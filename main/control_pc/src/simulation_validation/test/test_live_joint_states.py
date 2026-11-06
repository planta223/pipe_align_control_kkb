import math
import threading
import time
import unittest
from unittest.mock import patch

import rclpy

from simulation_validation.live_joint_states import ArmReader, Feedback, LiveJointStates


class FakeArm:
    # 조회에 필요한 메서드만 제공해 다른 SDK 호출을 검출한다.
    def __init__(self, responses):
        self.responses = iter(responses)
        self.calls = []

    def connect(self):
        self.calls.append('connect')

    def set_timeout(self, value):
        self.calls.append(('set_timeout', value))

    def get_joint_states(self, **kwargs):
        self.calls.append(('get_joint_states', kwargs))
        return next(self.responses, (1, []))

    def disconnect(self):
        self.calls.append('disconnect')


class ReaderTests(unittest.TestCase):
    def read_until_error(self, responses):
        arm = FakeArm(responses)
        arguments = []

        def factory(*args, **kwargs):
            arguments.append((args, kwargs))
            return arm

        stop = threading.Event()
        reader = ArmReader('left', '192.0.2.1', factory, stop, 0.001)
        reader.start()
        try:
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                feedback = reader.snapshot()
                if feedback.error and feedback.error != '연결 중':
                    return feedback, arm, arguments
                time.sleep(0.005)
            self.fail('조회 실패가 보고되지 않았습니다.')
        finally:
            stop.set()
            reader.join(timeout=2.0)
            self.assertFalse(reader.is_alive())

    def test_failure_preserves_radians_and_only_read_methods_are_called(self):
        angles = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
        feedback, arm, arguments = self.read_until_error([(0, [angles]), (1, [])])
        self.assertEqual(feedback.positions, tuple(angles))
        self.assertGreater(feedback.received_at, 0)
        self.assertIn('code=1', feedback.error)
        self.assertEqual(arm.calls, [
            'connect', ('set_timeout', 1.0),
            ('get_joint_states', {'is_radian': True, 'num': 1}),
            ('get_joint_states', {'is_radian': True, 'num': 1}), 'disconnect',
        ])
        self.assertTrue(arguments[0][1]['do_not_open'])
        for option in ('enable_report', 'enable_heartbeat', 'timed_comm'):
            self.assertFalse(arguments[0][1][option])

    def test_invalid_feedback_never_becomes_a_pose(self):
        for values in ([0.0] * 6, [math.nan] * 7, [math.inf] * 7, ['0'] * 7):
            with self.subTest(values=values):
                feedback, _, _ = self.read_until_error([(0, [values])])
                self.assertEqual(feedback.positions, ())
                self.assertEqual(feedback.received_at, 0)

    def test_invalid_packet_preserves_last_valid_pose(self):
        feedback, _, _ = self.read_until_error([(0, [[0.1] * 7]), (0, [[0.2] * 6])])
        self.assertEqual(feedback.positions, (0.1,) * 7)


class PublishTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # 실물 접속 없이 발행 내용과 수신 중단 처리를 검사한다.
        with patch('simulation_validation.live_joint_states.sdk_factory'), \
                patch.object(ArmReader, 'start'):
            self.node = LiveJointStates()
        self.messages = []
        self.publisher = patch.object(self.node.publisher, 'publish', self.messages.append)
        self.publisher.start()

    def tearDown(self):
        self.publisher.stop()
        with patch.object(ArmReader, 'join'):
            self.node.destroy_node()

    def test_no_initial_or_missing_arm_angles_are_invented(self):
        self.node.publish_feedback()
        self.assertEqual(self.messages, [])
        self.node.readers['left'].feedback = Feedback((0.2,) * 7, time.monotonic(), '')
        self.node.publish_feedback()
        self.assertEqual(self.messages[-1].name, [f'left_joint{i}' for i in range(1, 8)])
        self.assertEqual(list(self.messages[-1].position), [0.2] * 7)

    def test_stale_and_disconnected_arms_hold_last_pose_and_recover(self):
        now = time.monotonic()
        self.node.readers['left'].feedback = Feedback((0.1,) * 7, now - 2, '')
        self.node.readers['right'].feedback = Feedback((-0.3,) * 7, now, '접속 끊김')
        self.node.publish_feedback()
        self.assertNotEqual(self.node.health['left'], 'ok')
        self.assertEqual(self.node.health['right'], '접속 끊김')
        self.assertEqual(list(self.messages[-1].position), [0.1] * 7 + [-0.3] * 7)
        for reader in self.node.readers.values():
            reader.feedback = Feedback((0.4,) * 7, time.monotonic(), '')
        self.node.publish_feedback()
        self.assertEqual(self.node.health, {'left': 'ok', 'right': 'ok'})
        self.assertEqual(list(self.messages[-1].position), [0.4] * 14)


if __name__ == '__main__':
    unittest.main()
