import importlib.util
from pathlib import Path
import signal
import subprocess
import threading
import time
import copy
import tempfile
import unittest
import yaml
from unittest.mock import Mock, patch

from launch import LaunchContext
from launch_ros.actions import Node as LaunchNode
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from simulation_validation.control_feedback import ControlFeedback
from simulation_validation.kinematics import named_pose
from simulation_validation.live_control import (
    arguments, input_fingerprint, main, ready_target, stop_process, supervise, wait_for_visualization,
)
from simulation_validation.model import default_frames_file, load_yaml


PACKAGE = Path(__file__).resolve().parents[1]


class FeedbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        config = Path(default_frames_file()).parent
        self.node = ControlFeedback(('left',), load_yaml(config / 'fixed_frames.yaml')['frames'],
                                    named_pose(load_yaml(config / 'arm_poses.yaml'), 'init1'))
        self.messages = []
        self.patcher = patch.object(self.node.publisher, 'publish', self.messages.append)
        self.patcher.start()

    def tearDown(self):
        self.patcher.stop()
        self.node.destroy_node()

    def feedback(self, value=0.2):
        return JointState(name=[f'left_joint{i}' for i in range(1, 8)], position=[value] * 7)

    def test_no_initial_pose_and_no_unselected_arm(self):
        self.node.publish()
        self.assertEqual(self.messages, [])
        self.node.receive_feedback(self.feedback())
        self.node.publish()
        self.assertEqual(self.messages[-1].name, self.node.names)
        self.assertEqual(list(self.messages[-1].position), [0.2] * 7)

    def test_stale_pose_is_preserved_and_completion_freezes_it(self):
        self.node.receive_feedback(self.feedback())
        self.node.received_at = time.monotonic() - 2
        self.assertTrue(self.node.stale())
        self.node.publish()
        self.assertEqual(list(self.messages[-1].position), [0.2] * 7)
        self.node.receive_status(String(data='COMPLETED: done'))
        self.node.finish()
        self.node.receive_feedback(self.feedback(0.9))
        self.node.receive_status(String(data='RUNNING: another session'))
        self.node.publish()
        self.assertEqual(self.node.state, 'COMPLETED')
        self.assertEqual(list(self.messages[-1].position), [0.2] * 7)

    def test_full_fault_reason_is_kept_and_reset_between_sessions(self):
        self.node.receive_status(String(data='FAULT: left: 관절 증분 제한 초과 dt_ms=25'))
        self.assertEqual(self.node.state, 'FAULT')
        self.assertIn('dt_ms=25', self.node.status_text)
        self.node.reset()
        self.assertEqual(self.node.status_text, 'WAITING')

    def test_invalid_or_other_arm_feedback_is_rejected(self):
        for msg in (JointState(), self.feedback(float('nan')),
                    JointState(name=['right_joint1'], position=[0.0])):
            with self.subTest(msg=msg), self.assertRaises(RuntimeError):
                self.node.receive_feedback(msg)
        with self.assertRaises(RuntimeError):
            self.node.finish()

    def test_best_effort_input_reaches_reliable_visualization(self):
        # SDK 대신 ROS 발행기로 실제 QoS 연결을 확인한다.
        self.patcher.stop()
        source = Node('fake_motion_feedback')
        publisher = source.create_publisher(JointState, '/xarm_motion/joint_states',
                                            qos_profile_sensor_data)
        received = []
        source.create_subscription(JointState, '/simulation_validation/joint_states', received.append, 1)
        try:
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline and not received:
                publisher.publish(self.feedback())
                rclpy.spin_once(self.node, timeout_sec=0.02)
                rclpy.spin_once(source, timeout_sec=0.02)
            self.assertTrue(received, '실측→시각화 QoS 연결 실패')
            self.assertEqual(list(received[-1].position), [0.2] * 7)
        finally:
            source.destroy_node()
            self.patcher.start()


class RunnerTests(unittest.TestCase):
    def test_pre_align_step_uses_current_start_and_configured_target(self):
        args = arguments(['--arm', 'both', '--step', 'pre_align_ready'])
        self.assertIsNone(args.start_pose)
        self.assertIsNone(args.target_pose)
        for extra in (['--start-pose', 'init1'], ['--target-pose', 'init1']):
            with self.subTest(extra=extra), self.assertRaises(SystemExit):
                arguments(['--arm', 'both', '--step', 'pre_align_ready', *extra])
        with self.assertRaises(SystemExit):
            arguments(['--arm', 'left', '--step', 'pre_align_ready'])

    def test_ready_target_switches_both_visualization_goals(self):
        poses = load_yaml(Path(default_frames_file()).with_name('arm_poses.yaml'))
        poses['poses']['future_ready'] = copy.deepcopy(poses['poses']['pre_align_ready_v2'])
        poses['poses']['future_ready']['left']['base_T_flange'][0][3] = .4
        for name in ('pre_align_ready', 'pre_align_ready_v2', 'future_ready'):
            selected, goals = ready_target({'pre_align_ready': {'target_pose': name}}, poses)
            self.assertEqual(selected, name)
            for side in ('left', 'right'):
                self.assertAlmostEqual(goals[side].p.x(), poses['poses'][name][side]['base_T_flange'][0][3])
        for name in (None, False, 3, [], {}, '', ' ', 'unknown'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                ready_target({'pre_align_ready': {'target_pose': name}}, poses)
        broken = copy.deepcopy(poses)
        del broken['poses']['pre_align_ready_v2']['right']
        with self.assertRaises(ValueError):
            ready_target({'pre_align_ready': {'target_pose': 'pre_align_ready_v2'}}, broken)

    def test_input_fingerprint_matches_cpp(self):
        self.assertEqual(input_fingerprint(b'', b''), '08328807b4eb6fed')
        self.assertEqual(input_fingerprint(b'control', b'poses'), 'feeec3e03e80b89c')
        self.assertEqual(input_fingerprint('목표'.encode(), b'poses'), 'fe6319ad788cbf87')
        self.assertNotEqual(input_fingerprint(b'ab', b'c'), input_fingerprint(b'a', b'bc'))

    def test_invalid_target_never_starts_visualization_or_controller(self):
        parent = Path(default_frames_file()).parents[1] / '_colcon/build/simulation_validation'
        parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=parent) as tmp:
            config = Path(tmp)
            poses = load_yaml(Path(default_frames_file()).with_name('arm_poses.yaml'))
            for invalid in ('unknown', 'missing_right', 'bad_matrix'):
                broken = copy.deepcopy(poses)
                if invalid != 'unknown':
                    broken['poses'][invalid] = copy.deepcopy(poses['poses']['pre_align_ready_v2'])
                    if invalid == 'missing_right':
                        del broken['poses'][invalid]['right']
                    else:
                        broken['poses'][invalid]['left']['base_T_flange'][0][0] = float('nan')
                (config / 'control.yaml').write_text(yaml.safe_dump({'pre_align_ready': {'target_pose': invalid}}))
                (config / 'arm_poses.yaml').write_text(yaml.safe_dump(broken))
                with self.subTest(invalid=invalid), patch('simulation_validation.live_control.settings', return_value=config), \
                        patch('simulation_validation.live_control.sys.stdin.isatty', return_value=True), \
                        patch('simulation_validation.live_control.subprocess.Popen') as popen:
                    self.assertEqual(main(['--arm', 'both', '--step', 'pre_align_ready']), 1)
                    popen.assert_not_called()

    def test_runner_uses_selected_goal_and_checks_controller_inputs(self):
        source = Path(default_frames_file()).parent
        parent = source.parent / '_colcon/build/simulation_validation'
        parent.mkdir(parents=True, exist_ok=True)
        for target in ('pre_align_ready', 'pre_align_ready_v2'):
            with self.subTest(target=target), tempfile.TemporaryDirectory(dir=parent) as tmp:
                config = Path(tmp) / 'config'
                config.mkdir()
                control = load_yaml(source / 'control.yaml')
                control['pre_align_ready']['target_pose'] = target
                (config / 'control.yaml').write_text(yaml.safe_dump(control))
                for filename in ('arm_poses.yaml', 'fixed_frames.yaml'):
                    (config / filename).write_bytes((source / filename).read_bytes())
                executable = Path(tmp) / 'lib/auto_align/auto_align'
                executable.parent.mkdir(parents=True)
                executable.touch()
                expected = input_fingerprint((config / 'control.yaml').read_bytes(),
                                             (config / 'arm_poses.yaml').read_bytes())
                with patch('simulation_validation.live_control.settings', return_value=config), \
                        patch('simulation_validation.live_control.sys.stdin.isatty', return_value=True), \
                        patch('simulation_validation.live_control.get_package_prefix', return_value=tmp), \
                        patch('simulation_validation.live_control.ControlFeedback') as feedback, \
                        patch('simulation_validation.live_control.subprocess.Popen') as popen, \
                        patch('simulation_validation.live_control.wait_for_visualization'), \
                        patch('simulation_validation.live_control.supervise', return_value=0), \
                        patch('simulation_validation.live_control.stop_process'), \
                        patch('simulation_validation.live_control.rclpy.init'), \
                        patch('simulation_validation.live_control.rclpy.shutdown'):
                    self.assertEqual(main(['--arm', 'both', '--step', 'pre_align_ready']), 0)
                    self.assertEqual(popen.call_args_list[1].args[0],
                                     [str(executable), '--step', 'pre_align_ready', '--expected-inputs', expected])
                    goals = feedback.call_args.args[2]
                    coordinates = load_yaml(config / 'arm_poses.yaml')['poses'][target]
                    for side in ('left', 'right'):
                        self.assertAlmostEqual(goals[side].p.x(), coordinates[side]['base_T_flange'][0][3])

    def test_auto_align_gate_is_independent_of_standalone_gate(self):
        with patch('simulation_validation.live_control.load_yaml', return_value={
                'execution': {'standalone_enabled': True, 'auto_align_enabled': False}}), \
                patch('simulation_validation.live_control.subprocess.Popen') as popen:
            self.assertEqual(main(['--arm', 'both', '--step', 'pre_align_ready']), 1)
            popen.assert_not_called()

    def test_required_arguments_and_no_gate_override(self):
        for argv in ([], ['--arm', 'both'],
                     ['--arm', 'both', '--start-pose', 'init1', '--target-pose', 'init1', '--enabled']):
            with self.subTest(argv=argv), self.assertRaises(SystemExit):
                arguments(argv)

    def test_disabled_and_invalid_gate_never_start_processes(self):
        for enabled in (False, None, 'true', 1):
            with self.subTest(enabled=enabled), \
                    patch('simulation_validation.live_control.load_yaml',
                          return_value={'execution': {'standalone_enabled': enabled}}), \
                    patch('simulation_validation.live_control.subprocess.Popen') as popen:
                self.assertEqual(main(['--arm', 'both', '--start-pose', 'init1', '--target-pose', 'init1']), 1)
                popen.assert_not_called()

    def test_visualization_failure_prevents_connection(self):
        node = Mock()
        visual = Mock()
        visual.poll.return_value = 1
        with self.assertRaisesRegex(RuntimeError, '로봇에는 접속하지 않았습니다'):
            wait_for_visualization(node, visual, threading.Event())

    def test_visualization_exit_and_stale_feedback_stop_running_session(self):
        for exited, stale in ((True, False), (False, True)):
            with self.subTest(exited=exited, stale=stale):
                node, motion, visual = Mock(), Mock(), Mock()
                node.completed = False
                node.state = 'RUNNING'
                node.stale.return_value = stale
                motion.poll.return_value = None
                visual.poll.return_value = 0 if exited else None
                with patch('simulation_validation.live_control.rclpy.spin_once'), self.assertRaises(RuntimeError):
                    supervise(node, motion, visual, threading.Event())

    def test_terminal_fault_wins_over_stale_feedback_during_disconnect(self):
        for state in ('FAULT', 'STOPPED'):
            node, motion, visual = Mock(), Mock(), Mock()
            node.completed = False
            node.state = state
            node.status_text = f'{state}: left: 관절 증분 제한 초과 dt_ms=25'
            node.stale.return_value = True
            motion.poll.return_value = None
            visual.poll.return_value = None
            with patch('simulation_validation.live_control.rclpy.spin_once'), self.assertRaisesRegex(
                    RuntimeError, f'{state}: left: 관절 증분 제한 초과 dt_ms=25'):
                supervise(node, motion, visual, threading.Event())
            node.stale.assert_not_called()

    def test_success_keeps_viewer_until_user_stop(self):
        node, motion, visual = Mock(), Mock(), Mock()
        stopped = threading.Event()
        node.completed = False
        node.state = 'COMPLETED'
        node.latest = JointState()
        motion.poll.return_value = 0
        visual.poll.return_value = None

        def finish():
            node.completed = True

        node.finish.side_effect = finish
        calls = []

        def spin(*args, **kwargs):
            calls.append(1)
            if len(calls) == 3:
                stopped.set()

        with patch('simulation_validation.live_control.rclpy.spin_once', side_effect=spin):
            self.assertEqual(supervise(node, motion, visual, stopped), 0)
        node.finish.assert_called_once()
        self.assertEqual(len(calls), 3)

    def test_fault_and_user_stop_do_not_become_success(self):
        node, motion, visual = Mock(), Mock(), Mock()
        node.completed = False
        visual.poll.return_value = None
        motion.poll.return_value = 1
        with patch('simulation_validation.live_control.rclpy.spin_once'), self.assertRaises(RuntimeError):
            supervise(node, motion, visual, threading.Event())
        stopped = threading.Event()
        stopped.set()
        self.assertEqual(supervise(node, motion, visual, stopped), 130)

    def test_cleanup_sends_stop_signal_and_waits_without_killing(self):
        process = Mock()
        process.poll.return_value = None
        stop_process(process)
        process.send_signal.assert_called_once_with(signal.SIGINT)
        process.wait.assert_called_once_with(timeout=8)
        process.kill.assert_not_called()

    def test_shutdown_timeout_escalates_to_term_without_kill(self):
        process = Mock()
        process.poll.return_value = None
        process.wait.side_effect = subprocess.TimeoutExpired('fake', 8)
        with self.assertRaises(RuntimeError):
            stop_process(process)
        self.assertEqual([call.args[0] for call in process.send_signal.call_args_list],
                         [signal.SIGINT, signal.SIGTERM])
        process.kill.assert_not_called()


class LaunchTests(unittest.TestCase):
    def setUp(self):
        spec = importlib.util.spec_from_file_location('simulation_init1_launch', PACKAGE / 'launch/init1.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.module = module
        self.context = LaunchContext()
        self.context.launch_configurations.update({
            'mode': 'live_control', 'frames_file': default_frames_file(),
            'coordinates_file': str(Path(default_frames_file()).with_name('arm_poses.yaml')),
            'model1300_left': 'false', 'model1300_right': 'false',
            'show_environment': 'false', 'rviz': 'false',
            'left_ip': '192.0.2.1', 'right_ip': '192.0.2.2',
            'seed_file': str(PACKAGE / 'config/init1.yaml'), 'pose_name': 'init1',
            'start_pose': 'init1', 'target_pose': 'init1_floor_parallel_v2',
            'pose_sequence': '', 'duration': '5.0',
        })

    def test_live_control_has_no_sdk_reader_or_local_ik_publisher(self):
        nodes = [action for action in self.module.start(self.context) if isinstance(action, LaunchNode)]
        self.assertEqual(len(nodes), 2)
        self.context.launch_configurations['frames_file'] = str(PACKAGE / 'config/init1.yaml')
        with self.assertRaises(ValueError):
            self.module.start(self.context)

    def test_existing_modes_keep_exactly_one_original_joint_publisher(self):
        for mode, executable in (('static_rviz', 'rviz_joint_states'),
                                 ('dynamic_rviz', 'rviz_joint_states'),
                                 ('live_feedback', 'live_joint_states')):
            with self.subTest(mode=mode):
                self.context.launch_configurations['mode'] = mode
                nodes = [action for action in self.module.start(self.context) if isinstance(action, LaunchNode)]
                names = [node.node_executable for node in nodes]
                self.assertEqual(names, ['robot_state_publisher', executable, 'rviz2'])


if __name__ == '__main__':
    unittest.main()
