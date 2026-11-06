"""실제 관절각만 조회해 시각화 토픽에 발행한다."""

from dataclasses import dataclass, replace
import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def sdk_factory():
    # 정적 모드는 SDK 설치가 필요 없다.
    try:
        from xarm.wrapper import XArmAPI
    except ImportError as exc:
        raise RuntimeError('live 모드에는 xarm-python-sdk가 필요합니다.') from exc
    return XArmAPI


@dataclass(frozen=True)
class Feedback:
    positions: tuple = ()
    received_at: float = 0.0
    error: str = '연결 중'


class ArmReader(threading.Thread):
    def __init__(self, side, ip, factory, stop, period):
        super().__init__(name=f'{side}_joint_reader', daemon=True)
        self.ip = ip
        self.factory = factory
        self.stop = stop
        self.period = period
        self.lock = threading.Lock()
        self.feedback = Feedback()

    def snapshot(self):
        with self.lock:
            return self.feedback

    def run(self):
        while not self.stop.is_set():
            arm = None
            try:
                arm = self.factory(
                    self.ip, is_radian=True, do_not_open=True,
                    enable_report=False, timed_comm=False,
                    enable_heartbeat=False, max_callback_thread_count=0,
                )
                arm.connect()
                arm.set_timeout(1.0)
                while not self.stop.is_set():
                    started = time.monotonic()
                    code, states = arm.get_joint_states(is_radian=True, num=1)
                    if code != 0:
                        raise RuntimeError(f'관절 조회 실패: code={code}')
                    positions = tuple(states[0])
                    if len(positions) != 7 or any(
                            type(v) not in (float, int) or not math.isfinite(v)
                            for v in positions):
                        raise ValueError('유효한 관절각 7개가 필요합니다.')
                    with self.lock:
                        self.feedback = Feedback(positions, time.monotonic(), '')
                    self.stop.wait(max(0.0, self.period - (time.monotonic() - started)))
            except Exception as exc:
                with self.lock:
                    self.feedback = replace(self.feedback, error=str(exc))
            finally:
                if arm is not None:
                    try:
                        arm.disconnect()
                    except Exception:
                        pass
            # 연결 실패 시 재시도하며 마지막 관절각은 유지한다.
            self.stop.wait(2.0)


class LiveJointStates(Node):
    def __init__(self):
        super().__init__('live_joint_states')
        addresses = {side: self.declare_parameter(f'{side}_ip', ip).value
                     for side, ip in (('left', '192.168.2.31'), ('right', '192.168.2.32'))}
        factory = sdk_factory()
        self.stop = threading.Event()
        self.readers = {side: ArmReader(side, ip, factory, self.stop, 0.05)
                        for side, ip in addresses.items()}
        self.health = {}
        self.publisher = self.create_publisher(JointState, 'joint_states', 1)
        self.timer = self.create_timer(0.05, self.publish_feedback)
        for reader in self.readers.values():
            reader.start()
        self.get_logger().info(
            '실제 양팔 관절각을 20 Hz로 조회합니다. 로봇 이동·설정 변경 명령은 보내지 않습니다.')

    def publish_feedback(self):
        msg = JointState()
        # 시각화 시각이다. 수신 중단 시 마지막 자세를 표시한다.
        msg.header.stamp = self.get_clock().now().to_msg()
        now = time.monotonic()
        for side, reader in self.readers.items():
            feedback = reader.snapshot()
            if feedback.error:
                status = feedback.error
            elif now - feedback.received_at > 1.0:
                status = '1초 이상 새 관절각이 없습니다.'
            else:
                status = 'ok'
            if self.health.get(side) != status:
                if status == 'ok':
                    self.get_logger().info(f'{side}: 실제 관절각 수신 중 ({reader.ip})')
                else:
                    display = '마지막 자세 유지' if feedback.positions else '자세 미표시'
                    self.get_logger().warning(f'{side}: {status} ({reader.ip}), {display}')
                self.health[side] = status
            if feedback.positions:
                msg.name.extend(f'{side}_joint{i}' for i in range(1, 8))
                msg.position.extend(feedback.positions)
        if msg.name:
            self.publisher.publish(msg)

    def destroy_node(self):
        self.timer.cancel()
        self.stop.set()
        for reader in self.readers.values():
            reader.join(timeout=5.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LiveJointStates()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
