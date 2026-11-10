"""제어기의 실측값을 시각화에 전달하고 완료 후 마지막 자세를 유지한다."""

from copy import deepcopy
import math
import time

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .rviz_joint_states import goal_markers


class ControlFeedback(Node):
    def __init__(self, sides, frames, goals, source='xarm_motion'):
        super().__init__('control_feedback', namespace='simulation_validation')
        if source not in ('xarm_motion', 'auto_align'):
            raise ValueError('지원하지 않는 제어 피드백입니다.')
        self.sides = sides
        self.frames = frames
        self.goals = {side: goals[side] for side in sides}
        self.names = [f'{side}_joint{i}' for side in sides for i in range(1, 8)]
        self.latest = None
        self.received_at = None
        self.state = 'WAITING'
        self.status_text = 'WAITING'
        self.completed = False
        self.description_received = False
        self.publisher = self.create_publisher(JointState, 'joint_states', 1)
        self.goal_publisher = self.create_publisher(MarkerArray, 'goal_axes', 1)
        self.status_publisher = self.create_publisher(Marker, 'control_status', 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(JointState, f'/{source}/joint_states',
                                 self.receive_feedback, qos_profile_sensor_data)
        self.create_subscription(String, f'/{source}/status', self.receive_status, latched)
        self.create_subscription(String, 'robot_description', self.receive_description, latched)
        self.timer = self.create_timer(0.05, self.publish)

    def receive_description(self, msg):
        self.description_received = bool(msg.data)

    def receive_feedback(self, msg):
        if self.completed:
            return
        if (list(msg.name) != self.names or len(msg.position) != len(self.names)
                or any(not math.isfinite(v) for v in msg.position)):
            raise RuntimeError('선택한 팔과 일치하는 유효한 실측 관절각이 필요합니다.')
        self.latest = deepcopy(msg)
        self.received_at = time.monotonic()

    def receive_status(self, msg):
        # 이전 실행의 transient 상태는 새 제어기 실행 전에 reset한다.
        if not self.completed:
            self.status_text = msg.data
            self.state = msg.data.split(':', 1)[0]

    def reset(self):
        self.latest = None
        self.received_at = None
        self.state = 'WAITING'
        self.status_text = 'WAITING'

    def stale(self):
        return self.received_at is not None and time.monotonic() - self.received_at > 1.0

    def finish(self):
        if self.state != 'COMPLETED' or self.latest is None:
            raise RuntimeError('정상 완료 상태와 최종 실측 피드백을 확인하지 못했습니다.')
        self.completed = True
        self.get_logger().info('이동 완료: 마지막 수신 자세를 유지합니다. 실시간 조회는 종료됐습니다. Ctrl+C로 닫으세요.')

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        if self.latest is not None:
            msg = deepcopy(self.latest)
            # 화면 유지용 시각이며 실측 수신 여부는 별도로 표시한다.
            msg.header.stamp = stamp
            self.publisher.publish(msg)
        self.goal_publisher.publish(goal_markers(self.goals, self.frames, stamp))
        marker = Marker()
        marker.header.frame_id = self.frames['base']
        marker.header.stamp = stamp
        marker.ns = 'live_control_status'
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.z = 0.65
        marker.scale.z = 0.035
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.text = self.state
        if self.completed:
            marker.text = 'COMPLETED - last received pose (not live)'
        elif self.stale():
            marker.text = 'STALE - last received pose'
            marker.color.r, marker.color.g = 1.0, 0.2
        elif self.latest is None:
            marker.text += ' - waiting for measured joints'
            marker.color.r = 1.0
        self.status_publisher.publish(marker)
