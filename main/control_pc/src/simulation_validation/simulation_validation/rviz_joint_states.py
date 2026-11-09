"""이름으로 선택한 자세·경로를 로컬 IK로 계산해 화면에만 표시한다."""

import math
import time

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray

from .kinematics import ArmKinematics, SIDES, dynamic_pose_names, make_plan, named_pose
from .model import joint_positions, load_yaml


class RvizJointStates(Node):
    def __init__(self):
        super().__init__('rviz_joint_states')
        mode = self.declare_parameter('mode', 'static_rviz').value
        if mode not in ('static_rviz', 'dynamic_rviz'):
            raise ValueError('화면 전용 모드만 사용할 수 있습니다.')
        config = load_yaml(self.declare_parameter('frames_file', '').value)
        coordinates = load_yaml(self.declare_parameter('coordinates_file', '').value)
        seed_file = self.declare_parameter('seed_file', '').value
        self.names, seed = joint_positions(load_yaml(seed_file))
        description = self.declare_parameter('robot_description', '').value
        self.frames = config['frames']
        arms = {side: ArmKinematics(description, self.frames[f'{side}_base'],
                                    self.frames[f'{side}_flange']) for side in SIDES}
        if self.names != sum((arms[side].names for side in SIDES), []):
            raise ValueError('모델의 관절 순서와 초기 추정값의 순서가 다릅니다.')
        if mode == 'static_rviz':
            selected = [self.declare_parameter('pose_name', 'init1').value]
        else:
            selected = dynamic_pose_names(
                self.declare_parameter('start_pose', '').value,
                self.declare_parameter('target_pose', '').value,
                self.declare_parameter('pose_sequence', '').value)
        duration = self.declare_parameter('duration', 5.0).value
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError('duration은 양수 초 단위여야 합니다.')
        poses = [named_pose(coordinates, name) for name in selected]
        steps = max(1, math.ceil(duration * 20))
        self.plan = make_plan(arms, poses, dict(left=seed[:7], right=seed[7:]), steps)
        self.goal = poses[-1]
        self.interval = duration / steps
        self.dynamic = mode == 'dynamic_rviz'
        self.started = time.monotonic()
        self.finished = False
        self.publisher = self.create_publisher(JointState, 'joint_states', 1)
        self.goal_publisher = self.create_publisher(MarkerArray, 'goal_axes', 1)
        self.timer = self.create_timer(0.05 if self.dynamic else 0.5, self.publish_pose)
        self.get_logger().info(
            f'{mode}: {" → ".join(selected)}, {len(self.plan)}개 자세 계산 완료. '
            '실물 접속·제어 명령 없이 로컬 IK 결과만 표시합니다.')
        if self.dynamic:
            self.get_logger().info('시작 자세를 2초 표시한 뒤 재생하고 마지막 자세를 유지합니다.')
        self.publish_pose()

    def publish_pose(self):
        elapsed = max(0.0, time.monotonic() - self.started - 2.0)
        index = min(len(self.plan) - 1, int(elapsed / self.interval)) if self.dynamic else 0
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = self.plan[index]
        self.publisher.publish(msg)
        self.publish_goals(msg.header.stamp)
        if self.dynamic and index == len(self.plan) - 1 and not self.finished:
            self.get_logger().info('화면 재생 완료. 목표 자세를 유지합니다.')
            self.finished = True

    def publish_goals(self, stamp):
        markers = MarkerArray()
        for side in SIDES:
            frame = self.goal[side]
            for axis in range(3):
                marker = Marker()
                marker.header.frame_id = self.frames[f'{side}_base']
                marker.header.stamp = stamp
                marker.ns = f'{side}_flange_target'
                marker.id = axis
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.scale.x, marker.scale.y, marker.scale.z = 0.004, 0.008, 0.012
                marker.color.r = float(axis == 0)
                marker.color.g = float(axis == 1)
                marker.color.b = float(axis == 2)
                marker.color.a = 0.6
                direction = (frame.M.UnitX, frame.M.UnitY, frame.M.UnitZ)[axis]()
                tip = frame.p + direction * 0.1
                marker.points = [Point(x=frame.p[0], y=frame.p[1], z=frame.p[2]),
                                 Point(x=tip[0], y=tip[1], z=tip[2])]
                markers.markers.append(marker)
        self.goal_publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RvizJointStates()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
