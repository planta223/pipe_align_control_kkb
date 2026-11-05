"""init 1 목표 관절각을 시각화 토픽에 발행한다."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .model import joint_positions, load_yaml


class Init1JointStates(Node):
    def __init__(self):
        super().__init__('init1_joint_states')
        path = self.declare_parameter('pose_file', '').value
        self.names, self.positions = joint_positions(load_yaml(path))
        self.publisher = self.create_publisher(JointState, 'joint_states', 1)
        self.timer = self.create_timer(0.5, self.publish_pose)
        self.get_logger().info('init 1 목표 자세를 표시합니다. 실제 로봇 피드백이 아닙니다.')
        self.publish_pose()

    def publish_pose(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = self.positions
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Init1JointStates()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
