"""Exercise the real ROS input path in an isolated domain; never open an SDK backend."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import SingleThreadedExecutor


def executable():
    return os.environ.get('AUTO_ALIGN_TEST_EXE', str(
        Path(__file__).resolve().parents[3] / '_colcon/build/auto_align/auto_align'))


@pytest.mark.parametrize('hz', [3.0, 12.0])
def test_measurement_process_accepts_variable_rate_without_robot(hz, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '117')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('synthetic_pipe_source', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    publishers = [node.create_publisher(PoseStamped, '/vision/' + side + '_pipe/pose',
                                       qos_profile_sensor_data) for side in ('left', 'right')]
    child = subprocess.Popen([executable(), '--step', 'vision_measure'],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        started = next_frame = time.monotonic()
        while time.monotonic() - started < 6.0:
            if child.poll() is not None:
                break
            if time.monotonic() >= next_frame:
                stamp = node.get_clock().now().to_msg()
                for side, publisher in enumerate(publishers):
                    m = PoseStamped()
                    m.header.stamp = stamp
                    m.header.frame_id = 'camera_left_optical_frame'
                    m.pose.position.x = -.416 if side == 0 else .416
                    m.pose.position.y = side * .01
                    m.pose.position.z = .4
                    m.pose.orientation.y = (1 if side == 0 else -1) * math.sqrt(.5)
                    m.pose.orientation.w = math.sqrt(.5)
                    publisher.publish(m)
                next_frame = time.monotonic() + 1 / hz
            executor.spin_once(timeout_sec=.01)
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
        output, _ = child.communicate(timeout=5)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
    assert output.count('VISION:') >= 2, output
    assert 'source_age_s=' in output
    assert child.returncode in (0, 130), output


def test_full_control_refuses_unattended_start_before_any_sdk_connection():
    # Even if a user enables the YAML gate later, stdin=DEVNULL prevents SDK
    # construction. This test must never depend on an accessible real robot.
    child = subprocess.run([executable(), '--step', 'vision_align'],
                           stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, text=True, timeout=5)
    assert child.returncode != 0
    assert 'vision_align_enabled' in child.stdout or '터미널' in child.stdout


def test_old_images_never_produce_a_measurement(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '118')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node('old_pipe_source', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    publishers = [node.create_publisher(PoseStamped, '/vision/' + side + '_pipe/pose',
                                       qos_profile_sensor_data) for side in ('left', 'right')]
    child = subprocess.Popen([executable(), '--step', 'vision_measure'],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        started = time.monotonic()
        while time.monotonic() - started < 10 and child.poll() is None:
            stamp = node.get_clock().now().to_msg()
            stamp.sec -= 2
            for publisher in publishers:
                m = PoseStamped()
                m.header.stamp = stamp
                m.header.frame_id = 'camera_left_optical_frame'
                m.pose.orientation.w = 1.0
                publisher.publish(m)
            executor.spin_once(timeout_sec=.1)
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
        output, _ = child.communicate(timeout=5)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
    assert child.returncode == 1, output
    assert 'VISION:' not in output
    assert '새 정지 영상' in output
