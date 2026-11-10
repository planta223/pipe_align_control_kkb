"""같은 터미널의 Enter·Ctrl+C를 유지하며 제어와 시각화를 실행한다."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.signals import SignalHandlerOptions

from .control_feedback import ControlFeedback
from .kinematics import named_pose
from .model import default_frames_file, load_yaml


def settings(gate='standalone_enabled'):
    config_dir = Path(default_frames_file()).parent
    control = load_yaml(config_dir / 'control.yaml')
    # 원본 파일만 검사한다. 최종 허용 검사는 C++ 실행기가 다시 수행한다.
    if control.get('execution', {}).get(gate) is not True:
        raise ValueError(f'control.yaml의 execution.{gate}=true가 필요합니다. 실행 옵션으로 변경할 수 없습니다.')
    for side in ('left', 'right'):
        if type(control['robots'][side]['model1300']) is not bool:
            raise ValueError(f'{side} model1300은 bool이어야 합니다.')
    return config_dir


def arguments(argv):
    parser = argparse.ArgumentParser(description='실측 시각화와 C++ 제어기를 함께 실행합니다. READY 후 Enter로 이동합니다.')
    parser.add_argument('--arm', required=True, choices=('left', 'right', 'both'))
    parser.add_argument('--start-pose')
    parser.add_argument('--target-pose')
    parser.add_argument('--step', choices=('pre_align_ready',), help='현재 실측 자세에서 auto_align 단계를 실행합니다.')
    parser.add_argument('--no-rviz', action='store_true', help='GUI 없이 관절각·TF만 발행합니다.')
    args = parser.parse_args(argv)
    if args.step:
        if args.arm != 'both' or args.start_pose or args.target_pose:
            parser.error('pre_align_ready는 --arm both를 사용하며 시작·목표 자세 인자를 받지 않습니다.')
        args.target_pose = 'pre_align_ready'
    elif not args.start_pose or not args.target_pose:
        parser.error('--start-pose와 --target-pose는 모두 필수입니다.')
    return args


def stop_process(process, group=False):
    if process is None or (not group and process.poll() is not None):
        return
    send = (lambda sig: os.killpg(process.pid, sig)) if group else process.send_signal
    try:
        send(signal.SIGINT)
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        send(signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired as exc:
            # 제어기를 강제 제거해 중지 처리가 사라지는 것을 피한다.
            raise RuntimeError('종료 요청에 응답하지 않습니다. 로봇 상태와 프로세스를 직접 확인하세요.') from exc
    except ProcessLookupError:
        pass


def wait_for_visualization(node, visual, stopped, rviz=True):
    deadline = time.monotonic() + 20.0
    while not stopped.is_set():
        if visual.poll() is not None:
            raise RuntimeError('시각화가 시작하지 못했습니다. 로봇에는 접속하지 않았습니다.')
        rclpy.spin_once(node, timeout_sec=0.05)
        if (node.description_received and node.publisher.get_subscription_count() > 0
                and (not rviz or node.status_publisher.get_subscription_count() > 0)):
            return
        if time.monotonic() > deadline:
            raise RuntimeError('시각화 준비 시간 초과입니다. 로봇에는 접속하지 않았습니다.')


def supervise(node, motion, visual, stopped):
    completed_at = None
    while not stopped.is_set():
        rclpy.spin_once(node, timeout_sec=0.05)
        if visual.poll() is not None:
            if node.completed:
                return 0
            raise RuntimeError('시각화가 종료되어 제어기에 중지를 요청합니다.')
        code = motion.poll()
        if code is not None and not node.completed:
            if code != 0:
                raise RuntimeError(f'제어기가 오류로 종료했습니다: code={code}')
            # 서로 다른 DDS 토픽의 마지막 메시지를 처리할 시간을 둔다.
            if completed_at is None:
                completed_at = time.monotonic()
            if node.state == 'COMPLETED' and node.latest is not None:
                node.finish()
            elif time.monotonic() - completed_at > 1.0:
                raise RuntimeError('제어기 종료 후 완료 상태·실측 피드백을 받지 못했습니다.')
        if not node.completed and node.state != 'COMPLETED' and node.stale():
            raise RuntimeError('실측 시각화가 1초 이상 갱신되지 않아 제어기에 중지를 요청합니다.')
    return 0 if node.completed else 130


def main(argv=None):
    args = arguments(argv)
    node = motion = visual = None
    stopped = threading.Event()
    previous_signals = {}
    initialized = False
    result = 1
    try:
        source = 'auto_align' if args.step else 'xarm_motion'
        config_dir = settings('auto_align_enabled' if args.step else 'standalone_enabled')
        if not sys.stdin.isatty():
            raise ValueError('Enter·Ctrl+C 입력이 가능한 터미널에서 실행하세요.')
        poses = load_yaml(config_dir / 'arm_poses.yaml')
        if args.start_pose:
            named_pose(poses, args.start_pose)
        goals = named_pose(poses, args.target_pose)
        frames = load_yaml(config_dir / 'fixed_frames.yaml')['frames']
        executable = Path(get_package_prefix(source)) / f'lib/{source}/{source}'
        if not executable.is_file():
            raise FileNotFoundError(f'{source}을 먼저 빌드하세요.')
        log_dir = config_dir.parent / '_log/simulation_validation'
        log_dir.mkdir(parents=True, exist_ok=True)
        os.environ['ROS_LOG_DIR'] = str(log_dir)
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous_signals[sig] = signal.signal(sig, lambda *_: stopped.set())
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        sides = ('left', 'right') if args.arm == 'both' else (args.arm,)
        node = ControlFeedback(sides, frames, goals, source=source)
        command = ['ros2', 'launch', 'simulation_validation', 'init1.launch.py',
                   'mode:=live_control', f'rviz:={str(not args.no_rviz).lower()}']
        visual = subprocess.Popen(command, stdin=subprocess.DEVNULL, start_new_session=True)
        wait_for_visualization(node, visual, stopped, rviz=not args.no_rviz)
        if not stopped.is_set():
            node.reset()
            # stdin을 상속해 C++ 실행기의 TTY 확인·Enter·중지 처리를 유지한다.
            motion_args = (['--step', args.step] if args.step else
                           ['--arm', args.arm, '--start-pose', args.start_pose,
                            '--target-pose', args.target_pose])
            motion = subprocess.Popen([str(executable), *motion_args])
            result = supervise(node, motion, visual, stopped)
        else:
            result = 130
    except Exception as exc:
        print(f'[경고] {exc}', file=sys.stderr, flush=True)
        result = 1
    finally:
        # 제어기의 중지 처리를 먼저 완료한다.
        for process, group in ((motion, False), (visual, True)):
            try:
                stop_process(process, group)
            except Exception as exc:
                print(f'[경고] {exc}', file=sys.stderr, flush=True)
                result = 1
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous_signals.items():
            signal.signal(sig, handler)
    return result
