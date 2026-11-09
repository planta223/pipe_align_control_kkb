"""Own one existing pedal process; keyboard keys only publish its task selection."""

import argparse
import codecs
from contextlib import contextmanager
from datetime import datetime
import fcntl
import os
from pathlib import Path
import queue
import re
import select
import signal
import subprocess
import sys
import termios
import threading
import time
import tty


ACK = re.compile(r'Received task_num=(\d+) from ')


def key_modes(config):
    keys = config.get('keys')
    if not isinstance(keys, dict) or set(keys) != {'left', 'right', 'both'}:
        raise ValueError('keys에 left/right/both 키 목록이 필요합니다.')
    modes = {}
    for mode, aliases in keys.items():
        if not isinstance(aliases, list) or not aliases:
            raise ValueError(f'keys.{mode}는 비어 있지 않은 목록이어야 합니다.')
        for alias in aliases:
            if not isinstance(alias, str) or len(alias) != 1 or not alias.isprintable():
                raise ValueError(f'keys.{mode}: 키는 출력 가능한 한 문자여야 합니다.')
            alias = alias.lower()
            if len(alias) != 1:
                raise ValueError(f'keys.{mode}: 소문자 변환 후 한 문자여야 합니다.')
            if alias in modes and modes[alias] != mode:
                raise ValueError(f'선택 간 키가 중복됩니다: {alias!r}')
            modes[alias] = mode
    return modes


def load_config(path):
    import yaml

    config = yaml.safe_load(Path(path).read_text())
    if not isinstance(config, dict):
        raise ValueError('설정은 YAML mapping이어야 합니다.')
    tasks = [config.get(mode + '_task') for mode in ('left', 'right', 'both')]
    if any(type(task) is not int or task < 0 for task in tasks) or len(set(tasks)) != 3:
        raise ValueError('left/right/both_task는 서로 다른 음이 아닌 정수여야 합니다.')
    for name in ('left_label', 'right_label', 'both_label', 'input_device', 'log_dir'):
        if not isinstance(config.get(name), str) or not config[name].strip():
            raise ValueError(f'{name} 설정이 필요합니다.')
    if not re.fullmatch(r'/[A-Za-z_][A-Za-z_0-9]*(/[A-Za-z_][A-Za-z_0-9]*)*',
                        config.get('selection_topic', '')):
        raise ValueError('selection_topic은 절대 ROS topic 이름이어야 합니다.')
    key_modes(config)
    return config


def validate_routes(config, path):
    """Changing selection must not silently change the configured grasp profile."""
    import yaml

    routes = yaml.safe_load(Path(path).read_text())
    hands = routes['task_hand_routes']
    profiles = routes['task_grasp_profiles']
    selected = [config[m + '_task'] for m in ('left', 'right', 'both')]
    if {hands.get(selected[0]), hands.get(selected[1])} != {'L', 'R'} or hands.get(selected[2]) != 'D':
        raise ValueError('설정 task가 각각 한 손/반대 손/양손 route인지 확인하세요.')
    if any(t not in profiles for t in selected) or not all(
            profiles[t] == profiles[selected[0]] for t in selected):
        raise ValueError('세 task의 grasp profile이 다릅니다. 손 선택용 profile을 먼저 확인하세요.')


def existing_pedals(proc_root=Path('/proc')):
    found = []
    for entry in proc_root.iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            args = (entry / 'cmdline').read_bytes().split(b'\0')
        except OSError:
            continue
        if any(Path(a.decode(errors='replace')).name in
               ('foot_pedal_hand_controller', 'foot_pedal_hand_controller.launch.py')
               for a in args[:8] if a):
            found.append(int(entry.name))
    return found


def pedal_command(executable, config, routes):
    # Both existing task subscriptions use the private selection topic. Its callback
    # ignores repeated task values. No original /task_recog input is left connected.
    params = {
        'task_routing_enabled': 'true',
        'default_task_num': config['both_task'],
        'left_task_topic': config['selection_topic'],
        'right_task_topic': config['selection_topic'],
        'task_routes_file': str(routes),
        'input_device': config['input_device'],
        'input_mode': 'event',
        'both_hands_key_code': -1,
        'publish_initial_pose': 'false',
    }
    command = [str(executable), '--ros-args', '-r', '__node:=rohand_selector_pedal',
               '--log-level', 'info']
    for name, value in params.items():
        command += ['-p', f'{name}:={value}']
    return command


class PedalProcess:
    def __init__(self, command, log_path):
        self.events = queue.Queue()
        self.log_path = log_path
        self.log = log_path.open('x', encoding='utf-8')
        self.process = None
        self.reader = None
        env = dict(os.environ, ROS_LOG_DIR=str(log_path.parent / 'ros'),
                   RCUTILS_COLORIZED_OUTPUT='0', PYTHONUNBUFFERED='1')
        try:
            # DEVNULL alone is insufficient: the original code also opens /dev/tty.
            # setsid removes its controlling TTY, so only this selector reads keys.
            self.process = subprocess.Popen(
                command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, encoding='utf-8', errors='replace',
                start_new_session=True, env=env,
            )
        except BaseException:
            self.log.close()
            raise
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        for line in self.process.stdout:
            self.log.write(line)
            self.log.flush()
            if 'Loaded task hand routes from ' in line:
                self.events.put(('ready', None))
            match = ACK.search(line)
            if match:
                self.events.put(('ack', int(match[1])))
            if any(level in line for level in ('[WARN]', '[ERROR]', '[FATAL]')):
                self.events.put(('warning', line.strip()))

    def close(self):
        # Signal only the child session created here, never another controller.
        try:
            for sig in (signal.SIGINT, signal.SIGTERM):
                if self.process.poll() is not None:
                    break
                try:
                    os.killpg(self.process.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    self.process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    continue
            if self.process.poll() is None:
                raise RuntimeError(f'발판 프로세스 PID {self.process.pid} 종료를 확인하세요.')
        finally:
            if self.reader:
                self.reader.join(timeout=1)
            if not self.reader or not self.reader.is_alive():
                self.process.stdout.close()
                self.log.close()


@contextmanager
def keyboard():
    if not sys.stdin.isatty():
        raise RuntimeError('키보드를 입력할 터미널에서 실행하세요.')
    fd = sys.stdin.fileno()
    previous = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield fd
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, previous)


class Display:
    def __init__(self, config):
        self.text = ''
        help_parts = []
        for mode in ('left', 'right', 'both'):
            keys = '/'.join('Space' if key == ' ' else key for key in config['keys'][mode])
            help_parts.append(f'{keys}:{config[mode + "_label"]}')
        print('  '.join(help_parts) + '  Ctrl+C:종료', flush=True)

    @staticmethod
    def colored(text):
        if text.startswith('선택: '):
            return '선택: \033[31m' + text[len('선택: '):] + '\033[0m'
        return text

    def status(self, text):
        if text != self.text:
            self.text = text
            print('\r\033[2K' + self.colored(text), end='', flush=True)

    def warning(self, text):
        text = text.split(']: ', 1)[-1]
        if len(text) > 72:
            text = text[:69] + '...'
        print('\r\033[2K경고: ' + text, flush=True)
        print(self.colored(self.text), end='', flush=True)


def control_loop(node, publisher, child, fd, config, ui):
    import rclpy
    from std_msgs.msg import Int32

    desired = 'both'
    keys = key_modes(config)
    decoder = codecs.getincrementaldecoder('utf-8')(errors='ignore')
    applied = requested = config['both_task']
    ready = False
    started = changed_at = time.monotonic()
    ui.status('발판 프로그램 준비 중...')
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0)
        if child.process.poll() is not None:
            raise RuntimeError(f'발판 프로그램 종료(code={child.process.returncode}). 로그: {child.log_path}')
        while True:
            try:
                event, value = child.events.get_nowait()
            except queue.Empty:
                break
            if event == 'ready':
                ready = True
            elif event == 'ack':
                applied = value
            elif event == 'warning':
                ui.warning(value)
        readable, _, _ = select.select([fd], [], [], 0.05)
        if readable:
            data = os.read(fd, 64)
            if not data:
                break
            # 나누어 수신된 UTF-8도 완성된 문자부터 처리한다.
            for char in decoder.decode(data).lower():
                desired = keys.get(char, desired)
        target = config[desired + '_task']
        if ready and publisher.get_subscription_count() and target != requested:
            publisher.publish(Int32(data=target))
            requested = target
            changed_at = time.monotonic()
        if not ready:
            if time.monotonic() - started > 10:
                raise RuntimeError(f'발판 시작 시간 초과. 로그: {child.log_path}')
        elif applied == target and requested == target:
            ui.status('선택: ' + config[desired + '_label'])
        else:
            ui.status('변경 대기: ' + config[desired + '_label'])
            if time.monotonic() - changed_at > 5:
                raise RuntimeError(f'손 선택 적용을 확인하지 못했습니다. 로그: {child.log_path}')


def main(args=None):
    from ament_index_python.packages import get_package_prefix, get_package_share_directory

    parser = argparse.ArgumentParser(description='ROHand keyboard selector')
    parser.add_argument('--config', default=str(
        Path(get_package_share_directory('rohand_selector')) / 'config/selector.yaml'))
    options = parser.parse_args(args)
    child = node = lock = None
    context_started = False
    return_code = 0
    previous_term = signal.getsignal(signal.SIGTERM)
    try:
        config = load_config(options.config)
        if not sys.stdin.isatty():
            raise RuntimeError('키보드를 입력할 터미널에서 실행하세요.')
        pids = existing_pedals()
        if pids:
            raise RuntimeError(f'기존 발판 프로그램이 실행 중입니다(PID {pids}). 해당 창에서 종료 후 실행하세요.')
        share = Path(get_package_share_directory('rohand_foot_pedal_controller'))
        executable = Path(get_package_prefix('rohand_foot_pedal_controller')) / 'lib/rohand_foot_pedal_controller/foot_pedal_hand_controller'
        routes = share / 'config/task_hand_routes.yaml'
        validate_routes(config, routes)
        if not executable.is_file():
            raise RuntimeError(f'기존 발판 실행 파일이 없습니다: {executable}')
        log_root = Path(config['log_dir']).expanduser()
        log_root.mkdir(parents=True, exist_ok=True)
        lock = (log_root / 'selector.lock').open('a')
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('다른 rohand_selector가 실행 중입니다.') from None
        run_dir = log_root / (datetime.now().strftime('%Y%m%d_%H%M%S_%f') + f'_{os.getpid()}')
        run_dir.mkdir()
        os.environ['ROS_LOG_DIR'] = str(run_dir / 'ros')
        import rclpy
        from rclpy.node import Node
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import Int32

        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        context_started = True
        node = Node('rohand_selector', enable_rosout=False, start_parameter_services=False)
        publisher = node.create_publisher(Int32, config['selection_topic'], 1)
        def interrupted(signum, frame):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, interrupted)
        with keyboard() as fd:
            print(f'로그: {run_dir / "pedal.log"}', flush=True)
            child = PedalProcess(pedal_command(executable, config, routes), run_dir / 'pedal.log')
            control_loop(node, publisher, child, fd, config, Display(config))
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'\n오류: {exc}', file=sys.stderr, flush=True)
        return_code = 1
    finally:
        if child:
            try:
                child.close()
            except Exception as exc:
                print(f'\n오류: {exc}', file=sys.stderr)
                return_code = 1
        if node:
            node.destroy_node()
        if context_started:
            rclpy.shutdown()
        if lock:
            lock.close()
        signal.signal(signal.SIGTERM, previous_term)
        print()
    return return_code
