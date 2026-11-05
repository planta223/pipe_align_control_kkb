"""No ROS graph, USB, input device, or hand driver is used by these tests."""

import ast
import copy
import json
import os
from pathlib import Path
import pty
import queue
import select
import shutil
import signal
import subprocess
import sys
import tempfile
import termios
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml

from rohand_selector import selector as s


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parent
LEGACY = Path('/home/rico/project/robothand_ws/rohand_ctrl_ws/install/rohand_foot_pedal_controller')
ROUTES = LEGACY / 'share/rohand_foot_pedal_controller/config/task_hand_routes.yaml'
SOURCE = LEGACY / 'lib/python3.10/site-packages/rohand_foot_pedal_controller/foot_pedal_hand_controller.py'


class SelectorTests(unittest.TestCase):
    def setUp(self):
        directory = WORKSPACE / 'build/rohand_selector/test_tmp'
        directory.mkdir(parents=True, exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=directory)
        self.addCleanup(self.temp.cleanup)
        self.temp_path = Path(self.temp.name)
        self.config = s.load_config(PACKAGE / 'config/selector.yaml')

    def legacy_callbacks(self):
        # Execute only the existing selection/toggle methods with fake publishers.
        # Do not instantiate its ROS Node or start its hardware event reader.
        tree = ast.parse(SOURCE.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'FootPedalHandController')
        names = {'_on_task_num', '_activate_profile_for_task', '_selected_hands_for_task',
                 '_toggle_task_routed_hands'}
        methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
        module = ast.Module(body=[ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0),
                                  ast.ClassDef(name='Callbacks', bases=[], keywords=[], body=methods, decorator_list=[])],
                            type_ignores=[])
        namespace = {}
        exec(compile(ast.fix_missing_locations(module), str(SOURCE), 'exec'), namespace)
        pedal = namespace['Callbacks']()
        data = yaml.safe_load(ROUTES.read_text())
        hand_names = {'L': ('left',), 'R': ('right',), 'D': ('left', 'right')}
        pedal._task_hand_routes = {k: hand_names[v] for k, v in data['task_hand_routes'].items()}
        pedal._task_grasp_profiles = {
            k: SimpleNamespace(open_class=v['open_class'], closed_class=v['closed_class'],
                               open_pose=data['classification_poses'][v['open_class']],
                               closed_pose=data['classification_poses'][v['closed_class']])
            for k, v in data['task_grasp_profiles'].items()
        }
        pedal._latest_task_num = self.config['both_task']
        pedal._both_hands_mode = False
        pedal._task_routing_enabled = True
        pedal._hand_pinch_active = {'left': False, 'right': False}
        pedal._active_profiles = {}
        pedal._left_task_topic = pedal._right_task_topic = self.config['selection_topic']
        pedal._log_available_hands = Mock()
        pedal.get_logger = Mock(return_value=Mock())
        pedal._publish_pose_to_hand = Mock()
        return pedal

    def test_current_routes_have_matching_profiles(self):
        s.validate_routes(self.config, ROUTES)

    def test_config_rejects_duplicate_tasks(self):
        config = dict(self.config, left_task=self.config['right_task'])
        path = self.temp_path / 'invalid.yaml'
        path.write_text(yaml.safe_dump(config))
        with self.assertRaises(ValueError):
            s.load_config(path)

    def test_different_grasp_profile_is_rejected(self):
        data = yaml.safe_load(ROUTES.read_text())
        data['task_grasp_profiles'][1] = {'open_class': 1, 'closed_class': 2}
        path = self.temp_path / 'routes.yaml'
        path.write_text(yaml.safe_dump(data))
        with self.assertRaises(ValueError):
            s.validate_routes(self.config, path)

    def test_backend_parameters_disable_old_b_and_startup_open(self):
        command = s.pedal_command('/fake/pedal', self.config, ROUTES)
        self.assertIn('both_hands_key_code:=-1', command)
        self.assertIn('publish_initial_pose:=false', command)
        self.assertIn('input_mode:=event', command)
        for side in ('left', 'right'):
            self.assertIn(f'{side}_task_topic:={self.config["selection_topic"]}', command)

    def test_selecting_hands_does_not_release_existing_grasp(self):
        pedal = self.legacy_callbacks()
        for task in (self.config['right_task'], self.config['left_task']):
            before = copy.deepcopy(pedal._hand_pinch_active)
            pedal._on_task_num(SimpleNamespace(data=task), 'left')
            self.assertEqual(pedal._hand_pinch_active, before)
            pedal._publish_pose_to_hand.assert_not_called()
            pedal._toggle_task_routed_hands()  # Simulate one pedal event, not hardware.
            self.assertEqual(pedal._publish_pose_to_hand.call_args.args[0], 'pinch')
            pedal._publish_pose_to_hand.reset_mock()
        self.assertEqual(pedal._hand_pinch_active, {'left': True, 'right': True})
        pedal._on_task_num(SimpleNamespace(data=self.config['both_task']), 'left')
        pedal._publish_pose_to_hand.assert_not_called()
        pedal._toggle_task_routed_hands()
        self.assertEqual(pedal._hand_pinch_active, {'left': False, 'right': False})

    def test_keyboard_loop_publishes_only_selection_and_ignores_b(self):
        pedal = self.legacy_callbacks()
        events = queue.Queue()
        events.put(('ready', None))
        child = SimpleNamespace(process=SimpleNamespace(poll=lambda: None), events=events)
        publisher = Mock()
        publisher.get_subscription_count.return_value = 2
        def publish(message):
            pedal._on_task_num(message, 'left')
            events.put(('ack', message.data))
        publisher.publish.side_effect = publish
        fake_rclpy = SimpleNamespace(ok=lambda: True, spin_once=lambda *a, **k: None)
        fake_msgs = SimpleNamespace(Int32=lambda **kwargs: SimpleNamespace(**kwargs))
        with patch.dict(sys.modules, {'rclpy': fake_rclpy, 'std_msgs.msg': fake_msgs}), \
                patch.object(s.select, 'select', return_value=([9], [], [])), \
                patch.object(s.os, 'read', side_effect=[b'r', b'l', b' ', b'b', b'B', b' ', b'']):
            s.control_loop(Mock(), publisher, child, 9, self.config, Mock())
        self.assertEqual([c.args[0].data for c in publisher.publish.call_args_list], [2, 1, 0])
        pedal._publish_pose_to_hand.assert_not_called()

    def test_duplicate_pedal_detection_reads_only_process_metadata(self):
        root = self.temp_path / 'proc'
        (root / '123').mkdir(parents=True)
        (root / '123/cmdline').write_bytes(b'python3\0/example/foot_pedal_hand_controller\0')
        (root / '456').mkdir()
        (root / '456/cmdline').write_bytes(b'python3\0/example/rohand_modbus_ap001\0')
        self.assertEqual(s.existing_pedals(root), [123])

    def test_terminal_is_restored_after_exception(self):
        master, slave = pty.openpty()
        stream = os.fdopen(os.dup(slave), 'r')
        try:
            before = termios.tcgetattr(slave)
            with patch.object(s.sys, 'stdin', stream):
                with self.assertRaises(RuntimeError):
                    with s.keyboard():
                        self.assertFalse(termios.tcgetattr(slave)[3] & termios.ICANON)
                        raise RuntimeError('test interruption')
            self.assertEqual(termios.tcgetattr(slave), before)
        finally:
            stream.close()
            os.close(master)
            os.close(slave)

    def test_child_cannot_open_controlling_tty_and_closes_cleanly(self):
        # A dummy Python child only: no ROS executable and no device access.
        probe = self.temp_path / 'probe.py'
        probe.write_text('''import os, signal, time
try:
    fd = os.open('/dev/tty', os.O_RDONLY)
except OSError:
    print('NO_CONTROLLING_TTY', flush=True)
else:
    os.close(fd)
    print('UNEXPECTED_TTY', flush=True)
print('Loaded task hand routes from FAKE', flush=True)
print('[WARN] fake warning', flush=True)
signal.signal(signal.SIGINT, lambda *_: exit(0))
while True: time.sleep(0.01)
''')
        log = self.temp_path / 'pedal.log'
        child = s.PedalProcess([sys.executable, '-u', str(probe)], log)
        try:
            self.assertEqual(child.events.get(timeout=3), ('ready', None))
            self.assertEqual(child.events.get(timeout=3), ('warning', '[WARN] fake warning'))
        finally:
            child.close()
        self.assertEqual(child.process.returncode, 0)
        self.assertIn('NO_CONTROLLING_TTY', log.read_text())
        self.assertNotIn('UNEXPECTED_TTY', log.read_text())

    @unittest.skipUnless(os.environ.get('ROHAND_SELECTOR_ROS_TEST') == '1',
                         'Run explicitly in a sourced ROS environment; backend is a dummy.')
    def test_real_ros_cli_with_dummy_backend(self):
        # Real ROS/DDS and a real controlling PTY, but never the real pedal program.
        prefix = self.temp_path / 'fake_backend'
        share = prefix / 'share/rohand_foot_pedal_controller'
        (share / 'config').mkdir(parents=True)
        shutil.copyfile(ROUTES, share / 'config/task_hand_routes.yaml')
        executable = prefix / 'lib/rohand_foot_pedal_controller/foot_pedal_hand_controller'
        executable.parent.mkdir(parents=True)
        records = self.temp_path / 'events.jsonl'
        executable.write_text('''#!/usr/bin/python3
import json, os, sys
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
records = Path(os.environ['SELECTOR_TEST_RECORDS'])
def record(event):
    with records.open('a') as f: f.write(json.dumps(event) + '\\n')
try:
    fd = os.open('/dev/tty', os.O_RDONLY)
except OSError:
    has_tty = False
else:
    os.close(fd)
    has_tty = True
record({'tty': has_tty, 'stdin_tty': sys.stdin.isatty(), 'pid': os.getpid()})
rclpy.init()
n = Node('dummy_pedal')
topic = n.declare_parameter('left_task_topic', '').value
current = n.declare_parameter('default_task_num', 0).value
def receive(msg):
    global current
    if msg.data != current:
        current = msg.data
        record({'task': current})
        n.get_logger().info(f'Received task_num={current} from {topic}')
n.create_subscription(Int32, topic, receive, 10)
print('BANNER_SHOULD_BE_HIDDEN', flush=True)
n.get_logger().info('Loaded task hand routes from FAKE')
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
finally:
    record({'stopped': True})
    n.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
''')
        executable.chmod(0o755)
        config = dict(self.config, log_dir=str(self.temp_path / 'logs'))
        config_path = self.temp_path / 'config.yaml'
        config_path.write_text(yaml.safe_dump(config))
        runner = self.temp_path / 'runner.py'
        runner.write_text(f'''import ament_index_python.packages as packages
import rohand_selector.selector as selector
original_prefix = packages.get_package_prefix
original_share = packages.get_package_share_directory
packages.get_package_prefix = lambda name: {str(prefix)!r} if name == 'rohand_foot_pedal_controller' else original_prefix(name)
packages.get_package_share_directory = lambda name: {str(share)!r} if name == 'rohand_foot_pedal_controller' else {str(PACKAGE)!r} if name == 'rohand_selector' else original_share(name)
selector.existing_pedals = lambda: []  # Dummy backend has no relationship to live controllers.
raise SystemExit(selector.main(['--config', {str(config_path)!r}]))
''')
        master, slave = pty.openpty()
        before = termios.tcgetattr(slave)
        def controlling_terminal():
            os.setsid()
            import fcntl
            fcntl.ioctl(0, termios.TIOCSCTTY, 0)
        env = dict(os.environ, ROS_DOMAIN_ID='191', ROS_LOCALHOST_ONLY='1',
                   SELECTOR_TEST_RECORDS=str(records), PYTHONUNBUFFERED='1')
        process = subprocess.Popen([sys.executable, str(runner)], stdin=slave, stdout=slave,
                                   stderr=slave, preexec_fn=controlling_terminal, env=env)
        output = bytearray()
        def drain():
            while select.select([master], [], [], 0)[0]:
                output.extend(os.read(master, 4096))
        def events():
            if not records.exists():
                return []
            return [json.loads(line) for line in records.read_text().splitlines() if line.endswith('}')]
        def wait_for(predicate):
            end = time.monotonic() + 12
            while time.monotonic() < end:
                drain()
                if predicate():
                    return
                if process.poll() is not None:
                    break
                time.sleep(0.02)
            self.fail(output.decode(errors='replace'))
        try:
            wait_for(lambda: '선택: 양손' in output.decode(errors='replace'))
            self.assertFalse(events()[0]['tty'])
            self.assertFalse(events()[0]['stdin_tty'])
            for key, task in ((b'r', 2), (b'l', 1), (b' ', 0)):
                os.write(master, key)
                wait_for(lambda: events()[-1].get('task') == task)
                wait_for(lambda: ('선택: ' + config[{2: 'right', 1: 'left', 0: 'both'}[task] + '_label'])
                         in output.decode(errors='replace'))
            os.write(master, b'bB')
            time.sleep(0.2)
            self.assertEqual([e['task'] for e in events() if 'task' in e], [2, 1, 0])
            os.write(master, b'\x03')
            process.wait(timeout=6)
            drain()
            self.assertEqual(process.returncode, 0, output.decode(errors='replace'))
            self.assertTrue(events()[-1]['stopped'])
            self.assertEqual(termios.tcgetattr(slave), before)
            self.assertNotIn('BANNER_SHOULD_BE_HIDDEN', output.decode(errors='replace'))
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=8)
            os.close(master)
            os.close(slave)


if __name__ == '__main__':
    unittest.main()
