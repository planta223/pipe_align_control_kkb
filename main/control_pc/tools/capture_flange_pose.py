#!/usr/bin/env python3
"""현재 양팔 자세를 조회해 육안 수직 보정용 샘플로 저장한다."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import hashlib
from pathlib import Path
import time
from zoneinfo import ZoneInfo

import numpy as np
import yaml


CONTROL_PC = Path(__file__).resolve().parents[1]


def timestamp():
    return datetime.now(ZoneInfo('Asia/Seoul')).isoformat(timespec='milliseconds')


def transform(pose_mm_rad):
    x, y, z, roll, pitch, yaw = pose_mm_rad
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = np.array([
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp, cp*sr, cp*cr],
    ])
    result[:3, 3] = np.array([x, y, z]) / 1000.0
    return result


def checked(result, name):
    code, values = result
    if code != 0:
        raise RuntimeError(f'{name}: SDK code={code}')
    return values


def numeric(values, size, name):
    result = np.asarray(values, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f'{name}: 유한한 값 {size}개가 필요합니다.')
    return result


def capture_arm(ip, factory, count=20):
    arm = factory(ip, is_radian=True, do_not_open=True, report_type='rich',
                  enable_report=True, enable_heartbeat=False, timed_comm=False,
                  max_callback_thread_count=0)
    try:
        arm.connect()
        arm.set_timeout(0.5)
        deadline = time.monotonic() + 4.0
        # SDK의 rich 보고 완료 표식을 확인해 초기 영점 속성을 사용하지 않는다.
        while not arm._arm._first_report_over:
            if time.monotonic() >= deadline:
                raise RuntimeError('rich 상태 보고 수신 시간 초과')
            time.sleep(0.02)
        time.sleep(0.25)
        errors = checked(arm.get_err_warn_code(), 'get_err_warn_code')
        if any(errors):
            raise RuntimeError(f'로봇 오류·경고: {errors}. 자동 해제하지 않습니다.')
        if arm._arm._is_simulation_robot:
            raise RuntimeError('실물 대신 컨트롤러 시뮬레이션에 연결됐습니다.')
        offset = numeric(arm.tcp_offset, 6, 'tcp_offset')
        world = numeric(arm.world_offset, 6, 'world_offset')
        if np.max(np.abs(world)) > 1e-6:
            raise RuntimeError('SDK world offset이 0이 아니어서 base 기준으로 기록할 수 없습니다.')
        samples = []
        for _ in range(count):
            started = time.monotonic()
            if started - arm._arm._last_report_time > 1.0:
                raise RuntimeError('rich 상태 보고가 1초 이상 갱신되지 않았습니다.')
            states = checked(arm.get_joint_states(is_radian=True, num=2), 'get_joint_states')
            joints = numeric(states[0], 7, 'joints')
            velocities = numeric(states[1], 7, 'velocities')
            tcp = numeric(checked(arm.get_position(is_radian=True), 'get_position'), 6, 'tcp')
            flange = transform(tcp) @ np.linalg.inv(transform(offset))
            samples.append({
                'captured_at': timestamp(), 'joints_rad': joints.tolist(),
                'joint_velocities_rad_s': velocities.tolist(),
                'sdk_tcp_pose_mm_rad': tcp.tolist(),
                'base_T_flange': flange.tolist(),
            })
            time.sleep(max(0.0, 0.1 - (time.monotonic() - started)))
        final_errors = checked(arm.get_err_warn_code(), 'get_err_warn_code')
        if any(final_errors):
            raise RuntimeError(f'측정 중 로봇 오류·경고: {final_errors}')
        if not np.allclose(arm.tcp_offset, offset, rtol=0, atol=1e-6):
            raise RuntimeError('측정 중 TCP offset이 변경됐습니다.')
        if not np.allclose(arm.world_offset, world, rtol=0, atol=1e-6):
            raise RuntimeError('측정 중 world offset이 변경됐습니다.')
        q = np.array([sample['joints_rad'] for sample in samples])
        v = np.array([sample['joint_velocities_rad_s'] for sample in samples])
        speed = float(np.max(np.abs(v)))
        span = float(np.max(np.ptp(q, axis=0)))
        if speed > 0.01 or span > 0.005:
            raise RuntimeError(f'정지 상태가 아닙니다: 최대 속도={speed:.6f}rad/s, 관절 변화={span:.6f}rad')
        return {
            'ip': ip, 'sdk_tcp_offset_mm_rad': offset.tolist(),
            'sdk_world_offset_mm_rad': world.tolist(),
            'stationarity': {'max_joint_speed_rad_s': speed, 'max_joint_span_rad': span},
            'representative': samples[-1], 'samples': samples,
        }
    finally:
        arm.disconnect()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f'기존 측정 파일을 덮어쓰지 않습니다: {args.output}')
    from xarm.wrapper import XArmAPI
    from xarm.version import __version__
    control = yaml.safe_load((CONTROL_PC / 'config/control.yaml').read_text())
    legacy = CONTROL_PC / 'config/legacy/fixed_frames.yaml'
    result = {
        'captured_at': timestamp(), 'reference': 'visual_flange_z_down',
        'height_reference': 'flange_centers', 'alignment_verified': False,
        'notes': '사용자가 육안으로 양팔 flange 축을 아래로 맞추고 높이를 유사하게 맞춘 자세다.',
        'units': {'translation': 'm', 'joint_angles': 'rad', 'sdk_pose': 'mm, rad'},
        'sdk_version': __version__,
        'legacy_frames_sha256': hashlib.sha256(legacy.read_bytes()).hexdigest(),
    }
    with ThreadPoolExecutor(max_workers=2) as executor:
        jobs = {side: executor.submit(capture_arm, control['robots'][side]['ip'], XArmAPI)
                for side in ('left', 'right')}
        result['arms'] = {side: job.result() for side, job in jobs.items()}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        stream.write('# 육안 flange 수직 기준 측정값. 정밀 정합값이 아니다.\n')
        yaml.safe_dump(result, stream, allow_unicode=True, sort_keys=False)
    print(f'저장 완료: {args.output}')
    for side, arm in result['arms'].items():
        flange = np.array(arm['representative']['base_T_flange'])
        print(f'{side}: {arm["stationarity"]}, base 기준 flange +Z={flange[:3, 2].tolist()}')


if __name__ == '__main__':
    main()
