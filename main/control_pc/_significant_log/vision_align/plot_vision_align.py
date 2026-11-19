#!/usr/bin/env python3
"""Plot archived vision_align logs without contacting ROS or the robot.

The source log contains averaged, stopped measurements, not every received
camera frame. Never interpret the intervals below as camera reception rate.
Position components retain their signs. New logs include pipe axis vectors and
signed projected angles; legacy logs without them retain unknown angular signs.
"""
import argparse
import csv
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import re
import statistics

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import yaml


OUT = Path(__file__).resolve().parent
CONTROL_PC = OUT.parents[1]
KST = dt.timezone(dt.timedelta(hours=9))
LINE = re.compile(r'^\[\w+\] \[(\d+)\.(\d+)\] \[([^]]+)\]: (.*)$')
PHASES = ['STEP4', 'READY', 'STEP5', 'STEP6', 'STEP7']
COLORS = {'STEP4': '#94a3b8', 'READY': '#eab66b', 'STEP5': '#b8b4eb',
          'STEP6': '#9bd5bd', 'STEP7': '#e7bbd6'}


def scalar(text, key):
    match = re.search(r'\b' + re.escape(key) + r'=([-+\deE.]+)', text)
    if match is None:
        raise ValueError(f'Missing {key}: {text}')
    return float(match[1])


def vector(text, pattern, count):
    match = re.search(pattern, text)
    if match is None:
        raise ValueError(f'Missing vector {pattern}: {text}')
    result = [float(item) for item in match[1].split()]
    if len(result) != count:
        raise ValueError(f'Expected {count} values: {match[1]}')
    return result


def axis_fields(text, side):
    fields = {f'{side}_axis_{axis}': None for axis in 'xyz'}
    fields.update({f'{side}_axis_sign': None,
                   f'{side}_angle_xy_deg': None, f'{side}_angle_xz_deg': None})
    if 'axis_camera=' not in text:
        return fields
    axis = vector(text, r'axis_camera=\[([^]]+)\]', 3)
    sign = scalar(text, 'axis_sign') if 'axis_sign=' in text else (1 if axis[0] >= 0 else -1)
    if sign not in (-1, 1):
        raise ValueError(f'Invalid axis_sign: {sign}')
    fields.update({f'{side}_axis_{name}': value for name, value in zip('xyz', axis)})
    fields[f'{side}_axis_sign'] = sign
    for index, plane in [(1, 'xy'), (2, 'xz')]:
        key = f'angle_{plane}_deg'
        fields[f'{side}_{key}'] = (scalar(text, key) if key + '=' in text else
                                    math.degrees(math.atan2(sign*axis[index], sign*axis[0])))
    return fields


def read_log(path):
    raw = path.read_bytes()
    records = []
    for number, line in enumerate(raw.decode('utf-8').splitlines(), 1):
        match = LINE.match(line)
        if match:
            sec, fraction, node, message = match.groups()
            ns = int(sec) * 10**9 + int(fraction.ljust(9, '0'))
            records.append((number, ns, node, message))
    if not records or not any(node == 'auto_align' and 'STEP4:' in message
                              for _, _, node, message in records):
        return None
    return {'path': path, 'raw': raw, 'records': records,
            'start_ns': records[0][1], 'end_ns': records[-1][1]}


def extract(run):
    start = run['start_ns']
    samples, events, phases = [], [], []
    current_phase = 'STEP4'
    previous_stamp = previous_log = None
    for line, ns, _, message in run['records']:
        seconds = (ns - start) * 1e-9
        phase_match = re.search(r'\bSTEP([4-7]):', message)
        phase = ('STEP' + phase_match[1]) if phase_match else None
        if message.startswith('READY:'):
            phase = 'READY'
        if phase and (not phases or phase != current_phase):
            current_phase = phase
            phases.append({'start_s': seconds, 'phase': phase})
        if message.startswith(('FAULT:', 'STOPPED:', 'COMPLETED:')) or phase:
            events.append({'source_line': line, 'elapsed_s': seconds, 'message': message})
        if not message.startswith('VISION:'):
            continue
        left = vector(message, r'left endpoint_m=\[([^]]+)\]', 3)
        right = vector(message, r'right endpoint_m=\[([^]]+)\]', 3)
        goal = vector(message, r'goal_yz_m=([^/]+)', 2)
        stamp = int(re.search(r'\bstamp=(\d+)', message)[1])
        left_text, right_text = message.split(' / right ', 1)
        angular = {**axis_fields(left_text, 'left'), **axis_fields(right_text, 'right')}
        for plane in ('xy', 'xz'):
            left_angle, right_angle = (angular[f'{side}_angle_{plane}_deg'] for side in ('left', 'right'))
            angular[f'relative_angle_{plane}_deg'] = (None if left_angle is None or right_angle is None
                                                      else right_angle-left_angle)
        source_interval = None if previous_stamp is None else (stamp - previous_stamp) * 1e-9
        log_interval = None if previous_log is None else (ns - previous_log) * 1e-9
        if source_interval is not None and source_interval <= 0:
            raise ValueError(f'Non-increasing source stamp: {run["path"]}:{line}')
        samples.append({
            'source_line': line, 'phase': current_phase, 'elapsed_s': seconds,
            'log_time_ns': ns, 'source_stamp_ns': stamp,
            'left_angle_x_deg': scalar(left_text, 'axis_X_deg'),
            'right_angle_x_deg': scalar(right_text, 'axis_X_deg'),
            'relative_angle_deg': scalar(message, 'relative_angle_deg'),
            **angular,
            'left_yz_error_mm': math.hypot(left[1]-goal[0], left[2]-goal[1])*1000,
            'right_yz_error_mm': math.hypot(right[1]-goal[0], right[2]-goal[1])*1000,
            'relative_yz_error_mm': math.hypot(left[1]-right[1], left[2]-right[2])*1000,
            **{f'{side}_{axis}_error_mm': (point[index]-goal[index-1])*1000
               for side, point in [('left', left), ('right', right)]
               for index, axis in [(1, 'y'), (2, 'z')]},
            **{f'relative_{axis}_error_mm': (right[index]-left[index])*1000
               for index, axis in [(1, 'y'), (2, 'z')]},
            'source_age_s': scalar(message, 'source_age_s'),
            'source_measurement_interval_s': source_interval,
            'log_measurement_interval_s': log_interval,
            'source_measurement_frequency_hz': None if source_interval is None else 1/source_interval,
            **{f'{side}_{axis}_m': value for side, values in [('left', left), ('right', right)]
               for axis, value in zip('xyz', values)},
            'goal_y_m': goal[0], 'goal_z_m': goal[1],
        })
        previous_stamp, previous_log = stamp, ns
    duration = (run['end_ns'] - start) * 1e-9
    for i, phase in enumerate(phases):
        phase['end_s'] = phases[i+1]['start_s'] if i+1 < len(phases) else duration
    return samples, events, phases, duration


def stats(values):
    if not values:
        return None
    return {'count': len(values), 'min': min(values), 'median': statistics.median(values),
            'mean': statistics.mean(values), 'max': max(values)}


def draw(run, samples, phases, duration, stem, reference):
    for font in font_manager.fontManager.ttflist:
        if font.name.startswith('Noto Sans CJK'):
            plt.rcParams['font.family'] = font.name
            break
    plt.rcParams.update({'axes.unicode_minus': False, 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False})
    has_signed_angles = any(s[f'{side}_angle_xy_deg'] is not None
                            for s in samples for side in ('left', 'right'))
    heights = [1.25, 2.5] + ([2, 2] if has_signed_angles else []) + [2, 2, 2.3, 2, 1.8]
    fig, axes = plt.subplots(len(heights), 1, figsize=(15, 22 if has_signed_angles else 18), sharex=True,
                             gridspec_kw={'height_ratios': heights})
    fig.subplots_adjust(left=.085, right=.97, top=.87, bottom=.12, hspace=.27)
    started = dt.datetime.fromtimestamp(run['start_ns']/1e9, KST)
    end_message = run['records'][-1][3]
    end_label = end_message.split(':', 1)[0]
    fig.suptitle(f'vision_align | {started:%Y-%m-%d %H:%M:%S} KST | {end_label}',
                 fontsize=17, y=.975)
    fig.text(.085, .935, run['path'].name, fontsize=10, color='#475569')
    fig.text(.085, .909, f'기록 길이 {duration:.2f} s · 정지 측정 {len(samples)}회 · '
             '수신 정보: 측정 결과 간격 + source_age (원본 프레임 수신 주기는 로그 없음)', fontsize=10)
    top = axes[0]
    for index, name in enumerate(PHASES):
        segments = [(p['start_s'], p['end_s']-p['start_s']) for p in phases if p['phase'] == name]
        top.broken_barh(segments, (index-.3, .6), facecolors=COLORS[name])
        if not segments:
            top.text(.985, index, '미진입', va='center', ha='right', fontsize=9,
                     color='#94a3b8', transform=top.get_yaxis_transform())
    top.set_yticks(range(len(PHASES)), ['STEP4 측정/준비', 'READY 대기', 'STEP5 회전',
                                       'STEP6 병진', 'STEP7 통합 검증'])
    top.set_ylim(4.5, -.5)
    top.set_title('로그에 기록된 단계 구간 (STEP5에는 시험 이동·복귀·보정 포함)', loc='left', fontsize=11)
    times = [s['elapsed_s'] for s in samples]
    def series(ax, key, label, color, style='-'):
        ax.plot(times, [s[key] for s in samples], style, color=color,
                marker='o', markersize=3.8, linewidth=1.35, label=label)
    angle = axes[1]
    signed_y, signed_z, position, cadence, age = axes[-5:]
    signed_angle_axes = list(axes[2:4]) if has_signed_angles else []
    series(angle, 'left_angle_x_deg', '좌측 축–카메라 X축', '#2563eb')
    series(angle, 'right_angle_x_deg', '우측 축–카메라 X축', '#ea580c')
    series(angle, 'relative_angle_deg', '좌우 축 사이 상대 각도', '#334155', '--')
    angle.axhline(reference['angle_deg'], color='#dc2626', linestyle=':', label='각도 2° 참고선*')
    angle.set_ylabel('각도 오차 크기 (°)')
    angle.set_title('좌·우 축 방향 오차 크기' + ('' if has_signed_angles else
                    ' (부호 없음: 원본 로그에 축 벡터가 없어 복원 불가)'),
                    loc='left', fontsize=11)
    for ax, plane in zip(signed_angle_axes, ('xy', 'xz')):
        series(ax, f'left_angle_{plane}_deg', '좌측: +X 기준 투영각', '#2563eb')
        series(ax, f'right_angle_{plane}_deg', '우측: +X 기준 투영각', '#ea580c')
        series(ax, f'relative_angle_{plane}_deg', '상대 투영각 차이: 우측 − 좌측', '#334155', '--')
        ax.axhline(0, color='#64748b', linewidth=1)
        values = [abs(s[f'{side}_angle_{plane}_deg']) for s in samples
                  for side in ('left', 'right', 'relative') if s[f'{side}_angle_{plane}_deg'] is not None]
        extent = max([1] + values)*1.15
        ax.set_ylim(-extent, extent)
        ax.set_ylabel(f'{plane.upper()} 투영각 (°)')
        ax.set_title(f'부호 있는 축 {plane.upper()} 투영각 (axis_sign으로 ±X 정규화 · 미기록 구간은 공백)',
                     loc='left', fontsize=11)
    for ax, component in [(signed_y, 'y'), (signed_z, 'z')]:
        series(ax, f'left_{component}_error_mm', '좌측: 측정 − 목표', '#2563eb')
        series(ax, f'right_{component}_error_mm', '우측: 측정 − 목표', '#ea580c')
        series(ax, f'relative_{component}_error_mm', '상대: 우측 − 좌측', '#334155', '--')
        ax.axhline(0, color='#64748b', linewidth=1)
        extent = max(1, max(abs(s[f'{side}_{component}_error_mm'])
                            for s in samples for side in ('left', 'right', 'relative')))*1.15
        ax.set_ylim(-extent, extent)
        ax.set_ylabel(f'{component.upper()} 오차 (mm)')
        ax.set_title(f'부호 있는 끝단 {component.upper()} 위치 오차 (카메라 좌표계)',
                     loc='left', fontsize=11)
    series(position, 'left_yz_error_mm', '좌측 끝단 → 공통 목표 Y/Z', '#2563eb')
    series(position, 'right_yz_error_mm', '우측 끝단 → 공통 목표 Y/Z', '#ea580c')
    series(position, 'relative_yz_error_mm', '좌우 끝단 Y/Z 거리', '#334155', '--')
    position.axhline(reference['endpoint_mm'], color='#dc2626', linestyle=':', label='개별 3 mm 참고선*')
    position.axhline(reference['relative_mm'], color='#9333ea', linestyle=':', label='상대 5 mm 참고선*')
    position.set_ylabel('Y/Z 오차 크기 (mm)')
    position.set_title('끝단 Y/Z 오차 크기 (부호 없음 · X 제외 · 허용값 비교용)', loc='left', fontsize=11)
    intervals = [s for s in samples if s['source_measurement_interval_s'] is not None]
    cadence.set_title('수신 관련 지표 ①: 기록된 측정 결과 사이 간격', loc='left', fontsize=11)
    if intervals:
        tx = [s['elapsed_s'] for s in intervals]
        source_dt = [s['source_measurement_interval_s'] for s in intervals]
        cadence.plot(tx, source_dt, '-o', markersize=3.5, color='#7c3aed', label='영상 stamp 사이 간격')
        cadence.plot(tx, [s['log_measurement_interval_s'] for s in intervals],
                     '--o', markersize=3, color='#0891b2', label='로그 출력 사이 간격')
        median = statistics.median(source_dt)
        cadence.text(.985, .94, f'stamp 간격 중앙값 {median:.3f} s\n'
                     f'역수 {1/median:.3f} 회/s = 측정 기록 빈도',
                     transform=cadence.transAxes, ha='right', va='top', fontsize=9,
                     bbox={'facecolor': 'white', 'alpha': .9, 'edgecolor': '#cbd5e1'})
    else:
        cadence.text(.5, .5, '측정 1회: 측정 결과 간격·빈도 계산 불가',
                     ha='center', va='center', transform=cadence.transAxes, fontsize=12)
        cadence.set_ylim(0, 1)
    cadence.set_ylabel('측정 결과 간격 (s)')
    series(age, 'source_age_s', 'source_age_s (로그 출력 시각 − 영상 stamp)', '#059669')
    age.set_ylabel('영상 나이 (s)')
    age.set_title('수신 관련 지표 ②: 사용한 영상의 나이', loc='left', fontsize=11)
    age.set_xlabel('첫 로그 기록부터 경과 시간 (s)')
    if samples:
        ages = [s['source_age_s'] for s in samples]
        age.text(.985, .94, f'중앙값 {statistics.median(ages)*1000:.0f} ms · 최대 {max(ages)*1000:.0f} ms',
                 transform=age.transAxes, ha='right', va='top', fontsize=9)
    for ax in axes[1:]:
        for phase in phases:
            ax.axvspan(phase['start_s'], phase['end_s'], color=COLORS[phase['phase']], alpha=.12, zorder=-2)
            if phase['start_s'] > 0:
                ax.axvline(phase['start_s'], color=COLORS[phase['phase']], alpha=.8, linewidth=.8, zorder=-1)
        ax.axvline(duration, color='#dc2626', linestyle='--', linewidth=1)
        if ax not in [signed_y, signed_z] + signed_angle_axes:
            ax.set_ylim(bottom=0)
        ax.grid(True, alpha=.2)
        handles, _ = ax.get_legend_handles_labels()
        if handles:
            ax.legend(loc='upper left', fontsize=8, framealpha=.9)
    for ax in axes:
        ax.set_xlim(0, max(duration*1.015, 1))
    angle_note = ('각도 성분은 ±X 정규화 축의 XY/XZ 투영각입니다.' if has_signed_angles else
                  '각도는 원본 로그에 부호 정보가 없습니다.')
    fig.text(.085, .086, '부호: 개별 위치 = 측정 − 목표, 상대 위치 = 우측 − 좌측. ' + angle_note, fontsize=9)
    fig.text(.085, .071, '점은 정지 후 평균 측정값이며, 선은 점 사이 연결입니다. 운동 중 오차를 복원한 곡선은 아닙니다.', fontsize=9)
    fig.text(.085, .048, '* 참고선: 현재 설정의 2° / 3 mm / 5 mm. 당시 설정 스냅샷이 없어 실행 당시 임계치로 단정하지 않습니다.', fontsize=9)
    fig.text(.085, .025, '원본 프레임 수신 주기·Hz는 복원 불가. 영상 stamp 간격은 정지·이동·검증 대기를 포함한 평균 측정 결과 간격입니다.', fontsize=9)
    fig.savefig(OUT / f'{stem}.png', dpi=180, bbox_inches='tight', pad_inches=.2)
    fig.savefig(OUT / f'{stem}.pdf', bbox_inches='tight', pad_inches=.2)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-dir', type=Path)
    args = parser.parse_args()
    archive = OUT / 'source_logs'
    source = args.source_dir or (archive if archive.exists() else CONTROL_PC / '_log/auto_align')
    runs = [run for p in source.glob('*.log') if (run := read_log(p))]
    runs.sort(key=lambda r: r['start_ns'], reverse=True)
    if len(runs) < 3:
        raise ValueError('At least three vision_align runs are required.')
    OUT.mkdir(parents=True, exist_ok=True)
    archive.mkdir(exist_ok=True)
    reference_path = OUT / 'reference_thresholds.json'
    if reference_path.exists():
        reference = json.loads(reference_path.read_text())
    else:
        raw = (CONTROL_PC / 'config/control.yaml').read_bytes()
        c = yaml.safe_load(raw)['vision_align']
        reference = {'origin': 'current control.yaml; historical run values unconfirmed',
                     'control_yaml_sha256': hashlib.sha256(raw).hexdigest(),
                     'angle_deg': math.degrees(c['angle_tolerance_rad']),
                     'endpoint_mm': c['position_tolerance_m']*1000,
                     'relative_mm': c['relative_position_tolerance_m']*1000}
        reference_path.write_text(json.dumps(reference, ensure_ascii=False, indent=2)+'\n')
    summaries = []
    for rank, run in enumerate(runs[:3], 1):
        started = dt.datetime.fromtimestamp(run['start_ns']/1e9, KST)
        pid = run['path'].stem.split('_')[-2]
        stem = f'{rank:02d}_{started:%Y%m%d_%H%M%S}_pid{pid}'
        destination = archive / run['path'].name
        if destination.resolve() != run['path'].resolve():
            destination.write_bytes(run['raw'])
        samples, events, phases, duration = extract(run)
        signed_count = sum(all(s[f'{side}_angle_xy_deg'] is not None for side in ('left', 'right'))
                           for s in samples)
        with (OUT / f'{stem}_measurements.csv').open('w', newline='') as f:
            if samples:
                writer = csv.DictWriter(f, fieldnames=list(samples[0]), lineterminator='\n')
                writer.writeheader()
                writer.writerows(samples)
        metadata = {'source_log': run['path'].name, 'source_sha256': hashlib.sha256(run['raw']).hexdigest(),
                    'start_kst': started.isoformat(), 'duration_s': duration,
                    'measurements': len(samples), 'phases': phases, 'events': events,
                    'end_message': run['records'][-1][3],
                    'signed_position_error_definition': {
                        'frame': 'camera coordinates',
                        'individual': 'measured endpoint minus logged goal, Y/Z components in mm',
                        'relative': 'right endpoint minus left endpoint, Y/Z components in mm'},
                    'signed_angles_available': bool(samples) and signed_count == len(samples),
                    'signed_angle_measurements': signed_count,
                    'signed_angle_definition': 'atan2(sign*axis.y, sign*axis.x), atan2(sign*axis.z, sign*axis.x), degrees; relative = right minus left projected angles',
                    'signed_angles_unavailable_reason': (None if samples and signed_count == len(samples) else
                        'Some or all source measurements lack axis vectors; their signed angles remain unknown.'),
                    'raw_camera_receive_period_available': False,
                    'source_measurement_interval_s': stats([s['source_measurement_interval_s'] for s in samples if s['source_measurement_interval_s'] is not None]),
                    'source_age_s': stats([s['source_age_s'] for s in samples])}
        (OUT / f'{stem}_metadata.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
        draw(run, samples, phases, duration, stem, reference)
        summaries.append({'artifact': stem, **metadata})
        print(json.dumps({key: value for key, value in summaries[-1].items()
                          if key in ('artifact', 'start_kst', 'measurements', 'duration_s', 'source_measurement_interval_s')}, ensure_ascii=False))
    (OUT / 'summary.json').write_text(json.dumps(summaries, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    (OUT / 'analysis_notes.txt').write_text(
        '최신 vision_align 실행 3개: 첫 로그 시각 기준 내림차순. vision_measure와 pre_align_ready 제외.\n'
        '이동 전에 중지된 실행도 생략하지 않았다. 세 번째 실행은 측정 1회이며 주기 계산 불가.\n'
        '좌우 각도 크기: 각 파이프 축과 카메라 X축의 예각 및 상대 각도.\n'
        '부호 있는 각도: 새 로그의 axis_camera와 axis_sign 및 angle_xy_deg/angle_xz_deg를 사용. 없는 과거 측정은 공란이며 복원 불가.\n'
        '상대 각도 성분은 우측 − 좌측 투영각 차이이며, 3D 상대 각도 크기와 다름. 측정별 가용 여부는 CSV, 전체 여부는 metadata에 기록.\n'
        '부호 있는 개별 위치: 카메라 좌표계에서 측정 끝단 − 각 로그의 goal_yz_m. Y와 Z를 별도 표시, 단위 mm.\n'
        '부호 있는 상대 위치: 우측 끝단 − 좌측 끝단. 원본 로그의 delta_yz_mm와 같은 부호 규칙.\n'
        'CSV: left/right_y/z_error_mm는 개별 성분, relative_y/z_error_mm는 상대 성분. 기존 yz_error_mm는 크기.\n'
        '기존 Y/Z 오차 크기는 허용값 비교용으로 함께 표시. X 오차를 제외하므로 전체 3D 완료 판정과 다름.\n'
        '초기 STEP4의 목표는 당시 표시 목표이며, 제어 시작 STEP4에서 고정 목표가 설정된다.\n'
        '시간축: 각 로그 출력 시각에서 해당 실행의 첫 로그 시각을 뺀 값.\n'
        '원본 프레임 수신 시각이 없어 원본 수신 주기/Hz는 알 수 없다.\n'
        'source_measurement_interval_s: 연속 VISION 결과의 원본 영상 stamp 간격.\n'
        'log_measurement_interval_s: 연속 VISION 로그 출력 간격. 둘 다 운동/정지/측정 대기를 포함.\n'
        'source_age_s: 로그에서 그대로 추출한 영상 나이. 순수 네트워크 지연으로 해석하지 않는다.\n'
        '각 선은 정지 평균 측정점 연결이며, 측정되지 않은 운동 중 상태를 복원하지 않는다.\n'
        'STEP5는 UVS 시험 이동/복귀도 포함. STEP6, STEP7은 세 실행 모두 미진입.\n'
        '참고선은 reference_thresholds.json에 보관한 현재 YAML 값이며 실행 당시 임계치 보증이 아님.\n'
        '원본 로그는 source_logs/에 바이트 그대로 보관했으며 metadata.json에 SHA256과 원본 행 번호를 기록.\n'
        '재생성: python3 -B main/control_pc/_significant_log/vision_align/plot_vision_align.py\n', encoding='utf-8')


if __name__ == '__main__':
    main()
