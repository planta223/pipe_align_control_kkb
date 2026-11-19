#!/usr/bin/env python3
"""Two-page diagnostic dashboard for a single vision-align run; no ROS/SDK access."""
import argparse
import csv
import datetime as dt
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('vision_log', HERE / 'plot_vision_align.py')
log = importlib.util.module_from_spec(spec)
spec.loader.exec_module(log)
plt = log.plt
LEFT, RIGHT = '#2563eb', '#ea580c'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    run = log.read_log(args.log)
    if run is None:
        parser.error('A vision-align control log is required.')
    samples, events, phases, duration = log.extract(run)
    if not samples:
        parser.error('No settled measurements were recorded.')
    start = run['start_ns']
    records = [(line, (ns-start)*1e-9, message) for line, ns, _, message in run['records']]
    initialization = next(t for _, t, m in records if '고정 목표 설정' in m)
    controlled = [s for s in samples if s['elapsed_s'] >= initialization]
    baseline = controlled[0]
    config_path = log.CONTROL_PC / 'config/control.yaml'
    config_raw = config_path.read_bytes()
    cfg = yaml.safe_load(config_raw)['vision_align']
    for sample in samples:
        for side in ('left', 'right'):
            sample[f'{side}_x_error_mm'] = (sample[f'{side}_x_m']-baseline[f'{side}_x_m'])*1000
            sample[f'{side}_y_error_mm'] = (sample[f'{side}_y_m']-baseline['goal_y_m'])*1000
            sample[f'{side}_z_error_mm'] = (sample[f'{side}_z_m']-baseline['goal_z_m'])*1000
            sample[f'{side}_xyz_error_mm'] = math.sqrt(sum(sample[f'{side}_{axis}_error_mm']**2 for axis in 'xyz'))
        sample['control_active'] = sample['elapsed_s'] >= initialization
        sample['gap_drift_mm'] = (sample['right_x_m']-sample['left_x_m'] -
                                  baseline['right_x_m']+baseline['left_x_m'])*1000
    # Extend the existing parser with the explicitly logged uncertainty fields.
    messages = {line: message for line, _, message in records}
    for sample in samples:
        parts = messages[sample['source_line']].split(' / right ', 1)
        for side, text in zip(('left', 'right'), parts):
            for kind in ('position', 'angle'):
                key = f'mean_{kind}_error_' + ('m' if kind == 'position' else 'rad')
                sample[f'{side}_{kind}_uncertainty'] = log.scalar(text, key)
    probes = [(t, m) for _, t, m in records if 'PROBE:' in m]
    corrections = [(t, m) for _, t, m in records if re.search(r'STEP[56]: (left|right) error', m)]
    cycles = []
    last_measurement = None
    pending_probe = None
    for line, t, message in records:
        if message.startswith('VISION:'):
            if cycles and cycles[-1]['end_s'] is None:
                cycles[-1]['end_s'] = t
                cycles[-1]['measurement_recorded'] = True
            last_measurement = t
            pending_probe = None
        elif 'PROBE:' in message:
            pending_probe = (t, message)
        elif 'PERF: COLLISION phase=vision_move ' in message:
            cycles.append({'start_s': pending_probe[0] if pending_probe else last_measurement,
                           'end_s': None, 'source_line': line,
                           'kind': 'probe' if pending_probe else 'correction',
                           'initial_collision_s': log.scalar(message, 'total_s'),
                           'connection_collision_s': 0., 'measurement_recorded': False})
        elif 'PERF: COLLISION phase=start_connection ' in message and cycles:
            cycles[-1]['connection_collision_s'] += log.scalar(message, 'total_s')
    for cycle in cycles:
        if cycle['end_s'] is None:
            cycle['end_s'] = duration
        cycle['duration_s'] = cycle['end_s']-cycle['start_s']
        cycle['other_s'] = cycle['duration_s']-cycle['initial_collision_s']-cycle['connection_collision_s']
        if cycle['other_s'] < 0:
            raise ValueError('Collision timings exceed cycle interval.')
    started = dt.datetime.fromtimestamp(start/1e9, log.KST)
    out = args.output_dir or HERE / started.strftime('%Y%m%d_%H%M%S')
    out.mkdir(parents=True, exist_ok=True)
    (out / args.log.name).write_bytes(run['raw'])
    for font in log.font_manager.fontManager.ttflist:
        if font.name.startswith('Noto Sans CJK'):
            plt.rcParams['font.family'] = font.name
            break
    plt.rcParams.update({'axes.unicode_minus': False, 'font.size': 9,
                         'axes.spines.top': False, 'axes.spines.right': False})
    times = [s['elapsed_s'] for s in controlled]

    def series(ax, key, label, color, scale=1., style='-'):
        ax.plot(times, [s[key]*scale for s in controlled], style, color=color,
                marker='o', markersize=3, linewidth=1.25, label=label)

    def decorate(ax, title, ylabel):
        ax.set_title(title, loc='left', fontsize=10)
        ax.set_ylabel(ylabel)
        ax.grid(alpha=.22)
        ax.legend(fontsize=8, loc='best')
        ax.set_xlim(0, duration*1.015)
        ax.axvline(duration, color='#dc2626', linestyle='--', linewidth=1)
        ax.set_xlabel('첫 로그부터 시간 (s)')
        for t, _ in probes:
            ax.axvline(t, color='#a855f7', alpha=.22, linewidth=.8)

    fig = plt.figure(figsize=(15, 13), layout='constrained')
    grid = fig.add_gridspec(4, 2, height_ratios=[.65, 1.5, 1.5, 1.5])
    timeline = fig.add_subplot(grid[0, :])
    for index, name in enumerate(log.PHASES):
        spans = [(p['start_s'], p['end_s']-p['start_s']) for p in phases if p['phase']==name]
        timeline.broken_barh(spans, (index-.3, .6), facecolors=log.COLORS[name])
        if not spans:
            timeline.text(duration*.85, index, '미진입', va='center', color='#64748b')
    timeline.set_yticks(range(5), ['STEP4', 'READY', 'STEP5 회전', 'STEP6 병진', 'STEP7 검증'])
    timeline.set_ylim(4.5, -.5)
    timeline.set_xlim(0, duration*1.015)
    timeline.set_title('단계와 시험 시작점 · 보라색 세로선 = 회전 시험 시작', loc='left')
    axes = [fig.add_subplot(grid[row, col]) for row in range(1, 4) for col in range(2)]
    ax = axes[0]
    for key, name, color in [('left_angle_x_deg','좌측',LEFT), ('right_angle_x_deg','우측',RIGHT),
                              ('relative_angle_deg','좌우 상대','#334155')]:
        series(ax, key, name, color)
    ax.axhline(math.degrees(cfg['angle_tolerance_rad']), color='#dc2626', linestyle=':', label='개별 각도 참고 한도')
    ax.axhline(math.degrees(cfg['relative_angle_tolerance_rad']), color='#7e22ce', linestyle='--', label='상대 각도 참고 한도')
    decorate(ax, '① 실제 단계 진입 조건: 개별·상대 각도 크기', '각도 (°)')
    ax = axes[1]
    for side, color in [('left', LEFT), ('right', RIGHT)]:
        for plane, style in [('xy','-'), ('xz','--')]:
            series(ax, f'{side}_angle_{plane}_deg', f'{side} {plane.upper()}', color, style=style)
    ax.axhline(0, color='#64748b', linewidth=.8)
    decorate(ax, '② 부호 있는 각도 성분: 어느 방향 오차가 남는가', '±X 정규화 투영각 (°)')
    ax = axes[2]
    for side, color in [('left',LEFT), ('right',RIGHT)]:
        series(ax, f'{side}_xyz_error_mm', side+' 개별 XYZ', color)
    series(ax, 'relative_yz_error_mm', '좌우 상대 YZ', '#334155')
    ax.axhline(cfg['position_tolerance_m']*1000, color='#dc2626', linestyle=':', label='개별 XYZ 참고 한도')
    ax.axhline(cfg['relative_position_tolerance_m']*1000, color='#7e22ce', linestyle='--', label='상대 YZ 참고 한도')
    decorate(ax, '③ 끝단 오차: 상대 YZ와 개별 XYZ를 구분', '거리 (mm)')
    ax = axes[3]
    for side, color in [('left',LEFT), ('right',RIGHT)]:
        for axis, style in [('y','-'), ('z','--')]:
            series(ax, f'{side}_{axis}_error_mm', f'{side} {axis.upper()}', color, style=style)
    ax.axhline(0, color='#64748b', linewidth=.8)
    decorate(ax, '④ 부호 있는 위치 성분: 측정 − 실행 시작 고정 목표', '끝단 Y/Z (mm)')
    ax = axes[4]
    series(ax, 'gap_drift_mm', '현재 X 간격 − 초기 X 간격', '#0891b2')
    for sign in (-1,1):
        ax.axhline(sign*cfg['max_gap_change_m']*1000, color='#dc2626', linestyle=':', label='간격 변화 참고 한도' if sign==1 else None)
    decorate(ax, '⑤ 회전에 따른 끝단 간격 변화', '간격 변화 (mm)')
    ax = axes[5]
    series(ax, 'source_age_s', '사용 영상 나이', '#059669', scale=1000)
    ax.axhline(cfg['max_pose_age_s']*1000, color='#dc2626', linestyle=':', label='영상 나이 참고 한도')
    decorate(ax, '⑥ 영상 나이: 수신 Hz나 순수 네트워크 지연이 아님', '영상 나이 (ms)')
    end_status = records[-1][2].split(':', 1)[0]
    fig.suptitle(f'vision_align | {started:%Y-%m-%d %H:%M:%S} KST | {duration:.2f}s | {end_status}\n'
                 '점은 정지 평균 측정, 선은 연결선 · 마지막 실패 측정창은 기록되지 않음', fontsize=14)
    fig.savefig(out/'01_alignment.png', dpi=160)
    fig.savefig(out/'01_alignment.pdf')
    plt.close(fig)

    fig, axs = plt.subplots(2, 2, figsize=(15, 10), layout='constrained')
    ax = axs[0,0]
    index = np.arange(1, len(cycles)+1)
    initial = np.array([c['initial_collision_s'] for c in cycles])
    recheck = np.array([c['connection_collision_s'] for c in cycles])
    other = np.array([c['other_s'] for c in cycles])
    ax.bar(index, initial, label='최초 충돌 순수 계측', color='#7c3aed')
    ax.bar(index, recheck, bottom=initial, label='연결 재검사 순수 계측', color='#0e7490')
    bars = ax.bar(index, other, bottom=initial+recheck, label='잔여: 경로·이동·정지·측정 등 미분리', color='#cbd5e1')
    for bar, cycle in zip(bars, cycles):
        if not cycle['measurement_recorded']:
            bar.set_hatch('///')
    ax.set_xticks(index, ['P' if c['kind']=='probe' else str(i) for i,c in enumerate(cycles,1)], fontsize=8)
    ax.set_xlabel('경로 검사 회차: P=시험, 숫자=보정 · 사선=검증 측정 미기록')
    ax.set_ylabel('주기 시간 (s)')
    ax.set_title('⑦ 보정 주기 비용: 잔여 시간을 실제 이동 시간으로 단정하지 않음', loc='left')
    ax.legend(fontsize=8)
    ax.grid(axis='y',alpha=.2)
    for ax, kind, factor, unit in [(axs[0,1], 'position',1000,'mm'), (axs[1,0],'angle',180/math.pi,'°')]:
        for side,color in [('left',LEFT), ('right',RIGHT)]:
            series(ax, f'{side}_{kind}_uncertainty', side, color, scale=factor)
        decorate(ax, f'⑧ 정지 평균 {kind} 불확실성 · 실패 판정 RMS와 다른 값', unit)
    ax = axs[1,1]
    intervals = np.diff([s['source_stamp_ns']*1e-9 for s in controlled])
    ax.plot(times[1:], intervals, 'o-', color='#475569', label='연속 평균 측정 stamp 사이 간격')
    decorate(ax, '⑨ 측정 간격: 이동·정지·평균 대기 포함, 카메라 FPS 아님', '측정 결과 간격 (s)')
    fig.suptitle('시간·측정 품질 | 실패창의 팔별 위치/각도 RMS는 로그에 없어 원인 팔을 확정할 수 없음\n'
                 '참고 한도는 현재 control.yaml 값이며 실행 당시 설정 스냅샷이 아님', fontsize=13)
    fig.savefig(out/'02_cost_quality.png', dpi=160)
    fig.savefig(out/'02_cost_quality.pdf')
    plt.close(fig)

    with (out/'measurements.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(samples[0]));writer.writeheader();writer.writerows(samples)
    summary={'source':args.log.name,'sha256':hashlib.sha256(run['raw']).hexdigest(),
             'start_kst':started.isoformat(),'duration_s':duration,'end':records[-1][2],
             'control_reference_line':baseline['source_line'],'reference_control_sha256':hashlib.sha256(config_raw).hexdigest(),
             'thresholds_are_current_config_reference':True,'samples':len(samples),'probes':len(probes),
             'completed_corrections':len(corrections),'path_checks':len(cycles),
             'unmeasured_last_cycle':not cycles[-1]['measurement_recorded'],
             'initial_collision_s':float(initial.sum()),'connection_collision_s':float(recheck.sum()),
             'phases':phases,'events':events,'cycles':cycles,
             'limitations':['Failure window RMS and responsible side unlogged.',
                            'Physical motion/settle/measurement times not separately logged.',
                            'Snapshots are stopped averaged observations, not raw camera frames.',
                            'Individual XYZ errors use fresh control-start targets; no cached targets.']}
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'output_dir':str(out),**{k:summary[k] for k in ('duration_s','samples','probes','completed_corrections','path_checks','end')}},ensure_ascii=False))


if __name__=='__main__':
    main()
