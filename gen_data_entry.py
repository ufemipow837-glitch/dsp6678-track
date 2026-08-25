#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将无人机实测数据转换为data_entry.c格式
选取前N帧(默认100帧)生成C测试数据
"""
import csv, os, math
from collections import defaultdict

PLOT_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'drone_data_parsed.csv')
CPI_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'drone_data_cpi.csv')
OUT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data_entry.c')

NUM_FRAMES = 100  # target_data数组大小为100

def fmt_float(v, prec=4):
    """格式化浮点数，始终保留小数点(兼容C6000编译器)"""
    s = f"{v:.{prec}f}"
    return s

def generate():
    # 加载CPI数据(确定帧顺序和空帧)
    cpi_list = []
    with open(CPI_CSV, 'r', encoding='utf-8-sig') as f:
        for row in csv.DictReader(f):
            cpi_list.append({
                't': int(row['时间戳(ms)']),
                'beam': int(row['波位号']),
                'interval': int(row['帧间隔(ms)']),
                'n_targets': int(row['目标数']),
            })

    # 加载点迹数据
    plots_by_t = defaultdict(list)
    with open(PLOT_CSV, 'r', encoding='utf-8-sig') as f:
        for row in csv.DictReader(f):
            if int(row['是否有效点']) == 1:
                t = int(row['时间戳(ms)'])
                plots_by_t[t].append({
                    'az': float(row['方位(度)']),
                    'el': float(row['俯仰(度)']),
                    'r': float(row['距离(m)']),
                    'v': float(row['径向速度(m/s)']),
                    'beam': int(row['波位号']),
                })

    # 取前NUM_FRAMES个CPI
    selected_cpis = cpi_list[:NUM_FRAMES]
    if len(selected_cpis) < NUM_FRAMES:
        print(f"警告: 只有{len(selected_cpis)}帧数据")

    lines = []
    lines.append('#include <stdio.h>')
    lines.append('#include <c6x.h>')
    lines.append('#include <math.h>')
    lines.append('#include "struct.h"')
    lines.append('#include <stdint.h>')
    lines.append('#include <float.h>')
    lines.append('/*')
    lines.append(' * data_entry.c - 无人机实测数据(100帧/约16秒)')
    lines.append(' * 数据来源: 无人机实测数据.txt')
    lines.append(f' * 时间范围: {selected_cpis[0]["t"]}ms ~ {selected_cpis[-1]["t"]}ms ({(selected_cpis[-1]["t"]-selected_cpis[0]["t"])/1000:.1f}s)')
    lines.append(' * 雷达参数: 17波位机扫, 波位间隔164ms, 扫描周期~2788ms')
    lines.append(' */')
    lines.append('void data_entry(struct TARGETPIONT_1 (*data)){')
    lines.append('')
    lines.append('    float pi = 3.141592653589793f;')
    lines.append('    int i, j;')
    lines.append('')
    lines.append('    /* 先清零所有帧 */')
    lines.append('    for(i = 0; i < %d; i++){' % NUM_FRAMES)
    lines.append('        data[i].targetNum = 0;')
    lines.append('        data[i].frameSn = 0;')
    lines.append('        data[i].mSecond = 0.0f;')
    lines.append('        data[i].workMode = 0;')
    lines.append('        data[i].beamNo = 0;')
    lines.append('        data[i].tgtnum = 1;')
    lines.append('        data[i].Year = 0;')
    lines.append('        data[i].Month = 0;')
    lines.append('        data[i].Day = 0;')
    lines.append('        data[i].Hour = 0;')
    lines.append('        data[i].Minute = 0;')
    lines.append('        data[i].Second = 0;')
    lines.append('        for(j = 0; j < 60; j++){')
    lines.append('            data[i].azi[j] = 0.0f;')
    lines.append('            data[i].ele[j] = 0.0f;')
    lines.append('            data[i].range[j] = 0.0f;')
    lines.append('            data[i].velocity[j] = 0.0f;')
    lines.append('            data[i].Use_Flag_1[j] = 0;')
    lines.append('        }')
    lines.append('    }')
    lines.append('')

    # 填充每帧数据
    for frame_idx, cpi in enumerate(selected_cpis):
        t = cpi['t']
        beam = cpi['beam']
        n = cpi['n_targets']
        plots = plots_by_t.get(t, [])

        # 使用实际有效点数(从点迹数据中统计,可能与CPI标注的n_targets略有出入因为筛选条件)
        real_plots = []
        for p in plots:
            real_plots.append(p)
        actual_n = len(real_plots)

        lines.append(f'    /* Frame {frame_idx}: t={t}ms, beam={beam}, targets={actual_n} */')
        lines.append(f'    data[{frame_idx}].targetNum = {actual_n};')
        lines.append(f'    data[{frame_idx}].frameSn = {frame_idx};')
        lines.append(f'    data[{frame_idx}].mSecond = {t}.0f;')
        lines.append(f'    data[{frame_idx}].workMode = 0;')
        lines.append(f'    data[{frame_idx}].beamNo = {beam};')
        lines.append(f'    data[{frame_idx}].Month = 0;')

        for j, p in enumerate(real_plots[:60]):  # 最多60个点
            az = p['az']
            el = p['el']
            r = p['r']
            v = abs(p['v'])  # velocity取绝对值(与data_process_func一致)
            lines.append(f'    data[{frame_idx}].azi[{j}] = {fmt_float(az,4)}/180.0f*pi;')
            lines.append(f'    data[{frame_idx}].ele[{j}] = {fmt_float(el,4)}/180.0f*pi;')
            lines.append(f'    data[{frame_idx}].range[{j}] = {fmt_float(r,2)}f;')
            lines.append(f'    data[{frame_idx}].velocity[{j}] = {fmt_float(v,4)}f;')
            lines.append(f'    data[{frame_idx}].Use_Flag_1[{j}] = 1;')

        lines.append('')

    lines.append('}')
    lines.append('')

    content = '\n'.join(lines)
    with open(OUT_FILE, 'w', encoding='utf-8') as f:
        f.write(content)

    print(f"生成完成: {OUT_FILE}")
    print(f"  帧数: {NUM_FRAMES}")
    total_plots = sum(len(plots_by_t.get(cpi['t'],[])) for cpi in selected_cpis)
    nonempty = sum(1 for cpi in selected_cpis if len(plots_by_t.get(cpi['t'],[]))>0)
    empty = NUM_FRAMES - nonempty
    print(f"  有目标帧: {nonempty}, 空帧: {empty}")
    print(f"  总点迹数: {total_plots}")

if __name__ == '__main__':
    generate()
