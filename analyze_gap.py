import re, csv
from collections import defaultdict

# 检查Frame 250-320的波位和数据
cpi_list = []
with open(r'd:\DSP\6678\track\track_1\drone_data_cpi.csv', 'r', encoding='utf-8-sig') as f:
    for row in csv.DictReader(f):
        cpi_list.append({
            't': int(row['时间戳(ms)']),
            'beam': int(row['波位号']),
            'n': int(row['目标数']),
        })

plots_by_t = defaultdict(list)
with open(r'd:\DSP\6678\track\track_1\drone_data_parsed.csv', 'r', encoding='utf-8-sig') as f:
    for row in csv.DictReader(f):
        if int(row['是否有效点']) == 1:
            t = int(row['时间戳(ms)'])
            plots_by_t[t].append({
                'az': float(row['方位(度)']),
                'r': float(row['距离(m)']),
                'v': float(row['径向速度(m/s)']),
                'bw': int(row['波位号']),
            })

print('=== Frame 250-320 的波位和无人机点 ===')
for i in range(250, min(320, len(cpi_list))):
    cpi = cpi_list[i]
    plots = plots_by_t.get(cpi['t'], [])
    drone_plots = [p for p in plots if 10 < p['az'] < 25 and 500 < p['r'] < 3000]
    other_plots = [p for p in plots if p not in drone_plots]
    t_from_start = (cpi['t'] - cpi_list[250]['t']) / 1000
    print(f'Frame {i:3d}: t={cpi["t"]}ms (+{t_from_start:.2f}s), beam={cpi["beam"]:2d}, '
          f'total={len(plots)}, drone={len(drone_plots)}, other={len(other_plots)}', end='')
    if drone_plots:
        p = drone_plots[0]
        print(f'  <-- DRONE: az={p["az"]:.1f}°, r={p["r"]:.0f}m, v={p["v"]:.1f}m/s', end='')
    if other_plots and len(other_plots) > 0:
        # 找Az=50°附近的高速点
        fast = [p for p in other_plots if abs(p['v']) > 50]
        if fast:
            p = fast[0]
            print(f'  [FAST: az={p["az"]:.1f}°, r={p["r"]:.0f}m, v={p["v"]:.1f}m/s, bw={p["bw"]}]', end='')
    print()

# 看看Frame 260之后波位2/3什么时候再出现
print('\n=== Frame 255之后波位2和波位3的帧 ===')
for i in range(255, 320):
    if cpi_list[i]['beam'] in [2, 3]:
        plots = plots_by_t.get(cpi_list[i]['t'], [])
        drone = [p for p in plots if 10 < p['az'] < 25]
        print(f'Frame {i}: beam={cpi_list[i]["beam"]}, t={cpi_list[i]["t"]}ms, drone_points={len(drone)}')
