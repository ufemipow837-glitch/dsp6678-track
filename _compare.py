import re, math

with open(r'd:\DSP\6678\track\track_1\无人机实测数据.txt', 'r', encoding='utf-8', errors='ignore') as f:
    content = f.read()

beam2 = []
for line in content.split('\n'):
    if '波位号:2' in line and '速度' in line and '目标数:0' not in line:
        ts_m = re.search(r'时间戳:(\d+)', line)
        azi_m = re.search(r'方位:([\d.\-]+)', line)
        rng_m = re.search(r'距离:([\d.\-]+)', line)
        ele_m = re.search(r'俯仰:([\d.\-]+)', line)
        vel_m = re.search(r'速度:([\d.\-]+)', line)
        h_m = re.search(r'高度:([\d.\-]+)', line)
        if all([ts_m, azi_m, rng_m, ele_m, vel_m]):
            ts = float(ts_m.group(1))
            azi = float(azi_m.group(1))
            rng = float(rng_m.group(1))
            ele = float(ele_m.group(1))
            vel = float(vel_m.group(1))
            h = float(h_m.group(1)) if h_m else 0
            beam2.append((ts, azi, rng, ele, vel, h))

print("=" * 80)
print(f"Beam 2 drone points: {len(beam2)}")
print("=" * 80)
header = "{:>10} {:>10} {:>10} {:>10} {:>10} {:>10}".format("t(ms)", "azi(deg)", "R(m)", "ele(deg)", "Vr(m/s)", "H(m)")
print(header)
print("-" * 80)
for ts, azi, rng, ele, vel, h in beam2:
    print("{:>10.0f} {:>10.2f} {:>10.2f} {:>10.4f} {:>10.4f} {:>10.2f}".format(ts, azi, rng, ele, vel, h))

# Get all unique timestamps sorted
all_ts = set()
beam_per_ts = {}
for line in content.split('\n'):
    ts_m = re.search(r'时间戳:(\d+)', line)
    bm = re.search(r'波位号:(\d+)', line)
    if ts_m and bm:
        ts = int(ts_m.group(1))
        b = int(bm.group(1))
        all_ts.add(ts)
        if ts not in beam_per_ts:
            beam_per_ts[ts] = b

sorted_ts = sorted(all_ts)
print("\n" + "=" * 80)
print(f"All frames: {len(sorted_ts)}, t=[{sorted_ts[0]}..{sorted_ts[-1]}]ms, span={((sorted_ts[-1]-sorted_ts[0])/1000):.1f}s")
print("=" * 80)

print("\nFrame index -> beam mapping:")
for i, ts in enumerate(sorted_ts[:30]):
    print("  Frame {:>3}: t={}, beam={}".format(i, ts, beam_per_ts.get(ts, "?")))

print("\nBeam 2 frames in full sequence:")
for i, ts in enumerate(sorted_ts):
    if beam_per_ts.get(ts) == 2:
        print("  Frame {:>3}: t={}".format(i, ts))
