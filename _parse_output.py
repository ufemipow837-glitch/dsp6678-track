import re
with open(r'd:\DSP\6678\track\track_1\输出数据.txt', 'r', encoding='utf-8', errors='ignore') as f:
    content = f.read()

lines = content.split('\n')
track_frames = []
i = 0
while i < len(lines):
    m_frame = re.search(r'Frame (\d+) \| t=(\d+)ms', lines[i])
    m_tracks = re.search(r'Reliable Tracks: (\d+)', lines[i])
    if m_frame and m_tracks:
        fi = int(m_frame.group(1))
        ts = int(m_frame.group(2))
        n_tracks = int(m_tracks.group(1))
        
        tracks = []
        for j in range(n_tracks):
            if i + 1 + j < len(lines):
                tm = re.search(r'Track\[(\d+)\]: Az=([\d.\-]+)deg El=([\d.\-]+)deg R=([\d.\-]+)m V=([\d.\-]+)m/s', lines[i+1+j])
                if tm:
                    tracks.append({
                        'idx': int(tm.group(1)),
                        'azi': float(tm.group(2)),
                        'ele': float(tm.group(3)),
                        'rng': float(tm.group(4)),
                        'vel': float(tm.group(5))
                    })
        track_frames.append({'frame': fi, 't': ts, 'tracks': tracks})
    i += 1

print(f'Total frames: {len(track_frames)}')
print(f'Frames with tracks: {sum(1 for f in track_frames if f["tracks"])}')
print()

hdr = "{:>5} {:>10} {:>7} {:>10} {:>10} {:>10} {:>10}".format("Frame", "t", "Track#", "Az(deg)", "El(deg)", "R(m)", "V(m/s)")
print(hdr)
print("-" * 70)
for f in track_frames:
    for t in f['tracks']:
        print("{:>5} {:>10} {:>7} {:>10.2f} {:>10.2f} {:>10.1f} {:>10.1f}".format(
            f['frame'], f['t'], t['idx'], t['azi'], t['ele'], t['rng'], t['vel']))
