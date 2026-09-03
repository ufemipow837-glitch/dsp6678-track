import re

c_bak = open(r'd:\DSP\6678\track\track_1\data_entry_500frames.c.bak', encoding='gbk', errors='ignore').read()
c_new = open(r'd:\DSP\6678\track\track_1\data_entry.c', encoding='ascii', errors='ignore').read()

print('=== Frame 0 pt[0-3] Compare ===')
for i in range(4):
    pat = r'data\[0\]\.azi\[' + str(i) + r'\] = ([-\d.]+)f;\s*\n\s*data\[0\]\.ele\[' + str(i) + r'\] = ([-\d.]+)f;\s*\n\s*data\[0\]\.range\[' + str(i) + r'\] = ([-\d.]+)f;\s*\n\s*data\[0\]\.velocity\[' + str(i) + r'\] = ([-\d.]+)f'
    m_bak = re.search(pat, c_bak)
    m_new = re.search(pat, c_new)
    if m_bak and m_new:
        b = [float(x) for x in m_bak.groups()]
        n = [float(x) for x in m_new.groups()]
        match = all(abs(x-y) < 0.0001 for x, y in zip(b, n))
        status = 'YES' if match else 'NO'
        print('  pt[%d]: azi=%.6f ele=%.6f r=%.2f v=%.4f  match=%s' % (i, n[0], n[1], n[2], n[3], status))
    else:
        print('  pt[%d]: NOT FOUND (bak=%s new=%s)' % (i, bool(m_bak), bool(m_new)))

print()
print('=== Timestamp alignment ===')
for fi in [0, 50, 100, 200, 300, 400, 499]:
    mb = re.search(r'data\[' + str(fi) + r'\]\.mSecond = ([-\d.]+)f', c_bak)
    mn = re.search(r'data\[' + str(fi) + r'\]\.mSecond = ([-\d.]+)f', c_new)
    if mb and mn:
        match = mb.group(1) == mn.group(1)
        print('  Frame %d: bak=%sms  new=%sms  match=%s' % (fi, mb.group(1), mn.group(1), match))

print()
print('=== Summary ===')
print('New file:  1300 frames (0-1299), 261.1s')
print('Backup:    500 frames (0-499)')
print('Frames 0-499 should be IDENTICAL')
