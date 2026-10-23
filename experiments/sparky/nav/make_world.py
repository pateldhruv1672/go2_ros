#!/usr/bin/env python3
"""Generate world SDF and matching occupancy map from scenario.json (single source of geometry)."""
import json, sys, hashlib
from pathlib import Path
scn = json.loads(Path(sys.argv[1]).read_text())
default_sdf = Path(sys.argv[2]).read_text()
out = Path(sys.argv[3]); out.mkdir(parents=True, exist_ok=True)
head = default_sdf[:default_sdf.index('<model name="box1">')]
head = head.rstrip()
models = []
for r in scn['rects']:
    models.append(f'''    <model name="{r['name']}">
      <static>true</static>
      <pose>{r['cx']} {r['cy']} 0.5 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><box><size>{r['sx']} {r['sy']} 1.0</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{r['sx']} {r['sy']} 1.0</size></box></geometry></visual>
      </link>
    </model>''')
sdf = head + '\n' + '\n'.join(models) + '\n  </world>\n</sdf>\n'
(out / 'sparky_arena.sdf').write_text(sdf)
m = scn['map']; res = m['resolution']
W = int(round(m['width_m'] / res)); H = int(round(m['height_m'] / res))
rows = []
for rr in range(H):
    y = m['origin_y'] + m['height_m'] - (rr + 0.5) * res
    row = bytearray()
    for cc in range(W):
        x = m['origin_x'] + (cc + 0.5) * res
        occ = any(abs(x - r['cx']) <= r['sx'] / 2 and abs(y - r['cy']) <= r['sy'] / 2 for r in scn['rects'])
        row.append(0 if occ else 254)
    rows.append(bytes(row))
(out / 'arena.pgm').write_bytes(f'P5\n{W} {H}\n255\n'.encode() + b''.join(rows))
(out / 'arena.yaml').write_text(
    f"image: {out.resolve() / 'arena.pgm'}\nmode: trinary\nresolution: {res}\n"
    f"origin: [{m['origin_x']}, {m['origin_y']}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
(out / 'world_map_hashes.json').write_text(json.dumps({
    'sdf_sha256': hashlib.sha256(sdf.encode()).hexdigest(),
    'pgm_sha256': hashlib.sha256((out / 'arena.pgm').read_bytes()).hexdigest()}, indent=1))
print('world+map written to', out)
