#!/usr/bin/env python3
"""Metrics + plots from recorded Nav2 runs. Refuses empty evidence. Usage:
analyze.py OUTDIR LABEL=ARMDIR [LABEL=ARMDIR ...]   (ARMDIR contains rep*/ with run/*.csv)"""
import sys, json, math, csv
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt

out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
arms = [a.split('=', 1) for a in sys.argv[2:]]
FOOT = [(0.36, 0.2), (0.36, -0.2), (-0.45, -0.2), (-0.45, 0.2)]
STATUS = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}

def load(p):
    with open(p) as f: r = list(csv.DictReader(f))
    if not r: raise SystemExit(f'EMPTY evidence: {p}')
    return {k: np.array([float(x[k]) for x in r]) for k in r[0]}

def seg_dist(P, A, B):
    AB = B - A; L = (AB ** 2).sum(1); L[L == 0] = 1e-12
    t = np.clip(((P - A) * AB).sum(1) / L, 0, 1)
    return np.linalg.norm(P - (A + AB * t[:, None]), axis=1)

def wilson(k, n, z=1.96):
    if n == 0: return (float('nan'),) * 2
    p = k / n; d = 1 + z * z / n; c = p + z * z / (2 * n); h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return (c - h) / d, (c + h) / d

def analyze_rep(d):
    d = Path(d); m = json.loads((d / 'run' / 'run_manifest.json').read_text()); scn = m['scenario']
    if 'goal_sent_sim_time' not in m: return {'status': m['status'], 'ok': False, 'blocked': True, 'dir': str(d)}
    gt = load(d / 'run' / 'ground_truth.csv'); cmd = load(d / 'run' / 'cmd_vel.csv')
    t0, t1 = m['goal_sent_sim_time'], m['goal_end_sim_time']
    sel = (gt['stamp'] >= t0) & (gt['stamp'] <= t1 + 0.05)
    t = gt['stamp'][sel]; xy = np.c_[gt['x'][sel], gt['y'][sel]]; yaw = gt['yaw'][sel]
    plans = [json.loads(l) for l in open(d / 'run' / 'plans.jsonl') if l.strip()]
    plans = [p for p in plans if len(p['xy']) >= 2]
    cte = np.full(len(t), np.nan)
    if plans:
        pt = np.array([p['recv_sim'] for p in plans])
        for i, ti in enumerate(t):
            j = np.searchsorted(pt, ti, side='right') - 1
            if j < 0: continue
            P = np.array(plans[j]['xy']); cte[i] = seg_dist(xy[i:i+1].repeat(len(P) - 1, 0), P[:-1], P[1:]).min()
    goal = scn['goal']; fin = xy[-1]
    err = math.hypot(fin[0] - goal['x'], fin[1] - goal['y'])
    yaw_err = abs((yaw[-1] - goal['yaw'] + math.pi) % (2 * math.pi) - math.pi)
    # oscillation: sign reversals of angular command outside a 0.1 rad/s deadband, with a 0.5 s persistence
    cs = (cmd['sim_time'] >= t0) & (cmd['sim_time'] <= t1); wz = cmd['wz'][cs]; ct = cmd['sim_time'][cs]
    sig = np.where(np.abs(wz) > 0.1, np.sign(wz), 0); rev = 0; last = 0; last_t = -9
    for s, tt in zip(sig, ct):
        if s != 0:
            if last != 0 and s != last and tt - last_t >= 0.5: rev += 1
            if s != last: last_t = tt
            last = s
    # geometric collision proxy: any footprint corner/edge sample of the GT pose inside a mapped rectangle
    def inside(px, py): return any(abs(px - r['cx']) <= r['sx'] / 2 and abs(py - r['cy']) <= r['sy'] / 2 for r in scn['rects'])
    hit = []
    for (x, y), th in zip(xy, yaw):
        c, s = math.cos(th), math.sin(th); h = False
        for k in range(4):
            a, b = FOOT[k], FOOT[(k + 1) % 4]
            for u in np.linspace(0, 1, 6):
                lx, ly = a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u
                if inside(x + c * lx - s * ly, y + s * lx + c * ly): h = True; break
            if h: break
        hit.append(h)
    hit = np.array(hit); episodes = int(((hit[1:] & ~hit[:-1]).sum()) + (1 if hit[:1].any() else 0))
    path_len = float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum())
    age = {}
    tf = d / 'tf_delay_log.csv'
    if tf.exists():
        a = load(tf)['age_s']; age = {'tf_age_median_s': float(np.median(a)), 'tf_age_p95_s': float(np.percentile(a, 95))}
    return {'dir': str(d), 'status': m['status'], 'action_status': m.get('action_status'), 'ok': m.get('action_status') == 4,
            'reached_goal_gt': err <= 0.3, 'duration_s': t1 - t0, 'final_err_m': err, 'final_yaw_err_rad': yaw_err,
            'path_len_m': path_len, 'cte_rms_m': float(np.sqrt(np.nanmean(cte ** 2))) if np.isfinite(cte).any() else float('nan'),
            'cte_max_m': float(np.nanmax(cte)) if np.isfinite(cte).any() else float('nan'),
            'wz_reversals': rev, 'collision_proxy_episodes': episodes, 'min_z_m': float(gt['z'][sel].min()),
            'max_abs_roll_pitch_rad': float(max(np.abs(gt['roll'][sel]).max(), np.abs(gt['pitch'][sel]).max())),
            'fell': bool(max(np.abs(gt['roll'][sel]).max(), np.abs(gt['pitch'][sel]).max()) > 1.0 or gt['z'][sel].min() < 0.12),
            't': t, 'xy': xy, 'cte': cte, 'cmd_t': ct, 'cmd_wz': wz, 'cmd_vx': cmd['vx'][cs], **age}

rows = []; data = {}
for label, ad in arms:
    reps = sorted(p for p in Path(ad).glob('rep*') if (p / 'run' / 'run_manifest.json').exists())
    if not reps: print('no runs for', label); continue
    data[label] = []
    for r in reps:
        res = analyze_rep(r); res['label'] = label; res['rep'] = r.name; data[label].append(res)
        rows.append({k: v for k, v in res.items() if not isinstance(v, np.ndarray)})
if not rows: raise SystemExit('no evidence')
keys = sorted({k for r in rows for k in r}); first = ['label', 'rep', 'status', 'action_status', 'ok', 'reached_goal_gt']
with open(out / 'metrics.csv', 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=first + [k for k in keys if k not in first]); w.writeheader(); w.writerows(rows)

summ = []
for label, rs in data.items():
    good = [r for r in rs if not r.get('blocked')]; n = len(rs); k = sum(1 for r in rs if r['ok']); lo, hi = wilson(k, n)
    def mean(key, only_ok=False):
        v = [r[key] for r in good if key in r and (r['ok'] or not only_ok) and np.isfinite(r[key])]; return float(np.mean(v)) if v else float('nan')
    summ.append({'label': label, 'n': n, 'success': k, 'rate': k / n, 'ci_lo': lo, 'ci_hi': hi, 'duration_ok': mean('duration_s', True),
                 'final_err': mean('final_err_m'), 'cte_rms': mean('cte_rms_m'), 'cte_max': mean('cte_max_m'), 'wz_rev': mean('wz_reversals'),
                 'collisions': sum(r.get('collision_proxy_episodes', 0) for r in good), 'falls': sum(1 for r in good if r.get('fell')), 'tf_age_med': mean('tf_age_median_s')})
with open(out / 'summary.json', 'w') as f: json.dump(summ, f, indent=1)
labels = [s['label'] for s in summ]; colors = plt.cm.viridis(np.linspace(0, 0.9, len(labels)))
scn = json.loads((Path(arms[0][1]) / 'rep1' / 'run' / 'run_manifest.json').read_text())['scenario']

# Trajectories
fig, ax = plt.subplots(figsize=(9, 6))
for r in scn['rects']: ax.add_patch(plt.Rectangle((r['cx'] - r['sx'] / 2, r['cy'] - r['sy'] / 2), r['sx'], r['sy'], color='0.7'))
for c, l in zip(colors, labels):
    for i, r in enumerate(data[l]):
        if 'xy' in r: ax.plot(r['xy'][:, 0], r['xy'][:, 1], color=c, lw=1.3, alpha=0.85, ls='-' if r['ok'] else '--', label=l if i == 0 else None)
ax.plot(*[scn['start'][k] for k in 'xy'], 'ko'); ax.plot(scn['goal']['x'], scn['goal']['y'], 'r*', ms=14)
ax.set_xlim(-1.7, 6.7); ax.set_ylim(-3.2, 3.2); ax.set_aspect('equal'); ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
ax.set_title('Ground-truth trajectories (solid = action SUCCEEDED, dashed = not)'); ax.legend(fontsize=8, ncol=2, loc='lower right'); fig.tight_layout(); fig.savefig(out / 'trajectories.png', dpi=140); plt.close(fig)
# Tracking error
fig, ax = plt.subplots(figsize=(9, 4.5))
for c, l in zip(colors, labels):
    for i, r in enumerate(data[l]):
        if 'cte' in r: ax.plot(r['t'] - r['t'][0], r['cte'], color=c, lw=1.1, alpha=0.8, label=l if i == 0 else None)
ax.set_xlabel('time since goal (sim s)'); ax.set_ylabel('cross-track error to current global plan (m)'); ax.legend(fontsize=8, ncol=2); ax.grid(alpha=0.3)
fig.tight_layout(); fig.savefig(out / 'tracking_error.png', dpi=140); plt.close(fig)
# Angular command
fig, ax = plt.subplots(figsize=(9, 4.5))
for c, l in zip(colors, labels):
    for i, r in enumerate(data[l]):
        if 'cmd_wz' in r: ax.plot(r['cmd_t'] - r['cmd_t'][0], r['cmd_wz'], color=c, lw=0.9, alpha=0.7, label=l if i == 0 else None)
ax.set_xlabel('time since goal (sim s)'); ax.set_ylabel('commanded angular velocity (rad/s)'); ax.legend(fontsize=8, ncol=2); ax.grid(alpha=0.3)
fig.tight_layout(); fig.savefig(out / 'angular_velocity.png', dpi=140); plt.close(fig)
# Success + metrics bars
fig, axs = plt.subplots(1, 3, figsize=(13, 4))
x = np.arange(len(labels)); rate = np.array([s['rate'] for s in summ]); lo = np.array([s['ci_lo'] for s in summ]); hi = np.array([s['ci_hi'] for s in summ])
axs[0].bar(x, rate, yerr=[rate - lo, hi - rate], color=colors, capsize=3); axs[0].set_ylim(0, 1.05); axs[0].set_title('Action success rate (Wilson 95% CI)')
axs[1].bar(x, [s['cte_rms'] for s in summ], color=colors); axs[1].set_title('Mean RMS cross-track error (m)')
axs[2].bar(x, [s['wz_rev'] for s in summ], color=colors); axs[2].set_title('Mean angular-command reversals')
for a in axs: a.set_xticks(x); a.set_xticklabels(labels, rotation=60, ha='right', fontsize=8)
fig.tight_layout(); fig.savefig(out / 'summary_bars.png', dpi=140); plt.close(fig)
for s in summ: print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in s.items()})
