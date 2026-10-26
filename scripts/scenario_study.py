"""Packaging scenario study: accumulator width x rear axle position, at a weight cap.

Every scenario starts from candidate 009c's node layout and search settings (design/frame-009c.json
paramSearch: VR3 catalog, two-phase layout/size search, 2 restarts, seed 51, 240 s per evaluation,
75.5 lb cap) on the team's source main hoop. The scenario sets the accumulator box and the halfshaft
corridor (scripts/packaging.py). The nodes that bound the battery bay are then refined by a
parallel compass search: the brace-rear node BR (x, y, z) and, when the box is wider than the
main-hoop legs and must sit aft against the powertrain, the battery-post foot BAT0 (x). Each
evaluation is the full search of param_search.evaluate (rule audit, keep-outs, templates, Percy
sweeps, halfshaft and battery checks). Beam-model numbers only; SolidWorks is the score of record.

  python scripts/scenario_study.py --out-dir design/scenarios-009d --workers 8

Writes <out-dir>/<scenario>.json (best recipe), <scenario>-search.json (every evaluation),
summary.json and summary.md.
"""
import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'): os.environ.setdefault(_v, '1')
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, wait, FIRST_COMPLETED
import argparse, itertools, json, sys, time, types
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import frame_geometry as fg
import packaging as pkg
import param_search as ps

ROOT = ps.ROOT
BAY_KEYS = ('braceRearXM', 'braceRearYM', 'braceRearZM')
WIDE_KEYS = BAY_KEYS + ('batteryPostXM',)


class Scenario:
    def __init__(self, sid, width_in, frac, start, a):
        self.sid, self.width_in, self.frac = sid, width_in, frac
        self.results, self.best, self.seen = [], None, set()
        self.step, self.iter, self.pending, self.done = 0.25, 0, 0, False
        self.improved = False; self.status = 'running'; self.note = None
        try:
            box = pkg.accumulator_box(width_in, a.length_in) if width_in else pkg.accumulator_box()
        except ValueError as e:
            self.done, self.status, self.note = True, 'infeasible', str(e); return
        self.wide = box['placement'].startswith('rear face')
        limits = {'braceRearYM': (0.26, a.brace_rear_y_max)} if self.wide else {}
        self.keys = WIDE_KEYS if self.wide else BAY_KEYS
        self.bounds = {k: ({**ps.BOUNDS, **fg.BOUNDS, **limits} if k == 'batteryPostXM' else {**ps.BOUNDS, **limits})[k] for k in self.keys}
        self.args = types.SimpleNamespace(catalog='vr3', max_weight=a.max_weight, restarts=a.restarts, seed=a.seed,
                                          budget=a.budget, main_hoop=a.main_hoop, limits=limits or None,
                                          packaging={'accumulator': box, 'axle': {'frac': frac, 'zM': a.axle_z}})
        self.box, self.start = box, dict(start)

    def key(self, p): return tuple(sorted((k, round(v, 4) if isinstance(v, float) else v) for k, v in p.items()))

    def initial(self):
        """009c's layout; for a box wider than the hoop legs also the screened layouts with the
        smallest feasible BR outboard offset (the 009c layout cannot hold it)."""
        pts = [dict(self.start)]
        if self.wide:
            ok = SCREENED.get(self.width_in)
            if ok is None:   # pre-checks depend on the box only, not on the axle: once per width
                ok = []
                for yb in np.arange(0.30, self.bounds['braceRearYM'][1] + 1e-9, 0.005):
                    for bx, xb, zb in itertools.product((0.24, 0.23, 0.22, 0.21, 0.20, 0.19), (0.10, 0.05), (0.34, 0.38, self.start['braceRearZM'], 0.42)):
                        p = dict(self.start, braceRearYM=round(float(yb), 4), batteryPostXM=bx, braceRearXM=xb, braceRearZM=zb)
                        if ps.precheck(p, self.args)[0] is not None: ok.append(p)
                    if len(ok) >= 3: break
                SCREENED[self.width_in] = ok
                print(f'w{self.width_in:g}: {len(ok)} screened start layouts, smallest BR y {ok[0]["braceRearYM"] if ok else None}', flush=True)
            if not ok:
                self.note = f"no layout within braceRearYM <= {self.bounds['braceRearYM'][1]} holds the box (pre-check)"
            # smallest offset first; prefer 009c's BR x/z and the most forward battery post
            ok.sort(key=lambda p: (p['braceRearYM'], p['braceRearXM'] != self.start['braceRearXM'],
                                   p['braceRearZM'] != self.start['braceRearZM'], -p['batteryPostXM']))
            pts += ok[:2] + [dict(ok[0], braceRearYM=round(min(ok[0]['braceRearYM'] + d, self.bounds['braceRearYM'][1]), 4)) for d in (0.02, 0.05)] if ok else []
        return pts

    def polls(self):
        """Next compass batch around the incumbent (halve the step after a batch with no gain)."""
        if self.iter > 0 and not self.improved: self.step /= 2
        self.iter += 1; self.improved = False
        if self.best is None or self.iter > (A.refine if self.wide else A.refine_narrow): return []
        x = self.best['params']; out = []
        for k in self.keys:
            lo, hi = self.bounds[k]
            for s in (1, -1):
                p = dict(x); p[k] = round(float(np.clip(x[k] + s * self.step * (hi - lo), lo, hi)), 4)
                out.append(p)
        return out

    def record(self, r, tag):
        r = dict(r); r['tag'] = tag; rec = r.pop('recipe', None)
        self.results.append(r)
        if r['valid'] and (self.best is None or r['ratio'] > self.best['ratio'] + 1e-9):
            self.best = r; self.improved = True
            out = {**rec, 'paramSearch': {k: v for k, v in r.items()}, 'scenario': self.describe()}
            (OUT / f'{self.sid}.json').write_text(json.dumps(out, indent=2, default=float) + '\n', encoding='utf-8')
        (OUT / f'{self.sid}-search.json').write_text(json.dumps({'scenario': self.describe(), 'results': self.results}, indent=2, default=float) + '\n', encoding='utf-8')
        print(f"[{self.sid} {tag}] {'ok ' if r['valid'] else 'REJ'} ratio {r.get('ratio', float('nan')):.3f} TR {r.get('tr', float('nan')):.0f} "
              f"W {r.get('weightLb', float('nan')):.2f}  best {self.best['ratio'] if self.best else float('nan'):.3f}  {(r.get('reason') or '')[:160]}", flush=True)

    def describe(self):
        d = {'id': self.sid, 'accumulatorWidthIn': self.width_in, 'axleFrac': self.frac, 'status': self.status, 'note': self.note}
        if hasattr(self, 'box'): d.update(box=self.box, wide=self.wide, searchedNodes=list(self.keys), windows=self.bounds,
                                          limits=self.args.limits, mainHoop=self.args.main_hoop)
        return d


def launch(pool, futs, sc, points, tag):
    """Submit the points not yet evaluated; pre-check failures are recorded without a worker."""
    while not sc.done:
        n = 0
        for i, p in enumerate(points):
            k = sc.key(p)
            if k in sc.seen: continue
            sc.seen.add(k)
            rec, why = ps.precheck(p, sc.args)
            if rec is None: sc.record({'params': p, 'valid': False, 'reason': why}, f'{tag}{i}-pre'); continue
            futs[pool.submit(ps.safe_evaluate, p, sc.args)] = (sc, f'{tag}{i}'); sc.pending += 1; n += 1
        if n: return
        points = sc.polls(); tag = f'poll{sc.iter}-'
        if not points: sc.done = True


def tube_changes(rec, ref):
    A_, B_ = {m['name']: m for m in rec['members']}, {m['name']: m for m in ref['members']}
    resized = sorted(f"{n}: {B_[n]['profile'].rsplit('.', 1)[0]} -> {A_[n]['profile'].rsplit('.', 1)[0]}"
                     for n in A_ if n in B_ and A_[n]['profile'].lower() != B_[n]['profile'].lower())
    nodes = {k: round(float(np.linalg.norm(np.subtract(rec['nodes'][k], ref['nodes'][k]))) * 1000, 1)
             for k in ('BRL', 'BAT0L', 'MHUL', 'MHBL', 'CS0L', 'CSUL') if k in rec['nodes'] and k in ref['nodes']}
    return {'added': sorted(set(A_) - set(B_)), 'removed': sorted(set(B_) - set(A_)), 'resized': resized,
            'nodeMovesMm': {k: v for k, v in nodes.items() if v > 0.5}, 'members': len(A_), 'membersRef': len(B_)}


def summarize(scs, ref):
    rows = []
    for sc in scs:
        d = sc.describe(); b = sc.best
        if b is None:
            reasons = sorted({(r.get('reason') or '')[:200] for r in sc.results})
            d.update(status='infeasible', reasons=reasons[:5]); rows.append(d); continue
        rec = json.loads((OUT / f'{sc.sid}.json').read_text(encoding='utf-8'))
        same = [r['ratio'] for r in sc.results if r['valid'] and r['params'] == b['params']]
        d['bestLayoutRepeats'] = {'n': len(same), 'min': min(same), 'median': float(np.median(same)), 'max': max(same)}
        d.update(status='valid', tr=b['tr'], weightLb=b['weightLb'], trPerLb=b['ratio'], trWork=b['trWork'],
                 halfshaftClearanceMm=b.get('halfshaftClearanceMm'), battery=b.get('battery'), params=b['params'],
                 evaluations=len(sc.results), changesVs009c=tube_changes(rec, ref),
                 braceRearM=rec['nodes']['BRL'], batteryPostFootM=rec['nodes']['BAT0L'],
                 rearFrameApexT=rec.get('halfshaftCorridor', {}).get('rearFrameApexT'))
        rows.append(d)
    (OUT / 'summary.json').write_text(json.dumps({'args': vars(A), 'scenarios': rows}, indent=2, default=float) + '\n', encoding='utf-8')
    lines = ['| Scenario | Width in | Axle % | TR lb-ft/deg | Weight lb | TR/lb | Same layout, other seeds (n: min-median) | BR y mm | BAT0 x mm | Shaft clr mm | Battery clr mm | Status |',
             '|---|---|---|---|---|---|---|---|---|---|---|---|']
    for d in rows:
        if d['status'] == 'valid':
            rp = d['bestLayoutRepeats']
            lines.append(f"| {d['id']} | {d['accumulatorWidthIn'] or 'starter 18.0'} | {d['axleFrac'] * 100:.0f} | {d['tr']:.0f} | {d['weightLb']:.2f} | "
                         f"{d['trPerLb']:.2f} | {rp['n']}: {rp['min']:.2f}-{rp['median']:.2f} | {d['braceRearM'][1] * 1000:.0f} | {d['batteryPostFootM'][0] * 1000:.0f} | "
                         f"{d['halfshaftClearanceMm']:.1f} | {d['battery']['clearanceMm']:.1f} | valid |")
        else:
            lines.append(f"| {d['id']} | {d['accumulatorWidthIn'] or 'starter 18.0'} | {d['axleFrac'] * 100:.0f} | - | - | - | - | - | - | - | - | "
                         f"infeasible: {(d.get('note') or (d.get('reasons') or [''])[0])[:160]} |")
    (OUT / 'summary.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print('\n'.join(lines), flush=True)


def main():
    global A, OUT
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--start', default='design/frame-009c.json')
    ap.add_argument('--widths', type=float, nargs='+', default=[19, 20, 21])
    ap.add_argument('--axle', type=float, nargs='+', default=[0.5, 0.6, 0.7, 0.8], help='fractions from the aft to the fore rear pickups')
    ap.add_argument('--axle-z', type=float, help='axle height, m (default: mean of the rear pickups, as 009c)')
    ap.add_argument('--length-in', type=float, help='accumulator length (default: the starter solid, 19.3 in)')
    ap.add_argument('--no-baseline', action='store_true', help='skip the starter-box / 50 %% reference scenario')
    ap.add_argument('--brace-rear-y-max', type=float, default=0.46, help='BR y window for boxes wider than the hoop legs (Codex: 0.33)')
    ap.add_argument('--max-weight', type=float, default=75.5)
    ap.add_argument('--restarts', type=int, default=2)
    ap.add_argument('--seed', type=int, default=51)
    ap.add_argument('--budget', type=float, default=240)
    ap.add_argument('--refine', type=int, default=3, help='compass iterations, box wider than the hoop legs')
    ap.add_argument('--refine-narrow', type=int, default=2, help='compass iterations, box between the hoop legs')
    ap.add_argument('--main-hoop', default='source', choices=['source', 'repair-002'])
    ap.add_argument('--workers', type=int, default=8)
    ap.add_argument('--only', nargs='*', help='scenario ids to run (default all)')
    ap.add_argument('--out-dir', default='design/scenarios-009d')
    ap.add_argument('--replicate-seeds', type=int, nargs='*', default=[],
                    help='instead of searching: re-run each scenario\'s best layout (from <out-dir>) with these inner-search seeds')
    ap.add_argument('--warm-seeds', type=int, nargs='*', help='replicate mode: also warm-started runs (from --warm-from) with these seeds')
    ap.add_argument('--warm-from', help='recipe whose tubes start every layout search (search mode) or the warm replicates; e.g. design/frame-009c.json')
    A = ap.parse_args(); OUT = ROOT / A.out_dir; OUT.mkdir(parents=True, exist_ok=True)
    ref = json.loads((ROOT / A.start).read_text(encoding='utf-8-sig'))
    start = {**ps.CODEX, **ref['paramSearch']['params']}
    warm = json.loads((ROOT / A.warm_from).read_text(encoding='utf-8-sig')) if A.warm_from else None
    scs = [] if A.no_baseline else [Scenario('starter-a50', None, 0.5, start, A)]
    scs += [Scenario(f'w{w:g}-a{round(f * 100)}', w, f, start, A) for w in A.widths for f in A.axle]
    if A.only: scs = [s for s in scs if s.sid in A.only]
    replicate = A.replicate_seeds or A.warm_seeds
    if warm and not replicate:
        for sc in scs:
            if hasattr(sc, 'args'): sc.args.warm = warm
    print(f"{len(scs)} scenarios, start {A.start} params {start}", flush=True)
    t0 = time.time(); futs = {}
    with ProcessPoolExecutor(max_workers=A.workers, initializer=ps._worker_init) as pool:
        if replicate:
            # The inner layout/size search is time-budgeted and path-dependent: one evaluation of
            # a layout varies by several TR/lb. Re-run each best layout with other seeds.
            for sc in scs:
                log = OUT / f'{sc.sid}-search.json'
                if sc.done or not log.exists(): continue
                saved = json.loads(log.read_text(encoding='utf-8'))
                sc.results, sc.note = saved['results'], saved['scenario'].get('note')
                valid = [r for r in sc.results if r['valid']]
                if not valid: sc.done = True; continue
                sc.best = max(valid, key=lambda r: r['ratio']); p = sc.best['params']
                jobs = [(s, None) for s in A.replicate_seeds] + [(s, warm) for s in (A.warm_seeds or [])]
                for s, wr in jobs:
                    a = types.SimpleNamespace(**{**vars(sc.args), 'seed': s, 'warm': wr})
                    futs[pool.submit(ps.safe_evaluate, p, a)] = (sc, f"{'warm' if wr else 'rep'}-s{s}"); sc.pending += 1
                sc.iter = 10 ** 6   # no compass polls after the replicates
            for sc in scs:
                if sc.best is None and not sc.done:   # never searched in this out-dir
                    sc.done = True
        for sc in scs:
            if not sc.done and not replicate: launch(pool, futs, sc, sc.initial(), 'start')
        while futs:
            done, _ = wait(futs, return_when=FIRST_COMPLETED)
            for f in done:
                sc, tag = futs.pop(f); sc.pending -= 1
                sc.record(f.result(), tag)
                if sc.pending == 0: launch(pool, futs, sc, sc.polls(), f'poll{sc.iter}-')
    for sc in scs:
        if sc.status == 'running': sc.status = 'valid' if sc.best else 'infeasible'
    print(f'done in {(time.time() - t0) / 60:.1f} min', flush=True)
    summarize(scs, ref)


A = OUT = None
SCREENED = {}
if __name__ == '__main__':
    main()
