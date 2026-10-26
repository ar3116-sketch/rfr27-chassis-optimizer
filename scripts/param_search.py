"""Outer search over node positions: Codex's rule-bounded frame parameters x inner
layout/size engine (scripts/layout_engine.py) for each sampled geometry.

frame_geometry.recipe(parameters) moves the free nodes (cockpit side posts, side-upper
heights, main-brace attachment and landing point, ...) only inside the rule windows
Codex derived. Each sample is rejected if a rule-required base tube enters a keep-out,
the driver template or a rule cockpit-template sweep. The inner engine then chooses
tubes and sizes. Latin-hypercube exploration, then coordinate refinement of the best.

Evaluations run in parallel worker processes (--workers): the Latin-hypercube samples
all at once, then a parallel compass search that polls every +/- step of every
parameter (and each discrete flip) around the incumbent, halving the step when no
poll improves.

  python scripts/param_search.py --samples 24 --refine 8 --workers 6 [--max-weight 85] --out design/frame-008.json
"""
import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'): os.environ.setdefault(_v, '1')
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
import argparse, importlib.util, json, math, random, sys, time, traceback
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import frame_geometry as fg
import ground_structure as gs
import layout_engine as le

ROOT = gs.ROOT
# Codex's generator windows, with the side-upper and brace heights narrowed to where the
# hoop bends stay supported (F.5.6.3; 1-D scans of the geometric audit at Codex's values).
BOUNDS = {'cockpitHalfWidthM': (.295, .335), 'cockpitStationXM': (.91, 1.0), 'sideUpperZM': (.350, .377),
          'frontSideUpperZM': (.3455, .3562), 'mainBraceZM': (1.246, 1.276), 'braceRearXM': (0, .10),
          'braceRearYM': (.26, .33), 'braceRearZM': (.34, .42)}
DISCRETE = {'rearBayProfile': ('B', 'C'), 'floorDiagonal': (True, False)}
CODEX = {'cockpitHalfWidthM': .305, 'cockpitStationXM': .945, 'sideUpperZM': .360, 'frontSideUpperZM': .3465,
         'mainBraceZM': 1.247, 'braceRearXM': .045, 'braceRearYM': .280, 'braceRearZM': .380, 'batteryPostXM': .240,
         'rearBayProfile': 'B', 'floorDiagonal': True}
# The supplied +10 mm templates are stricter than the rules (rule-constraints.md G24);
# base tubes are checked against keep-outs, the driver template and the rule sweeps.
# The native packaging gate also checks the supplied +10 mm opening sweep, so base tubes must
# clear it too (the supplied internal templates stay excluded: the hull estimate is conservative there).
RULE_HULLS = gs.RULE_HULL
# Audit checks fixed by the node positions alone (hoop bend support F.5.6.3, brace
# attachment and angles, hoop base width): tested before the inner optimization.
_spec = importlib.util.spec_from_file_location('audit', ROOT / 'scripts/audit-frame-rules.py')
AUDIT = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(AUDIT)
GEOMETRIC = ('_bend_support_', '_bend_radius', 'main_brace_attachment', 'main_brace_angle', 'main_brace_direction',
             'front_brace_attachment', 'front_brace_direction', 'main_hoop_base_width', 'main_hoop_side_inclination')


def geometry_failures(rec):
    return [c['name'] for c in AUDIT.audit(rec)['checks'] if not c['accepted'] and any(g in c['name'] for g in GEOMETRIC)]


_FORBIDDEN = {}


def _worker_init():
    sys.stdout = open(os.devnull, 'w')   # the engine narrates every move


def precheck(params, args, forbidden=None):
    """The checks fixed by the node positions alone, before any tube search: generator window,
    geometric rule audit, rule-required tubes against keep-outs and templates, and (scenario
    accumulator) the battery rules on the rule-required frame. Returns (recipe, None) or
    (None, reason)."""
    box = (getattr(args, 'packaging', None) or {}).get('accumulator')
    if forbidden is None:
        key = json.dumps(box, sort_keys=True)
        if key not in _FORBIDDEN: _FORBIDDEN[key] = gs.Forbidden(box)
        forbidden = _FORBIDDEN[key]
    try:
        rec = fg.recipe(params, main_hoop=getattr(args, 'main_hoop', 'source'), limits=getattr(args, 'limits', None))
    except ValueError as e:
        return None, f'generator: {e}'
    bad = geometry_failures(rec)
    if bad: return None, f'geometry audit {bad}'
    rule_hulls = [(n, eq) for n, eq in forbidden.hulls if RULE_HULLS(n)]
    free = getattr(args, 'skeleton', 'fixed') == 'free'
    for m in rec['members']:
        if free and m['role'] in gs.SKELETON_ROLES: continue   # released: the inner search judges each one
        a, b = np.array(m['a']), np.array(m['b'])
        for name, eq in rule_hulls:   # exact (photo_keepouts.seg_hull_min); sampling missed grazing contacts
            if gs.photo_keepouts.seg_hull_min(a, b, eq) < .0127:
                return None, f"{m['name']} enters {name}"
    if box:   # rule-required structure must leave the scenario battery inside its side protection
        bat = gs.pkg.battery_checks(rec, box)
        if not bat['pass']: return None, f'base frame vs accumulator: {bat}'
    return rec, None


def warm_state(pb, ref):
    """Start state carrying a reference design's tubes (matched by optimizer member name) onto
    this problem's geometry. Built up from the base state: each reference tube (and its mirror) is
    kept only if it passes the engine's own move test (valid_change), repeating until nothing more
    fits, so
    tubes that no longer exist, fit or satisfy the constraints are skipped one at a time.
    (Before 20 Oct the reference was applied wholesale and repaired by dropping optional tubes;
    one dropped pickup rail made the whole state invalid and it silently fell back to the base.)"""
    member_group = {name: gi for gi, g in enumerate(pb.groups) for name, *_ in g['members']}
    cat = {c.lower(): c for c in pb.catalog}
    want = {}
    for m in ref['members']:
        g = member_group.get(m.get('optimizerName', m['name'])); prof = cat.get(m['profile'].lower())
        if g is not None and prof in pb.allowed[g]: want[g] = prof
    state = pb.base_state()
    if getattr(pb, 'free', False):
        # Free skeleton: carry the reference's side and bulkhead-support panels over in one compound
        # move (one tube at a time cannot replace a crossing diagonal), then drop released Codex
        # tubes the reference does not have, where that stays valid.
        panel = {g for side in (1, -1) for g in pb.panel_groups[side]}
        ch = pb.close(state, {g: p for g, p in want.items() if g in panel})
        if ch:
            trial = list(state)
            for g, p in ch.items(): trial[g] = p
            if pb.valid_change(state, trial, set(ch)): state = trial
        for g in sorted(panel):
            mg = pb.mirror[g]
            if state[g] and pb.groups[g].get('skeleton') and g not in want and mg >= 0:
                trial = list(state); trial[g] = trial[mg] = None
                if pb.valid_change(state, trial, {g, mg}): state = trial
    for _ in range(4):
        added = 0
        for g, prof in sorted(want.items()):
            if state[g] == prof: continue
            trial = list(state); trial[g] = prof
            mg = pb.mirror[g]
            if mg != g and mg in want: trial[mg] = want[mg]
            changed = {g, mg}
            if pb.valid_change(state, trial, changed):   # the same per-move test the search engine uses
                state = trial; added += 1
        if not added: break
    return state


def evaluate(params, args, forbidden=None, budget=None):
    """args may carry packaging (layout_engine.packaging_from_args), main_hoop and limits
    (parameter windows wider than frame_geometry.BOUNDS); absent = the 009c inputs, except the
    main hoop, which defaults to the team's source sketch. args.warm (a recipe dict): start the
    layout search from that design's tubes on the stocked catalog instead of the cold two-phase
    search (a cold start can stall far from the X-braced designs when the geometry changes)."""
    packaging = getattr(args, 'packaging', None); warm = getattr(args, 'warm', None)
    skeleton = getattr(args, 'skeleton', 'fixed')
    t = time.time()
    rec, why = precheck(params, args, forbidden)
    if rec is None: return {'params': params, 'valid': False, 'reason': why, 'skeleton': skeleton}
    try:
        cat = 'vr3' if warm and args.catalog == 'vr3' else 'vr3-search' if args.catalog == 'vr3' else args.catalog
        pb = gs.Problem(rec, gs.CATALOG[cat], packaging=packaging, skeleton=skeleton); pb.brace = False
        eng = le.Engine(pb); eng.state = warm_state(pb, warm) if warm else pb.base_state(); eng.max_weight = args.max_weight
        run = le.search if warm or args.catalog != 'vr3' else le.two_phase_search
        start, state, best, log = run(eng, restarts=args.restarts, seed=args.seed, budget_s=budget or args.budget)
    except RuntimeError as e:
        return {'params': params, 'valid': False, 'reason': str(e), 'skeleton': skeleton}
    out = gs.to_recipe(pb, state)
    v = le.verify_design(out)
    over = args.max_weight is not None and v['weightLb'] > args.max_weight + 0.05
    shaft = v.get('halfshaftClearanceMm') is not None and v['halfshaftClearanceMm'] < -0.01
    battery = v.get('battery') is not None and not v['battery']['pass']
    ok = pb.valid_state(state) and v['ruleAudit']['pass'] and not over and not shaft and not battery
    reason = None if ok else (f"{v['weightLb']:.2f} lb over the cap" if over else
                              f"halfshaft clearance {v['halfshaftClearanceMm']:.1f} mm" if shaft else
                              f"accumulator {v['battery']}" if battery else f"audit {v['ruleAudit']['failed']}")
    return {'params': params, 'valid': ok, 'reason': reason, 'start': 'warm' if warm else 'cold', 'seed': args.seed, 'skeleton': skeleton,
            'ratio': v['trPerLb'], 'tr': v['trLbFtPerDeg'], 'weightLb': v['weightLb'], 'trWork': v['trWorkConjugate'],
            'halfshaftClearanceMm': v.get('halfshaftClearanceMm'), 'battery': v.get('battery'),
            'members': len(out['members']), 'seconds': round(time.time() - t, 1), 'recipe': out}


def lhs(n, rng, bounds=BOUNDS):
    keys = list(bounds); cols = {}
    for k in keys:
        lo, hi = bounds[k]; perm = rng.permutation(n)
        cols[k] = [lo + (hi - lo) * (perm[i] + rng.random()) / n for i in range(n)]
    out = []
    for i in range(n):
        p = {k: round(float(cols[k][i]), 4) for k in keys}
        p.update({k: rng.choice(v) if isinstance(v[0], str) else bool(rng.integers(2)) for k, v in DISCRETE.items()})
        out.append(p)
    return out


def safe_evaluate(params, args, budget=None):
    try:
        return evaluate(params, args, budget=budget)
    except Exception as e:
        return {'params': params, 'valid': False, 'reason': f'error: {e}', 'trace': traceback.format_exc()[-600:]}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--samples', type=int, default=24)
    ap.add_argument('--refine', type=int, default=8, help='compass-search iterations')
    ap.add_argument('--max-weight', type=float)
    ap.add_argument('--catalog', default='vr3')
    ap.add_argument('--restarts', type=int, default=2)
    ap.add_argument('--budget', type=float, default=120, help='inner search seconds per refinement evaluation')
    ap.add_argument('--screen-budget', type=float, default=60, help='inner search seconds per sample')
    ap.add_argument('--workers', type=int, default=6)
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--out', required=True)
    ap.add_argument('--main-hoop', default='source', choices=['source', 'repair-002'],
                    help="main hoop centreline: the team's sketch (default) or the 3.25 in die repair used up to 009c")
    ap.add_argument('--limit', action='append', default=[], metavar='PARAM=LO:HI',
                    help='widen/narrow a parameter window, e.g. braceRearYM=0.26:0.42 (searched and allowed by the generator)')
    ap.add_argument('--start', help='recipe (paramSearch.params or parameters) or JSON of parameters to start from instead of Codex values')
    ap.add_argument('--warm-from', help="recipe whose tubes start every inner layout search (param_search.warm_state), e.g. design/frame-009c.json")
    ap.add_argument('--skeleton', choices=['fixed', 'free'], default='fixed',
                    help="free: Codex's side impact and bulkhead-support tubes are optional and any layout that passes the "
                         "name-free F.6.4/F.6.2.3 predicates (scripts/rule_structure.py) may replace them")
    le.add_packaging_args(ap)
    args = ap.parse_args()
    args.warm = json.loads((ROOT / args.warm_from).read_text(encoding='utf-8-sig')) if args.warm_from else None
    args.limits = {}
    for s in args.limit:
        k, v = s.split('='); lo, hi = (float(x) for x in v.split(':')); args.limits[k] = (lo, hi)
    args.packaging = le.packaging_from_args(args, args.main_hoop)
    bounds = {**BOUNDS, **args.limits}
    start = dict(CODEX)
    if args.start:
        s = json.loads((ROOT / args.start).read_text(encoding='utf-8-sig'))
        start.update(s.get('paramSearch', {}).get('params') or s.get('parameters') or s)
    rng = np.random.default_rng(args.seed)
    results = []; best = None
    log_path = ROOT / (args.out.replace('.json', '') + '-search.json')

    def record(r, tag):
        nonlocal best
        r['tag'] = tag
        results.append({k: v for k, v in r.items() if k != 'recipe'})
        improved = r['valid'] and (best is None or r['ratio'] > best['ratio'])
        if improved:
            best = r
            (ROOT / args.out).write_text(json.dumps({**r['recipe'], 'paramSearch': {k: v for k, v in r.items() if k != 'recipe'}}, indent=2) + '\n', encoding='utf-8')
        print(f"[{tag}] {'ok ' if r['valid'] else 'REJ'} ratio {r.get('ratio', float('nan')):.3f} "
              f"W {r.get('weightLb', float('nan')):.1f} lb  best {best['ratio'] if best else float('nan'):.3f}  "
              f"{r.get('reason') or ''}", flush=True)
        log_path.write_text(json.dumps({'args': vars(args), 'results': results}, indent=2, default=str) + '\n', encoding='utf-8')
        return improved

    def batch(pool, jobs, budget):
        futs = {pool.submit(safe_evaluate, p, args, budget): tag for p, tag in jobs}
        for f in as_completed(futs): record(f.result(), futs[f])

    with ProcessPoolExecutor(max_workers=args.workers, initializer=_worker_init) as pool:
        batch(pool, [(start, 'start' if args.start else 'codex')] + [(p, f'lhs{i + 1}') for i, p in enumerate(lhs(args.samples, rng, bounds))], args.screen_budget)
        # Parallel compass search around the incumbent.
        step = 0.25
        for it in range(args.refine):
            if best is None: break
            x = best['params']; polls = []
            for k, (lo, hi) in bounds.items():
                for sgn in (1, -1):
                    p = dict(x); p[k] = round(float(np.clip(x[k] + sgn * step * (hi - lo), lo, hi)), 4)
                    if p != x: polls.append((p, f'poll{it + 1}{"+" if sgn > 0 else "-"}{k}'))
            for k, opts in DISCRETE.items():
                for o in opts:
                    if o != x[k]: p = dict(x); p[k] = o; polls.append((p, f'poll{it + 1}:{k}={o}'))
            before = best['ratio']
            batch(pool, polls + [(dict(x), f'poll{it + 1}:repeat')], args.budget)
            if best['ratio'] <= before + 1e-6:
                step /= 2; print(f'-- no poll improved; step -> {step:.4f}', flush=True)
    print(f"Best ratio {best['ratio']:.3f} at {best['weightLb']:.1f} lb with {best['params']}" if best else 'No valid design')


if __name__ == '__main__':
    main()
