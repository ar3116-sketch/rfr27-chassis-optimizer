"""Skeleton sweep around one design: every distinct rule-legal re-routing of its side impact and
bulkhead-support structure that one compound move reaches (layout_engine.skeleton_moves, free
skeleton), each re-optimised by the inner search and checked independently.

For a design D at its own geometry: build the free-skeleton problem, put D's tubes in exactly,
list the valid compound skeleton moves, keep the best-gain move per topology (the set of tubes
added and removed, whatever the profiles), and run, for each one, the inner search (le.search)
warm from D with that re-routing applied. Results are verified with layout_engine.verify_design
(beam model, rule audit, photo zones at 1 mm, work-conjugate TR).

  python scripts/skeleton_sweep.py --recipe design/frame-011b.json --workers 12 --out design/skeleton-20261020/sweep-011b.json
"""
import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'): os.environ.setdefault(_v, '1')
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
import argparse, io, contextlib, json, sys, time

sys.path.insert(0, str(Path(__file__).parent))
import frame_geometry as fg, ground_structure as gs, layout_engine as le

ROOT = gs.ROOT


def problem(path):
    r = json.loads((ROOT / path).read_text(encoding='utf-8-sig'))
    params = r.get('paramSearch', {}).get('params') or r['parameters']
    pb = gs.Problem(fg.recipe(params), gs.CATALOG['vr3'], skeleton='free')
    group = {name: gi for gi, g in enumerate(pb.groups) for name, *_ in g['members']}
    cat = {c.lower(): c for c in pb.catalog}
    st = [None] * len(pb.groups)
    for m in r['members']: st[group[m.get('optimizerName', m['name'])]] = cat[m['profile'].lower()]
    return r, params, pb, st


def describe(pb, ch):
    rep = lambda g: g == min(g, pb.mirror[g])
    return {'added': sorted(f"{pb.groups[g]['name'].replace('Weld_', '')}:{p.rsplit('.', 1)[0]}" for g, p in ch.items() if p and rep(g)),
            'removed': sorted(pb.groups[g]['name'].replace('Weld_', '') for g, p in ch.items() if p is None and rep(g))}


def run(path, key, max_weight, restarts, seed, budget):
    t = time.time()
    r, params, pb, st = problem(path)
    eng = le.Engine(pb); eng.max_weight = max_weight; eng.factor(st)
    moves = {}
    for _, c, _, _ in le.skeleton_moves(eng):   # best gain first, as in main()
        moves.setdefault(tuple(sorted((g, p is None) for g, p in c.items())), c)
    ch = moves.get(key)
    if ch is None: return {'key': key, 'valid': False, 'reason': 'move not found in this process'}
    eng.apply_move(ch); eng.factor(eng.state)
    with contextlib.redirect_stdout(io.StringIO()):
        try: start, state, best, log = le.search(eng, restarts=restarts, seed=seed, budget_s=budget)
        except RuntimeError as e: return {'key': key, 'move': describe(pb, ch), 'valid': False, 'reason': str(e)}
    out = gs.to_recipe(pb, state)
    v = le.verify_design(out)
    over = v['weightLb'] > max_weight + 0.05
    ok = pb.valid_state(state) and v['ruleAudit']['pass'] and not over and not v['photoKeepoutHits']
    return {'key': key, 'move': describe(pb, ch), 'valid': ok, 'ratio': v['trPerLb'], 'tr': v['trLbFtPerDeg'],
            'weightLb': v['weightLb'], 'trWork': v['trWorkConjugate'], 'audit': v['ruleAudit'],
            'photoHits': len(v['photoKeepoutHits']), 'members': len(out['members']), 'seconds': round(time.time() - t),
            'recipe': out}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--recipe', required=True)
    ap.add_argument('--max-weight', type=float, default=75.5)
    ap.add_argument('--restarts', type=int, default=2)
    ap.add_argument('--seed', type=int, default=51)
    ap.add_argument('--budget', type=float, default=240)
    ap.add_argument('--workers', type=int, default=12)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    r, params, pb, st = problem(a.recipe)
    eng = le.Engine(pb); eng.max_weight = a.max_weight; eng.factor(st)
    base = le.verify_design(gs.to_recipe(pb, st))
    print(f"{a.recipe}: engine ratio {eng.score()['score']:.3f}, beam {base['trPerLb']:.3f}", flush=True)
    topo = {}
    for gain, ch, th, w in le.skeleton_moves(eng):
        k = frozenset((g, p is None) for g, p in ch.items())
        if k not in topo: topo[k] = (gain, ch)
    jobs = [(tuple(sorted((g, p is None) for g, p in ch.items())), gain, ch) for gain, ch in topo.values()]
    print(f'{len(jobs)} distinct legal skeleton re-routings', flush=True)
    res = []
    with ProcessPoolExecutor(max_workers=a.workers) as pool:
        futs = {pool.submit(run, a.recipe, key, a.max_weight, a.restarts, a.seed, a.budget): (gain, ch) for key, gain, ch in jobs}
        for f in as_completed(futs):
            x = f.result(); x['immediateGain'] = round(futs[f][0], 3); res.append(x)
            print(json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in x.items() if k not in ('recipe', 'key', 'audit')}), flush=True)
    res.sort(key=lambda x: -(x.get('ratio') or 0) if x.get('valid') else 0)
    out = ROOT / a.out
    out.write_text(json.dumps({'recipe': a.recipe, 'base': {k: base[k] for k in ('trPerLb', 'trLbFtPerDeg', 'weightLb', 'trWorkConjugate')},
                               'results': [{k: v for k, v in x.items() if k not in ('recipe', 'key')} for x in res]}, indent=1) + '\n', encoding='utf-8')
    best = next((x for x in res if x.get('valid')), None)
    if best:
        out.with_name(out.stem + '-best.json').write_text(json.dumps({**best['recipe'], 'paramSearch': {
            'params': params, 'skeleton': 'free', 'ratio': best['ratio'], 'tr': best['tr'], 'weightLb': best['weightLb'],
            'trWork': best['trWork'], 'valid': True, 'tag': f"skeleton sweep around {a.recipe}: {best['move']}"}}, indent=2) + '\n', encoding='utf-8')
    print('BEST', json.dumps({k: v for k, v in best.items() if k not in ('recipe', 'key', 'audit')}) if best else 'none valid', flush=True)


if __name__ == '__main__':
    main()
