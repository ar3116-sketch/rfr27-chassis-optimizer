"""Continuation over the photo keep-out zones (WORK_LOG 2026-10-20, recreated from the 20 Oct
scratchpad script). Optimise at one frame geometry with some zones switched off, then switch them
back on one step at a time, warm-starting each step from the previous step's answer. The last step
of every chain has every zone on, so its answer is the one that counts; the relaxed steps only
steer where the search starts.

  python scripts/continuation.py --params design/frame-011b.json --start-from design/frame-009c.json \
      --chain D --skeleton free --out design/frame-013-chainD.json

Chains (zone name prefixes still off at each step; photo_keepouts.load / PHOTO_KEEPOUTS_OFF):
  A: all off -> chain on -> front shock on -> switch on -> tie rod on -> rear shock on
  C: all off -> tie rod and rear shock off -> rear shock off -> all on
  D: all off -> tie rod on -> rear shock on -> switch on -> chain on (front shock last)
--steps '[{"off": "TieRod,RearShock"}, {"off": ""}]' gives a custom chain.
"""
import os
for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'): os.environ.setdefault(_v, '1')
from pathlib import Path
import argparse, io, contextlib, json, sys, time, types

sys.path.insert(0, str(Path(__file__).parent))
import param_search as ps

ROOT = ps.ROOT
ALL = 'TieRod,FrontShock,RearShock,MasterSwitch,Chain'
CHAINS = {
    'A': [ALL, 'TieRod,FrontShock,RearShock,MasterSwitch', 'TieRod,RearShock,MasterSwitch', 'TieRod,RearShock', 'RearShock', ''],
    'C': [ALL, 'TieRod,RearShock', 'RearShock', ''],
    'D': [ALL, 'FrontShock,RearShock,MasterSwitch,Chain', 'FrontShock,MasterSwitch,Chain', 'MasterSwitch,Chain', 'Chain', ''],
}


def load(p):
    return json.loads(Path(p).read_text(encoding='utf-8-sig'))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--params', required=True, help='recipe whose geometry parameters are used')
    ap.add_argument('--start-from', required=True, help='recipe that warm-starts the first step')
    ap.add_argument('--chain', choices=sorted(CHAINS))
    ap.add_argument('--steps', help='JSON list of {"off": "...", "cap": lb}; overrides --chain')
    ap.add_argument('--skeleton', choices=['fixed', 'free'], default='fixed')
    ap.add_argument('--max-weight', type=float, default=75.5)
    ap.add_argument('--budget', type=float, default=240)
    ap.add_argument('--restarts', type=int, default=2)
    ap.add_argument('--seed', type=int, default=51)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    steps = json.loads(a.steps) if a.steps else [{'off': o} for o in CHAINS[a.chain or 'A']]
    if steps[-1].get('off'): raise SystemExit('The last step must have every zone on ("off": "")')
    base = load(ROOT / a.params); params = base.get('paramSearch', {}).get('params') or base['parameters']
    warm = load(ROOT / a.start_from); hist = []; final = None
    for i, st in enumerate(steps):
        os.environ['PHOTO_KEEPOUTS_OFF'] = st.get('off', ''); ps._FORBIDDEN.clear()
        args = types.SimpleNamespace(catalog='vr3', restarts=a.restarts, seed=a.seed, budget=a.budget,
                                     max_weight=st.get('cap', a.max_weight), main_hoop='source', limits=None,
                                     packaging=None, warm=warm, skeleton=a.skeleton)
        t = time.time()
        with contextlib.redirect_stdout(io.StringIO()):
            r = ps.evaluate(params, args, budget=a.budget)
        h = {'step': i, **st, **{k: r.get(k) for k in ('valid', 'reason', 'ratio', 'tr', 'weightLb', 'trWork', 'members')},
             'seconds': round(time.time() - t)}
        hist.append(h); print(json.dumps(h), flush=True)
        if r.get('valid') and r.get('recipe'):
            warm = r['recipe']
            if not st.get('off') and st.get('cap', a.max_weight) == a.max_weight: final = r
    os.environ['PHOTO_KEEPOUTS_OFF'] = ''
    out = ROOT / a.out
    tag = f"continuation chain {a.chain or 'custom'} ({a.skeleton} skeleton) at {a.params} params, first step warm from {a.start_from}"
    if final:
        prov = {'params': params, **{k: final.get(k) for k in ('valid', 'ratio', 'tr', 'weightLb', 'trWork', 'skeleton')},
                'start': 'warm', 'seed': a.seed, 'tag': tag, 'history': hist}
        out.write_text(json.dumps({**final['recipe'], 'paramSearch': prov}, indent=2) + '\n', encoding='utf-8')
    out.with_name(out.stem + '-history.json').write_text(json.dumps({'args': vars(a), 'history': hist}, indent=1) + '\n', encoding='utf-8')
    print('FINAL', json.dumps({k: final.get(k) for k in ('valid', 'ratio', 'tr', 'weightLb', 'members')}) if final else 'none valid', flush=True)


if __name__ == '__main__':
    main()
