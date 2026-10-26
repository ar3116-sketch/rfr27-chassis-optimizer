"""Unattended native build/trim driver with visual + git checkpoints.

Chains the existing typed native jobs (build-connecting-frame, trim-connecting-frame,
snapshot) so a candidate goes from recipe to fully trimmed weldment without hand-written
job files. Each successful batch leaves: the saved CAD, its trim journal, PNG snapshots
under checkpoints/, a WORK_LOG entry and a git commit. Stops at the first failure; it
never retries, relaxes tolerances or edits recipes. Trimming is not a frame gate: the
independent joint/rule/driver/simulation checks are still required afterwards.

  python scripts/chassis_pipeline.py status   --candidate 005
  python scripts/chassis_pipeline.py snapshot --candidate 005 [--label note]
  python scripts/chassis_pipeline.py trim     --candidate 005 [--batch 40] [--max-batches N]
  python scripts/chassis_pipeline.py build    --candidate 006 [--recipe design/frame-006.json] [--source part.SLDPRT]
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse, math, hashlib, json, os, re, shutil, subprocess, sys

ROOT = Path(__file__).resolve().parents[1]
JOBS = ROOT / 'scripts/jobs/auto'
CHECKPOINTS = ROOT / 'checkpoints'
LOCK = ROOT / 'handoff/native-cad.lock'
# node: RFR_NODE, then PATH, then the default install folder.
NODE = os.environ.get('RFR_NODE') or shutil.which('node') or 'C:/Program Files/nodejs/node.exe'


def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load(p): return json.loads(Path(p).read_text(encoding='utf-8-sig'))
def now(): return datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
def rel(p): return Path(p).resolve().relative_to(ROOT).as_posix()


class Candidate:
    def __init__(self, cid, recipe=None):
        self.id = cid
        d = ROOT / f'cad/working/candidate-{cid}'
        self.part = d / f'RFR27-candidate-{cid}.SLDPRT'
        self.layout = d / f'RFR27-driver-layout-{cid}.SLDASM'
        self.journal = self.part.with_suffix('.trim-progress.json')
        self.build_journal = self.part.with_suffix('.build-progress.json')
        # Recipe: explicit, else the one a shop-order build recorded, else design/frame-<id>.json.
        if recipe: self.recipe = Path(recipe).resolve()
        elif self.build_journal.exists(): self.recipe = Path(load(self.build_journal)['recipe'])
        else: self.recipe = ROOT / f'design/frame-{cid}.json'
        self.shots = CHECKPOINTS / f'candidate-{cid}'

    def total(self): return len(load(self.recipe)['trims'])

    def accept_recipe_change(self, j):
        """Allow a regenerated recipe only if everything already built/trimmed is identical.

        The previous recipe is taken from git (it must match the journal hash), then
        nodes/groups/members and every already-processed trim are compared exactly."""
        try:
            prev_bytes = subprocess.run(['git', 'show', 'HEAD:' + rel(self.recipe)], cwd=ROOT, check=True,
                                        stdout=subprocess.PIPE).stdout
        except subprocess.CalledProcessError:
            sys.exit(f'Recipe changed and no committed version to compare against ({rel(self.recipe)}).')
        if hashlib.sha256(prev_bytes).hexdigest() != j['recipeSha256']:
            sys.exit('Recipe changed and the committed version is not the one the CAD was trimmed with. Inspect before resuming.')
        prev, cur, n = json.loads(prev_bytes.decode('utf-8-sig')), load(self.recipe), j['nextTrimIndex']
        for k in ('nodes', 'groups', 'members', 'parameters'):
            if prev.get(k) != cur.get(k): sys.exit(f'Recipe {k} changed; built geometry no longer matches. Make a new candidate instead.')
        if prev['trims'][:n] != cur['trims'][:n]:
            sys.exit(f'Recipe changed within the {n} already-processed trims. Make a new candidate instead.')
        print(f'Recipe changed only after processed index {n}: {len(prev["trims"])} -> {len(cur["trims"])} trims. Accepted.')
        j['recipeSha256'] = sha(self.recipe); j['recipeChangeAcceptedUtc'] = now()
        self.journal.write_text(json.dumps(j, indent=2) + '\n', encoding='utf-8')

    def next_index(self):
        """Resume point from the native per-save journal, refusing if CAD or recipe drifted."""
        if not self.journal.exists(): return 0
        j = load(self.journal)
        if j['recipeSha256'] != sha(self.recipe):
            self.accept_recipe_change(j)
        if j['cadSha256'] != sha(self.part):
            sys.exit(f'{rel(self.part)} changed outside the trim journal. Inspect it (git diff/log) before resuming.')
        return j['nextTrimIndex']


def stale_lock():
    """True if the native lock's owning process no longer exists."""
    try: pid = int(load(LOCK)['pid'])
    except (OSError, ValueError, KeyError): return False
    out = subprocess.run(['tasklist', '/FI', f'PID eq {pid}', '/NH'], stdout=subprocess.PIPE, text=True).stdout
    return str(pid) not in out


def run_job(name, job):
    if LOCK.exists(): sys.exit(f'Native CAD lock present ({rel(LOCK)}); another job owns SolidWorks.')
    JOBS.mkdir(parents=True, exist_ok=True)
    path = JOBS / f'{name}.json'
    path.write_text(json.dumps(job, indent=2) + '\n', encoding='utf-8')
    print(f'[{now()}] native {job["command"]} -> {rel(path)}', flush=True)
    p = subprocess.run([NODE, str(ROOT / 'scripts/run-native.mjs'), str(path)], cwd=ROOT,
                       stdout=subprocess.PIPE, text=True)
    lines = [l for l in p.stdout.splitlines() if l.startswith('{')]
    summary = json.loads(lines[-1]) if lines else {}
    evidence = Path(summary.get('evidence', ''))
    ok = p.returncode == 0 and not summary.get('error')
    out, err = {}, summary.get('error')
    if (evidence / 'stdout.json').exists():
        try:
            raw = load(evidence / 'stdout.json')
            out = raw.get('result', {}) if ok else {}
            if not ok and not err: err = (str(raw.get('error') or '').splitlines() or [''])[0]
        except (ValueError, OSError):
            pass
    return ok, evidence, out, err


def crop(img, pad=40):
    """Trim the empty viewport around the chassis: keep dark (tubes) or saturated (keep-outs, datums) pixels."""
    import numpy as np
    a = np.asarray(img).astype(int)
    mask = (a.max(2) - a.min(2) > 40) | (a.max(2) < 150)
    ys, xs = np.nonzero(mask)
    if len(xs) == 0: return img
    return img.crop((max(xs.min() - pad, 0), max(ys.min() - pad, 0),
                     min(xs.max() + pad, img.width), min(ys.max() + pad, img.height)))


def to_png(images):
    """Convert native BMP snapshots to PNG (BMPs are 4.8 MB each and git-ignored)."""
    from PIL import Image
    pngs = []
    for bmp in images:
        bmp = Path(bmp)
        if not bmp.exists(): continue
        png = bmp.with_suffix('.png')
        crop(Image.open(bmp).convert('RGB')).save(png, optimize=True)
        bmp.unlink(); pngs.append(png)
    return pngs


def snapshot(c, label, include_part=True):
    folder = c.shots / label
    paths = ([str(c.part)] if include_part else []) + [str(c.layout)]
    ok, ev, out, err = run_job(f'snapshot-{c.id}-{label}', {
        'command': 'snapshot', 'paths': paths, 'outputDir': str(folder),
        'restore': str(c.layout), 'timeoutMs': 180000})
    if not ok:
        print(f'Snapshot failed ({err}); evidence {rel(ev) if ev.name else "none"}'); return []
    return to_png([v for s in out['shots'] for v in s['images'].values()])


def update_index():
    rows = []
    for cdir in sorted(CHECKPOINTS.glob('candidate-*')):
        for f in sorted(p for p in cdir.iterdir() if p.is_dir()):
            meta = f / 'checkpoint.json'
            m = load(meta) if meta.exists() else {}
            imgs = ' '.join(f'![{p.stem}]({p.relative_to(CHECKPOINTS).as_posix()})' for p in sorted(f.glob('*-iso.png')))
            rows.append(f"| {cdir.name} | {f.name} | {m.get('utc', '')} | {m.get('note', '')} | {imgs} |")
    CHECKPOINTS.mkdir(exist_ok=True)
    (CHECKPOINTS / 'INDEX.md').write_text(
        '# Visual checkpoints\n\nGenerated by `scripts/chassis_pipeline.py`. Each folder has isometric and side PNGs of the '
        'bare frame part and the driver layout (frame + Percy). Commit history pairs each folder with the exact CAD.\n\n'
        '| Candidate | Checkpoint | UTC | Note | Isometric |\n|---|---|---|---|---|\n' + '\n'.join(rows) + '\n', encoding='utf-8')


def record(c, label, note, evidence=None):
    folder = c.shots / label; folder.mkdir(parents=True, exist_ok=True)
    meta = {'utc': now(), 'candidate': c.id, 'note': note, 'part': rel(c.part),
            'cadSha256': sha(c.part) if c.part.exists() else None,
            'recipe': rel(c.recipe), 'recipeSha256': sha(c.recipe),
            'evidence': rel(evidence) if evidence and evidence.name else None}
    (folder / 'checkpoint.json').write_text(json.dumps(meta, indent=2) + '\n', encoding='utf-8')
    update_index()
    with open(ROOT / 'handoff/WORK_LOG.md', 'a', encoding='utf-8') as log:
        log.write(f"\n## {meta['utc']} - pipeline checkpoint candidate-{c.id} {label}\n- {note}\n"
                  f"- CAD sha256 {meta['cadSha256']}; evidence {meta['evidence']}; images checkpoints/candidate-{c.id}/{label}/\n")
    return meta


def commit(message):
    if not (ROOT / '.git').exists() or ARGS.no_commit: return
    subprocess.run(['git', 'add', '-A'], cwd=ROOT, check=True)
    if subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=ROOT).returncode == 0: return
    subprocess.run(['git', 'commit', '-q', '-m', message + '\n\nAutomated checkpoint from scripts/chassis_pipeline.py'],
                   cwd=ROOT, check=True)
    print('  committed: ' + message, flush=True)


def trim(c):
    total, start = c.total(), c.next_index()
    batches = 0
    while start < total and (ARGS.max_batches is None or batches < ARGS.max_batches):
        count = min(ARGS.batch, total - start)
        print(f'\n=== candidate-{c.id}: trims {start}..{start + count - 1} of {total} ===', flush=True)
        ok, ev, out, err = run_job(f'trim-{c.id}-{start:03d}', {
            'command': 'trim-connecting-frame', 'path': str(c.part), 'restore': str(c.layout),
            'recipe': str(c.recipe), 'start': start, 'count': count,
            'snapshotDir': str(c.shots / f'{start + count:04d}-trims'), 'timeoutMs': 600000})
        reached = load(c.journal)['nextTrimIndex'] if c.journal.exists() else start
        if not ok:
            label = f'{reached:04d}-FAILED'
            note = f'Trim batch from {start} FAILED at index {reached}: {err or "native error"}. Last good save kept; nothing retried.'
            print('\n' + note); print(f'Evidence: {rel(ev)}  (see stdout.json / stderr.txt)')
            if not LOCK.exists(): snapshot(c, label)
            record(c, label, note, ev); commit(f'candidate-{c.id}: trim FAILED at {reached}/{total}')
            sys.exit(1)
        if reached != start + count: sys.exit(f'Journal says next={reached}, expected {start + count}; stopping.')
        label = f'{reached:04d}-trims'
        to_png((out.get('snapshot') or {}).values())
        snapshot(c, label, include_part=False)
        skipped = sum(1 for x in out.get('cuts', []) if x.get('skipped'))
        record(c, label, f'Trims {start}..{reached - 1} passed ({count - skipped} cuts, {skipped} no-overlap skips); {reached}/{total} processed.', ev)
        commit(f'candidate-{c.id}: trims {start}-{reached - 1} ({reached}/{total})')
        start, batches = reached, batches + 1
    if start >= total: print(f'\ncandidate-{c.id}: all {total} trim operations processed. Next: independent joint-fit/rule/driver audits.')


def build(c):
    if c.part.exists(): sys.exit(f'{rel(c.part)} already exists; build only makes fresh candidates.')
    r = load(c.recipe); r['trims'] = []
    build_only = c.recipe.with_name(c.recipe.stem + '-build-only.json')
    build_only.write_text(json.dumps(r, indent=2) + '\n', encoding='utf-8')
    job = {'command': 'build-connecting-frame', 'recipe': str(build_only), 'output': str(c.part), 'timeoutMs': 900000}
    if ARGS.source: job['source'] = str(Path(ARGS.source).resolve())
    ok, ev, out, err = run_job(f'build-{c.id}', job)
    if not ok: print(f'Build failed: {err}; evidence {rel(ev)}'); sys.exit(1)
    for b in c.part.parent.glob('*.bmp'): b.unlink()
    record(c, '0000-built', f'Built {len(r["groups"])} weldment groups from {rel(c.recipe)}; untrimmed.', ev)
    commit(f'candidate-{c.id}: built untrimmed weldment frame')
    print(f'Built {rel(c.part)}. Create/relink {rel(c.layout)} with Percy before trimming (trim jobs restore it).')


def build_vr3(c):
    """VR3 shop-order build: each group is added and its tubes coped immediately, in batches."""
    journal = c.part.with_suffix('.build-progress.json')
    total = len(load(c.recipe)['groups'])
    start = load(journal)['nextGroup'] if journal.exists() else 0
    if start == 0 and c.part.exists(): sys.exit(f'{rel(c.part)} exists without a build journal; use a fresh candidate id.')
    retried, timed_out = set(), set()   # groups already retried once after a failure / timeout
    while start < total:
        count = min(ARGS.batch, total - start)
        job = {'command': 'build-trim-frame', 'recipe': str(c.recipe), 'output': str(c.part), 'start': start, 'count': count,
               'defineSketches': True, 'snapshotDir': str(c.shots / f'g{start + count:03d}-groups'), 'timeoutMs': 900000}
        if start == 0 and ARGS.source: job['source'] = str(Path(ARGS.source).resolve())
        if c.layout.exists(): job['restore'] = str(c.layout)
        print(f'\n=== candidate-{c.id}: groups {start}..{start + count - 1} of {total} (build + trim as added) ===', flush=True)
        ok, ev, out, err = run_job(f'build-vr3-{c.id}-{start:03d}', job)
        reached = load(journal)['nextGroup'] if journal.exists() else start
        # Native timeout (a batch outgrew the 15 min job limit as the part got heavier): the runner
        # leaves the lock. If its process is gone, archive the lock, close the unsaved part without
        # saving, check the saved CAD still matches the journal, halve the batch and continue.
        if not ok and 'timeout' in (err or '').lower() and ARGS.batch > 1 and reached not in timed_out and stale_lock():
            timed_out.add(reached)
            LOCK.rename(Path(ev) / 'stale-native-cad.lock')
            rok, rev, rout, rerr = run_job(f'recover-{c.id}-{reached:03d}', {'command': 'recover-part', 'path': str(c.part), 'timeoutMs': 120000})
            if not rok or (journal.exists() and load(journal)['cadSha256'] != sha(c.part)):
                sys.exit(f'Timeout recovery failed at group {reached}: {rerr or "CAD differs from the build journal"}; evidence {rel(rev)}')
            ARGS.batch = max(1, ARGS.batch // 2)
            note = f'VR3 build batch from group {start} timed out at group {reached}; recovered (unsaved changes discarded), continuing with {ARGS.batch} groups per job.'
            print('\n' + note); record(c, f'g{reached:03d}-timeout', note, ev); commit(f'candidate-{c.id}: VR3 build timeout recovered at group {reached}/{total}')
            start = reached; continue
        # One automatic retry from the last saved group: SolidWorks occasionally refuses a
        # feature once (seen: InsertStructuralWeldment4 null at a group that built on retry).
        # Native timeouts are not retried; they need the lock/CAD inspection described in the log.
        if not ok and reached not in retried and 'timeout' not in (err or '').lower() and not LOCK.exists():
            retried.add(reached)
            note = f'VR3 build from group {start} failed at group {reached}: {err or "native error"}; retrying once from the last saved group.'
            print('\n' + note); record(c, f'g{reached:03d}-retry', note, ev); commit(f'candidate-{c.id}: VR3 build retry at group {reached}/{total}')
            start = reached; continue
        if not ok:
            note = f'VR3 build from group {start} FAILED at group {reached}: {err or "native error"}. Last saved group kept.'
            print('\n' + note); print(f'Evidence: {rel(ev)}')
            record(c, f'g{reached:03d}-FAILED', note, ev); commit(f'candidate-{c.id}: VR3 build FAILED at group {reached}/{total}')
            sys.exit(1)
        to_png((out.get('snapshot') or {}).values())
        label = f'g{reached:03d}-groups'
        cuts = out.get('cuts', [])
        gaps = [f"{x['operation']} {x['gapWarning']['gapM'] * 1000:.2f} mm" for x in cuts if x.get('gapWarning')]
        waste = [x['operation'] for x in cuts if x.get('copeWaste')]
        record(c, label, f'Groups {start}..{reached - 1} built and coped in shop order; {len(cuts)} cope operations; '
                         f'sketches fully defined: {all(x.get("fullyDefined") for x in out.get("sketchStatus", []))}.'
                         + (f' Multiway gaps over 0.05 in: {", ".join(gaps)}.' if gaps else '')
                         + (f' Cope waste removed: {", ".join(waste)}.' if waste else ''), ev)
        commit(f'candidate-{c.id}: VR3 build groups {start}-{reached - 1} ({reached}/{total})')
        start = reached
    print(f'candidate-{c.id}: VR3-order build complete ({total} groups).')


def stamp(): return datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')


def layout(c):
    """Driver-layout assembly: the candidate part plus Percy in the proposed seated pose."""
    if c.layout.exists(): sys.exit(f'{rel(c.layout)} already exists.')
    ident = [1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0]
    ok, ev, out, err = run_job(f'layout-{c.id}', {
        'command': 'pose-assembly', 'output': str(c.layout), 'poseDefinition': str(ROOT / 'design/percy-pose-001.json'),
        'components': [{'path': str(c.part), 'name': f'Chassis_candidate_{c.id}', 'transform': ident, 'configuration': ARGS.configuration},
                       {'path': str(ROOT / ARGS.percy), 'name': 'Percy_proposed_pose', 'transform': ident}],
        'timeoutMs': 600000})
    if not ok: sys.exit(f'Layout failed: {err}; evidence {rel(ev)}')
    snapshot(c, 'layout', include_part=False); record(c, 'layout', 'Driver layout assembly with Percy created.', ev)
    commit(f'candidate-{c.id}: driver layout assembly')


def audit(c):
    """Native gates on the built, trimmed part: templates/driver circles and Percy/keep-out/overlap audit."""
    checks = ROOT / f'design/driver-checks-{c.id}.json'
    subprocess.run([sys.executable, str(ROOT / 'scripts/make-driver-checks.py'), str(c.recipe), str(checks)], cwd=ROOT, check=True)
    # Photo-estimated packaging keep-outs (scripts/photo_keepouts.py) are audited as extra envelopes.
    photo = ROOT / 'design/keepouts-photo-001-layout.json'
    if photo.exists():
        d = load(checks); d['sweptEnvelopes'] += load(photo)['sweptEnvelopes']
        checks.write_text(json.dumps(d, indent=2) + '\n', encoding='utf-8')
    t = stamp(); results = {}
    ok, ev, out, err = run_job(f'packaging-{c.id}', {
        'command': 'packaging-audit', 'layoutRecipe': str(checks), 'recipe': str(c.recipe), 'path': str(c.part),
        'output': str(ROOT / f'cad/tests/packaging-{c.id}-{t}/envelopes.SLDPRT'), 'restore': str(c.layout), 'timeoutMs': 900000})
    reports = out.get('reports', []) if ok else []
    results['packaging'] = {'ok': ok, 'evidence': rel(ev) if ev.name else None,
                            'accepted': ok and bool(reports) and all(r.get('accepted', False) for r in reports),
                            'failed': [r['name'] for r in reports if not r.get('accepted')] if ok else [err]}
    # The native main-hoop audit (HoopAudit.cs) compares the built one-piece hoop with the path it was
    # built from: the 3.25 in die repair (up to 009c), or for mainHoopPath 'source' (010 on) the team's
    # sketch with its two 0.383 deg corners rounded (design/main-hoop-source-001.json, candidate-002s).
    hoop_path = load(c.recipe).get('mainHoopPath', 'repair-002')
    job = {'command': 'chassis-audit', 'path': str(c.part), 'snapshot': str(ROOT / f'cad/tests/chassis-audit-{c.id}-{t}/chassis.SLDPRT'),
           'restore': str(c.layout), 'percy': str(ROOT / ARGS.percy),
           'configuration': ARGS.configuration, 'baseline': str(ROOT / ARGS.baseline), 'timeoutMs': 900000}
    hoop_recipe = {'repair-002': 'design/main-hoop-repair-002.json', 'source': 'design/main-hoop-source-001.json'}.get(hoop_path)
    if hoop_recipe and (ROOT / hoop_recipe).exists(): job['mainHoopRecipe'] = str(ROOT / hoop_recipe)
    ok, ev, out, err = run_job(f'chassis-audit-{c.id}', job)
    keys = ('nativeRebuild', 'bodyValidity', 'protectedReferences', 'keepoutClearance', 'driverSolidIntersectionFree', 'tubeIntersectionsFree')
    chassis = {'ok': ok, 'evidence': rel(ev) if ev.name else None, 'mainHoopPath': hoop_path,
               'mainHoopBodies': out.get('mainHoopBodies') if ok else None}
    chassis.update({k: out.get(k) for k in keys} if ok else {'error': err})
    chassis['weightLb'] = out.get('mass', {}).get('weightLb') if ok else None
    results['chassis'] = chassis
    passed = bool(results['packaging']['accepted'] and ok and all(out.get(k) for k in keys))
    (ROOT / f'design/gates-{c.id}.json').write_text(json.dumps({'utc': now(), 'passed': passed, **results}, indent=2) + '\n', encoding='utf-8')
    note = (f"Native gates {'PASSED' if passed else 'FAILED'}: packaging {results['packaging']['failed'] or 'all accepted'}; "
            f"chassis { {k: chassis.get(k) for k in keys} }; structural weight {chassis.get('weightLb')} lb.")
    print(note); record(c, f'gates-{t}', note); commit(f'candidate-{c.id}: native gates {"pass" if passed else "FAIL"}')
    if not passed: sys.exit(1)


def simulate(c):
    """Prescribed TR study on the trimmed part, validated, scored like the tracker."""
    import importlib.util
    spec = importlib.util.spec_from_file_location('vcs', ROOT / 'scripts/validate-chassis-simulation.py')
    v = importlib.util.module_from_spec(spec); spec.loader.exec_module(v)
    recipe = load(c.recipe); pts = recipe['fixturePoints'] + recipe['loadPoints']; prefer = set()
    # A point pick can return a joint in front of the requested one. If the result validation
    # finds no restrained/loaded node near a requested point, re-pick that point and its mirror
    # from the car's side (preferSideView) and solve again.
    for attempt in range(3):
        t = stamp()
        ok, ev, out, err = run_job(f'sim-{c.id}', {
            'command': 'chassis-simulation', 'source': str(c.part), 'recipe': str(c.recipe), 'restore': str(c.layout),
            'output': str(ROOT / f'cad/tests/chassis-sim-{c.id}-{t}/RFR27-candidate-{c.id}-study.SLDPRT'),
            'preferSideView': sorted(prefer), 'timeoutMs': 900000})
        if not ok: sys.exit(f'Simulation failed: {err}; evidence {rel(ev)}')
        try:
            res = v.validate(out, recipe); break
        except ValueError as e:
            m = re.search(r'No (?:fixture|load) node within \d+ mm of \[([^\]]+)\]', str(e))
            if not m or attempt == 2: raise
            q = [float(x) for x in m.group(1).split(',')]
            idx = {i for i, p in enumerate(pts) if math.dist(p, q) < 1e-6 or math.dist(p, [q[0], -q[1], q[2]]) < 1e-6}
            if not idx or idx <= prefer: raise
            prefer |= idx; print(f'  validation: {e}; re-picking points {sorted(idx)} from the side view', flush=True)
    (ev / 'validation.json').write_text(json.dumps(res, indent=2, default=str) + '\n', encoding='utf-8')
    gpath = ROOT / f'design/gates-{c.id}.json'
    gates = load(gpath) if gpath.exists() else {}
    w = (gates.get('chassis') or {}).get('weightLb')
    rx = res['rxDeg']; tr = 545 / abs(sum(rx) / 2)
    score = {'utc': now(), 'rx1ForeDeg': rx[1], 'rx2AftDeg': rx[0], 'trLbFtPerDeg': tr, 'weightLb': w,
             'trPerLb': tr / w if w else None, 'evidence': rel(ev), 'gatesPassed': gates.get('passed')}
    (ROOT / f'design/score-{c.id}.json').write_text(json.dumps(score, indent=2) + '\n', encoding='utf-8')
    note = f"SolidWorks TR {tr:.0f} lb-ft/deg (RX1 fore {rx[1]:.4f}, RX2 aft {rx[0]:.4f} deg), weight {w} lb, ratio {score['trPerLb']}"
    print(note); record(c, f'sim-{t}', note, ev); commit(f'candidate-{c.id}: validated SolidWorks TR study')


def package(c):
    """VR3 quote package (numbered cut list, two-sheet drawing + PDF, Excel BOM)."""
    out_dir = ROOT / f'cad/exports/candidate-{c.id}-vr3'
    vr3_part, numbers = out_dir / f'{c.part.stem}-VR3.SLDPRT', out_dir / f'{c.part.stem}-partnumbers.json'
    # Two native jobs, each with its own time budget: numbered cut list, then the drawing.
    if vr3_part.exists() and numbers.exists():
        print(f'Numbered cut list already in {rel(vr3_part)}; making the drawing.')
    else:
        if out_dir.exists() and any(out_dir.iterdir()): sys.exit(f'{rel(out_dir)} holds a partial package; inspect or remove it first.')
        ok, ev, out, err = run_job(f'vr3-package-{c.id}', {'command': 'vr3-package', 'path': str(c.part), 'recipe': str(c.recipe),
                                                           'outputDir': str(out_dir), 'restore': str(c.layout), 'timeoutMs': 900000})
        if not ok: sys.exit(f'VR3 cut list failed: {err}; evidence {rel(ev)}')
        record(c, 'vr3-cutlist', f"VR3 cut list: {out['tubes']} tubes, one item and part number each, in {rel(vr3_part)}", ev)
    # Title block (VR3 checklist): weight from the native gates of this exact part.
    gpath = ROOT / f'design/gates-{c.id}.json'
    w = (load(gpath).get('chassis') or {}).get('weightLb') if gpath.exists() else None
    if not w: sys.exit(f'No structural weight in {rel(gpath)}; run audit before package.')
    title_block = {'drawnBy': ARGS.drawn_by, 'date': datetime.now(timezone.utc).strftime('%m/%d/%y'), 'company': 'RUTGERS FORMULA RACING',
                   'title': f'RFR27 CHASSIS FRAME\nCANDIDATE {c.id.upper()}', 'material': 'AISI 4130N', 'weight': f'{w:.2f} lb',
                   'revision': ARGS.revision}
    ok, ev, out, err = run_job(f'vr3-drawing-{c.id}', {'command': 'vr3-drawing', 'path': str(vr3_part), 'restore': str(c.layout),
                                                     'timeoutMs': 900000, 'titleBlock': title_block})
    if not ok: sys.exit(f'VR3 drawing failed: {err}; evidence {rel(ev)}')
    subprocess.run([sys.executable, str(ROOT / 'scripts/vr3_bom.py'), str(out_dir / f'{c.part.stem}-cutlist.csv')], cwd=ROOT, check=True)
    record(c, 'vr3-package', f"VR3 package: drawing ({out['balloons']} balloons, {out['sheets']} sheets) + PDF + BOM in {rel(out_dir)}", ev)
    commit(f'candidate-{c.id}: VR3 quote package')


def main():
    global ARGS
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('action', choices=['status', 'snapshot', 'trim', 'build', 'build-vr3', 'layout', 'audit', 'simulate', 'package'])
    ap.add_argument('--configuration', default='Default<As Machined>')
    ap.add_argument('--candidate', required=True)
    ap.add_argument('--recipe'); ap.add_argument('--source')
    ap.add_argument('--batch', type=int, default=40)
    ap.add_argument('--max-batches', type=int)
    ap.add_argument('--label', default='')
    ap.add_argument('--no-commit', action='store_true')
    # SolidWorks keeps one open document per file name. A second checkout (worktree) running while the
    # main checkout's Percy assembly or starter part is open points these at renamed copies of them.
    ap.add_argument('--percy', default='cad/working/percy/RFR27-Percy-seated-001.SLDASM', help='layout/audit: Percy assembly')
    ap.add_argument('--baseline', default='cad/working/RFR27-working-starter.SLDPRT', help='audit: protected-geometry baseline part')
    ap.add_argument('--drawn-by', default='A. RENJITH', help='package: title block DRAWN BY')
    ap.add_argument('--revision', default='A', help='package: title block revision')
    ARGS = ap.parse_args()
    if not 1 <= ARGS.batch <= 40: sys.exit('--batch must be 1..40 (native bound)')
    if ARGS.action == 'build-vr3' and ARGS.batch == 40: ARGS.batch = 8  # groups per native job
    c = Candidate(ARGS.candidate, ARGS.recipe)
    if ARGS.action == 'status':
        st = {'part': rel(c.part), 'recipe': rel(c.recipe), 'totalTrims': c.total(), 'lock': LOCK.exists()}
        if c.build_journal.exists():
            j = load(c.build_journal); st.update(buildOrder='VR3 (trim as added)', nextGroup=j['nextGroup'], groupCount=j['groupCount'])
        else: st['nextTrimIndex'] = c.next_index()
        print(json.dumps(st, indent=2))
    elif ARGS.action == 'snapshot':
        slug = re.sub(r'[^a-z0-9]+', '-', ARGS.label.lower()).strip('-')
        n = c.next_index(); label = f'{n:04d}-trims' + (f'-{slug}' if slug else '')
        snapshot(c, label); record(c, label, ARGS.label or f'Manual snapshot at {n}/{c.total()} trims.')
        commit(f'candidate-{c.id}: snapshot {label}')
    elif ARGS.action == 'trim': trim(c)
    elif ARGS.action == 'build-vr3': build_vr3(c)
    elif ARGS.action in ('layout', 'audit', 'simulate', 'package'): globals()[ARGS.action](c)
    else: build(c)


if __name__ == '__main__':
    main()
