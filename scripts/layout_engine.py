"""Exact, fast layout/size search engine for the chassis ground structure.

Built on scripts/ground_structure.Problem (candidates, conflicts, symmetry, rule sizes).
Each beam's stiffness is a sum of six rank-one modes,
    k_e = sum_t s_et c_et c_et^T,   s = [EA/L, GJ/L, 3EI/L, EI/L, 3EI/L, EI/L],
so the probe rotation theta = c^T u (K u = f) has
  * adjoint sensitivities for every candidate from two solves:
        d theta / d s_et = -(c_et^T u)(c_et^T lambda),  K lambda = c
  * exact discrete moves (add / remove / resize a mirror pair) by Woodbury:
        theta' = theta - (U^T lambda)^T (I + D S)^-1 D (U^T u),  S = U^T K^-1 U
    using a cached K^-1 on the free DOFs, updated by the same low-rank formula
    when a move is accepted (periodic refactorisation keeps it exact).
Absent tubes keep a 1e-6 stiffness floor so K stays non-singular.

Search: exact best-improvement local search + tabu, MILP multi-moves (HiGHS,
local branching over the exact single-move gains with the conflict graph), random
kicks, several restarts. Objective = the tracker ratio TR / weight.
"""
from pathlib import Path
import argparse, contextlib, io, json, math, random, sys, time
import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla
from scipy.optimize import milp, LinearConstraint, Bounds

sys.path.insert(0, str(Path(__file__).parent))
import beam_model as bm
import ground_structure as gs

FLOOR = 1e-6
OVER_PEN = 2.0
SCREEN_K = None  # adjoint screening (number of exact checks per step); None = exact evaluation of every move


def local_modes(L):
    """6 x 12 rank-one modes in local DOFs [u v w tx ty tz]x2 (verified against beam_model)."""
    C = np.zeros((6, 12))
    C[0, 6], C[0, 0] = 1, -1                                    # axial              EA/L
    C[1, 9], C[1, 3] = 1, -1                                    # torsion            GJ/L
    C[2, 5], C[2, 11], C[2, 7], C[2, 1] = 1, 1, -2 / L, 2 / L   # xy bend symmetric  3EI/L
    C[3, 5], C[3, 11] = 1, -1                                   # xy bend antisym    EI/L
    C[4, 4], C[4, 10], C[4, 8], C[4, 2] = -1, -1, -2 / L, 2 / L # xz bend symmetric  3EI/L
    C[5, 4], C[5, 10] = 1, -1                                   # xz bend antisym    EI/L
    return C


def mode_coefs(sec, L):
    """Mode stiffnesses; Timoshenko shear softens only the symmetric bending modes:
    3EI/(L(1+phi)), phi = 12EI/(k G A L^2) (exact rank-one form of the shear-deformable beam)."""
    A, I, J = sec['A'], sec['I'], sec['J']; f = 1 + bm.shear_phi(L, sec)
    return np.array([bm.E * A / L, bm.G * J / L, 3 * bm.E * I / (L * f), bm.E * I / L, 3 * bm.E * I / (L * f), bm.E * I / L])


class Engine:
    def __init__(self, pb):
        self.pb = pb
        rec = dict(pb.base); rec['members'] = []
        owner = {}
        for gi, g in enumerate(pb.groups):
            for name, a, b in g['members']:
                rec['members'].append({'name': name, 'a': pb.nodes[a].tolist(), 'b': pb.nodes[b].tolist(), 'profile': '1.000 x 0.065.sldlfp'})
                owner[name] = gi
        f = bm.build(rec)
        self.xyz = f.xyz; n = len(f.xyz)
        hoop = bm.section(bm.HOOP_PROFILE)
        el = []
        for i, j, s, name in f.elems:
            a, b = f.xyz[i], f.xyz[j]; L = float(np.linalg.norm(b - a))
            C = local_modes(L) @ bm._rotation(a, b)
            if name in owner: kind, g1, g2 = 'member', owner[name], -1
            elif name.startswith('link|'):
                _, m, h = name.split('|'); g1, g2 = owner.get(m, -1), owner.get(h, -1)
                if g1 < 0: g1, g2 = g2, -1
                kind = 'link' if g1 >= 0 else 'fixed'
            else: kind, g1, g2 = 'fixed', -1, -1
            el.append((i, j, C, L, kind, g1, g2))
        self.n_el = len(el)
        self.ei = np.array([e[0] for e in el]); self.ej = np.array([e[1] for e in el])
        self.C = np.array([e[2] for e in el])                      # (n_el, 6, 12)
        self.L = np.array([e[3] for e in el])
        self.kind = np.array([e[4] for e in el])
        self.g1 = np.array([e[5] for e in el]); self.g2 = np.array([e[6] for e in el])
        self.hoop_coef = np.array([mode_coefs(hoop, L) for L in self.L])
        names = [e[3] for e in f.elems]
        self.hoop_mass = float(sum(L * hoop['kgPerM'] for L, nm in zip(self.L, names) if nm in ('MainHoop_1', 'FrontHoop_1')))
        # element DOFs and free-DOF map
        self.dofs = np.stack([np.r_[6 * i:6 * i + 6, 6 * j:6 * j + 6] for i, j in zip(self.ei, self.ej)])
        used = set(self.ei) | set(self.ej)
        fixed = {6 * f.node(p) + c for p in pb.base['fixturePoints'] for c in range(3)}
        fixed |= {6 * k + c for k in range(n) if k not in used for c in range(6)}
        self.free = np.array(sorted(set(range(6 * n)) - fixed)); self.ndof = 6 * n
        self.fmap = -np.ones(6 * n, int); self.fmap[self.free] = np.arange(len(self.free))
        self.F = np.zeros(6 * n)
        for p in pb.base['loadPoints']: self.F[6 * f.node(p) + 2] += math.copysign(100 * bm.LBF, p[1])
        self.c = np.zeros(6 * n)
        for p in pb.base['probes']: self.c[6 * f.node(p) + 3] += 1 / len(pb.base['probes'])
        self.fixed_mask = self.kind == 'fixed'
        # per-group element lists; section coefficient tables per element and option
        self.sections = pb.sections
        self.min_sec = min(pb.sections.values(), key=lambda s: s['A'])
        self.el_of = {}
        for e in range(self.n_el):
            for g in (self.g1[e], self.g2[e]):
                if g >= 0: self.el_of.setdefault(int(g), []).append(e)
        self.floor_coef = np.array([mode_coefs(self.min_sec, L) * FLOOR for L in self.L])
        self.unit = {p: np.array([mode_coefs(s, L) for L in self.L]) for p, s in pb.sections.items()}
        self.kg = {p: s['kgPerM'] for p, s in pb.sections.items()}
        # Trimmed tube length per group: each end sits ~one carrier radius inside its carrier.
        self.member_len = {gi: sum(max(np.linalg.norm(pb.nodes[b] - pb.nodes[a]) - 2 * gs.CARRIER_R,
                                       0.5 * np.linalg.norm(pb.nodes[b] - pb.nodes[a])) for _, a, b in g['members'])
                           for gi, g in enumerate(pb.groups)}
        self.max_weight = None

    # ---------------------------------------------------------------- state
    def coefs(self, state, elements=None):
        idx = np.arange(self.n_el) if elements is None else np.asarray(elements)
        out = np.empty((len(idx), 6))
        for k, e in enumerate(idx):
            kd = self.kind[e]
            if kd == 'fixed': out[k] = self.hoop_coef[e]
            elif kd == 'member':
                p = state[self.g1[e]]; out[k] = self.unit[p][e] if p else self.floor_coef[e]
            else:  # offset-joint link: present iff both owners present
                on = state[self.g1[e]] and (self.g2[e] < 0 or state[self.g2[e]])
                out[k] = self.hoop_coef[e] if on else self.floor_coef[e]
        return out

    def weight_lb(self, state):
        kg = self.hoop_mass + sum(self.member_len.get(g, 0) * self.kg[p] for g, p in enumerate(state) if p)
        return kg / 0.45359237

    def factor(self, state):
        """Assemble, factor, solve primal + adjoint, and cache K^-1 on free DOFs."""
        coef = self.coefs(state)
        ke = np.einsum('eti,et,etj->eij', self.C, coef, self.C)
        rows = np.repeat(self.dofs, 12, axis=1).ravel(); cols = np.tile(self.dofs, (1, 12)).ravel()
        K = sp.coo_matrix((ke.ravel(), (rows, cols)), shape=(self.ndof, self.ndof)).tocsc()
        Kff = K[self.free][:, self.free]
        lu = spla.splu(Kff.tocsc())
        self.u = np.zeros(self.ndof); self.lam = np.zeros(self.ndof)
        self.u[self.free] = lu.solve(self.F[self.free]); self.lam[self.free] = lu.solve(self.c[self.free])
        self.Z = lu.solve(np.eye(len(self.free)))
        self.Z = 0.5 * (self.Z + self.Z.T)
        self.coef = coef; self.state = list(state); self.accepts = 0
        return self.theta()

    def snapshot(self):
        """Copy of the factorised state (cheaper than refactorising when a trial is undone)."""
        return (list(self.state), self.Z.copy(), self.u.copy(), self.lam.copy(), self.coef.copy(), self.accepts)

    def restore(self, snap):
        st, Z, u, lam, coef, acc = snap
        self.state, self.Z, self.u, self.lam, self.coef, self.accepts = list(st), Z.copy(), u.copy(), lam.copy(), coef.copy(), acc

    def theta(self): return float(self.c @ self.u)

    def ratio(self, theta, weight):
        return bm.TRACKER_TORQUE_LBFT / abs(math.degrees(theta)) / weight

    def merit(self, theta, weight):
        """Search objective: the ratio, less a steep penalty for weight over the cap (no state over the
        cap at all while hard_cap is set, see trim_to_cap)."""
        over = max(0.0, weight - self.max_weight) if self.max_weight else 0.0
        if over > 0 and getattr(self, 'hard_cap', False): return -1e9
        return self.ratio(theta, weight) - (getattr(self, 'over_pen', None) or OVER_PEN) * over

    def score(self):
        th = self.theta(); w = self.weight_lb(self.state)
        return {'thetaDeg': math.degrees(th), 'tr': bm.TRACKER_TORQUE_LBFT / abs(math.degrees(th)), 'weightLb': w,
                'score': self.ratio(th, w), 'merit': self.merit(th, w),
                'underCap': bool(not self.max_weight or w <= self.max_weight)}

    # ---------------------------------------------------------------- moves
    def _move_data(self, changes):
        """changes: {group: new profile or None}. Returns (U_local, D, dof list) for the change."""
        trial = list(self.state)
        for g, p in changes.items(): trial[g] = p
        els = sorted({e for g in changes for e in self.el_of.get(g, [])})
        new = self.coefs(trial, els); old = self.coef[els]
        dco = new - old
        cols, dvals, dofsets = [], [], []
        for k, e in enumerate(els):
            for t in range(6):
                if abs(dco[k, t]) > 0: cols.append((e, t)); dvals.append(dco[k, t])
        if not cols: return None, trial, els, new
        dset = sorted({int(x) for e, _ in cols for x in self.dofs[e] if self.fmap[x] >= 0})
        pos = {d: i for i, d in enumerate(dset)}
        U = np.zeros((len(dset), len(cols)))
        for c_i, (e, t) in enumerate(cols):
            for a, d in enumerate(self.dofs[e]):
                if self.fmap[d] >= 0: U[pos[d], c_i] = self.C[e, t, a]
        fd = self.fmap[dset]
        return (U, np.array(dvals), np.array(dset), fd), trial, els, new

    def evaluate_move(self, changes):
        """Exact theta and weight after a move (Woodbury), without committing."""
        data, trial, _, _ = self._move_data(changes)
        w = self.weight_lb(trial)
        if data is None: return self.theta(), w
        U, D, dset, fd = data
        S = U.T @ self.Z[np.ix_(fd, fd)] @ U
        Uu = U.T @ self.u[dset]; Ul = U.T @ self.lam[dset]
        M = np.eye(len(D)) + D[:, None] * S
        try: x = np.linalg.solve(M, D * Uu)
        except np.linalg.LinAlgError: return None, w
        if np.linalg.cond(M) > 1e12: return None, w  # removal disconnects part of the frame
        return self.theta() - Ul @ x, w

    def evaluate_compound(self, changes):
        """Exact theta and weight after a compound move by the symmetric Woodbury form,
        theta' = theta - (U'lam)' (D^-1 + U'ZU)^-1 (U'u), solved after diagonal equilibration. For the
        12-13 group skeleton moves cond(I + D U'ZU) reaches 1e15 only through the stiffness scaling
        (evaluate_move's guard rejects them although they are accurate to 1e-9); the equilibrated
        matrix stays near 1e7 and agrees with refactorisation to 1e-12. A disconnection makes it
        singular, so a condition number above 1e13 still rejects the move."""
        data, trial, _, _ = self._move_data(changes)
        w = self.weight_lb(trial)
        if data is None: return self.theta(), w
        U, D, dset, fd = data
        C = np.diag(1.0 / D) + U.T @ self.Z[np.ix_(fd, fd)] @ U
        sc = 1.0 / np.sqrt(np.abs(np.diag(C))); Cs = C * sc[:, None] * sc[None, :]
        try:
            if np.linalg.cond(Cs) > 1e13: return None, w
            x = sc * np.linalg.solve(Cs, sc * (U.T @ self.u[dset]))
        except np.linalg.LinAlgError:
            return None, w
        return self.theta() - (U.T @ self.lam[dset]) @ x, w

    def evaluate_exact(self, changes):
        """theta and weight after a move by refactorising (for compound moves too large for the
        Woodbury update to be accurate); the current factorisation is restored."""
        snap = self.snapshot()
        try:
            trial = list(self.state)
            for g, p in changes.items(): trial[g] = p
            return self.factor(trial), self.weight_lb(trial)
        except Exception:
            return None, None
        finally:
            self.restore(snap)

    def apply_move(self, changes, refactor_every=30):
        data, trial, els, new = self._move_data(changes)
        if data is not None:
            U, D, dset, fd = data
            ZU = self.Z[:, fd] @ U
            S = U.T @ ZU[fd, :]
            M = np.linalg.solve(np.eye(len(D)) + D[:, None] * S, np.diag(D))   # (I + D S)^-1 D
            self.Z -= ZU @ M @ ZU.T
            uf = self.u[self.free]; lf = self.lam[self.free]
            uf -= ZU @ (M @ (U.T @ self.u[dset])); lf -= ZU @ (M @ (U.T @ self.lam[dset]))
            self.u[self.free] = uf; self.lam[self.free] = lf
            self.coef[els] = new
        self.state = trial; self.accepts += 1
        if self.accepts % refactor_every == 0: self.factor(self.state)

    # ---------------------------------------------------------------- neighbourhood
    def pairs(self):
        return [g for g in range(len(self.pb.groups)) if self.pb.mirror[g] >= g]

    def first_order(self):
        """Adjoint screen: d theta / d s_et = -(c_et . u)(c_et . lambda) for every element mode,
        then the linear theta change of setting each group to each profile (or removing it)."""
        # Per mode, the exact single rank-one update saturates: dtheta = -ds a b / (1 + ds q),
        # a = c.u, b = c.lambda, q = c' K^-1 c (from the cached inverse on free DOFs).
        a = np.einsum('eti,ei->et', self.C, self.u[self.dofs]); b = np.einsum('eti,ei->et', self.C, self.lam[self.dofs])
        fd = self.fmap[self.dofs]; Cf = np.where(fd[:, None, :] >= 0, self.C, 0.0); fdc = np.maximum(fd, 0)
        Ze = self.Z[fdc[:, :, None], fdc[:, None, :]]                        # (n_el, 12, 12)
        q = np.einsum('eti,eij,etj->et', Cf, Ze, Cf)
        if not hasattr(self, '_prof'):
            self._prof = list(self.pb.catalog); self._U = np.stack([self.unit[p] for p in self._prof])   # (P, n_el, 6)
        def dtheta(ds, ab, qq): return (-ds * ab / (1 + ds * qq)).sum(axis=(-2, -1))
        est = {}
        for g, els in self.el_of.items():
            els = [e for e in els if self.kind[e] == 'member']
            if not els: continue
            ab, qq, cur = a[els] * b[els], q[els], self.coef[els]
            est[g] = (dtheta(self._U[:, els] - cur, ab, qq), float(dtheta(self.floor_coef[els] - cur, ab, qq)))
        return est

    def neighbourhood(self, tabu=(), screen=SCREEN_K):
        """Score of feasible single pair moves: list of (gain, changes, theta, w). With `screen`,
        every move is ranked by its first-order (adjoint) merit change and only the best
        `screen` plus every removal are evaluated exactly (Woodbury)."""
        th0 = self.theta(); w0 = self.weight_lb(self.state); base = self.merit(th0, w0)
        est = self.first_order() if screen else None
        pidx = {p: i for i, p in enumerate(self.pb.catalog)}
        moves = []
        for g in self.pairs():
            if g in tabu: continue
            m = self.pb.mirror[g]
            for p in [None] + self.pb.allowed[g]:
                if p == self.state[g] or not self.pb.feasible(self.state, g, p): continue
                ch = {g: p, m: p}
                if est is None: moves.append((0.0, ch)); continue
                dth = sum((est[x][1] if p is None else est[x][0][pidx[p]]) if x in est else 0.0 for x in {g, m})
                trial = list(self.state); trial[g] = trial[m] = p
                thl = th0 + dth; w = self.weight_lb(trial)
                gain = self.merit(thl, w) - base if thl * th0 > 0 else -np.inf
                moves.append((np.inf if p is None else gain, ch))   # removals always checked exactly
        if est is not None and len(moves) > screen:
            moves.sort(key=lambda x: -x[0])
            moves = [mv for mv in moves if mv[0] == np.inf] + [mv for mv in moves if mv[0] != np.inf][:screen]
        out = []
        for _, ch in moves:
            th, w = self.evaluate_move(ch)
            if th is None or th * th0 <= 0: continue
            out.append((self.merit(th, w) - base, ch, th, w))
        return out


# -------------------------------------------------------------------- search
def repair_pickups(eng, log):
    """A15: every suspension node must be a joint. Add the best-scoring feasible tube
    pair that ends at each pickup no present tube ends at."""
    pb = eng.pb
    for k, enders in enumerate(pb.pickup_enders):
        if any(eng.state[g] for g in enders): continue
        best = None
        for g in sorted(enders):
            for p in pb.allowed[g]:
                trial = list(eng.state); trial[g] = trial[pb.mirror[g]] = p
                if not pb.valid_change(eng.state, trial, {g, pb.mirror[g]} - {None}): continue
                th, w = eng.evaluate_move({g: p, pb.mirror[g]: p})
                if th is None: continue
                r = eng.merit(th, w)
                if best is None or r > best[0]: best = (r, g, p)
        if best is None: raise RuntimeError(f'No feasible tube can make suspension node {k} a joint')
        _, g, p = best; eng.apply_move({g: p, pb.mirror[g]: p})
        log.append({'move': {pb.groups[g]['name']: p}, 'reason': f'suspension node {k} must be a joint', **eng.score()})
        print(f"  joint at suspension node {k}: {pb.groups[g]['name']} {p}", flush=True)


def repair_kinks(eng, log):
    """F.5.2.3: give every kink a same-size support, choosing the best-scoring option."""
    pb = eng.pb
    for kk in pb.kinks:
        target = eng.state[kk['group']]
        if target is None or any(eng.state[s] == target for s in kk['supports']): continue
        best = None
        for g in sorted(kk['supports']):
            trial = list(eng.state); trial[g] = trial[pb.mirror[g]] = target
            if not pb.valid_change(eng.state, trial, {g, pb.mirror[g]}): continue
            th, w = eng.evaluate_move({g: target, pb.mirror[g]: target})
            if th is not None and (best is None or eng.merit(th, w) > best[0]): best = (eng.merit(th, w), g)
        if best is None: raise RuntimeError(f"No feasible same-size support for the kink of {pb.groups[kk['group']]['name']}")
        g = best[1]; eng.apply_move({g: target, pb.mirror[g]: target})
        log.append({'move': {pb.groups[g]['name']: target}, 'reason': 'F.5.2.3 kink support', **eng.score()})
        print(f"  kink support for {pb.groups[kk['group']]['name']}: {pb.groups[g]['name']} {target}", flush=True)


# ---------------------------------------------------------------- free skeleton (F.6.4 / F.6.2.3)
# With ground_structure.Problem(skeleton='free') Codex's side impact and bulkhead-support tubes are
# optional and the generic predicates (scripts/rule_structure.py) decide whether a state still has
# a legal structure on each side. Going from one legal structure to another usually needs several
# tubes to change at once (a new diagonal crosses the old one), which no single pair move can do,
# so the search also tries compound moves: add one panel tube pair at a rule-class profile (or drop
# one tube of the current structure), drop whatever the addition conflicts with (Problem.close), then
# rejoin orphaned suspension pickups and add tubes aimed at what the predicates report missing
# (untriangulated bays, unsupported kinks) until the state is valid (_complete).
def _apply(state, ch):
    t = list(state)
    for g, p in ch.items(): t[g] = p
    return t


def _rule_profiles(pb, state, g):
    """Profiles worth trying for a panel tube: the lightest allowed one meeting Size B and the
    lightest meeting Size C (the predicates only see the size class and OD), plus the profiles of
    present kinked panel members (an F.5.2.3 support must match its member exactly)."""
    ok = pb.allowed[g]; kg = lambda q: pb.sections[q]['kgPerM']
    want = {min((q for q in ok if gs.rs.meets(q, c)), key=kg, default=None) for c in ('B', 'C')}
    want |= {state[k['group']] for k in pb.kinks if state[k['group']]} & set(ok)
    return sorted(q for q in want if q)


def _panel_pairs(pb):
    if not hasattr(pb, '_panel_pairs'):
        pb._panel_pairs = sorted({min(h, pb.mirror[h]) for s in (1, -1) for h in pb.panel_groups[s]
                                  if pb.mirror[h] >= 0 and not pb.groups[h]['required']})
    return pb._panel_pairs


def _rejoin_pickups(pb, state, ch, removed):
    """Give every suspension pickup the move left without a joint its lightest ending tube (one
    that displaces no structural tube: the front lower rail, for one, would displace the bulkhead's
    lower support at its 5.9 deg joint)."""
    kg = lambda q: pb.sections[q]['kgPerM']
    for _ in range(8):
        trial = _apply(state, ch)
        k = next((k for k, en in enumerate(pb.pickup_enders) if not any(trial[x] for x in en)), None)
        if k is None: return ch
        best = None
        for x in sorted(pb.pickup_enders[k]):
            if x in removed or pb.mirror[x] in removed or pb.mirror[x] < 0 or not pb.allowed[x]: continue
            q = min(pb.allowed[x], key=kg)
            c2 = pb.close(trial, {x: q, pb.mirror[x]: q})
            if c2 is None or any(ch.get(g) is not None and v is None for g, v in c2.items()): continue
            if any(v is None and trial[g] and gs.rs.level(trial[g]) for g, v in c2.items()): continue
            t2 = _apply(trial, c2)
            if any(not t2[h] for y in (x, pb.mirror[x]) for h in pb.depends.get(y, ())): continue   # lands on an absent tube
            if any(t2[c] for y in (x, pb.mirror[x]) for c in pb.conflicts[y]): continue           # e.g. crosses its own mirror
            cost = sum(np.linalg.norm(pb.nodes[b] - pb.nodes[a]) for _, a, b in pb.groups[x]['members']) * kg(q)
            if best is None or cost < best[0]: best = (cost, {**ch, **c2})
        if best is None: return None
        ch = best[1]
    return None


def _defect_additions(pb, state, removed):
    """Additions aimed at what blocks the panels (Problem.panel_defects): a panel tube with both
    ends in one untriangulated bay, or a support for an unsupported kink, each closed over its
    conflicts."""
    faces, kinks = pb.panel_defects(state)
    if not hasattr(pb, '_pair_nodes'):
        pb._pair_nodes = {x: [{n for _, a, b in pb.groups[y]['members'] for n in (a, b)} for y in {x, pb.mirror[x]}]
                          for x in _panel_pairs(pb)}
    out = []
    for x, ends in pb._pair_nodes.items():
        # present tubes below Size B may still be upgraded (a side impact bay needs Size B)
        if x in removed or pb.mirror[x] in removed or (state[x] and gs.rs.meets(state[x], 'B')): continue
        if any(e <= F for F in faces for e in ends):
            out += [(x, q) for q in _rule_profiles(pb, state, x) if q != state[x]]
    for k in kinks:
        need = state[k['tags'][0]]
        for x, u in pb._supporters(k['node']):
            if x in removed or pb.mirror[x] < 0 or pb.groups[x]['required'] or state[x]: continue
            if k['exempt']: q = min((q for q in pb.allowed[x] if gs.rs.level(q)), key=lambda q: pb.sections[q]['kgPerM'], default=None)
            else:
                q = need if need in pb.allowed[x] else None
                if q and math.degrees(math.asin(min(1.0, abs(u @ k['normal'])))) > gs.rs.PLANE_DEG: q = None
            if q: out.append((x, q))
    seen = set()
    for x, q in out:
        if (min(x, pb.mirror[x]), q) in seen: continue
        seen.add((min(x, pb.mirror[x]), q))
        c = pb.close(state, {x: q, pb.mirror[x]: q})
        if c is not None: yield c


def _complete(pb, state, ch, depth=3, width=12):
    """Make a compound move valid: rejoin orphaned pickups, then up to `depth` rounds of additions
    aimed at the remaining panel defects (at most `width` partial moves carried per round). Nothing
    the move removed is put back."""
    removed = {g for g, p in ch.items() if p is None}
    frontier, out = [ch], []
    for _ in range(depth):
        nxt = []
        for c in frontier:
            c = _rejoin_pickups(pb, state, c, removed)
            if c is None: continue
            trial = _apply(state, c)
            if pb.valid_change(state, trial, set(c)): out.append(c); continue
            for c2 in _defect_additions(pb, trial, removed):
                if any(c.get(g) is not None and v is None for g, v in c2.items()): continue
                nxt.append({**c, **c2})
        if out or not nxt: break
        frontier = nxt[:width]
    return out


def skeleton_moves(eng):
    """Valid compound moves (see above) as (merit gain, changes, theta, weight), best first: an
    addition that displaces a tube of the current side/bulkhead structures, or the removal of one,
    each completed by _complete."""
    pb = eng.pb; st = eng.state
    if not getattr(pb, 'free', False): return []
    th0 = eng.theta(); base = eng.merit(th0, eng.weight_lb(st))
    wit = pb.witness(st)
    primary = [pb.close(st, {g: None, pb.mirror[g]: None}) for g in sorted({min(g, pb.mirror[g]) for g in wit})
               if pb.mirror[g] >= 0 and not pb.groups[g]['required']]
    for g in _panel_pairs(pb):
        m = pb.mirror[g]
        if (st[g] and gs.rs.level(st[g])) or not (pb.conflicts[g] | pb.conflicts[m]) & wit: continue
        primary += [pb.close(st, {g: q, m: q}) for q in _rule_profiles(pb, st, g)]
    found = {}
    for ch in primary:
        if ch is None: continue
        for c in _complete(pb, st, ch): found.setdefault(tuple(sorted(c.items(), key=lambda kv: kv[0])), c)
    out = []
    for ch in found.values():
        th, w = eng.evaluate_compound(ch)
        if th is None or th * th0 <= 0: continue
        out.append((eng.merit(th, w) - base, ch, th, w))
    return sorted(out, key=lambda x: -x[0])


SKELETON_LOOKAHEAD = 4   # distinct skeleton re-routings tried per round, each re-optimised by local search


def skeleton_descent(eng, log, rounds=4, lookahead=None):
    """Free skeleton: try the best few distinct compound skeleton moves (one per set of tubes added
    or removed, whatever the profiles), re-optimise each by local search (trimmed to the weight cap
    if it ends over it), and keep the best result if it beats the current state (better()). A
    re-routing usually starts worse (new tubes at rule size, over the weight cap) and only pays off
    after the rest of the frame adapts, so a move is judged after that descent, not on its
    immediate gain."""
    lookahead = SKELETON_LOOKAHEAD if lookahead is None else lookahead
    for _ in range(rounds):
        start, before = eng.snapshot(), trimmed_outcome(eng)
        picks, seen = [], set()
        for gain, ch, th, w in skeleton_moves(eng):
            topo = frozenset((g, p is None) for g, p in ch.items())
            if topo in seen: continue
            seen.add(topo); picks.append(ch)
            if len(picks) >= lookahead: break
        best = None
        for ch in picks:
            eng.restore(start); eng.factor(_apply(start[0], ch))      # compound move: refactorise, no low-rank update
            trial_log = [{'move': {eng.pb.groups[k]['name']: v for k, v in ch.items()}, 'reason': 'skeleton', **eng.score()}]
            local_search(eng, trial_log)
            s = trim_to_cap(eng, trial_log)          # a re-routing is judged, and kept, under the cap
            same = s['underCap'] == before['underCap']
            if better(s, before) and (not same or (s['score'] if s['underCap'] else s['merit'])
                                      > (before['score'] if before['underCap'] else before['merit']) + 1e-9)                     and (best is None or better(s, best[0])):
                best = (s, eng.snapshot(), trial_log)
        eng.restore(start if best is None else best[1])
        if best is None: break
        log.extend(best[2])
        print(f"  skeleton re-routing kept: ratio {eng.score()['score']:.3f} (was {before['score']:.3f})", flush=True)
    return eng.score()


def repair_panels(eng, log):
    """Free skeleton: if a side has no legal F.6.4/F.6.2.3 structure (a released Codex tube is
    blocked at this geometry), add the best completion of one or two panel tube pairs."""
    pb = eng.pb
    if not getattr(pb, 'free', False) or pb.panels_ok(eng.state): return
    st = eng.state; best = None
    frontier = [{}]
    for _ in range(3):
        nxt, done = [], []
        for c in frontier:
            for c2 in _defect_additions(pb, _apply(st, c), set()):
                full = {**c, **c2}
                (done if pb.panels_ok(_apply(st, full)) else nxt).append(full)
        for c in done:
            th, w = eng.evaluate_compound(c)
            if th is not None and (best is None or eng.merit(th, w) > best[0]): best = (eng.merit(th, w), c)
        if best or not nxt: break
        frontier = nxt[:12]
    if best is None: raise RuntimeError('No rule-legal side impact / bulkhead support structure at this geometry')
    eng.factor(_apply(eng.state, best[1]))
    log.append({'move': {pb.groups[k]['name']: v for k, v in best[1].items()}, 'reason': 'F.6.4/F.6.2.3 structure', **eng.score()})


def better(s, b):
    """Search preference: under the weight cap first, then the highest ratio (merit while over)."""
    return (s['underCap'], s['score'] if s['underCap'] else s['merit']) > (b['underCap'], b['score'] if b['underCap'] else b['merit'])


def trim_to_cap(eng, log):
    """Bring a state over the weight cap under it. The soft penalty (OVER_PEN per lb) lets a search
    end up to about 1 lb over the cap, and param_search rejects such results; this only ever acts on
    a state that would have been rejected. Local search is repeated with a steeper penalty (8, 32,
    128 per lb), so the search itself sheds the weight that costs the least ratio; if that is not
    enough, the feasible move that removes weight at the smallest ratio cost per lb is applied until
    the state is under the cap. A local search that may not cross the cap again then polishes it."""
    if not eng.max_weight or eng.weight_lb(eng.state) <= eng.max_weight: return eng.score()
    pb = eng.pb
    try:
        for pen in (8.0, 32.0, 128.0):
            eng.over_pen = pen
            local_search(eng, log)
            if eng.weight_lb(eng.state) <= eng.max_weight: break
    finally:
        eng.over_pen = None
    for _ in range(40):
        w0 = eng.weight_lb(eng.state)
        if w0 <= eng.max_weight: break
        th0 = eng.theta(); r0 = eng.ratio(th0, w0); best = None
        for g in eng.pairs():
            m = pb.mirror[g]
            for p in [None] + pb.allowed[g]:
                if p == eng.state[g] or not pb.feasible(eng.state, g, p): continue
                trial = list(eng.state); trial[g] = trial[m] = p
                if eng.weight_lb(trial) >= w0 - 1e-6: continue
                th, w = eng.evaluate_move({g: p, m: p})
                if th is None or th * th0 <= 0: continue
                eff = (eng.ratio(th, w) - r0) / (w0 - w)
                if best is None or eff > best[0]: best = (eff, {g: p, m: p})
        if best is None: break
        eng.apply_move(best[1])
        log.append({'move': {pb.groups[k]['name']: v for k, v in best[1].items()}, 'reason': 'weight cap', **eng.score()})
    eng.factor(eng.state)
    if eng.weight_lb(eng.state) <= eng.max_weight:
        eng.hard_cap = True
        try: local_search(eng, log)
        finally: eng.hard_cap = False
    return eng.score()


def trimmed_outcome(eng):
    """Score the current state would have after trim_to_cap; the state itself is left unchanged."""
    if eng.score()['underCap']: return eng.score()
    snap = eng.snapshot()
    with contextlib.redirect_stdout(io.StringIO()): s = trim_to_cap(eng, [])
    eng.restore(snap)
    return s


def local_search(eng, log, tabu_len=8, max_moves=400):
    tabu = []
    for _ in range(max_moves):
        nb = eng.neighbourhood(tabu=set(tabu))
        if not nb: break
        gain, ch, th, w = max(nb, key=lambda x: x[0])
        if gain <= 1e-9: break
        eng.apply_move(ch)
        g = min(ch); tabu.append(g); tabu = tabu[-tabu_len:]
        s = eng.score(); log.append({'move': {eng.pb.groups[k]['name']: v for k, v in ch.items()}, **s})
        print(f"  {eng.pb.groups[g]['name']:<40} -> {ch[g] or 'removed':<22} TR {s['tr']:7.1f} {s['weightLb']:6.1f} lb ratio {s['score']:.3f}", flush=True)
    return eng.score()


def milp_step(eng, radius=6, limit_s=20):
    """Choose a compatible set of up to `radius` pair moves maximising the summed exact
    single-move gains (conflicts respected), then evaluate it exactly."""
    nb = eng.neighbourhood()
    if not nb: return None
    pb = eng.pb; idx = {}
    for k, (gain, ch, _, _) in enumerate(nb): idx.setdefault(min(ch), []).append(k)
    nvar = len(nb); cvec = -np.array([max(x[0], -1.0) for x in nb])
    A, lo, hi = [], [], []
    for g, ks in idx.items():                           # one option per pair
        row = np.zeros(nvar); row[ks] = 1; A.append(row); lo.append(0); hi.append(1)
    row = np.ones(nvar); A.append(row); lo.append(0); hi.append(radius)   # local branching radius
    present = {g: eng.state[g] is not None for g in eng.pairs()}
    reps = set(eng.pairs())
    for g in reps:                                       # conflict edges between pairs
        for q in pb.conflicts[g]:
            q = min(q, pb.mirror[q])
            if q <= g or q not in reps: continue
            # presence_after(g) + presence_after(q) <= 1
            row = np.zeros(nvar); const = 0
            for gg in (g, q):
                if present[gg]: const += 1; [row.__setitem__(k, row[k] - 1) for k in idx.get(gg, []) if nb[k][1][gg] is None]
                else: [row.__setitem__(k, row[k] + 1) for k in idx.get(gg, []) if nb[k][1][gg] is not None]
            A.append(row); lo.append(-np.inf); hi.append(1 - const)
    res = milp(cvec, integrality=np.ones(nvar), bounds=Bounds(0, 1),
               constraints=LinearConstraint(np.array(A), lo, hi), options={'time_limit': limit_s})
    if res.x is None: return None
    chosen = [nb[k][1] for k in np.where(res.x > 0.5)[0]]
    if len(chosen) < 2: return None
    merged = {}
    for ch in chosen: merged.update(ch)
    return merged


# Two-phase search for the VR3-stocked catalog. Phase 1 uses the 1.000/0.750 in families and
# the (unstocked) 1.375 x 0.049 as a search aid: with the large thin stocked tubes available
# from the start, the search spends a tight weight budget on them early and stalls. Phase 2
# replaces every aid tube by its best stocked substitute (exact evaluation), then continues on
# stocked sizes only, so no unstocked tube can remain in the result.
SEARCH_AID = '1.375 x 0.049.SLDLFP'
PHASE1 = {'0.750 x 0.035', '0.750 x 0.049', '0.750 x 0.065', '1.000 x 0.035', '1.000 x 0.049', '1.000 x 0.058',
          '1.000 x 0.065', '1.000 x 0.083', '1.000 x 0.095', '1.375 x 0.049'}


def two_phase_search(eng, restarts=6, seed=1, budget_s=900):
    pb = eng.pb; full = pb.allowed
    stem = lambda p: p.rsplit('.', 1)[0]
    stocked = [[p for p in ok if p != SEARCH_AID] or ok for ok in full]
    pb.allowed = [[p for p in ok if stem(p) in PHASE1] or ok for ok in full]
    eng.state = [p if (p is None or p in pb.allowed[g]) else pb.allowed[g][0] for g, p in enumerate(eng.state)]
    try:
        start, state, best, log = search(eng, restarts=restarts, seed=seed, budget_s=budget_s * 0.65)
    finally:
        pb.allowed = stocked
    eng.factor(state)
    for g in eng.pairs():                       # best stocked substitute for each aid tube pair
        if eng.state[g] != SEARCH_AID: continue
        m = pb.mirror[g]; base = eng.merit(eng.theta(), eng.weight_lb(eng.state)); opts = []
        for p in pb.allowed[g]:
            th, w = eng.evaluate_move({g: p, m: p})
            if th is not None: opts.append((eng.merit(th, w) - base, p))
        if not opts: raise RuntimeError('No stocked substitute for ' + pb.groups[g]['name'])
        eng.apply_move({g: max(opts)[1], m: max(opts)[1]})
    eng.factor(eng.state)
    s2, state2, best2, log2 = search(eng, restarts=max(3, restarts // 2), seed=seed + 1000, budget_s=budget_s * 0.35)
    if any(p == SEARCH_AID for p in state2): raise RuntimeError('Search aid tube left in the result')
    return start, state2, best2, log + log2


def search(eng, restarts=6, seed=1, kick=4, budget_s=900):
    rng = random.Random(seed); t0 = time.time()
    eng.factor(eng.state); log = []
    start = eng.score(); print(f"Start: TR {start['tr']:.1f}, {start['weightLb']:.1f} lb, ratio {start['score']:.3f}")
    free = getattr(eng.pb, 'free', False)
    if free: repair_panels(eng, log)
    repair_pickups(eng, log); repair_kinks(eng, log)
    if not eng.pb.valid_state(eng.state): raise RuntimeError('Start state violates a constraint after repair')
    # Best = highest ratio under the weight cap (highest merit until one is found): see better().
    # Candidates are compared by the score they would have once trimmed to the weight cap
    # (trimmed_outcome); the search itself still explores with the soft penalty.
    local_search(eng, log)
    if free: skeleton_descent(eng, log)
    best_state, best = list(eng.state), trimmed_outcome(eng)
    for r in range(restarts):
        if time.time() - t0 > budget_s: break
        multi = milp_step(eng)
        if multi:
            trial = list(eng.state)
            for g, p in multi.items(): trial[g] = p
            if not eng.pb.valid_change(eng.state, trial, set(multi)): multi = None
        if multi:
            old = list(eng.state); before = eng.score()['merit']
            eng.factor(eng.state); eng.apply_move(multi); eng.factor(eng.state)
            if eng.score()['merit'] > before + 1e-9:
                print(f'-- MILP multi-move ({len(multi)} groups) accepted: ratio {eng.score()["score"]:.3f}')
                local_search(eng, log)
            else: eng.factor(old)
        s = trimmed_outcome(eng)
        if better(s, best): best, best_state = s, list(eng.state)
        # kick: random feasible pair moves from the best state, then descend again
        eng.factor(best_state)
        if free and rng.random() < 0.5:   # free skeleton: half the kicks re-route the skeleton instead
            moves = skeleton_moves(eng)
            if moves: eng.factor(_apply(eng.state, rng.choice(moves[:8])[1]))
        for _ in range(kick):
            g = rng.choice(eng.pairs()); opts = [None] + eng.pb.allowed[g]
            p = rng.choice(opts)
            if p != eng.state[g] and eng.pb.feasible(eng.state, g, p) and eng.evaluate_move({g: p, eng.pb.mirror[g]: p})[0] is not None:
                eng.apply_move({g: p, eng.pb.mirror[g]: p})
        eng.factor(eng.state)
        print(f'-- kick {r + 1}: ratio {eng.score()["score"]:.3f}')
        local_search(eng, log)
        if free: skeleton_descent(eng, log)
        s = trimmed_outcome(eng)
        if better(s, best):
            best, best_state = s, list(eng.state); print(f'   new best ratio {best["score"]:.3f}')
    eng.factor(best_state)
    if not eng.score()['underCap']:
        trim_to_cap(eng, log)   # would otherwise be rejected over the cap
    return start, list(eng.state), eng.score(), log


def verify_design(rec):
    """Independent checks of a recipe: beam_model score, work-conjugate twist (load-weighted
    rotation, hard to game by stiffening the probe bars) and Codex's rule audit."""
    import importlib.util
    r = bm.evaluate(rec)
    f = bm.build(rec); sol = bm.solve(f, rec); u = sol['u']; work = 0.0; T = 0.0
    for p in rec['loadPoints']:
        fz = math.copysign(100 * bm.LBF, p[1]); k = f.node(p); work += fz * u[6 * k + 2]; T += p[1] * fz
    tr_work = abs(T / bm.LBFT) / abs(math.degrees(work / T))
    spec = importlib.util.spec_from_file_location('audit', gs.ROOT / 'scripts/audit-frame-rules.py')
    audit = importlib.util.module_from_spec(spec); spec.loader.exec_module(audit)
    try:
        res = audit.audit(rec); ra = {'pass': res['allImplementedChecksPass'], 'failed': [c['name'] for c in res['checks'] if not c['accepted']]}
    except Exception as e:
        ra = {'pass': False, 'failed': [f'audit error: {e}']}
    hs = None
    if rec.get('halfshaftCorridor'):   # clearance of every tube surface to the halfshaft corridors
        c = rec['halfshaftCorridor']; fb = gs.Forbidden(); fb.hulls = []
        fb.add_halfshaft_corridors(c['xM'], c['zM'], c['radiusM'], c['yInM'], c['yOutM'])
        hs = min(fb.clearance(np.array(m['a']), np.array(m['b']))[0] - bm.section(m['profile'])['od'] / 2 for m in rec['members'])
    battery = gs.pkg.battery_checks(rec, rec['accumulatorBox']) if rec.get('accumulatorBox') else None
    import photo_keepouts
    photo, photo_min = photo_keepouts.check_recipe(rec)
    return {**{k: r[k] for k in ('rxDeg', 'trLbFtPerDeg', 'weightLb', 'trPerLb')}, 'trWorkConjugate': tr_work, 'ruleAudit': ra,
            'halfshaftClearanceMm': None if hs is None else hs * 1000, 'battery': battery,
            'photoKeepoutHits': photo, 'photoKeepoutMin': photo_min}


def add_packaging_args(ap):
    """Accumulator box and rear axle inputs (scripts/packaging.py). All unset = 009c inputs."""
    g = ap.add_argument_group('packaging (unset: starter battery solid, axle midway between the rear pickups)')
    g.add_argument('--acc-width-in', type=float, help='accumulator width across the car, in')
    g.add_argument('--acc-length-in', type=float, help='accumulator length along the car, in')
    g.add_argument('--acc-front-x', type=float, help='accumulator front face x, m (default: automatic placement)')
    g.add_argument('--axle-frac', type=float, help='rear axle x as a fraction from the aft (0) to the fore (1) rear pickups')
    g.add_argument('--axle-z', type=float, help='rear axle height, m (default: mean of the four rear pickups)')


def packaging_from_args(a, main_hoop='source'):
    box = None
    if a.acc_width_in is not None or a.acc_length_in is not None or a.acc_front_x is not None:
        box = gs.pkg.accumulator_box(a.acc_width_in, a.acc_length_in, a.acc_front_x, main_hoop=main_hoop)
    axle = {'frac': a.axle_frac, 'zM': a.axle_z} if a.axle_frac is not None or a.axle_z is not None else None
    return {'accumulator': box, 'axle': axle} if box or axle else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--base', default='design/frame-005.json')
    ap.add_argument('--out')
    ap.add_argument('--catalog', choices=list(gs.CATALOG), default='round')
    ap.add_argument('--max-weight', type=float, help='trimmed-weight cap in lb')
    ap.add_argument('--restarts', type=int, default=6)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--budget', type=float, default=900)
    ap.add_argument('--brace-pickups', action='store_true', help='require 3D bracing at suspension pickups (not a rule)')
    ap.add_argument('--no-halfshafts', action='store_true', help='omit the rear halfshaft corridors (comparison only)')
    add_packaging_args(ap)
    a = ap.parse_args()
    base = json.loads((gs.ROOT / a.base).read_text(encoding='utf-8-sig'))
    cat = gs.CATALOG['vr3-search' if a.catalog == 'vr3' else a.catalog]
    t = time.time(); pb = gs.Problem(base, cat, halfshafts=not a.no_halfshafts, packaging=packaging_from_args(a, bm.main_hoop_path(base)))
    pb.brace = a.brace_pickups
    eng = Engine(pb); eng.state = pb.base_state(); eng.max_weight = a.max_weight
    if pb.dropped_from_base: print('Dropped from the base (constraint violations):', ', '.join(pb.dropped_from_base))
    if a.brace_pickups: eng.state = gs.brace_pickups(pb, eng.state, [])
    print(f'{len(pb.groups)} groups, {len(eng.pairs())} symmetric pair variables, {len(eng.free)} free DOF; setup {time.time() - t:.0f}s')
    run = two_phase_search if a.catalog == 'vr3' else search
    start, state, best, log = run(eng, a.restarts, a.seed, budget_s=a.budget)
    print(f"Best: TR {best['tr']:.1f} lb-ft/deg, {best['weightLb']:.1f} lb, ratio {best['score']:.3f} (start {start['score']:.3f}); {time.time() - t:.0f}s")
    rec = gs.to_recipe(pb, state)
    verify = verify_design(rec)
    print(f"Independent check (beam_model on the recipe): ratio {verify['trPerLb']:.3f}, TR {verify['trLbFtPerDeg']:.0f}, "
          f"{verify['weightLb']:.1f} lb; work-conjugate TR {verify['trWorkConjugate']:.0f}; constraints valid {pb.valid_state(state)}; "
          f"rule audit {'PASS' if verify['ruleAudit']['pass'] else 'FAIL ' + str(verify['ruleAudit']['failed'])}")
    if a.out:
        rec['layoutEngine'] = {'base': a.base, 'catalog': a.catalog, 'seed': a.seed, 'start': start, 'best': best, 'moves': log,
                               'bracePickups': a.brace_pickups, 'maxWeightLb': a.max_weight, 'verification': verify,
                               'droppedFromBase': pb.dropped_from_base}
        (gs.ROOT / a.out).write_text(json.dumps(rec, indent=2) + '\n', encoding='utf-8')
        print(f"Wrote {a.out}: {len(rec['groups'])} groups, {len(rec['members'])} members, {len(rec['trims'])} trims")


if __name__ == '__main__':
    main()
