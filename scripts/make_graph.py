"""Generator sintetickog 2D pose-grafa u .g2o formatu.

Izlaz:  <ime>.g2o      graf sa pocetnom aproksimacijom iz odometrije
        <ime>_gt.g2o   tacne poze, za racunanje greske
"""

import argparse
import math

import numpy as np


# ---------- SE(2) pomocne funkcije ----------

def v2t(v):
    c, s = math.cos(v[2]), math.sin(v[2])
    return np.array([[c, -s, v[0]],
                     [s,  c, v[1]],
                     [0.0, 0.0, 1.0]])


def t2v(T):
    return np.array([T[0, 2], T[1, 2], math.atan2(T[1, 0], T[0, 0])])


def between(a, b):
    """Relativna poza b izrazena u sistemu a."""
    return t2v(np.linalg.inv(v2t(a)) @ v2t(b))


def compose(a, d):
    """Nadovezi relativnu pozu d na pozu a."""
    return t2v(v2t(a) @ v2t(d))


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


# ---------- tacna putanja ----------

def trajectory(radius, step, laps):
    total = 2.0 * math.pi * radius * laps
    n = int(round(total / step))
    poses = []
    for k in range(n + 1):
        phi = -math.pi / 2.0 + (k * step) / radius
        poses.append(np.array([
            radius * math.cos(phi),
            radius + radius * math.sin(phi),
            wrap(phi + math.pi / 2.0),
        ]))
    return poses


# ---------- grane ----------

def odometry_edges(gt, rng, s_xy, s_th):
    edges = []
    for i in range(len(gt) - 1):
        m = between(gt[i], gt[i + 1])
        m = m + np.array([rng.normal(0, s_xy),
                          rng.normal(0, s_xy),
                          rng.normal(0, s_th)])
        m[2] = wrap(m[2])
        edges.append((i, i + 1, m, s_xy, s_th, False))
    return edges


def loop_edges(gt, rng, s_xy, s_th, max_dist, min_gap, prob, outlier_frac):
    edges = []
    n = len(gt)
    for i in range(n):
        for j in range(i + min_gap, n):
            if np.hypot(gt[j][0] - gt[i][0], gt[j][1] - gt[i][1]) > max_dist:
                continue
            if rng.random() > prob:
                continue

            m = between(gt[i], gt[j])
            if rng.random() < outlier_frac:
                # pogresno zatvaranje petlje: velika greska, normalna tezina
                m = m + np.array([rng.uniform(-2.0, 2.0),
                                  rng.uniform(-2.0, 2.0),
                                  rng.uniform(-0.6, 0.6)])
                outlier = True
            else:
                m = m + np.array([rng.normal(0, s_xy),
                                  rng.normal(0, s_xy),
                                  rng.normal(0, s_th)])
                outlier = False
            m[2] = wrap(m[2])
            edges.append((i, j, m, s_xy, s_th, outlier))
    return edges


# ---------- upis ----------

def info_line(s_xy, s_th):
    ixy = 1.0 / (s_xy ** 2)
    ith = 1.0 / (s_th ** 2)
    return f'{ixy:.6f} 0.000000 0.000000 {ixy:.6f} 0.000000 {ith:.6f}'


def write_g2o(path, poses, edges):
    with open(path, 'w') as f:
        for i, p in enumerate(poses):
            f.write(f'VERTEX_SE2 {i} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n')
        for i, j, m, s_xy, s_th, _ in edges:
            f.write(f'EDGE_SE2 {i} {j} {m[0]:.6f} {m[1]:.6f} {m[2]:.6f} '
                    f'{info_line(s_xy, s_th)}\n')


# ---------- glavni tok ----------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='graphs/sim')
    ap.add_argument('--radius', type=float, default=4.5)
    ap.add_argument('--step', type=float, default=0.5)
    ap.add_argument('--laps', type=float, default=3.0)
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--sigma-odom-xy', type=float, default=0.02)
    ap.add_argument('--sigma-odom-th', type=float, default=0.01)
    ap.add_argument('--sigma-lc-xy', type=float, default=0.01)
    ap.add_argument('--sigma-lc-th', type=float, default=0.005)
    ap.add_argument('--lc-dist', type=float, default=0.8)
    ap.add_argument('--lc-min-gap', type=int, default=20)
    ap.add_argument('--lc-prob', type=float, default=0.4)
    ap.add_argument('--outlier-frac', type=float, default=0.0)
    ap.add_argument('--plot', action='store_true')
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)

    gt = trajectory(args.radius, args.step, args.laps)
    odo = odometry_edges(gt, rng, args.sigma_odom_xy, args.sigma_odom_th)
    loops = loop_edges(gt, rng, args.sigma_lc_xy, args.sigma_lc_th,
                       args.lc_dist, args.lc_min_gap, args.lc_prob,
                       args.outlier_frac)

    # pocetna aproksimacija: cista odometrija, nadovezana od poze 0
    init = [gt[0].copy()]
    for _, _, m, _, _, _ in odo:
        init.append(compose(init[-1], m))

    write_g2o(f'{args.out}.g2o', init, odo + loops)
    write_g2o(f'{args.out}_gt.g2o', gt, [])

    drift = np.hypot(init[-1][0] - gt[-1][0], init[-1][1] - gt[-1][1])
    n_out = sum(1 for e in loops if e[5])

    print(f'cvorova            : {len(gt)}')
    print(f'odometrijskih grana: {len(odo)}')
    print(f'zatvaranja petlje  : {len(loops)}  (pogresnih: {n_out})')
    print(f'drift odometrije   : {drift:.3f} m na kraju')
    print(f'upisano            : {args.out}.g2o  i  {args.out}_gt.g2o')

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        g = np.array(gt)
        o = np.array(init)
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.plot(g[:, 0], g[:, 1], lw=2, label='tacna putanja')
        ax.plot(o[:, 0], o[:, 1], lw=1.2, label='odometrija (pocetna aproks.)')
        for i, j, _, _, _, out in loops:
            ax.plot([o[i, 0], o[j, 0]], [o[i, 1], o[j, 1]],
                    lw=0.5, alpha=0.6,
                    color='red' if out else 'gray')
        ax.set_aspect('equal')
        ax.legend()
        ax.set_title(f'{args.out}.g2o')
        fig.savefig(f'{args.out}.png', dpi=130, bbox_inches='tight')
        print(f'slika              : {args.out}.png')


if __name__ == '__main__':
    main()