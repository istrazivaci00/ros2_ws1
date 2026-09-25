"""Monte Carlo prolaz: vise seed-ova po udelu grubih gresaka.

Grafovi se prave u memoriji, bez pisanja .g2o fajlova.
"""

import argparse
import csv
import os
import statistics as st
import sys
import time

import numpy as np
import gtsam
from gtsam import Pose2, noiseModel

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_graph import trajectory, odometry_edges, loop_edges, compose
from optimize_gtsam import ape


def omega(sxy, sth):
    return np.diag([1.0 / sxy ** 2, 1.0 / sxy ** 2, 1.0 / sth ** 2])


def build(seed, frac, radius=4.5, step=0.5, laps=3.0,
          s_oxy=0.02, s_oth=0.01, s_lxy=0.01, s_lth=0.005,
          lc_dist=0.8, lc_gap=20, lc_prob=0.4):
    rng = np.random.default_rng(seed)
    gt_list = trajectory(radius, step, laps)
    odo = odometry_edges(gt_list, rng, s_oxy, s_oth)
    loops = loop_edges(gt_list, rng, s_lxy, s_lth,
                       lc_dist, lc_gap, lc_prob, frac)

    init = [gt_list[0].copy()]
    for _, _, m, _, _, _ in odo:
        init.append(compose(init[-1], m))

    edges = [(i, j, tuple(m), omega(sxy, sth))
             for i, j, m, sxy, sth, _ in odo + loops]
    verts = {i: tuple(p) for i, p in enumerate(init)}
    gt = {i: tuple(p) for i, p in enumerate(gt_list)}
    return verts, edges, gt, len(loops), sum(1 for e in loops if e[5])


def solve(verts, edges, keys, optimizer, robust, k, max_iter=100):
    graph = gtsam.NonlinearFactorGraph()
    prior = noiseModel.Diagonal.Variances(np.array([1e-6, 1e-6, 1e-8]))
    graph.add(gtsam.PriorFactorPose2(keys[0], Pose2(*verts[keys[0]]), prior))

    for i, j, m, om in edges:
        model = noiseModel.Gaussian.Information(om)
        if j != i + 1:
            if robust == 'huber':
                model = noiseModel.Robust.Create(
                    noiseModel.mEstimator.Huber.Create(k), model)
            elif robust == 'cauchy':
                model = noiseModel.Robust.Create(
                    noiseModel.mEstimator.Cauchy.Create(k), model)
        graph.add(gtsam.BetweenFactorPose2(i, j, Pose2(*m), model))

    initial = gtsam.Values()
    for kk in keys:
        initial.insert(kk, Pose2(*verts[kk]))

    params = {'gn': gtsam.GaussNewtonParams,
              'dogleg': gtsam.DoglegParams,
              'lm': gtsam.LevenbergMarquardtParams}[optimizer]()
    params.setMaxIterations(max_iter)
    cls = {'gn': gtsam.GaussNewtonOptimizer,
           'dogleg': gtsam.DoglegOptimizer,
           'lm': gtsam.LevenbergMarquardtOptimizer}[optimizer]
    opt = cls(graph, initial, params)

    t = time.perf_counter()
    try:
        res, iters = opt.optimize(), opt.iterations()
    except Exception:
        res, iters = initial, -1
    return res, iters, time.perf_counter() - t, 2.0 * graph.error(res)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fracs', nargs='+', type=float,
                    default=[0.0, 0.05, 0.10, 0.20, 0.30])
    ap.add_argument('--lc-probs', nargs='+', type=float,
                    default=[0.05, 0.10, 0.20, 0.40])
    ap.add_argument('--seeds', nargs='+', type=int, default=list(range(1, 11)))
    ap.add_argument('--robust', nargs='+', default=['none', 'huber'])
    ap.add_argument('--optimizer', default='lm')
    ap.add_argument('--robust-k', type=float, default=1.0)
    ap.add_argument('--csv', default='results/mc2d.csv')
    args = ap.parse_args()

    runs = []
    for p in args.lc_probs:
        for frac in args.fracs:
            for seed in args.seeds:
                verts, edges, gt, n_lc, n_out = build(seed, frac, lc_prob=p)
                keys = sorted(verts)
                n_obs = 3 * len(edges) + 3
                red = n_obs - 3 * len(keys)
                for r in args.robust:
                    res, it, dt, chi2 = solve(verts, edges, keys,
                                              args.optimizer, r, args.robust_k)
                    a = ape(res, gt, keys)
                    runs.append({'lc_prob': p, 'frac': frac, 'seed': seed,
                                 'robust': r, 'petlji': n_lc, 'grubih': n_out,
                                 'r_n': round(red / n_obs, 3),
                                 'iter': it, 'ms': round(dt * 1000, 2),
                                 'chi2': round(chi2, 1),
                                 'ape_cm': round(a[0] * 100, 3),
                                 'max_cm': round(a[1] * 100, 3)})
        print(f'  ... zavrsena gustina lc_prob={p}')

    os.makedirs(os.path.dirname(args.csv), exist_ok=True)
    with open(args.csv, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(runs[0]))
        w.writeheader()
        w.writerows(runs)

    print(f"\n{'lc_p':>6}{'r/n':>7}{'udeo':>7}{'jezgro':>9}"
          f"{'APE sred':>10}{'sd':>8}{'iter':>7}{'ms':>7}")
    for p in args.lc_probs:
        for frac in args.fracs:
            for r in args.robust:
                g = [x for x in runs if x['lc_prob'] == p
                     and x['frac'] == frac and x['robust'] == r]
                e = [x['ape_cm'] for x in g]
                sd = st.stdev(e) if len(e) > 1 else 0.0
                print(f'{p:>6.2f}{st.mean(x["r_n"] for x in g):>7.2f}'
                      f'{frac:>7.2f}{r:>9}{st.mean(e):>10.2f}{sd:>8.2f}'
                      f'{st.mean(x["iter"] for x in g):>7.1f}'
                      f'{st.mean(x["ms"] for x in g):>7.1f}')
    print(f'\nupisano: {args.csv}')


if __name__ == '__main__':
    main()