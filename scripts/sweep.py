"""Prolazi kroz matricu konfiguracija nad vise grafova i ispisuje tabelu."""

import argparse
import csv
import os
import sys
import time

import numpy as np
import gtsam
from gtsam import Pose2, noiseModel

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from optimize_gtsam import read_g2o, ape


def solve(verts, edges, keys, optimizer, linear, robust, k, max_iter):
    graph = gtsam.NonlinearFactorGraph()
    prior = noiseModel.Diagonal.Variances(np.array([1e-6, 1e-6, 1e-8]))
    graph.add(gtsam.PriorFactorPose2(keys[0], Pose2(*verts[keys[0]]), prior))

    for i, j, m, omega in edges:
        model = noiseModel.Gaussian.Information(omega)
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
    try:
        params.setLinearSolverType(linear)
    except Exception:
        pass

    cls = {'gn': gtsam.GaussNewtonOptimizer,
           'dogleg': gtsam.DoglegOptimizer,
           'lm': gtsam.LevenbergMarquardtOptimizer}[optimizer]
    opt = cls(graph, initial, params)

    t = time.perf_counter()
    try:
        result = opt.optimize()
        iters = opt.iterations()
    except Exception:
        result, iters = initial, -1
    dt = time.perf_counter() - t
    return result, iters, dt, 2.0 * graph.error(result)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--graphs', nargs='+', required=True,
                    help='osnovna imena bez .g2o, npr. graphs/sim_clean')
    ap.add_argument('--optimizers', nargs='+', default=['lm'])
    ap.add_argument('--robust', nargs='+', default=['none', 'huber', 'cauchy'])
    ap.add_argument('--robust-k', type=float, default=1.0)
    ap.add_argument('--linear', default='MULTIFRONTAL_CHOLESKY')
    ap.add_argument('--max-iter', type=int, default=100)
    ap.add_argument('--csv', default='results/sweep.csv')
    args = ap.parse_args()

    rows = []
    for base in args.graphs:
        verts, edges = read_g2o(f'{base}.g2o')
        gt, _ = read_g2o(f'{base}_gt.g2o')
        keys = sorted(verts)
        n_lc = sum(1 for i, j, _, _ in edges if j != i + 1)
        a0 = ape(gtsam_values(verts, keys), gt, keys)

        for o in args.optimizers:
            for r in args.robust:
                res, it, dt, chi2 = solve(verts, edges, keys, o, args.linear,
                                          r, args.robust_k, args.max_iter)
                a1 = ape(res, gt, keys)
                rows.append({
                    'graf': base.split('/')[-1],
                    'grana': len(edges), 'petlji': n_lc,
                    'opt': o, 'robust': r,
                    'iter': it, 'ms': round(dt * 1000, 1),
                    'chi2': round(chi2, 1),
                    'ape0_cm': round(a0[0] * 100, 2),
                    'ape_cm': round(a1[0] * 100, 2),
                    'max_cm': round(a1[1] * 100, 2),
                    'yaw_deg': round(a1[2], 3),
                })

    hdr = ['graf', 'grana', 'petlji', 'opt', 'robust',
           'iter', 'ms', 'chi2', 'ape0_cm', 'ape_cm', 'max_cm', 'yaw_deg']
    w = {h: max(len(h), *(len(str(r[h])) for r in rows)) + 2 for h in hdr}
    print(''.join(h.rjust(w[h]) for h in hdr))
    for r in rows:
        print(''.join(str(r[h]).rjust(w[h]) for h in hdr))

    os.makedirs(os.path.dirname(args.csv), exist_ok=True)
    with open(args.csv, 'w', newline='') as f:
        wr = csv.DictWriter(f, fieldnames=hdr)
        wr.writeheader()
        wr.writerows(rows)
    print(f'\nupisano: {args.csv}')


def gtsam_values(verts, keys):
    v = gtsam.Values()
    for k in keys:
        v.insert(k, Pose2(*verts[k]))
    return v


if __name__ == '__main__':
    main()