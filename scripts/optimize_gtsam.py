"""Optimizacija 2D pose-grafa iz .g2o fajla pomocu GTSAM-a.

Graf se gradi rucno iz fajla (ne preko readG2o) da bi se moglo
birati robusno jezgro po vrsti grane.
"""

import argparse
import math
import time

import numpy as np
import gtsam
from gtsam import Pose2, noiseModel


# ---------- citanje .g2o ----------

def read_g2o(path):
    verts, edges = {}, []
    with open(path) as f:
        for line in f:
            p = line.split()
            if not p:
                continue
            if p[0] == 'VERTEX_SE2':
                verts[int(p[1])] = (float(p[2]), float(p[3]), float(p[4]))
            elif p[0] == 'EDGE_SE2':
                i, j = int(p[1]), int(p[2])
                m = (float(p[3]), float(p[4]), float(p[5]))
                o = [float(x) for x in p[6:12]]
                omega = np.array([[o[0], o[1], o[2]],
                                  [o[1], o[3], o[4]],
                                  [o[2], o[4], o[5]]])
                edges.append((i, j, m, omega))
    return verts, edges


def write_g2o(path, values, keys):
    with open(path, 'w') as f:
        for k in keys:
            p = values.atPose2(k)
            f.write(f'VERTEX_SE2 {k} {p.x():.6f} {p.y():.6f} {p.theta():.6f}\n')


# ---------- greska ----------

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def ape(values, gt, keys):
    pos, ang = [], []
    for k in keys:
        p = values.atPose2(k)
        gx, gy, gth = gt[k]
        pos.append(math.hypot(p.x() - gx, p.y() - gy))
        ang.append(abs(wrap(p.theta() - gth)))
    pos = np.array(pos)
    return (math.sqrt((pos ** 2).mean()), pos.max(),
            math.degrees(np.array(ang).mean()))


# ---------- glavni tok ----------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--graph', required=True)
    ap.add_argument('--gt', required=True)
    ap.add_argument('--optimizer', default='lm', choices=['gn', 'lm', 'dogleg'])
    ap.add_argument('--linear', default='MULTIFRONTAL_CHOLESKY')
    ap.add_argument('--robust', default='none',
                    choices=['none', 'huber', 'cauchy'])
    ap.add_argument('--robust-k', type=float, default=1.0)
    ap.add_argument('--max-iter', type=int, default=100)
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    verts, edges = read_g2o(args.graph)
    gt, _ = read_g2o(args.gt)
    keys = sorted(verts)

    graph = gtsam.NonlinearFactorGraph()

    # datum: prvi cvor fiksiran (defekt ranga 3 se uklanja ovde)
    prior = noiseModel.Diagonal.Variances(np.array([1e-6, 1e-6, 1e-8]))
    x0, y0, t0 = verts[keys[0]]
    graph.add(gtsam.PriorFactorPose2(keys[0], Pose2(x0, y0, t0), prior))

    n_loop = 0
    for i, j, m, omega in edges:
        model = noiseModel.Gaussian.Information(omega)
        is_loop = (j != i + 1)
        if is_loop:
            n_loop += 1
            if args.robust == 'huber':
                model = noiseModel.Robust.Create(
                    noiseModel.mEstimator.Huber.Create(args.robust_k), model)
            elif args.robust == 'cauchy':
                model = noiseModel.Robust.Create(
                    noiseModel.mEstimator.Cauchy.Create(args.robust_k), model)
        graph.add(gtsam.BetweenFactorPose2(i, j, Pose2(*m), model))

    initial = gtsam.Values()
    for k in keys:
        initial.insert(k, Pose2(*verts[k]))

    if args.optimizer == 'gn':
        params = gtsam.GaussNewtonParams()
    elif args.optimizer == 'dogleg':
        params = gtsam.DoglegParams()
    else:
        params = gtsam.LevenbergMarquardtParams()
    params.setMaxIterations(args.max_iter)
    try:
        params.setLinearSolverType(args.linear)
    except Exception:
        print(f'upozorenje: linearni solver {args.linear} nije prihvacen')

    if args.optimizer == 'gn':
        opt = gtsam.GaussNewtonOptimizer(graph, initial, params)
    elif args.optimizer == 'dogleg':
        opt = gtsam.DoglegOptimizer(graph, initial, params)
    else:
        opt = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)

    chi2_0 = 2.0 * graph.error(initial)
    t = time.perf_counter()
    result = opt.optimize()
    dt = time.perf_counter() - t
    chi2_1 = 2.0 * graph.error(result)

    a0 = ape(initial, gt, keys)
    a1 = ape(result, gt, keys)

    print(f'graf            : {args.graph}')
    print(f'cvorova / grana : {len(keys)} / {len(edges)}  '
          f'(zatvaranja petlje: {n_loop})')
    print(f'optimizator     : {args.optimizer}  '
          f'linearni: {args.linear}  robusno: {args.robust}')
    print(f'iteracija       : {opt.iterations()}')
    print(f'vreme           : {dt * 1000:.1f} ms')
    print(f'chi2            : {chi2_0:.1f}  ->  {chi2_1:.1f}')
    print(f'APE RMS  [cm]   : {a0[0] * 100:.2f}  ->  {a1[0] * 100:.2f}')
    print(f'APE max  [cm]   : {a0[1] * 100:.2f}  ->  {a1[1] * 100:.2f}')
    print(f'ugao sred [deg] : {a0[2]:.3f}  ->  {a1[2]:.3f}')

    if args.out:
        write_g2o(args.out, result, keys)
        print(f'upisano         : {args.out}')


if __name__ == '__main__':
    main()