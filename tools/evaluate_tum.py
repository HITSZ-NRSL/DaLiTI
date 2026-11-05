#!/usr/bin/env python3
"""Evaluate a DaLiTI TUM trajectory (t x y z qx qy qz qw, t = time of the state).

usage: evaluate_tum.py EST.tum [--gt GT.tum] [--from T] [--to T] [--json OUT.json]

Without ground truth: path length and end-to-start distance.
With ground truth: absolute trajectory error after an SE(3) alignment (Umeyama, no scale) of the
estimated positions to the ground-truth positions, which are linearly interpolated at the estimate
timestamps (both trajectories must use the same clock).
"""
import argparse
import json

import numpy as np


def load_tum(path):
    data = np.loadtxt(path, ndmin=2)
    if data.shape[1] < 8:
        raise SystemExit(f"{path}: expected 8 columns (t x y z qx qy qz qw)")
    return data


def umeyama(est, ref):
    mean_e, mean_r = est.mean(0), ref.mean(0)
    u, _, vt = np.linalg.svd((est - mean_e).T @ (ref - mean_r))
    d = np.eye(3)
    d[2, 2] = np.sign(np.linalg.det(vt.T @ u.T))
    rot = vt.T @ d @ u.T
    return rot, mean_r - rot @ mean_e


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("est", help="estimated trajectory (TUM)")
    parser.add_argument("--gt", help="ground-truth trajectory (TUM)")
    parser.add_argument("--from", dest="t_from", type=float, help="evaluate from this time [s]")
    parser.add_argument("--to", dest="t_to", type=float, help="evaluate up to this time [s]")
    parser.add_argument("--json", help="write the results to this file")
    args = parser.parse_args()

    est = load_tum(args.est)
    t, pos = est[:, 0], est[:, 1:4]
    res = {
        "poses": int(len(t)),
        "duration_s": float(t[-1] - t[0]),
        "path_m": float(np.linalg.norm(np.diff(pos, axis=0), axis=1).sum()),
        "end_to_start_m": float(np.linalg.norm(pos[-1] - pos[0])),
    }
    if args.gt:
        gt = load_tum(args.gt)
        lo = max(gt[0, 0], t[0] if args.t_from is None else max(t[0], args.t_from))
        hi = min(gt[-1, 0], t[-1] if args.t_to is None else min(t[-1], args.t_to))
        sel = (t >= lo) & (t <= hi)
        if sel.sum() < 10:
            raise SystemExit("fewer than 10 estimated poses overlap the ground truth in time")
        ref = np.stack([np.interp(t[sel], gt[:, 0], gt[:, i]) for i in (1, 2, 3)], 1)
        rot, trans = umeyama(pos[sel], ref)
        err = np.linalg.norm(ref - (pos[sel] @ rot.T + trans), axis=1)
        gsel = (gt[:, 0] >= lo) & (gt[:, 0] <= hi)
        res.update({
            "ate_poses": int(sel.sum()),
            "ate_rmse_m": float(np.sqrt(np.mean(err ** 2))),
            "ate_mean_m": float(err.mean()),
            "ate_median_m": float(np.median(err)),
            "ate_max_m": float(err.max()),
            "final_error_m": float(err[-1]),
            "gt_path_m": float(np.linalg.norm(np.diff(gt[gsel, 1:4], axis=0), axis=1).sum()),
        })

    print(f"  poses:          {res['poses']} over {res['duration_s']:.1f} s, path {res['path_m']:.2f} m")
    print(f"  end-to-start:   {res['end_to_start_m']:.3f} m")
    if args.gt:
        print(f"  ATE (SE3):      RMSE {res['ate_rmse_m']:.3f} m, mean {res['ate_mean_m']:.3f} m, "
              f"max {res['ate_max_m']:.3f} m, final {res['final_error_m']:.3f} m "
              f"({res['ate_poses']} poses, ground-truth path {res['gt_path_m']:.2f} m)")
    if args.json:
        with open(args.json, "w") as f:
            json.dump(res, f, indent=1)


if __name__ == "__main__":
    main()
