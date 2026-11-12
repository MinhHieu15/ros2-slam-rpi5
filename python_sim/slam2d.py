"""
2D graph SLAM in pure Python, run on a simulated indoor environment.

The robot drives a loop through a few rooms with a 360-beam LiDAR and drifting wheel odometry.
Pipeline:
  front end  - keyframe every 0.3 m / 15 deg, scan-to-submap ICP starting from the odometry guess
  loop close - ICP against old keyframes near the current estimate, accepted on fit quality
  back end   - pose graph optimised with scipy least_squares (sparse Jacobian)
  mapping    - log-odds occupancy grid from the optimised keyframe poses

Outputs in results/: trajectories (CSV), metrics, maps and plots.
"""
import csv
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial import cKDTree

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
rng = np.random.default_rng(7)

DT = 0.1
N_BEAMS = 360
MAX_RANGE = 8.0
RANGE_NOISE = 0.015
V_NOISE, W_NOISE, W_BIAS, V_SCALE = 0.02, 0.03, 0.006, 1.015
KF_DIST, KF_ANGLE = 0.3, np.deg2rad(15)
RES = 0.05


# ---------------------------------------------------------------- world and sensors
def build_world():
    segs = []

    def box(x0, y0, x1, y1):
        segs.extend([(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)])

    box(0, 0, 14, 10)                       # outer walls
    segs += [(5, 0, 5, 3.5), (5, 5.5, 5, 10)]          # wall with a door
    segs += [(9, 10, 9, 6.5), (9, 4.5, 9, 0)]          # second wall with a door
    segs += [(9, 6.5, 11.5, 6.5), (12.8, 6.5, 14, 6.5)]
    box(1.5, 6.5, 2.7, 7.7)                 # furniture
    box(2.0, 1.5, 3.2, 2.2)
    box(6.5, 7.0, 7.5, 8.5)
    box(6.8, 1.2, 7.4, 1.8)
    box(11.0, 2.0, 12.5, 3.0)
    box(11.8, 8.0, 12.6, 8.8)
    segs += [(0, 4.5, 1.2, 4.5), (13.0, 4.0, 14, 4.0)]
    return np.array(segs, float)


def raycast(pose, segs):
    x, y, th = pose
    ang = th + np.linspace(-np.pi, np.pi, N_BEAMS, endpoint=False)
    d = np.stack([np.cos(ang), np.sin(ang)], 1)[:, None, :]          # N,1,2
    p = segs[None, :, :2]
    e = segs[None, :, 2:] - p                                         # 1,M,2
    o = np.array([x, y])[None, None, :]
    den = d[..., 0] * e[..., 1] - d[..., 1] * e[..., 0]
    w = p - o
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (w[..., 0] * e[..., 1] - w[..., 1] * e[..., 0]) / den
        u = (w[..., 0] * d[..., 1] - w[..., 1] * d[..., 0]) / den
    t = np.where((np.abs(den) > 1e-9) & (t > 0) & (u >= 0) & (u <= 1), t, np.inf)
    r = t.min(axis=1)
    r = np.where(r < MAX_RANGE, r + rng.normal(0, RANGE_NOISE, N_BEAMS), np.inf)
    return r


def scan_points(r):
    ang = np.linspace(-np.pi, np.pi, N_BEAMS, endpoint=False)
    ok = np.isfinite(r)
    return np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])], 1)


# ---------------------------------------------------------------- pose helpers
def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def transform(pose, pts):
    c, s = np.cos(pose[2]), np.sin(pose[2])
    return pts @ np.array([[c, s], [-s, c]]) + pose[:2]


def relative(a, b):
    """Pose of b expressed in the frame of a."""
    c, s = np.cos(a[2]), np.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return np.array([c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2])])


def compose(a, d):
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([a[0] + c * d[0] - s * d[1], a[1] + s * d[0] + c * d[1], wrap(a[2] + d[2])])


# ---------------------------------------------------------------- scan matching
def icp(src, dst, init, iters=40):
    """Align src (robot frame) to dst (world frame). Returns pose, inlier ratio, rms error."""
    tree = cKDTree(dst)
    pose = init.copy()
    gate = 0.5
    for _ in range(iters):
        p = transform(pose, src)
        dist, idx = tree.query(p)
        m = dist < gate
        if m.sum() < 30:
            return pose, 0.0, np.inf
        a, b = p[m], dst[idx[m]]
        ca, cb = a.mean(0), b.mean(0)
        H = (a - ca).T @ (b - cb)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[1] *= -1
            R = Vt.T @ U.T
        dth = np.arctan2(R[1, 0], R[0, 0])
        t = cb - R @ ca
        new_xy = R @ pose[:2] + t
        step = np.hypot(*(new_xy - pose[:2])) + abs(dth)
        pose = np.array([new_xy[0], new_xy[1], wrap(pose[2] + dth)])
        gate = max(0.1, gate * 0.8)
        if step < 1e-5:
            break
    dist, _ = tree.query(transform(pose, src))
    inl = dist < 0.1
    return pose, inl.mean(), np.sqrt(np.mean(dist[inl] ** 2)) if inl.any() else np.inf


# ---------------------------------------------------------------- pose graph
def optimise(poses, edges):
    n = len(poses)

    def residual(x):
        P = np.vstack([poses[0], x.reshape(-1, 3)])
        res = np.empty(3 * len(edges))
        for k, (i, j, z, w) in enumerate(edges):
            e = relative(P[i], P[j]) - z
            e[2] = wrap(e[2])
            res[3 * k:3 * k + 3] = e * w
        return res

    sp = lil_matrix((3 * len(edges), 3 * (n - 1)), dtype=int)
    for k, (i, j, _, _) in enumerate(edges):
        for v in (i, j):
            if v > 0:
                sp[3 * k:3 * k + 3, 3 * (v - 1):3 * v] = 1
    sol = least_squares(residual, poses[1:].ravel(), jac_sparsity=sp, method="trf", x_scale="jac")
    return np.vstack([poses[0], sol.x.reshape(-1, 3)])


# ---------------------------------------------------------------- occupancy grid
def occupancy(poses, scans, extent=(-1, 15, -1, 11)):
    x0, x1, y0, y1 = extent
    W, H = int((x1 - x0) / RES), int((y1 - y0) / RES)
    L = np.zeros((H, W))
    ang = np.linspace(-np.pi, np.pi, N_BEAMS, endpoint=False)
    for pose, r in zip(poses, scans):
        hit = np.isfinite(r)
        rr = np.where(hit, r, MAX_RANGE)
        a = pose[2] + ang
        for frac in np.arange(0, 1, RES / MAX_RANGE / 1.5):
            d = rr * frac
            px = ((pose[0] + d * np.cos(a) - x0) / RES).astype(int)
            py = ((pose[1] + d * np.sin(a) - y0) / RES).astype(int)
            ok = (px >= 0) & (px < W) & (py >= 0) & (py < H) & (d < rr - RES)
            L[py[ok], px[ok]] -= 0.04
        ex = ((pose[0] + rr[hit] * np.cos(a[hit]) - x0) / RES).astype(int)
        ey = ((pose[1] + rr[hit] * np.sin(a[hit]) - y0) / RES).astype(int)
        ok = (ex >= 0) & (ex < W) & (ey >= 0) & (ey < H)
        np.add.at(L, (ey[ok], ex[ok]), 0.9)
    return 1 / (1 + np.exp(-np.clip(L, -10, 10))), extent


# ---------------------------------------------------------------- driving
def route():
    return [(2.5, 3.5), (4.0, 4.5), (7.0, 4.5), (8.0, 5.5), (10.5, 5.5), (12.5, 5.0), (12.8, 8.2), (10.0, 8.5),
            (9.8, 5.5), (7.5, 3.0), (6.0, 1.0), (4.0, 4.5), (2.5, 8.8), (0.8, 8.5), (0.8, 2.5), (2.5, 3.5),
            (4.0, 4.5), (7.0, 4.5), (8.0, 5.5), (10.5, 5.5), (12.5, 1.2), (10.0, 1.0), (8.0, 5.5), (4.0, 4.5),
            (2.5, 3.5)]


def drive(segs):
    gt = np.array([1.2, 3.0, 0.3])
    odo = gt.copy()
    truth, odom, scans = [gt.copy()], [odo.copy()], [raycast(gt, segs)]
    for wx, wy in route():
        for _ in range(4000):
            dx, dy = wx - gt[0], wy - gt[1]
            if np.hypot(dx, dy) < 0.15:
                break
            err = wrap(np.arctan2(dy, dx) - gt[2])
            w = np.clip(2.0 * err, -1.2, 1.2)
            v = 0.45 * max(0.0, np.cos(err)) ** 2
            gt = compose(gt, np.array([v * DT * np.cos(w * DT / 2), v * DT * np.sin(w * DT / 2), w * DT]))
            vm = v * V_SCALE + rng.normal(0, V_NOISE) * (v > 0)
            wm = w + W_BIAS + rng.normal(0, W_NOISE)
            odo = compose(odo, np.array([vm * DT * np.cos(wm * DT / 2), vm * DT * np.sin(wm * DT / 2), wm * DT]))
            truth.append(gt.copy())
            odom.append(odo.copy())
            scans.append(raycast(gt, segs))
    return np.array(truth), np.array(odom), scans


# ---------------------------------------------------------------- main
def main():
    os.makedirs(OUT, exist_ok=True)
    segs = build_world()
    truth, odom, scans = drive(segs)
    print(f"{len(truth)} steps, {np.sum(np.hypot(*np.diff(truth[:, :2], axis=0).T)):.1f} m driven")

    kf_idx, kf_est, edges, loops = [0], [odom[0].copy()], [], []
    last_odo = odom[0]
    for k in range(1, len(odom)):
        d = relative(last_odo, odom[k])
        if np.hypot(d[0], d[1]) < KF_DIST and abs(d[2]) < KF_ANGLE:
            continue
        guess = compose(kf_est[-1], d)
        sub = np.vstack([transform(kf_est[i], scan_points(scans[kf_idx[i]])) for i in range(max(0, len(kf_idx) - 6), len(kf_idx))])
        pose, inl, _ = icp(scan_points(scans[k]), sub, guess)
        if inl < 0.6:
            pose = guess
        z = relative(kf_est[-1], pose)
        edges.append((len(kf_idx) - 1, len(kf_idx), z, np.array([20.0, 20.0, 40.0])))
        kf_idx.append(k)
        kf_est.append(pose)
        last_odo = odom[k]

        # loop closure against old keyframes near the current estimate
        j = len(kf_idx) - 1
        P = np.array(kf_est)
        near = [i for i in range(j - 40) if np.hypot(*(P[i, :2] - P[j, :2])) < 2.0]
        if near and j % 3 == 0:
            i = min(near, key=lambda i: np.hypot(*(P[i, :2] - P[j, :2])))
            ref = np.vstack([transform(P[m], scan_points(scans[kf_idx[m]])) for m in range(max(0, i - 2), min(j - 40, i + 3))])
            pose_lc, inl, rms = icp(scan_points(scans[k]), ref, P[j])
            if inl > 0.85 and rms < 0.04:
                edges.append((i, j, relative(P[i], pose_lc), np.array([30.0, 30.0, 60.0])))
                loops.append((i, j))
                kf_est = list(optimise(np.array(kf_est), edges))

    kf_idx = np.array(kf_idx)
    slam = optimise(np.array(kf_est), edges)
    kt, ko = truth[kf_idx], odom[kf_idx]

    def ate(est):
        e = np.hypot(est[:, 0] - kt[:, 0], est[:, 1] - kt[:, 1])
        return np.sqrt(np.mean(e ** 2)), e.max(), e

    a_odo, m_odo, e_odo = ate(ko)
    a_slam, m_slam, e_slam = ate(slam)
    h_odo = np.rad2deg(np.sqrt(np.mean(wrap(ko[:, 2] - kt[:, 2]) ** 2)))
    h_slam = np.rad2deg(np.sqrt(np.mean(wrap(slam[:, 2] - kt[:, 2]) ** 2)))
    print(f"keyframes {len(kf_idx)}, loop closures {len(loops)}")
    print(f"odometry  ATE rmse {a_odo:.3f} m  max {m_odo:.3f} m  heading rmse {h_odo:.2f} deg")
    print(f"SLAM      ATE rmse {a_slam:.3f} m  max {m_slam:.3f} m  heading rmse {h_slam:.2f} deg")

    with open(os.path.join(OUT, "metrics.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["method", "ate_rmse_m", "ate_max_m", "heading_rmse_deg", "keyframes", "loop_closures"])
        w.writerow(["odometry", f"{a_odo:.4f}", f"{m_odo:.4f}", f"{h_odo:.3f}", len(kf_idx), 0])
        w.writerow(["graph_slam", f"{a_slam:.4f}", f"{m_slam:.4f}", f"{h_slam:.3f}", len(kf_idx), len(loops)])
    with open(os.path.join(OUT, "trajectories.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["keyframe", "step", "t_s", "gt_x", "gt_y", "gt_th", "odo_x", "odo_y", "odo_th", "slam_x", "slam_y", "slam_th"])
        for n, (k, g, o, s) in enumerate(zip(kf_idx, kt, ko, slam)):
            w.writerow([n, k, f"{k * DT:.1f}", *[f"{v:.4f}" for v in (*g, *o, *s)]])

    kf_scans = [scans[k] for k in kf_idx]
    grid_odo, ext = occupancy(ko, kf_scans)
    grid_slam, _ = occupancy(slam, kf_scans)
    plt.imsave(os.path.join(OUT, "map_slam.png"), 1 - grid_slam, cmap="gray", origin="lower")

    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    for ax, grid, est, title in ((axes[0], grid_odo, ko, f"Odometry only (ATE {a_odo:.2f} m)"),
                                 (axes[1], grid_slam, slam, f"Graph SLAM (ATE {a_slam:.3f} m, {len(loops)} loop closures)")):
        ax.imshow(1 - grid, cmap="gray", origin="lower", extent=ext, vmin=0, vmax=1)
        ax.plot(kt[:, 0], kt[:, 1], "g-", lw=1.2, label="ground truth")
        ax.plot(est[:, 0], est[:, 1], "r-", lw=1.0, label="estimate")
        ax.set_title(title)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "map_comparison.png"), dpi=130)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 6.5))
    for s in segs:
        ax.plot(s[[0, 2]], s[[1, 3]], "k-", lw=2)
    ax.plot(kt[:, 0], kt[:, 1], "g-", lw=1.5, label="ground truth")
    ax.plot(ko[:, 0], ko[:, 1], "-", color="orange", lw=1, label="odometry")
    ax.plot(slam[:, 0], slam[:, 1], "b--", lw=1.2, label="graph SLAM")
    for i, j in loops:
        ax.plot(slam[[i, j], 0], slam[[i, j], 1], "m-", lw=0.8, alpha=0.6)
    ax.plot([], [], "m-", label="loop closure edge")
    ax.set_aspect("equal")
    ax.set_title("Trajectories in the simulated environment")
    ax.legend(loc="upper left", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "trajectories.png"), dpi=130)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4))
    t = kf_idx * DT
    ax.plot(t, e_odo, color="orange", label="odometry")
    ax.plot(t, e_slam, "b", label="graph SLAM")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position error (m)")
    ax.set_title("Position error against ground truth")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "error_over_time.png"), dpi=130)
    plt.close(fig)


if __name__ == "__main__":
    main()
