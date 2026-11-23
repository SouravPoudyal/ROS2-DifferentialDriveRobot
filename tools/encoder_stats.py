#!/usr/bin/env python3
"""Encoder / wheel-state statistics for real-vs-simulated noise modelling.

  python3 encoder_stats.py record  --label real_straight --duration 12
  python3 encoder_stats.py compare real_straight.json sim_straight.json
  python3 encoder_stats.py suggest real_straight.json

Reads sensor_msgs/JointState (wheel positions [rad], velocities [rad/s]) and,
optionally, nav_msgs/Odometry. Only the standard library is needed besides rclpy.
"""
import argparse
import json
import math
import statistics as st
import sys
import time

SLEW = 50.0   # firmware ramp constant [rad/s^2] (Part I of the report)


# ------------------------------------------------------------------ helpers
def mean(x):
    return st.fmean(x) if len(x) else float("nan")


def sd(x):
    return st.pstdev(x) if len(x) > 1 else float("nan")


def smooth(x, w=5):
    h = w // 2
    return [mean(x[max(0, i - h):i + h + 1]) for i in range(len(x))]


def fit_quantum(incs):
    """Largest step d such that every position increment is a multiple of d."""
    v = sorted(set(round(x, 7) for x in incs if x > 1e-6))
    if len(v) < 3:
        return None
    gap = min(b - a for a, b in zip(v, v[1:]))
    if gap < 1e-4:                       # finer than any plausible encoder
        return None
    for k in (1, 2, 3):
        d = gap / k
        if max(abs(x / d - round(x / d)) for x in incs if x > 1e-6) < 0.05:
            return d
    return None


def pred_rise(w0, slew, tau, dt=0.002):
    """10-90 % rise time of slew limiter followed by first-order lag (eq. 9-10)."""
    alpha = 1 - math.exp(-dt / tau)
    ws = y = t = 0.0
    t10 = t90 = None
    while t < 5.0 and t90 is None:
        ws += max(-slew * dt, min(slew * dt, w0 - ws))
        y += alpha * (ws - y)
        t += dt
        if t10 is None and y >= 0.1 * w0:
            t10 = t
        if y >= 0.9 * w0:
            t90 = t
    return None if t90 is None else t90 - t10


def fit_tau(rise, w0, slew=SLEW):
    best, err = None, 1e9
    for k in range(2, 200):
        p = pred_rise(w0, slew, k * 0.005)
        if p is not None and abs(p - rise) < err:
            best, err = k * 0.005, abs(p - rise)
    return best if err < 0.03 else None   # None: faster than the ramp allows


# ----------------------------------------------------------------- analysis
def analyse(js, od, radius=0.033, sep=0.1402):
    """js rows: (t, pL, pR, vL, vR); od rows: (t, vx, wz)."""
    if len(js) < 20:
        raise ValueError("too few joint-state samples")
    t0 = js[0][0]
    t = [r[0] - t0 for r in js]
    pL, pR = [r[1] for r in js], [r[2] for r in js]
    vL, vR = [r[3] for r in js], [r[4] for r in js]
    n = len(t)
    for v, p in ((vL, pL), (vR, pR)):     # derive speed if the topic has none
        for i in range(n):
            if math.isnan(v[i]):
                dt = t[i] - t[i - 1] if i else 0.0
                v[i] = (p[i] - p[i - 1]) / dt if dt > 0 else 0.0

    o = {"samples": n, "duration_s": t[-1]}
    dts = [b - a for a, b in zip(t, t[1:]) if b > a]
    o["msg_rate_hz"] = 1 / mean(dts) if dts else None

    inc = []
    for p in (pL, pR):
        inc += [abs(p[i] - p[i - 1]) for i in range(1, n) if p[i] != p[i - 1]]
    q = fit_quantum(inc) if inc else None
    o["quantum_rad"] = q
    o["cpr_est"] = 2 * math.pi / q if q else None

    sp = smooth([max(abs(x), abs(y)) for x, y in zip(vL, vR)], 15)
    thr = max(0.3, 0.2 * max(sp))
    mv, run = [], []
    for i in range(n + 1):                 # longest contiguous run above thr
        if i < n and sp[i] > thr:
            run.append(i)
        else:
            if len(run) > len(mv):
                mv = run
            run = []
    o["moving"] = len(mv) > 20

    # sample-and-hold: times at which the position changes, measured only while moving
    # (a still wheel often keeps the same count, which would show up as long gaps)
    lo, hi = (mv[0], mv[-1]) if o["moving"] else (1, n - 1)
    ut = [t[i] for i in range(max(lo, 1), hi + 1)
          if pL[i] != pL[i - 1] or pR[i] != pR[i - 1]]
    ud = [b - a for a, b in zip(ut, ut[1:]) if 0.005 < b - a < 0.3]
    o["update_dt_mean_s"], o["update_dt_std_s"] = mean(ud), sd(ud)
    s_end = sum(1 for x in t if x < t[mv[0]] - 0.6) if o["moving"] else n
    s_beg = sum(1 for x in t if x < 1.0)
    if s_end - s_beg > 20:
        o["still_pos_std_rad"] = mean([sd(pL[s_beg:s_end]), sd(pR[s_beg:s_end])])
        o["still_vel_std_rads"] = mean([sd(vL[s_beg:s_end]), sd(vR[s_beg:s_end])])
    o["travel_L_m"] = (pL[-1] - pL[0]) * radius
    o["travel_R_m"] = (pR[-1] - pR[0]) * radius

    ia = ib = None
    if o["moving"]:
        i0, i1 = mv[0], mv[-1]
        ia, ib = i0 + (i1 - i0) // 5, i1 - (i1 - i0) // 5     # middle 60 %
        mL, mR = mean(vL[ia:ib]), mean(vR[ia:ib])
        o.update(steady_vel_L=mL, steady_vel_R=mR,
                 ripple_L=sd(vL[ia:ib]), ripple_R=sd(vR[ia:ib]))
        if abs(mL) > 0.5 and mL * mR > 0:
            o["ratio_R_over_L"] = mR / mL
        for name, v, m in (("L", vL, mL), ("R", vR, mR)):
            if abs(m) <= 1.0:
                continue
            ms = abs(m)
            xs = smooth([x if m > 0 else -x for x in v])
            k0 = sum(1 for x in t if x < t[i0] - 0.6)
            t10 = next((t[i] for i in range(k0, ib) if xs[i] >= 0.1 * ms), None)
            t90 = next((t[i] for i in range(k0, ib) if xs[i] >= 0.9 * ms), None)
            if t10 is not None and t90 is not None and t90 > t10:
                o[f"t10_{name}_s"] = t10
                o[f"rise_{name}_s"] = t90 - t10
                o[f"overshoot_{name}_pct"] = 100 * (max(xs[i0:ib]) - ms) / ms
        rises = [o[k] for k in ("rise_L_s", "rise_R_s") if k in o]
        if rises:
            o["tau_fit_s"] = fit_tau(mean(rises), mean([abs(mL), abs(mR)]))

    if od:
        ot = [r[0] - t0 for r in od]
        vx, wz = [r[1] for r in od], [r[2] for r in od]
        cut = t[mv[0]] - 0.6 if o["moving"] else 1e9
        pre = [vx[i] for i, x in enumerate(ot) if 1.0 <= x < cut]
        if len(pre) > 20:
            o["odom_vx_still_std"] = sd(pre)
        if ia is not None:
            sel = [i for i, x in enumerate(ot) if t[ia] <= x <= t[ib]]
            if len(sel) > 5:
                o["odom_vx_steady_mean"] = mean([vx[i] for i in sel])
                o["odom_vx_steady_std"] = sd([vx[i] for i in sel])
                o["odom_wz_steady_mean"] = mean([wz[i] for i in sel])
                o["odom_wz_steady_std"] = sd([wz[i] for i in sel])
        T = o["update_dt_mean_s"] if ud else 0.05
        # report eq. (7): sigma_p = sigma_v * T / r (two-point odometry only)
        if "odom_vx_still_std" in o:
            o["sigma_p_still_odom"] = o["odom_vx_still_std"] * T / radius
        if "odom_vx_steady_std" in o:
            o["sigma_p_moving_odom"] = o["odom_vx_steady_std"] * T / radius
        # independent wheel noise gives wz_std / vx_std = 2 / sep (14.3 here);
        # a much smaller ratio means the noise is common to both wheels
        if o.get("odom_vx_steady_std") and o.get("odom_wz_steady_std") is not None:
            o["wz_over_vx_std"] = o["odom_wz_steady_std"] / o["odom_vx_steady_std"]
            o["sigma_p_from_wz"] = o["odom_wz_steady_std"] * sep * T / (2 * radius)

    return {k: (None if isinstance(v, float) and math.isnan(v) else v)
            for k, v in o.items()}


# ------------------------------------------------------------------- output
ROWS = [
    ("samples", "joint-state samples"),
    ("msg_rate_hz", "message rate [Hz]"),
    ("update_dt_mean_s", "position update interval [s]"),
    ("update_dt_std_s", "update interval std (jitter) [s]"),
    ("quantum_rad", "position quantum [rad]"),
    ("cpr_est", "CPR estimate"),
    ("still_pos_std_rad", "still: position std [rad]"),
    ("still_vel_std_rads", "still: wheel speed std [rad/s]"),
    ("steady_vel_L", "steady speed L [rad/s]"),
    ("steady_vel_R", "steady speed R [rad/s]"),
    ("ripple_L", "steady ripple L, std [rad/s]"),
    ("ripple_R", "steady ripple R, std [rad/s]"),
    ("ratio_R_over_L", "speed ratio R/L"),
    ("t10_L_s", "10 % crossing L, time since recording start [s]"),
    ("rise_L_s", "10-90 % rise L [s]"),
    ("rise_R_s", "10-90 % rise R [s]"),
    ("overshoot_L_pct", "overshoot L [%]"),
    ("overshoot_R_pct", "overshoot R [%]"),
    ("tau_fit_s", "effective lag tau, slew 50 [s]"),
    ("travel_L_m", "wheel travel L over run [m]"),
    ("travel_R_m", "wheel travel R over run [m]"),
    ("odom_vx_still_std", "odom vx std, still [m/s]"),
    ("odom_vx_steady_mean", "odom vx steady mean [m/s]"),
    ("odom_vx_steady_std", "odom vx steady std [m/s]"),
    ("odom_wz_steady_mean", "odom wz steady mean [rad/s]"),
    ("odom_wz_steady_std", "odom wz steady std [rad/s]"),
    ("sigma_p_still_odom", "sigma_p from odom, still [rad]"),
    ("sigma_p_moving_odom", "sigma_p from odom vx, moving [rad]"),
    ("wz_over_vx_std", "odom wz std / vx std (independent: 2/sep)"),
    ("sigma_p_from_wz", "sigma_p from odom wz [rad]"),
]


def fmt(v):
    if v is None:
        return "-"
    return f"{v:.4g}" if isinstance(v, (int, float)) else str(v)


def table(files):
    data = [json.load(open(f)) for f in files]
    names = [d.get("label", f) for d, f in zip(data, files)]
    head = f"{'metric':44s}" + "".join(f"{n[:18]:>20s}" for n in names)
    print(head + (f"{'ratio 2/1':>12s}" if len(data) > 1 else ""))
    print("-" * len(head))
    for key, label in ROWS:
        vals = [d.get(key) for d in data]
        if all(v is None for v in vals):
            continue
        line = f"{label:44s}" + "".join(f"{fmt(v):>20s}" for v in vals)
        if len(vals) > 1 and all(isinstance(v, (int, float)) for v in vals[:2]) \
                and vals[0] not in (0, 0.0):
            line += f"{vals[1] / vals[0]:>12.3g}"
        print(line)


def suggest(path):
    d = json.load(open(path))
    print(f"Launch arguments from {path} (use a REAL recording):\n")
    sp = d.get("sigma_p_from_wz") or d.get("sigma_p_moving_odom")
    if sp:
        print(f"  noise_position_std:={sp:.4g}")
    if d.get("cpr_est"):
        print(f"  noise_cpr:={round(d['cpr_est'])}.0")
    if d.get("update_dt_mean_s"):
        print(f"  sample_rate_hz:={1 / d['update_dt_mean_s']:.3g}")
    if d.get("update_dt_std_s") is not None:
        print(f"  sample_jitter_s:={d['update_dt_std_s']:.4g}")
    if d.get("tau_fit_s"):
        print(f"  lag_tau:={d['tau_fit_s']:.3g}")
    if d.get("ratio_R_over_L"):
        print(f"  left_gain:=1.0 right_gain:={d['ratio_R_over_L']:.3g}"
              "   (valid only for a straight, equal-command run)")
    still = d.get("still_pos_std_rad")
    if still is not None and still < 1e-3:
        print("\nNote: the real encoders are quiet when still "
              f"(position std {still:.2g} rad). Constant position jitter then "
              "over-noises a standing robot; the real ripple comes from the "
              "count-based speed estimate while moving.")


# ------------------------------------------------------------------ record
def record(a):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = Node("encoder_stats")
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=500)
    js, od = [], []

    def stamp(h):
        t = h.stamp.sec + h.stamp.nanosec * 1e-9
        return t if t > 0 else time.monotonic()

    def on_js(m):
        if a.left not in m.name or a.right not in m.name:
            return
        i, j = m.name.index(a.left), m.name.index(a.right)
        v = list(m.velocity)
        nan = float("nan")
        js.append((stamp(m.header), m.position[i], m.position[j],
                   v[i] if len(v) > i else nan, v[j] if len(v) > j else nan))

    def on_od(m):
        od.append((stamp(m.header), m.twist.twist.linear.x, m.twist.twist.angular.z))

    node.create_subscription(JointState, a.joint_topic, on_js, qos)
    if a.odom_topic:
        node.create_subscription(Odometry, a.odom_topic, on_od, qos)
    print(f"Recording {a.duration:.0f} s from {a.joint_topic} ...")
    end = time.monotonic() + a.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

    if not js:
        sys.exit(f"No data on {a.joint_topic} for joints {a.left}/{a.right}. "
                 "Check the environment file you sourced and `ros2 topic list`.")
    out = analyse(js, od, a.radius, a.sep)
    out["label"], out["joint_topic"] = a.label, a.joint_topic
    json.dump(out, open(f"{a.label}.json", "w"), indent=2)
    if a.csv:
        t0 = js[0][0]
        with open(f"{a.label}.csv", "w") as f:
            f.write("t,pos_left,pos_right,vel_left,vel_right\n")
            for r in js:
                f.write(f"{r[0] - t0:.6f},{r[1]:.6f},{r[2]:.6f},{r[3]:.6f},{r[4]:.6f}\n")
    table([f"{a.label}.json"])
    print(f"\nSaved {a.label}.json" + (f" and {a.label}.csv" if a.csv else ""))


def reanalyse(a):
    lines = open(a.csv).read().splitlines()[1:]
    js = [tuple(float(x) for x in l.split(",")) for l in lines if l]
    out = analyse(js, [], a.radius, a.sep)
    out["label"] = a.csv.rsplit(".", 1)[0] + "_re"
    json.dump(out, open(out["label"] + ".json", "w"), indent=2)
    table([out["label"] + ".json"])
    print(f"\nSaved {out['label']}.json (joint-state rows only, no odometry)")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record")
    r.add_argument("--label", required=True)
    r.add_argument("--duration", type=float, default=12.0)
    r.add_argument("--joint-topic", default="/joint_states")
    r.add_argument("--odom-topic", default="/bumperbot_controller/odom")
    r.add_argument("--left", default="wheel_left_joint")
    r.add_argument("--right", default="wheel_right_joint")
    r.add_argument("--radius", type=float, default=0.033)
    r.add_argument("--sep", type=float, default=0.1402)
    r.add_argument("--csv", action="store_true", help="also save raw samples")
    c = sub.add_parser("compare")
    c.add_argument("files", nargs="+")
    s = sub.add_parser("suggest")
    s.add_argument("file")
    q = sub.add_parser("reanalyse", help="re-run the analysis on a saved CSV")
    q.add_argument("csv")
    q.add_argument("--radius", type=float, default=0.033)
    q.add_argument("--sep", type=float, default=0.1402)
    a = p.parse_args()
    if a.cmd == "record":
        record(a)
    elif a.cmd == "compare":
        table(a.files)
    elif a.cmd == "reanalyse":
        reanalyse(a)
    else:
        suggest(a.file)


if __name__ == "__main__":
    main()
