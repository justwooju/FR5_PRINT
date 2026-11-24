#!/usr/bin/env python3
"""작은 비커(공) → 큰 비커 붓기 한 판 (기준 자세를 비커 자리만큼 옮겨 IK 로 푼다).

  python pour_run.py --dry                         # 기준 자리로 계획만 (로봇 안 움직임)
  python pour_run.py                               # 기준 자리: 잡기 → 붓기 → 내려놓기 → 초기 자세
  python pour_run.py --S 40,640 --B 200,590        # 비커 자리를 바꿔서 (베이스 mm)
  python pour_run.py --phases place,home           # 쥔 비커를 S 에 내려놓고 초기 자세로
  (음수 좌표는 --S=-80,700 처럼 = 로 붙여 쓴다)

기준(poses.json, pose_go.py 로 저장): grasp = 작은 비커를 책상에서 쥔 자세(손끝 xy = 작은 비커 중심),
big = 작은 비커를 쥔 채 큰 비커와 동심원으로 맞춘 자세(손끝 xy = 큰 비커 중심, 기울이기 직전 높이),
pour = 붓는 자세(큰 비커 기준 상대 자리·자세를 그대로 옮겨 씀), home = 초기 자세.
세우고 옮기는 구간은 직선(자세 고정). 기울이기는 J5 가 12° 될 때까지 직선(손끝 자리·자세를 붓기 자세 쪽으로),
그 뒤 붓기 자세까지만 관절 공간(J5 = 0 손목 특이점을 직선으로 지나지 않으려고).
멈춤: Ctrl+C(보내기를 멈추면 제어판이 0.3 s 뒤 그 자리에 선다) · 제어판 [정지]/Esc · 비상정지.
numpy 가 필요하다 — fr5auto 와 같은 파이썬(openvla_py310)으로 돌린다.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pose_go import DH_A, DH_AL, DH_D, HZ, TOOL_MM, Abort, Sender, get, load  # noqa: E402

TABLE_Z = 36.8          # 책상 높이(베이스 mm) = grasp TCP 94.8 − 자로 잰 손끝 높이 58
SMALL_H = 66.5          # 작은 비커 높이
BIG_H = 105.0           # 큰 비커 높이
BIG_R = 46.0            # 큰 비커 위 반지름
PANEL_FLOOR = 45.0      # 제어판 바닥 한도(TCP z)
J6_WIN = 70.0           # 손목 카메라 선: J6 은 초기 자세 ±70° 안
LIMITS = [(-175, 175), (-260, 80), (-155, 155), (-260, 80), (-170, 170), (-170, 170)]


# ── 기구학 ──────────────────────────────────────────────────────────
def fk(q_deg):
    T = np.eye(4)
    for i in range(6):
        t = math.radians(q_deg[i])
        ca, sa, ct, st = math.cos(DH_AL[i]), math.sin(DH_AL[i]), math.cos(t), math.sin(t)
        T = T @ np.array([[ct, -st * ca, st * sa, DH_A[i] * ct], [st, ct * ca, -ct * sa, DH_A[i] * st],
                          [0, sa, ca, DH_D[i]], [0, 0, 0, 1]])
    T[:3, 3] += T[:3, 2] * TOOL_MM
    return T


def rotvec(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    a = math.acos(c)
    if a < 1e-9:
        return np.zeros(3)
    if a > math.pi - 1e-4:                      # 180° 근처
        w, V = np.linalg.eigh((R + np.eye(3)) / 2)
        return V[:, np.argmax(w)] * a
    return a / (2 * math.sin(a)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def expm(v):
    a = np.linalg.norm(v)
    if a < 1e-12:
        return np.eye(3)
    k = v / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def ik(q0, p, R, iters=80):
    """감쇠 최소제곱 IK. 돌려줌: 관절(°), 위치 오차 mm, 자세 오차 °."""
    q = np.array(q0, float)
    for _ in range(iters):
        T = fk(q)
        e = np.concatenate([p - T[:3, 3], rotvec(R @ T[:3, :3].T) * 100.0])   # 1 rad ≈ 100 mm 무게
        if np.linalg.norm(e[:3]) < 0.02 and np.linalg.norm(e[3:]) < 0.002:
            break
        J = np.zeros((6, 6))
        for j in range(6):
            dq = q.copy(); dq[j] += 1e-4
            Td = fk(dq)
            J[:3, j] = (Td[:3, 3] - T[:3, 3]) / math.radians(1e-4)
            J[3:, j] = rotvec(Td[:3, :3] @ T[:3, :3].T) * 100.0 / math.radians(1e-4)
        dqr = J.T @ np.linalg.solve(J @ J.T + 1.0 * np.eye(6), e)
        q = q + np.degrees(np.clip(dqr, -0.2, 0.2))
    T = fk(q)
    return q, float(np.linalg.norm(p - T[:3, 3])), math.degrees(np.linalg.norm(rotvec(R @ T[:3, :3].T)))


def vertical(R):
    """같은 손목 방향(공구 x 의 수평 성분)을 유지한 채 공구를 곧게 아래로."""
    x = R[:, 0].copy(); x[2] = 0; x /= np.linalg.norm(x)
    z = np.array([0.0, 0.0, -1.0])
    return np.column_stack([x, np.cross(z, x), z])


# ── 궤적 ────────────────────────────────────────────────────────────
def mj(n):
    t = np.linspace(0, 1, n)
    return 10 * t ** 3 - 15 * t ** 4 + 6 * t ** 5


class Plan:
    def __init__(self, q0, grip0):
        self.q, self.g, self.tag = [np.array(q0, float)], [grip0], ["start"]
        self.err = 0.0
        self.cart = []                      # 직선 구간 표본 번호

    @property
    def last(self):
        return self.q[-1]

    def hold(self, sec, grip=None, tag="hold"):
        g = self.g[-1] if grip is None else grip
        for _ in range(max(1, int(sec * HZ))):
            self.q.append(self.last.copy()); self.g.append(g); self.tag.append(tag)

    def joint(self, q1, vmax, tag):
        q1 = np.array(q1, float)
        T = max(1.0, 1.875 * float(np.max(np.abs(q1 - self.last))) / vmax)
        q0 = self.last.copy()
        for s in mj(int(T * HZ) + 1)[1:]:
            self.q.append(q0 + (q1 - q0) * s); self.g.append(self.g[-1]); self.tag.append(tag)

    def line(self, p1, R1, vlin, vang, tag):
        T0 = fk(self.last)
        p0, R0 = T0[:3, 3], T0[:3, :3]
        w = rotvec(R0.T @ R1)
        if np.linalg.norm(p1 - p0) < 0.5 and np.linalg.norm(w) < math.radians(0.5):
            return
        T = max(1.0, 1.875 * max(np.linalg.norm(p1 - p0) / vlin, math.degrees(np.linalg.norm(w)) / vang))
        for s in mj(int(T * HZ) + 1)[1:]:
            q, ep, er = ik(self.last, p0 + (p1 - p0) * s, R0 @ expm(w * s))
            self.err = max(self.err, ep, er / 10)
            self.q.append(q); self.g.append(self.g[-1]); self.tag.append(tag)
            self.cart.append(len(self.q) - 1)


def tilt_switch(q_pre, p_pre, R_pre, p_pour, R_pour, j5_switch):
    """기울이기 직선 구간의 끝: 직선 위에서 J5 가 j5_switch 아래로 내려가기 직전 자리."""
    w = rotvec(R_pre.T @ R_pour)
    q, s_sw = np.array(q_pre, float), 1.0
    for s in np.linspace(0, 1, 201)[1:]:
        qn = ik(q, p_pre + (p_pour - p_pre) * s, R_pre @ expm(w * s))[0]
        if qn[4] < j5_switch:
            break
        q, s_sw = qn, s
    return s_sw, p_pre + (p_pour - p_pre) * s_sw, R_pre @ expm(w * s_sw), q


def solve(p, R, seed, name):
    q, ep, er = ik(seed, p, R, iters=300)
    if ep > 0.5 or er > 0.3:
        sys.exit(f"{name}: IK 가 안 풀린다(위치 {ep:.2f} mm · 자세 {er:.2f}°) — 그 자리는 못 감")
    return q


def build(a, P, q_now, grip_now):
    Tg, Tb, Tp, home = fk(P["grasp"]), fk(P["big"]), fk(P["pour"]), np.array(P["home"], float)
    S_ref, B_ref = Tg[:3, 3][:2], Tb[:3, 3][:2]
    S = (np.array(a.S, float) if a.S else S_ref) + np.array(a.grasp_shift, float)   # 잡고 내려놓는 손끝 자리 옮김
    B = np.array(a.B, float) if a.B else B_ref
    Rg, Rb = vertical(Tg[:3, :3]), vertical(Tb[:3, :3])
    z_g = Tg[2, 3] + a.grasp_dz                        # 잡는 높이(기준보다 grasp_dz mm)
    hang = (z_g - TABLE_Z)                             # 쥔 비커 바닥이 TCP 아래 몇 mm
    z_pre = Tb[2, 3]                                   # 기울이기 직전 높이(big 그대로)
    z_lift = max(a.z_lift, z_pre)
    p_pour = Tp[:3, 3] + np.r_[B - B_ref + np.array(a.pour_shift, float), 0.0]   # 붓는 자리 옮김(베이스 mm)
    q_pour = solve(p_pour, Tp[:3, :3], P["pour"], "붓기 자세")
    info = {"S": S, "B": B, "S_ref": S_ref, "B_ref": B_ref, "z_g": z_g, "hang": hang, "z_lift": z_lift,
            "z_pre": z_pre, "pour_off": p_pour[:2] - B, "q_pour": q_pour}
    pl = Plan(q_now, grip_now)
    op, cl = a.open / 1000.0, a.close / 1000.0
    for ph in a.phases:
        if ph == "grasp":
            q_above = solve(np.r_[S, z_g + a.approach], Rg, P["grasp"], "작은 비커 위")
            pl.hold(0.5, op, "open")
            pl.joint(q_above, a.vmax, "to_small")
            pl.line(np.r_[S, z_g], Rg, a.v_down, a.v_ang, "descend")
            pl.hold(a.grip_wait, cl, "close")
            pl.line(np.r_[S, z_lift], Rg, a.v_down, a.v_ang, "lift")
        elif ph == "pour":
            if np.linalg.norm(fk(pl.last)[:3, 3][:2] - B) > 5.0:   # 이미 큰 비커 위면(멈춘 뒤 다시 시작) 바로 기울이기로
                pl.line(np.r_[S, z_lift], Rg, a.v_lin, a.v_ang, "lift")
                pl.line(np.r_[B, z_lift], Rb, a.v_lin, a.v_ang, "transport")
            pl.line(np.r_[B, z_pre], Rb, a.v_down, a.v_ang, "pre_pour")
            # 기울이기: J5 가 j5_switch 까지는 직선(손끝 자리·자세를 붓기 자세 쪽으로 곧게),
            # 거기서부터 붓기 자세까지는 관절 공간(J5 = 0 손목 특이점을 직선으로 지나면 J4·J6 이 튄다)
            T_pre = fk(pl.last)
            p_pre, R_pre = T_pre[:3, 3].copy(), T_pre[:3, :3].copy()
            s_sw, p_sw, R_sw, _ = tilt_switch(pl.last, p_pre, R_pre, p_pour, Tp[:3, :3], a.j5_switch)
            pl.line(p_sw, R_sw, a.v_down, a.v_ang, "tilt")
            q_sw = pl.last.copy()
            pl.joint(q_pour, a.v_tilt, "tilt_end")
            pl.hold(a.pour_hold, None, "pour_hold")
            pl.joint(q_sw, a.v_tilt, "untilt")
            pl.line(p_pre, R_pre, a.v_down, a.v_ang, "untilt")
            info["tilt_switch"] = (s_sw, float(q_sw[4]))
            pl.line(np.r_[B, z_lift], Rb, a.v_down, a.v_ang, "up")
            pl.line(np.r_[S, z_lift], Rg, a.v_lin, a.v_ang, "return")
        elif ph == "untilt":                            # 기울인 채 멈췄을 때: 붓기 길을 거꾸로 되짚어 세우고 S 위로
            p_pre, R_pre = np.r_[B, z_pre], Rb
            q_pre = solve(p_pre, R_pre, P["big"], "기울이기 직전 자세")
            q_sw = tilt_switch(q_pre, p_pre, R_pre, p_pour, Tp[:3, :3], a.j5_switch)[3]
            if pl.last[4] < a.j5_switch:
                pl.joint(q_sw, a.v_tilt, "untilt")
            pl.line(p_pre, R_pre, a.v_down, a.v_ang, "untilt")
            pl.line(np.r_[B, z_lift], Rb, a.v_down, a.v_ang, "up")
            pl.line(np.r_[S, z_lift], Rg, a.v_lin, a.v_ang, "return")
        elif ph == "place":
            T = fk(pl.last)
            if np.linalg.norm(T[:3, 3][:2] - S) > 1.0 or abs(T[2, 3] - z_lift) > 1.0:
                if T[2, 3] < z_lift - 1.0:
                    pl.line(np.r_[T[:3, 3][:2], z_lift], vertical(T[:3, :3]), a.v_down, a.v_ang, "up")
                pl.line(np.r_[S, z_lift], Rg, a.v_lin, a.v_ang, "return")
            pl.line(np.r_[S, z_g + a.place_gap], Rg, a.v_down, a.v_ang, "place")
            pl.hold(a.grip_wait, op, "release")
            pl.line(np.r_[S, z_g + a.approach], Rg, a.v_down, a.v_ang, "retreat")
        elif ph == "up":                                 # 낮은 자리에서 멈췄을 때: 그 자리에서 곧게 옮기는 높이까지
            T = fk(pl.last)
            if T[2, 3] < z_lift - 1.0:
                pl.line(np.r_[T[:3, 3][:2], z_lift], vertical(T[:3, :3]), a.v_down, a.v_ang, "up")
        elif ph == "home":
            pl.joint(home, a.vmax, "home")
        else:
            sys.exit(f"모르는 단계 '{ph}' (grasp·pour·untilt·place·up·home)")
    return pl, info


def check(pl, info, q_home):
    Q = np.array(pl.q)
    tcp = np.array([fk(q)[:3, 3] for q in Q[::5]])
    out = {"sec": len(Q) / HZ, "ik_err": pl.err,
           "z_min": float(tcp[:, 2].min()),
           "j6": (float(Q[:, 5].min()), float(Q[:, 5].max())),
           "jump": float(np.max(np.abs(np.diff(Q, axis=0)))) if len(Q) > 1 else 0.0,
           "j5_cart": float(np.min(np.abs(Q[pl.cart, 4]))) if pl.cart else None}
    bad = []
    for i, (lo, hi) in enumerate(LIMITS):
        if Q[:, i].min() < lo or Q[:, i].max() > hi:
            bad.append(f"J{i + 1} 한계 밖 ({Q[:, i].min():.0f}~{Q[:, i].max():.0f}°)")
    if max(abs(out["j6"][0] - q_home[5]), abs(out["j6"][1] - q_home[5])) > J6_WIN:
        bad.append(f"J6 이 초기 자세 ±{J6_WIN:.0f}° 밖 ({out['j6'][0]:.0f}~{out['j6'][1]:.0f}°)")
    if out["z_min"] < PANEL_FLOOR + 3:
        bad.append(f"TCP 가 제어판 바닥 한도 근처 ({out['z_min']:.0f} mm)")
    if out["jump"] > 2.0:
        bad.append(f"한 표본에 관절이 {out['jump']:.1f}° 뜀(IK 가지 바뀜)")
    if pl.err > 1.0:
        bad.append(f"직선 구간 IK 오차 {pl.err:.2f}")
    if out["j5_cart"] is not None and out["j5_cart"] < 5.0:
        bad.append(f"직선 구간 |J5| 최소 {out['j5_cart']:.1f}° — 손목 특이점")
    clear = info["z_pre"] - info["hang"] - (TABLE_Z + BIG_H)
    if clear < 5.0:
        bad.append(f"기울이기 직전 작은 비커 바닥이 큰 비커 테두리보다 {clear:.0f} mm 위뿐")
    out["clear"] = clear
    return out, bad


def stream(pl, a):
    st = get("/api/state")
    if st.get("mode") != "engaged":
        sys.exit("제어판이 조작 상태가 아니다 — [조작 시작]을 먼저 누른다")
    snd = Sender(a.udp_host, a.udp_port, pl.g[0])
    snd.q = list(pl.q[0])
    snd.start()
    try:
        input("지금 자세를 보내는 중. 제어판 /leader 에서 [리더 추종 시작]을 누른 뒤 Enter (취소 Ctrl+C) > ")
        t0, chk, worst, last_tag, over = time.monotonic(), 0.0, 0.0, None, 0
        n = len(pl.q)
        while True:
            k = min(n - 1, int((time.monotonic() - t0) * HZ))
            snd.q, snd.grip = list(pl.q[k]), pl.g[k]
            if pl.tag[k] != last_tag:
                last_tag = pl.tag[k]
                print(f"  {k / HZ:6.1f} s  {last_tag}")
            now = time.monotonic()
            if now - chk >= 0.2:
                chk = now
                st = get("/api/state", 1.0)
                if st.get("mode") != "engaged":
                    raise Abort("제어판 조작 상태가 꺼졌다")
                js = [float(x) for x in st["joints"]]
                err = max(abs(x - y) for x, y in zip(js, snd.q)) if all(js) else 0.0   # 관절 하나라도 0 으로 읽힌 순간은 건너뜀
                worst = max(worst, err)
                over = over + 1 if err > a.max_lag else 0
                if over >= 3:                        # 0.6 s 연속으로 뒤처질 때만 멈춤(한 번 튀는 읽기는 무시)
                    raise Abort(f"로봇이 명령보다 {err:.1f}° 뒤처진다")
            if k >= n - 1:
                break
            time.sleep(1.0 / HZ)
        time.sleep(1.0)
        st = get("/api/state", 1.0)
        print(f"끝: 최대 뒤처짐 {worst:.1f}° · TCP {st['tcp']} · 그리퍼 {st['gripper']['pos']}")
    except KeyboardInterrupt:
        print("\nCtrl+C — 보내기를 멈춘다")
    except Abort as e:
        print("멈춤:", e)
    except Exception as e:
        print("멈춤(제어판 읽기 실패):", repr(e)[:120])
    finally:
        snd.on = False
        print("보내기 끝 — 제어판이 0.3 s 뒤 그 자리에 선다")


def xy(s):
    v = [float(x) for x in s.split(",")]
    if len(v) != 2:
        raise argparse.ArgumentTypeError("x,y (mm)")
    return v


def make_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--S", type=xy, help="작은 비커 중심 x,y (기본: grasp 자리)")
    ap.add_argument("--B", type=xy, help="큰 비커 중심 x,y (기본: big 자리)")
    ap.add_argument("--phases", default="grasp,pour,place,home")
    ap.add_argument("--grasp-dz", type=float, default=0.0, help="기준 잡는 높이에서 더 내려가는 mm(음수)")
    ap.add_argument("--approach", type=float, default=80.0, help="잡기 전·놓은 뒤 위 높이 mm")
    ap.add_argument("--place-gap", type=float, default=2.0, help="내려놓을 때 잡는 높이보다 위 mm")
    ap.add_argument("--grasp-shift", type=xy, default=[0.0, 0.0], help="잡고 내려놓는 손끝 자리 옮김 dx,dy mm (베이스) — 비커의 어디를 쥐는지 맞춤")
    ap.add_argument("--pour-shift", type=xy, default=[10.0, 0.0], help="붓는 손끝 자리 옮김 dx,dy mm (베이스, +x = 로봇이 책상을 볼 때 오른쪽)")
    ap.add_argument("--z-lift", type=float, default=230.0, help="옮길 때 TCP 높이 mm")
    ap.add_argument("--open", type=int, default=900)
    ap.add_argument("--close", type=int, default=520)
    ap.add_argument("--grip-wait", type=float, default=1.5)
    ap.add_argument("--pour-hold", type=float, default=1.0)
    ap.add_argument("--vmax", type=float, default=10.0, help="관절 이동 °/s")
    ap.add_argument("--v-tilt", type=float, default=10.0, help="기울이기 끝(관절 공간) °/s")
    ap.add_argument("--j5-switch", type=float, default=12.0, help="기울이기를 직선에서 관절 공간으로 바꾸는 J5 °")
    ap.add_argument("--v-lin", type=float, default=50.0, help="옮기기 mm/s")
    ap.add_argument("--v-down", type=float, default=25.0, help="내리기·들기 mm/s")
    ap.add_argument("--v-ang", type=float, default=20.0, help="직선 구간 회전 °/s")
    ap.add_argument("--max-lag", type=float, default=8.0)
    ap.add_argument("--dry", action="store_true", help="계획만 보기")
    ap.add_argument("--udp-host", default="127.0.0.1")
    ap.add_argument("--udp-port", type=int, default=8097)
    return ap


def main():
    a = make_parser().parse_args()
    a.phases = [p.strip() for p in a.phases.split(",") if p.strip()]
    P = load()
    for k in ("home", "grasp", "big", "pour"):
        if k not in P:
            sys.exit(f"poses.json 에 '{k}' 가 없다 (pose_go.py save {k})")
    st = get("/api/state")
    q_now = [float(x) for x in st["cmd"]]
    grip_now = float(st["gripper"]["pos"]) / 1000.0
    pl, info = build(a, P, q_now, grip_now)
    out, bad = check(pl, info, np.array(P["home"], float))
    print(f"작은 비커 S {np.round(info['S'], 1)} (기준 {np.round(info['S_ref'], 1)}) · "
          f"큰 비커 B {np.round(info['B'], 1)} (기준 {np.round(info['B_ref'], 1)})")
    print(f"잡는 높이 {info['z_g']:.1f} · 옮기는 높이 {info['z_lift']:.0f} · 기울이기 직전 {info['z_pre']:.1f} mm · "
          f"붓기 TCP = B + {np.round(info['pour_off'], 1)}")
    print(f"붓기 관절 {np.round(info['q_pour'], 2)}")
    j5 = "—" if out["j5_cart"] is None else f"{out['j5_cart']:.1f}°"
    print(f"단계 {','.join(a.phases)} · {out['sec']:.1f} s · TCP 최저 {out['z_min']:.0f} mm · "
          f"J6 {out['j6'][0]:.0f}~{out['j6'][1]:.0f}° · 직선 구간 |J5| 최소 {j5} · "
          f"표본당 최대 {out['jump']:.2f}° · 직선 IK 오차 {pl.err:.3f}")
    print(f"기울이기 직전 작은 비커 바닥 – 큰 비커 테두리 {out['clear']:.0f} mm")
    if "tilt_switch" in info:
        print(f"기울이기: 직선 {info['tilt_switch'][0] * 100:.0f}% (J5 {info['tilt_switch'][1]:.1f}°) → 관절 공간으로 붓기 자세")
    if bad:
        print("보내지 않음:\n  - " + "\n  - ".join(bad))
        sys.exit(2)
    if a.dry:
        print("(--dry: 보내지 않음)")
        return
    stream(pl, a)


if __name__ == "__main__":
    main()
