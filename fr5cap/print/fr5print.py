#!/usr/bin/env python3
"""fr5print — FR5 를 프린터처럼 쓴다. 작업판(판) 좌표 위에 그린 그림 · 도형 · 글씨 · 배열 찍기 · G-code · SVG · 좌표 파일을
공구 끝으로 그대로 따라간다. 획 사이는 들어서 옮기고(빠르게), 획은 내려서 그린다(천천히).

    python3 ~/fr5cap/print/fr5print.py                 # http://localhost:8796
    python3 ~/fr5cap/print/fr5print.py --port 8797     # 다른 포트

순서: 제어판 [조작 시작] → [보내기 시작] → 제어판 /leader [리더 추종 시작] → '준비'
  1. 공구 길이(TCP → 공구 끝 mm)를 정한다.
  2. 작업판 맞추기: 조그로 공구 끝을 판 모서리 3점에 대고 기록(또는 1점 + 크기).
  3. 작업을 만든다(그리기 · 도형 · 글씨 · 사진/그림 → 선 · 배열 · 파일) → [계획 보기](로봇 안 움직임) → [시작](로봇 움직임).
     [바로 따라 그리기]: 판 그림에 획을 그릴 때마다 그 획만 계획 · 검사해 곧바로 그린다.
  멈춤: [멈춤](Space) — 그 자리에 선다 · [획 끝에서 멈춤] — 지금 획을 끝내고 들어 올린 뒤 선다 · 제어판 [정지] · 비상정지.
  멈춘 뒤 [이어서]: 멈춘 획(또는 다음 획)부터 다시 계획한다.
fr5jog 화면 · drop_run 과 동시에 보내지 않는다(모두 제어판 UDP 로 보낸다). 8793 화면이 보내는 중이면 [보내기 시작]을 거절하고,
보내는 도중 8793 이 보내기를 켜면 이쪽 보내기를 끈다.
데이터: 이 파일이 있는 폴더의 bed.json(판) · settings.json(설정) · jobs/(작업) · runs.jsonl(실행 기록).
"""
import argparse
import glob
import hashlib
import json
import math
import os
import re
import sys
import threading
import time
import urllib.request
import uuid
import xml.etree.ElementTree as ET
from http.server import ThreadingHTTPServer

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.expanduser("~/fr5cap/jog"))
import fr5jog_ui as J  # noqa: E402  (pose_go · pour_run 도 여기서 읽힌다)

PG, PR = J.PG, J.PR
VERSION = "0.2.0"
HZ = PR.HZ
BOX = {"x": (-260.0, 330.0), "y": (300.0, 720.0), "z_max": 620.0}         # 작업 상자 (TCP · 공구 끝) — fr5path 와 같음
MAX_SEC = 7200.0
MAX_LEN = 60000.0                                                         # 한 작업 길이(그리기 + 옮기기) mm
CURV_WIN, SHARP_DEG = 0.75, 30.0                                           # 그리기: 굽음을 재는 길이 mm · 이보다 크게 꺾이면 선다(°) — 글씨 모서리 가속을 정한 값 안으로
LIM = {"v_draw": (1.0, 60.0), "v_travel": (5.0, 60.0), "v_z": (2.0, 30.0), "v_touch": (0.5, 10.0), "touch_mm": (0.0, 20.0),
       "z_travel": (2.0, 100.0), "acc": (20.0, 500.0), "vj": (2.0, 30.0), "z_below": (0.0, 3.0), "air_dz": (0.0, 100.0),
       "ramp_s": (0.2, 10.0), "margin": (0.0, 50.0), "z_max": (5.0, 300.0), "tool_len": (0.0, 400.0)}
DEFAULTS = {"v_draw": 15.0, "v_travel": 40.0, "v_z": 15.0, "v_touch": 3.0, "touch_mm": 4.0, "z_travel": 15.0, "acc": 150.0,
            "vj": 15.0, "z_below": 0.0, "air_dz": 0.0, "ramp_s": 1.0, "margin": 0.0, "z_max": 150.0, "tool_len": 0.0, "back": True}


def _num(sp, k, d, lo, hi):
    try:
        v = float(sp.get(k, d))
    except (TypeError, ValueError):
        v = d
    if not math.isfinite(v):
        v = d
    return min(hi, max(lo, v))


def clean_opts(o):
    out = dict(DEFAULTS)
    for k, (lo, hi) in LIM.items():
        out[k] = _num(o or {}, k, DEFAULTS[k], lo, hi)
    out["back"] = bool((o or {}).get("back", DEFAULTS["back"]))
    return out


# ── 꺾은선 (fr5path 0.1.0 과 같은 계산) ─────────────────────────────────
def resample(P, ds):
    """꺾은선을 길이 ds 마다 다시 찍는다 (처음 · 끝 그대로)."""
    P = np.asarray(P, float)
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    keep = np.r_[True, seg > 1e-9]
    P = P[keep]
    if len(P) < 2:
        return P
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    n = max(2, int(math.ceil(s[-1] / ds)) + 1)
    t = np.linspace(0.0, s[-1], n)
    return np.column_stack([np.interp(t, s, P[:, k]) for k in range(P.shape[1])])


def smooth(P, sigma_mm, ds, closed):
    """가우스 다듬기 (손으로 그린 떨림). 열린 길은 처음 · 끝 점을 그대로 둔다."""
    if sigma_mm <= 0 or len(P) < 5:
        return P
    sg = sigma_mm / ds
    r = int(3 * sg) + 1
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / sg) ** 2)
    k /= k.sum()
    if closed:
        Q = np.vstack([P[:-1][-r:], P[:-1], P[:-1][:r]]) if len(P) - 1 > r else P
        out = np.column_stack([np.convolve(Q[:, j], k, "same") for j in range(P.shape[1])])
        if len(P) - 1 > r:
            out = out[r:r + len(P) - 1]
            return np.vstack([out, out[:1]])
        return out
    Q = np.vstack([np.repeat(P[:1], r, 0), P, np.repeat(P[-1:], r, 0)])
    out = np.column_stack([np.convolve(Q[:, j], k, "same") for j in range(P.shape[1])])[r:r + len(P)]
    out[0], out[-1] = P[0], P[-1]
    return out


def fillet(V, r, closed):
    """꺾은선 꼭짓점을 반지름 r 호로 둥글린다. 반지름은 이웃 변 길이 0.45 배 안에서. closed 면 끝에 처음 점을 붙인다."""
    V = np.asarray(V, float)
    if closed and len(V) > 1 and np.linalg.norm(V[0] - V[-1]) < 1e-9:
        V = V[:-1]
    n = len(V)
    if n < 3 or r <= 0:
        return np.vstack([V, V[:1]]) if closed else V
    out = [] if closed else [V[0]]
    for i in (range(n) if closed else range(1, n - 1)):
        p0, p1, p2 = V[i - 1], V[i], V[(i + 1) % n]
        d1, d2 = p0 - p1, p2 - p1
        l1, l2 = np.linalg.norm(d1), np.linalg.norm(d2)
        if l1 < 1e-9 or l2 < 1e-9:
            out.append(p1)
            continue
        u1, u2 = d1 / l1, d2 / l2
        ang = math.acos(max(-1.0, min(1.0, float(u1 @ u2))))
        if ang > math.pi - 1e-3:
            out.append(p1)
            continue
        t = min(r / math.tan(ang / 2), 0.45 * l1, 0.45 * l2)
        rr = t * math.tan(ang / 2)
        a = p1 + u1 * t
        bis = (u1 + u2) / np.linalg.norm(u1 + u2)
        c = p1 + bis * (rr / math.sin(ang / 2))
        e1 = (a - c) / rr
        e2 = (p1 + u2 * t - c) / rr
        phi = math.acos(max(-1.0, min(1.0, float(e1 @ e2))))
        w = e2 - (e2 @ e1) * e1
        w /= max(np.linalg.norm(w), 1e-12)
        for s in np.linspace(0.0, phi, max(3, int(phi * rr / 0.4) + 1)):
            out.append(c + rr * (math.cos(s) * e1 + math.sin(s) * w))
    if closed:
        out.append(out[0])
    else:
        out.append(V[-1])
    return np.array(out)


def plen(P):
    P = np.asarray(P, float)
    return float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1))) if len(P) > 1 else 0.0


# ── 기구학 (여러 자세 한 번에 — pour_run.fk 와 같은 DH) ─────────────────
def fk_batch(Q):
    Q = np.radians(np.atleast_2d(np.asarray(Q, float)))
    N = len(Q)
    T = np.tile(np.eye(4), (N, 1, 1))
    for i in range(6):
        ct, st = np.cos(Q[:, i]), np.sin(Q[:, i])
        ca, sa = math.cos(PG.DH_AL[i]), math.sin(PG.DH_AL[i])
        A = np.zeros((N, 4, 4))
        A[:, 0, 0], A[:, 0, 1], A[:, 0, 2], A[:, 0, 3] = ct, -st * ca, st * sa, PG.DH_A[i] * ct
        A[:, 1, 0], A[:, 1, 1], A[:, 1, 2], A[:, 1, 3] = st, ct * ca, -ct * sa, PG.DH_A[i] * st
        A[:, 2, 1], A[:, 2, 2], A[:, 2, 3] = sa, ca, PG.DH_D[i]
        A[:, 3, 3] = 1.0
        T = T @ A
    T[:, :3, 3] += T[:, :3, 2] * PG.TOOL_MM
    return T


def tip_of(q, L):
    T = PR.fk(q)
    return T[:3, 3] + T[:3, 2] * L


# ── 작업판 ─────────────────────────────────────────────────────────────
class Bed:
    """판 좌표: 원점 O(판 왼쪽 아래 모서리), X(오른쪽) · Y(안쪽 · 위) · Z(판에서 나오는 쪽) — 베이스 좌표 열벡터 R."""

    def __init__(self, d):
        self.d = d
        self.O = np.array(d["O"], float)
        self.R = np.array(d["R"], float)
        self.W, self.H = float(d["W"]), float(d["H"])

    def base(self, b):
        return self.O + np.asarray(b, float) @ self.R.T

    def bed(self, p):
        return (np.asarray(p, float) - self.O) @ self.R

    def key(self):
        return self.d.get("made", "") + f"|{self.W:.3f}|{self.H:.3f}|" + ",".join(f"{x:.3f}" for x in self.O)


def bed_three(P1, P2, P3):
    """3점 (공구 끝, 베이스 mm): 1 = 원점(왼쪽 아래) · 2 = +X 쪽(오른쪽 아래) · 3 = +Y 쪽(왼쪽 위, 위에서 볼 때 1→2 의 왼쪽)."""
    P1, P2, P3 = (np.asarray(p, float) for p in (P1, P2, P3))
    X = P2 - P1
    W = float(np.linalg.norm(X))
    if W < 20.0:
        raise ValueError(f"점 1 과 2 가 {W:.1f} mm 떨어져 있다 — 20 mm 넘게 떨어뜨린다")
    X /= W
    v = P3 - P1
    Yp = v - (v @ X) * X
    H = float(np.linalg.norm(Yp))
    if H < 20.0:
        raise ValueError(f"점 3 이 점 1→2 선에서 {H:.1f} mm 떨어져 있다 — 20 mm 넘게 (세 점이 거의 한 줄)")
    Y = Yp / H
    Z = np.cross(X, Y)
    if Z[2] < 0:
        raise ValueError("판 법선이 아래를 본다 — 위에서 볼 때 점 3 은 점 1→2 방향의 왼쪽이어야 한다 (1 왼쪽 아래 · 2 오른쪽 아래 · 3 왼쪽 위)")
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(Z[2])))))
    if tilt > 45.0:
        raise ValueError(f"판이 {tilt:.0f}° 기울었다 — 45° 까지만")
    return {"O": P1.tolist(), "R": np.column_stack([X, Y, Z]).tolist(), "W": W, "H": H, "tilt": tilt, "skew": float(v @ X)}


def bed_one(P1, W, H):
    """1점: 지금 공구 끝 = 판 왼쪽 아래 모서리. 축은 조그 위에서 보기와 같다 (오른쪽 = 베이스 −x, 위 = 로봇 쪽 −y), 판은 수평."""
    R = np.column_stack([[-1.0, 0, 0], [0, -1.0, 0], [0, 0, 1.0]])
    return {"O": np.asarray(P1, float).tolist(), "R": R.tolist(), "W": float(W), "H": float(H), "tilt": 0.0, "skew": 0.0}


# ── 글씨 (한 획 글꼴 — 폭 4 · 높이 6 칸, 대문자 · 숫자 · 기호) ─────────────
_G = {
    "A": [[(0, 0), (2, 6), (4, 0)], [(0.67, 2), (3.33, 2)]],
    "B": [[(0, 0), (0, 6), (3, 6), (4, 5), (4, 4), (3, 3), (0, 3)], [(3, 3), (4, 2), (4, 1), (3, 0), (0, 0)]],
    "C": [[(4, 5), (3, 6), (1, 6), (0, 5), (0, 1), (1, 0), (3, 0), (4, 1)]],
    "D": [[(0, 0), (0, 6), (2.5, 6), (4, 4.5), (4, 1.5), (2.5, 0), (0, 0)]],
    "E": [[(4, 6), (0, 6), (0, 0), (4, 0)], [(0, 3), (3, 3)]],
    "F": [[(4, 6), (0, 6), (0, 0)], [(0, 3), (3, 3)]],
    "G": [[(4, 5), (3, 6), (1, 6), (0, 5), (0, 1), (1, 0), (3, 0), (4, 1), (4, 3), (2, 3)]],
    "H": [[(0, 0), (0, 6)], [(4, 0), (4, 6)], [(0, 3), (4, 3)]],
    "I": [[(1, 6), (3, 6)], [(2, 6), (2, 0)], [(1, 0), (3, 0)]],
    "J": [[(4, 6), (4, 1), (3, 0), (1, 0), (0, 1)]],
    "K": [[(0, 0), (0, 6)], [(4, 6), (0, 2)], [(1.3, 3.3), (4, 0)]],
    "L": [[(0, 6), (0, 0), (4, 0)]],
    "M": [[(0, 0), (0, 6), (2, 3), (4, 6), (4, 0)]],
    "N": [[(0, 0), (0, 6), (4, 0), (4, 6)]],
    "O": [[(1, 0), (0, 1), (0, 5), (1, 6), (3, 6), (4, 5), (4, 1), (3, 0), (1, 0)]],
    "P": [[(0, 0), (0, 6), (3, 6), (4, 5), (4, 4), (3, 3), (0, 3)]],
    "Q": [[(1, 0), (0, 1), (0, 5), (1, 6), (3, 6), (4, 5), (4, 1), (3, 0), (1, 0)], [(2.5, 1.5), (4, 0)]],
    "R": [[(0, 0), (0, 6), (3, 6), (4, 5), (4, 4), (3, 3), (0, 3)], [(2, 3), (4, 0)]],
    "S": [[(4, 5), (3, 6), (1, 6), (0, 5), (0, 4), (1, 3), (3, 3), (4, 2), (4, 1), (3, 0), (1, 0), (0, 1)]],
    "T": [[(0, 6), (4, 6)], [(2, 6), (2, 0)]],
    "U": [[(0, 6), (0, 1), (1, 0), (3, 0), (4, 1), (4, 6)]],
    "V": [[(0, 6), (2, 0), (4, 6)]],
    "W": [[(0, 6), (1, 0), (2, 4), (3, 0), (4, 6)]],
    "X": [[(0, 0), (4, 6)], [(0, 6), (4, 0)]],
    "Y": [[(0, 6), (2, 3), (4, 6)], [(2, 3), (2, 0)]],
    "Z": [[(0, 6), (4, 6), (0, 0), (4, 0)]],
    "0": [[(1, 0), (0, 1), (0, 5), (1, 6), (3, 6), (4, 5), (4, 1), (3, 0), (1, 0)], [(0.4, 0.8), (3.6, 5.2)]],
    "1": [[(1, 5), (2, 6), (2, 0)], [(1, 0), (3, 0)]],
    "2": [[(0, 5), (1, 6), (3, 6), (4, 5), (4, 4), (0, 0), (4, 0)]],
    "3": [[(0, 5), (1, 6), (3, 6), (4, 5), (4, 4), (3, 3), (1.5, 3)], [(3, 3), (4, 2), (4, 1), (3, 0), (1, 0), (0, 1)]],
    "4": [[(3, 0), (3, 6), (0, 2), (4, 2)]],
    "5": [[(4, 6), (0, 6), (0, 3.5), (3, 3.5), (4, 2.5), (4, 1), (3, 0), (1, 0), (0, 1)]],
    "6": [[(4, 5), (3, 6), (1, 6), (0, 5), (0, 1), (1, 0), (3, 0), (4, 1), (4, 2.5), (3, 3.5), (0, 3.5)]],
    "7": [[(0, 6), (4, 6), (1.5, 0)]],
    "8": [[(1, 3), (0, 4), (0, 5), (1, 6), (3, 6), (4, 5), (4, 4), (3, 3), (1, 3), (0, 2), (0, 1), (1, 0), (3, 0), (4, 1), (4, 2), (3, 3)]],
    "9": [[(0, 1), (1, 0), (3, 0), (4, 1), (4, 5), (3, 6), (1, 6), (0, 5), (0, 3.5), (1, 2.5), (4, 2.5)]],
    "-": [[(1, 3), (3, 3)]],
    "+": [[(2, 1.5), (2, 4.5)], [(0.5, 3), (3.5, 3)]],
    "=": [[(0.5, 2), (3.5, 2)], [(0.5, 4), (3.5, 4)]],
    ".": [[(2, 0), (2, 0.3)]],
    ",": [[(2, 0.5), (1.5, -1)]],
    ":": [[(2, 1), (2, 1.3)], [(2, 4), (2, 4.3)]],
    "/": [[(0, 0), (4, 6)]],
    "(": [[(3, 6), (2, 5), (1.5, 3), (2, 1), (3, 0)]],
    ")": [[(1, 6), (2, 5), (2.5, 3), (2, 1), (1, 0)]],
    "_": [[(0, 0), (4, 0)]],
    "!": [[(2, 6), (2, 2)], [(2, 0), (2, 0.3)]],
    "?": [[(0, 5), (1, 6), (3, 6), (4, 5), (4, 4), (2, 2.5), (2, 1.8)], [(2, 0), (2, 0.3)]],
    "#": [[(1, 0), (1.5, 6)], [(2.5, 0), (3, 6)], [(0, 2), (4, 2)], [(0, 4), (4, 4)]],
    "*": [[(2, 1.5), (2, 4.5)], [(0.7, 2.2), (3.3, 3.8)], [(0.7, 3.8), (3.3, 2.2)]],
    "'": [[(2, 6), (2, 4.5)]],
    "%": [[(0, 0), (4, 6)], [(0.5, 5), (1, 5.5), (0.5, 6), (0, 5.5), (0.5, 5)], [(3.5, 0), (4, 0.5), (3.5, 1), (3, 0.5), (3.5, 0)]],
    " ": [],
}


def text_strokes(text, h, spacing=1.0, line_gap=1.6):
    """글씨 → 획들 (mm, 첫 줄 기준선 왼쪽 = 원점, 줄은 아래로). 모르는 글자는 빈칸으로 두고 알린다."""
    u = h / 6.0
    adv = (4.0 + 2.0 * spacing) * u
    out, miss = [], set()
    for li, line in enumerate(str(text).split("\n")):
        y0 = -li * line_gap * h
        for ci, ch in enumerate(line):
            g = _G.get(ch.upper())
            if g is None:
                miss.add(ch)
                continue
            for st in g:
                out.append(np.array([[ci * adv + x * u, y0 + y * u] for x, y in st], float))
    return out, sorted(miss)


# ── 도형 (가운데 = 원점, mm) ─────────────────────────────────────────────
def shape_strokes(sp):
    """도형 → 획들 (2D, 가운데 기준), 이름."""
    k = sp.get("kind")
    n = lambda key, d, lo, hi: _num(sp, key, d, lo, hi)  # noqa: E731
    cw = -1.0 if sp.get("cw") else 1.0
    if k == "circle":
        r = n("r", 20, 0.5, 250)
        t = np.linspace(0, 2 * math.pi, max(24, int(2 * math.pi * r / 0.4))) * cw
        return [np.c_[r * np.cos(t), r * np.sin(t)]], f"원 반지름 {r:g}"
    if k == "rect":
        w, h, c = n("w", 40, 1, 600), n("h", 30, 1, 600), n("corner", 0, 0, 100)
        V = [(0, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2), (-w / 2, -h / 2)]
        if cw < 0:
            V = [V[0]] + V[1:][::-1]
        return [fillet(np.array(V, float), c, True)], f"사각형 {w:g} × {h:g}"
    if k in ("polygon", "star"):
        m, r, c = int(n("n", 6 if k == "polygon" else 5, 3, 24)), n("r", 20, 1, 250), n("corner", 0, 0, 100)
        if k == "polygon":
            th = math.pi / 2 + 2 * math.pi * np.arange(m) / m * cw
            V, name = np.c_[r * np.cos(th), r * np.sin(th)], f"{m}각형 반지름 {r:g}"
        else:
            ri = r * n("inner", 0.45, 0.1, 0.95)
            th = math.pi / 2 + math.pi * np.arange(2 * m) / m * cw
            rad = np.where(np.arange(2 * m) % 2 == 0, r, ri)
            V, name = np.c_[rad * np.cos(th), rad * np.sin(th)], f"별 {m}꼭지 반지름 {r:g}"
        return [fillet(V, c, True)], name
    if k == "fig8":
        w, h = n("w", 60, 2, 600), n("h", 30, 1, 600)
        t = np.linspace(0, 2 * math.pi, max(60, int((w + h) * 2 / 0.4)))
        return [np.c_[w / 2 * np.sin(t) * cw, h / 2 * np.sin(2 * t)]], f"8자 {w:g} × {h:g}"
    if k == "line":
        L, ang = n("L", 40, 0.5, 600), math.radians(n("ang", 0, -360, 360))
        u = np.array([math.cos(ang), math.sin(ang)])
        return [np.array([-u * L / 2, u * L / 2])], f"선 {L:g}"
    if k == "spiral":
        r0, r1, turns = n("r0", 0, 0, 250), n("r", 20, 0.5, 250), n("turns", 4, 0.25, 60)
        tt = np.linspace(0, 2 * math.pi * turns, max(60, int(math.pi * (r0 + r1) * turns / 0.4)))
        rad = r0 + (r1 - r0) * tt / tt[-1]
        return [np.c_[rad * np.cos(tt * cw), rad * np.sin(tt * cw)]], f"나선 반지름 {r0:g} → {r1:g} · {turns:g}바퀴"
    if k == "cross":
        L = n("L", 10, 0.5, 300)
        return [np.array([[-L / 2, 0], [L / 2, 0]]), np.array([[0, -L / 2], [0, L / 2]])], f"십자 {L:g}"
    if k == "fill":
        return fill_strokes(sp)
    raise ValueError(f"모르는 도형 '{k}'")


def fill_strokes(sp):
    """채우기: 사각형 · 원 안을 지그재그 한 획으로 (간격 s, 각 a)."""
    n = lambda key, d, lo, hi: _num(sp, key, d, lo, hi)  # noqa: E731
    s, a = n("s", 2, 0.3, 50), math.radians(n("ang", 0, -360, 360))
    if sp.get("of") == "circle":
        r = n("r", 15, 1, 250)
        t = np.linspace(0, 2 * math.pi, 181)[:-1]
        V, name = np.c_[r * np.cos(t), r * np.sin(t)], f"채운 원 반지름 {r:g} · 간격 {s:g}"
    else:
        w, h = n("w", 30, 1, 600), n("h", 20, 1, 600)
        V, name = np.array([(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)], float), f"채운 사각형 {w:g} × {h:g} · 간격 {s:g}"
    c, si = math.cos(a), math.sin(a)
    Rm = np.array([[c, -si], [si, c]])
    U = V @ Rm                                  # 채우는 선을 가로로 놓는 자리 (−a 돌림)
    y0, y1 = U[:, 1].min(), U[:, 1].max()
    nl = int(math.floor((y1 - y0) / s))
    if nl < 1:
        raise ValueError("채우기 간격이 도형보다 크다")
    pts = []
    for i in range(nl + 1):
        y = y0 + (y1 - y0 - nl * s) / 2 + i * s
        xs = []
        for j in range(len(U)):
            p, q = U[j], U[(j + 1) % len(U)]
            if (p[1] - y) * (q[1] - y) <= 0 and abs(q[1] - p[1]) > 1e-12:
                xs.append(p[0] + (y - p[1]) * (q[0] - p[0]) / (q[1] - p[1]))
        if len(xs) < 2:
            continue
        xa, xb = min(xs), max(xs)
        if xb - xa < 1e-6:
            continue
        pts += [(xa, y), (xb, y)] if i % 2 == 0 else [(xb, y), (xa, y)]
    P = np.array(pts, float) @ Rm.T            # 되돌림 (+a)
    return [P], name


# ── 배열 찍기 (A1 = 원점, 열 +x · 행 −y — 위에서 본 판처럼) ─────────────────
ROWS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"


def grid_points(sp):
    rows, cols = int(_num(sp, "rows", 4, 1, 26)), int(_num(sp, "cols", 6, 1, 48))
    px, py = _num(sp, "px", 19.3, 0.5, 300), _num(sp, "py", 19.3, 0.5, 300)
    want = parse_wells(str(sp.get("wells") or ""), rows, cols)
    pts = []
    for i in range(rows):
        js = range(cols) if (sp.get("order") != "snake" or i % 2 == 0) else range(cols - 1, -1, -1)
        for j in js:
            lab = f"{ROWS[i]}{j + 1}"
            if want is None or (i, j) in want:
                pts.append((lab, np.array([j * px, -i * py])))
    if not pts:
        raise ValueError("고른 칸이 없다")
    return pts, f"배열 {rows}×{cols} · 간격 {px:g}×{py:g}" + ("" if want is None else f" · {len(pts)}칸")


def parse_wells(s, rows, cols):
    """'A1, B2-B5, C, 3' → 칸 (행, 열) 모음. 비면 None(전부)."""
    s = s.strip().upper()
    if not s:
        return None
    out = set()
    for tok in re.split(r"[,\s]+", s):
        if not tok:
            continue
        m = re.fullmatch(r"([A-Z])(\d+)(?:-([A-Z])(\d+))?", tok)
        if m:
            r0, c0 = ROWS.index(m.group(1)), int(m.group(2)) - 1
            r1, c1 = (ROWS.index(m.group(3)), int(m.group(4)) - 1) if m.group(3) else (r0, c0)
            for r in range(min(r0, r1), max(r0, r1) + 1):
                for c in range(min(c0, c1), max(c0, c1) + 1):
                    if r < rows and c < cols:
                        out.add((r, c))
            continue
        if re.fullmatch(r"[A-Z]", tok):
            r = ROWS.index(tok)
            out |= {(r, c) for c in range(cols)} if r < rows else set()
            continue
        if re.fullmatch(r"\d+", tok):
            c = int(tok) - 1
            out |= {(r, c) for r in range(rows)} if 0 <= c < cols else set()
            continue
        raise ValueError(f"칸 이름을 못 읽었다: '{tok}' (예: A1, B2-B5, C, 3)")
    return out


# ── G-code ──────────────────────────────────────────────────────────────
_WORD = re.compile(r"([A-Z])\s*([-+]?(?:\d+\.?\d*|\.\d+))")


def parse_gcode(text, z_start, v_draw, v_travel):
    """G-code (판 좌표 mm) → 요소들: ('rapid', 점들) · ('feed', 점들, mm/s) · ('grip', 값) · ('dwell', s) · ('pause', 글).
    처음 자리 = (0, 0, z_start). 지원: G0 G1 G2 G3 G4 G17 G20 G21 G90 G91 G94 · M0 M1 M2 M30 M3 M4 M5 M6 · F S P."""
    els, warn = [], []
    pos = np.array([0.0, 0.0, float(z_start)])
    mode, absm, unit, feed = 0, True, 1.0, v_draw * 60.0
    cur = None                                    # (kind, v, pts)

    def flush():
        nonlocal cur
        if cur and len(cur[2]) > 1:
            els.append(("rapid", np.array(cur[2])) if cur[0] == "rapid" else ("feed", np.array(cur[2]), cur[1]))
        cur = None

    def add_pts(kind, v, pts):
        nonlocal cur
        if cur is None or cur[0] != kind or abs(cur[1] - v) > 1e-9:
            flush()
            cur = (kind, v, [pos.copy()])
        cur[2].extend(pts)

    lines = str(text).splitlines()
    if len(lines) > 200000:
        raise ValueError("G-code 는 20만 줄까지")
    for ln, raw in enumerate(lines, 1):
        s = re.sub(r"\(.*?\)", " ", raw.split(";")[0]).strip().upper()
        if not s or s.startswith("%"):
            continue
        s = re.sub(r"^N\d+\s*", "", s)
        s = s.split("*")[0]
        words = _WORD.findall(s)
        rest = _WORD.sub(" ", s).strip()
        if rest:
            raise ValueError(f"{ln}번 줄을 못 읽었다: '{raw.strip()[:40]}'")
        W = {}
        gs, ms = [], []
        for L_, v in words:
            if L_ == "G":
                gs.append(float(v))
            elif L_ == "M":
                ms.append(int(float(v)))
            else:
                W[L_] = float(v)
        motion = None
        for g in gs:
            gi = round(g, 1)
            if gi in (0, 1, 2, 3):
                motion = int(gi)
            elif gi == 4:
                flush()
                sec = W.get("P", W.get("S", 0.0))
                if not 0 <= sec <= 60:
                    raise ValueError(f"{ln}번 줄: G4 기다리기 {sec:g} s — 0~60 s (P 또는 S = 초)")
                els.append(("dwell", float(sec)))
            elif gi == 17:
                pass
            elif gi in (18, 19):
                raise ValueError(f"{ln}번 줄: G{gi:g} 면은 지원 안 함 (G17 XY 만)")
            elif gi == 20:
                unit = 25.4
            elif gi == 21:
                unit = 1.0
            elif gi == 90:
                absm = True
            elif gi == 91:
                absm = False
            elif gi == 94:
                pass
            elif gi in (90.1, 91.1, 40, 49, 80, 54, 64, 61):
                pass
            else:
                raise ValueError(f"{ln}번 줄: G{g:g} 는 지원 안 함 (좌표를 바꾸는 명령은 거절 — G0 G1 G2 G3 G4 G17 G20 G21 G90 G91 만)")
        for m in ms:
            if m in (0, 1):
                flush()
                els.append(("pause", f"M{m} ({ln}번 줄)"))
            elif m in (2, 30):
                flush()
                return els, warn
            elif m in (3, 4):
                flush()
                if "S" not in W:
                    raise ValueError(f"{ln}번 줄: M{m} 에는 S(그리퍼 값 0~1000)가 있어야 한다")
                els.append(("grip", min(1000.0, max(0.0, W["S"]))))
            elif m == 5:
                flush()
                els.append(("grip", None))
            elif m == 6:
                flush()
                els.append(("pause", f"공구 바꾸기 M6 ({ln}번 줄)"))
            else:
                warn.append(f"{ln}번 줄 M{m} 무시")
        if motion is not None:
            mode = motion
        if "F" in W:
            if W["F"] <= 0:
                raise ValueError(f"{ln}번 줄: F 는 0 보다 커야 한다")
            feed = W["F"] * unit
        if not any(k in W for k in "XYZ"):
            if any(k in W for k in "IJR") and mode in (2, 3):
                pass
            else:
                continue
        tgt = pos.copy()
        for i, ax in enumerate("XYZ"):
            if ax in W:
                tgt[i] = W[ax] * unit if absm else pos[i] + W[ax] * unit
        if mode == 0:
            add_pts("rapid", 0.0, [tgt.copy()])
        elif mode == 1:
            add_pts("feed", min(feed / 60.0, LIM["v_draw"][1]), [tgt.copy()])
        else:
            add_pts("feed", min(feed / 60.0, LIM["v_draw"][1]), arc_pts(pos, tgt, W, unit, mode == 2, ln))
        pos = tgt
    flush()
    return els, warn


def arc_pts(p0, p1, W, unit, cw, ln, tol=0.02):
    if "R" in W:
        r = W["R"] * unit
        d = p1[:2] - p0[:2]
        q = float(np.linalg.norm(d))
        if q < 1e-9 or abs(r) < q / 2 - 1e-6:
            raise ValueError(f"{ln}번 줄: 호 반지름 R {abs(r):g} 이 두 점 거리의 절반보다 작다")
        hgt = math.sqrt(max(0.0, r * r - q * q / 4))
        mid = (p0[:2] + p1[:2]) / 2
        nrm = np.array([-d[1], d[0]]) / q
        sgn = (1.0 if cw else -1.0) * (1.0 if r > 0 else -1.0)
        c = mid - sgn * nrm * hgt
    else:
        c = p0[:2] + np.array([W.get("I", 0.0), W.get("J", 0.0)]) * unit
    r0 = float(np.linalg.norm(p0[:2] - c))
    r1 = float(np.linalg.norm(p1[:2] - c))
    if r0 < 1e-6:
        raise ValueError(f"{ln}번 줄: 호 중심이 시작점과 같다")
    if abs(r0 - r1) > max(0.05, 0.002 * r0):
        raise ValueError(f"{ln}번 줄: 호 끝점이 원 위에 있지 않다 (반지름 {r0:.3f} 대 {r1:.3f})")
    a0 = math.atan2(p0[1] - c[1], p0[0] - c[0])
    a1 = math.atan2(p1[1] - c[1], p1[0] - c[0])
    da = a1 - a0
    if cw:
        while da >= -1e-12:
            da -= 2 * math.pi
    else:
        while da <= 1e-12:
            da += 2 * math.pi
    if np.linalg.norm(p1[:2] - p0[:2]) > 1e-9 and abs(abs(da) - 2 * math.pi) < 1e-9:
        da = 0.0
    step = 2 * math.acos(max(-1.0, 1 - tol / max(r0, 1e-6)))
    n = max(2, int(math.ceil(abs(da) / max(step, 1e-3))))
    out = []
    for i in range(1, n + 1):
        t = i / n
        a = a0 + da * t
        rr = r0 + (r1 - r0) * t
        out.append(np.array([c[0] + rr * math.cos(a), c[1] + rr * math.sin(a), p0[2] + (p1[2] - p0[2]) * t]))
    out[-1] = p1.copy()
    return out


# ── SVG ─────────────────────────────────────────────────────────────────
_UNIT = {"mm": 1.0, "cm": 10.0, "in": 25.4, "pt": 25.4 / 72, "pc": 25.4 / 6, "px": 25.4 / 96, "": 25.4 / 96}
_NUM = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"


def _len_mm(s):
    m = re.fullmatch(r"\s*(" + _NUM + r")\s*([a-z%]*)\s*", s or "")
    if not m or m.group(2) not in _UNIT:
        return None
    return float(m.group(1)) * _UNIT[m.group(2)]


def _tf(s):
    """SVG transform → 3×3."""
    M = np.eye(3)
    for name, args in re.findall(r"(matrix|translate|scale|rotate|skewX|skewY)\s*\(([^)]*)\)", s or ""):
        a = [float(x) for x in re.findall(_NUM, args)]
        if name == "matrix" and len(a) == 6:
            T = np.array([[a[0], a[2], a[4]], [a[1], a[3], a[5]], [0, 0, 1]])
        elif name == "translate":
            T = np.array([[1, 0, a[0]], [0, 1, a[1] if len(a) > 1 else 0.0], [0, 0, 1]])
        elif name == "scale":
            T = np.diag([a[0], a[1] if len(a) > 1 else a[0], 1.0])
        elif name == "rotate":
            t = math.radians(a[0])
            Rr = np.array([[math.cos(t), -math.sin(t), 0], [math.sin(t), math.cos(t), 0], [0, 0, 1]])
            if len(a) == 3:
                T = np.array([[1, 0, a[1]], [0, 1, a[2]], [0, 0, 1]]) @ Rr @ np.array([[1, 0, -a[1]], [0, 1, -a[2]], [0, 0, 1]])
            else:
                T = Rr
        elif name == "skewX":
            T = np.array([[1, math.tan(math.radians(a[0])), 0], [0, 1, 0], [0, 0, 1]])
        elif name == "skewY":
            T = np.array([[1, 0, 0], [math.tan(math.radians(a[0])), 1, 0], [0, 0, 1]])
        else:
            continue
        M = M @ T
    return M


def _bez(P, n):
    t = np.linspace(0, 1, n + 1)[1:, None]
    if len(P) == 3:
        return (1 - t) ** 2 * P[0] + 2 * (1 - t) * t * P[1] + t ** 2 * P[2]
    return (1 - t) ** 3 * P[0] + 3 * (1 - t) ** 2 * t * P[1] + 3 * (1 - t) * t ** 2 * P[2] + t ** 3 * P[3]


def _svg_arc(p0, rx, ry, phi, fa, fs, p1, seg_len):
    """SVG 타원호(끝점 표기) → 점들 (p0 제외). SVG 1.1 부록 F.6.5."""
    if rx == 0 or ry == 0 or np.allclose(p0, p1):
        return [p1]
    rx, ry = abs(rx), abs(ry)
    ph = math.radians(phi)
    c, s = math.cos(ph), math.sin(ph)
    dx, dy = (p0 - p1) / 2
    x1, y1 = c * dx + s * dy, -s * dx + c * dy
    lam = x1 ** 2 / rx ** 2 + y1 ** 2 / ry ** 2
    if lam > 1:
        rx, ry = rx * math.sqrt(lam), ry * math.sqrt(lam)
    num = rx ** 2 * ry ** 2 - rx ** 2 * y1 ** 2 - ry ** 2 * x1 ** 2
    den = rx ** 2 * y1 ** 2 + ry ** 2 * x1 ** 2
    k = math.sqrt(max(0.0, num / den)) if den > 0 else 0.0
    if fa == fs:
        k = -k
    cx1, cy1 = k * rx * y1 / ry, -k * ry * x1 / rx
    cx, cy = c * cx1 - s * cy1 + (p0[0] + p1[0]) / 2, s * cx1 + c * cy1 + (p0[1] + p1[1]) / 2

    def ang(u, v):
        a = math.atan2(u[0] * v[1] - u[1] * v[0], u[0] * v[0] + u[1] * v[1])
        return a

    u = ((x1 - cx1) / rx, (y1 - cy1) / ry)
    v = ((-x1 - cx1) / rx, (-y1 - cy1) / ry)
    t1 = ang((1, 0), u)
    dt = ang(u, v)
    if not fs and dt > 0:
        dt -= 2 * math.pi
    elif fs and dt < 0:
        dt += 2 * math.pi
    n = max(4, int(math.ceil(abs(dt) * max(rx, ry) / max(seg_len, 1e-6))))
    out = []
    for i in range(1, n + 1):
        t = t1 + dt * i / n
        out.append(np.array([c * rx * math.cos(t) - s * ry * math.sin(t) + cx, s * rx * math.cos(t) + c * ry * math.sin(t) + cy]))
    out[-1] = np.array(p1, float)
    return out


def _path(d, seg):
    """SVG path d → 꺾은선들 (사용자 단위)."""
    toks = re.findall(r"[MmLlHhVvCcSsQqTtAaZz]|" + _NUM, d or "")
    out, cur, start, pos = [], None, np.zeros(2), np.zeros(2)
    i, cmd, prev_c, prev_q = 0, None, None, None

    def num():
        nonlocal i
        if i >= len(toks) or re.fullmatch(r"[A-Za-z]", toks[i]):
            raise ValueError("SVG 경로 숫자가 모자란다")
        i += 1
        return float(toks[i - 1])

    def flag():
        nonlocal i
        t = toks[i]
        if t in ("0", "1"):
            i += 1
            return int(t)
        if t[0] in "01" and len(t) > 1:              # '11' 같이 붙은 깃발
            toks[i] = t[1:]
            return int(t[0])
        return int(float(num()))

    def nb(a, b):
        return max(2, min(400, int(math.ceil((np.linalg.norm(np.diff(np.array([a] + b), axis=0), axis=1).sum()) / max(seg, 1e-6)))))

    while i < len(toks):
        if re.fullmatch(r"[A-Za-z]", toks[i]):
            cmd = toks[i]
            i += 1
        elif cmd is None:
            raise ValueError("SVG 경로가 명령으로 시작하지 않는다")
        rel = cmd.islower()
        C = cmd.upper()
        o = pos if rel else np.zeros(2)
        if C == "Z":
            if cur is not None:
                cur.append(start.copy())
                out.append(cur)
            cur, pos, prev_c, prev_q = None, start.copy(), None, None
            cmd = None if i >= len(toks) or re.fullmatch(r"[A-Za-z]", toks[i]) else ("l" if rel else "L")
            continue
        if C == "M":
            if cur is not None and len(cur) > 1:
                out.append(cur)
            pos = o + np.array([num(), num()])
            start, cur = pos.copy(), [pos.copy()]
            cmd = "l" if rel else "L"
            prev_c = prev_q = None
            continue
        if cur is None:
            cur = [pos.copy()]
        if C == "L":
            pos = o + np.array([num(), num()])
            cur.append(pos.copy())
            prev_c = prev_q = None
        elif C == "H":
            x = num()
            pos = np.array([pos[0] + x if rel else x, pos[1]])
            cur.append(pos.copy())
            prev_c = prev_q = None
        elif C == "V":
            y = num()
            pos = np.array([pos[0], pos[1] + y if rel else y])
            cur.append(pos.copy())
            prev_c = prev_q = None
        elif C in ("C", "S"):
            if C == "C":
                c1 = o + np.array([num(), num()])
            else:
                c1 = 2 * pos - prev_c if prev_c is not None else pos.copy()
            c2 = o + np.array([num(), num()])
            p = o + np.array([num(), num()])
            cur.extend(list(_bez(np.array([pos, c1, c2, p]), nb(pos, [c1, c2, p]))))
            prev_c, prev_q, pos = c2, None, p
        elif C in ("Q", "T"):
            if C == "Q":
                c1 = o + np.array([num(), num()])
            else:
                c1 = 2 * pos - prev_q if prev_q is not None else pos.copy()
            p = o + np.array([num(), num()])
            cur.extend(list(_bez(np.array([pos, c1, p]), nb(pos, [c1, p]))))
            prev_q, prev_c, pos = c1, None, p
        elif C == "A":
            rx, ry, phi = num(), num(), num()
            fa, fs = flag(), flag()
            p = o + np.array([num(), num()])
            cur.extend(_svg_arc(pos, rx, ry, phi, fa, fs, p, seg))
            pos, prev_c, prev_q = p, None, None
        else:
            raise ValueError(f"SVG 경로 명령 '{cmd}' 를 모른다")
    if cur is not None and len(cur) > 1:
        out.append(cur)
    return [np.array(c, float) for c in out]


def parse_svg(text, seg_mm=0.3):
    """SVG → 획들 (mm, 위 = +y, 왼쪽 아래 = 원점으로 옮김), 경고."""
    if len(text) > 4_000_000:
        raise ValueError("SVG 는 4 MB 까지")
    if "<!ENTITY" in text:
        raise ValueError("SVG 에 ENTITY 가 있다 — 지원 안 함")
    try:
        root = ET.fromstring(text)
    except ET.ParseError as e:
        raise ValueError(f"SVG 를 못 읽었다: {e}") from None
    strip = lambda t: t.split("}")[-1]  # noqa: E731
    if strip(root.tag) != "svg":
        raise ValueError("svg 파일이 아니다")
    vb = [float(x) for x in re.findall(_NUM, root.get("viewBox") or "")]
    wmm, hmm = _len_mm(root.get("width") or ""), _len_mm(root.get("height") or "")
    if len(vb) == 4 and vb[2] > 0 and vb[3] > 0:
        sx = (wmm / vb[2]) if wmm else _UNIT["px"]
        sy = (hmm / vb[3]) if hmm else sx
        base = np.array([[sx, 0, -vb[0] * sx], [0, sy, -vb[1] * sy], [0, 0, 1]])
    else:
        base = np.diag([_UNIT["px"], _UNIT["px"], 1.0])
    warn, out = [], []
    skip = {"defs", "clipPath", "mask", "symbol", "metadata", "title", "desc", "style", "pattern", "marker", "linearGradient", "radialGradient"}

    def ap(M, P):
        P = np.asarray(P, float)
        return (np.c_[P, np.ones(len(P))] @ M.T)[:, :2]

    def walk(el, M):
        tag = strip(el.tag)
        if tag in skip:
            return
        if (el.get("display") or "") == "none" or "display:none" in (el.get("style") or "").replace(" ", ""):
            return
        M = M @ _tf(el.get("transform"))
        sc = math.sqrt(abs(np.linalg.det(M[:2, :2]))) or 1.0
        seg = seg_mm / sc
        f = lambda k, d=0.0: float(re.findall(_NUM, el.get(k) or str(d))[0]) if re.findall(_NUM, el.get(k) or str(d)) else d  # noqa: E731
        polys = []
        if tag == "path":
            polys = _path(el.get("d"), seg)
        elif tag == "line":
            polys = [np.array([[f("x1"), f("y1")], [f("x2"), f("y2")]])]
        elif tag in ("polyline", "polygon"):
            v = [float(x) for x in re.findall(_NUM, el.get("points") or "")]
            P = np.array(v[:len(v) // 2 * 2], float).reshape(-1, 2)
            if len(P) >= 2:
                polys = [np.vstack([P, P[:1]]) if tag == "polygon" else P]
        elif tag == "rect":
            x, y, w, h = f("x"), f("y"), f("width"), f("height")
            if w > 0 and h > 0:
                polys = [np.array([[x, y], [x + w, y], [x + w, y + h], [x, y + h], [x, y]])]
                if el.get("rx") or el.get("ry"):
                    warn.append("둥근 사각형 모서리는 각지게 그린다")
        elif tag in ("circle", "ellipse"):
            cx, cy = f("cx"), f("cy")
            rx = f("r") if tag == "circle" else f("rx")
            ry = f("r") if tag == "circle" else f("ry")
            if rx > 0 and ry > 0:
                n = max(24, int(2 * math.pi * max(rx, ry) / seg))
                t = np.linspace(0, 2 * math.pi, n + 1)
                polys = [np.c_[cx + rx * np.cos(t), cy + ry * np.sin(t)]]
        elif tag in ("text", "tspan"):
            warn.append("SVG 글자는 그리지 않는다 — 경로로 바꿔(Inkscape: 객체를 경로로) 넣는다")
            return
        elif tag in ("image", "use"):
            warn.append(f"SVG <{tag}> 는 그리지 않는다")
            return
        for P in polys:
            if len(P) >= 2:
                out.append(ap(M, P))
        for ch in el:
            walk(ch, M)

    walk(root, base)
    if not out:
        raise ValueError("SVG 에 그릴 선이 없다")
    allp = np.vstack(out)
    x0, y1 = allp[:, 0].min(), allp[:, 1].max()
    strokes = [np.c_[P[:, 0] - x0, y1 - P[:, 1]] for P in out]       # 위 = +y, 왼쪽 아래 = 원점
    return strokes, sorted(set(warn))


def parse_csv(text):
    """줄마다 x, y[, z] (판 mm) · 빈 줄 = 펜 들기(새 획) · '#' 뒤는 무시."""
    strokes, cur = [], []
    for ln, line in enumerate(str(text).splitlines(), 1):
        line = line.split("#")[0].strip()
        if not line:
            if len(cur) >= 1:
                strokes.append(cur)
            cur = []
            continue
        try:
            v = [float(x) for x in re.split(r"[,\s;]+", line) if x]
        except ValueError:
            raise ValueError(f"{ln}번 줄을 못 읽었다: '{line[:40]}' (x, y 또는 x, y, z)") from None
        if len(v) == 2:
            v.append(0.0)
        if len(v) != 3:
            raise ValueError(f"{ln}번 줄을 못 읽었다: '{line[:40]}' (x, y 또는 x, y, z)")
        cur.append(v)
    if cur:
        strokes.append(cur)
    if not strokes:
        raise ValueError("좌표가 없다")
    if sum(len(s) for s in strokes) > 50000:
        raise ValueError("좌표는 5만 줄까지")
    return [np.array(s, float) for s in strokes]


# ── 작업 → 동작들 (판 좌표) ──────────────────────────────────────────────
_CACHE = {}


def _cached(kind, text, fn):
    h = hashlib.md5((kind + "\0" + text).encode()).hexdigest()
    if h not in _CACHE:
        if len(_CACHE) > 20:
            _CACHE.clear()
        _CACHE[h] = fn()
    return _CACHE[h]


def place(P, it, scale=1.0):
    """2D/3D 점들 → 판 좌표: 크기 · 돌림(원점 둘레) · 옮김 · 높이."""
    P = np.asarray(P, float)
    xy = P[:, :2] * scale
    a = math.radians(_num(it, "rot", 0, -360, 360))
    if a:
        c, s = math.cos(a), math.sin(a)
        xy = np.c_[c * xy[:, 0] - s * xy[:, 1], s * xy[:, 0] + c * xy[:, 1]]
    xy = xy + np.array([_num(it, "x", 0, -2000, 2000), _num(it, "y", 0, -2000, 2000)])
    z = (P[:, 2] if P.shape[1] > 2 else np.zeros(len(P))) + _num(it, "z", 0, -3, 300)
    return np.c_[xy, z]


def expand_item(it, idx, o):
    """작업 항목 하나 → 동작들, 이름, 경고."""
    t = it.get("type")
    lab = str(it.get("name") or "")
    ops, warn = [], []

    def stroke(P, label, v=None):
        P = np.asarray(P, float)
        if len(P) == 1:
            P = np.vstack([P, P])
        ops.append({"k": "stroke", "pts": P, "v": v, "label": label, "item": idx})

    if t == "draw":
        sm = _num(it, "smooth", 1, 0, 20)
        for j, S in enumerate(it.get("strokes") or []):
            P = np.asarray(S, float)
            if P.ndim != 2 or P.shape[1] < 2 or len(P) < 1:
                continue
            P = P[:, :2]
            if len(P) >= 2 and plen(P) > 1.0:
                P = resample(P, 0.5)
                P = smooth(P, sm, 0.5, False)
            stroke(place(P, it), f"그림 획 {j + 1}")
        name = f"그림 {len(ops)}획"
    elif t == "shape":
        S, name = shape_strokes(it)
        for j, P in enumerate(S):
            stroke(place(P, it), name + (f" {j + 1}" if len(S) > 1 else ""))
    elif t == "text":
        h = _num(it, "h", 10, 1, 300)
        S, miss = text_strokes(str(it.get("text") or ""), h, _num(it, "spacing", 1.0, 0, 5))
        if miss:
            warn.append("못 쓰는 글자(건너뜀): " + " ".join(miss))
        for P in S:
            stroke(place(P, it), "글씨 '" + str(it.get("text"))[:12] + "'")
        name = f"글씨 '{str(it.get('text'))[:16]}' 높이 {h:g}"
    elif t == "grid":
        pts, name = grid_points(it)
        act = {"down": _num(it, "dz", 0, -3, 100), "g1": it.get("g1"), "dwell": _num(it, "dwell", 0.5, 0, 60), "g2": it.get("g2")}
        for g in ("g1", "g2"):
            if act[g] in ("", None):
                act[g] = None
            else:
                act[g] = _num(act, g, 0, 0, 1000)
        for lab2, p in pts:
            P = place(np.array([[p[0], p[1]]]), it)[0]
            P[2] += act["down"]
            ops.append({"k": "point", "p": P, "act": act, "label": lab2, "item": idx})
    elif t == "gcode":
        txt = str(it.get("text") or "")
        els, w = _cached("g", txt + f"|{o['z_travel']}|{o['v_draw']}", lambda: parse_gcode(txt, o["z_travel"], o["v_draw"], o["v_travel"]))
        warn += w
        part, k = [], 0
        name = "G-code " + (lab or f"{len(txt.splitlines())}줄")

        def close(last):
            nonlocal part, k
            if part:
                ops.append({"k": "prog", "els": part, "first": k == 0, "last": last, "label": f"{name} 부분 {k + 1}", "item": idx})
                k += 1
            part = []

        for e in els:
            if e[0] == "pause":
                close(False)
                ops.append({"k": "pause", "msg": e[1], "item": idx, "label": e[1]})
            elif e[0] in ("rapid", "feed"):
                P = place(e[1], it)
                part.append((e[0], P) + ((e[2],) if e[0] == "feed" else ()))
            else:
                part.append(e)
        close(True)
        if ops and ops[-1]["k"] == "prog":
            ops[-1]["last"] = True
    elif t == "svg":
        txt = str(it.get("text") or "")
        S, w = _cached("s", txt, lambda: parse_svg(txt))
        warn += w
        sc = _num(it, "scale", 1.0, 0.01, 100)
        allp = np.vstack(S)
        for j, P in enumerate(S):
            stroke(place(P, it, sc), f"SVG 획 {j + 1}")
        name = "SVG " + (lab + " " if lab else "") + f"{allp[:, 0].max() * sc:.0f} × {allp[:, 1].max() * sc:.0f} mm ({len(S)}획)"
    elif t == "csv":
        txt = str(it.get("text") or "")
        S = _cached("c", txt, lambda: parse_csv(txt))
        for j, P in enumerate(S):
            stroke(place(P, it), f"좌표 획 {j + 1}")
        name = "좌표 " + (lab + " " if lab else "") + f"{len(S)}획"
    elif t == "action":
        a = it.get("act")
        if a == "grip":
            v = _num(it, "v", 500, 0, 1000)
            ops.append({"k": "grip", "v": v, "item": idx, "label": f"그리퍼 → {v:g}"})
            name = f"그리퍼 → {v:g}"
        elif a == "dwell":
            s = _num(it, "s", 1, 0, 600)
            ops.append({"k": "dwell", "s": s, "item": idx, "label": f"기다리기 {s:g} s"})
            name = f"기다리기 {s:g} s"
        else:
            msg = str(it.get("msg") or "확인 뒤 [이어서]")[:80]
            ops.append({"k": "pause", "msg": msg, "item": idx, "label": "멈춤: " + msg})
            name = "멈춤: " + msg
    else:
        raise ValueError(f"모르는 작업 항목 '{t}'")
    return ops, (lab + " · " if lab and t not in ("gcode", "svg", "csv") else "") + name, warn


def op_pts(op):
    """동작의 판 좌표 점들 (미리 보기 · 범위 검사)."""
    if op["k"] == "stroke":
        return op["pts"]
    if op["k"] == "point":
        return op["p"][None, :]
    if op["k"] == "prog":
        P = [e[1] for e in op["els"] if e[0] in ("rapid", "feed")]
        return np.vstack(P) if P else np.zeros((0, 3))
    return np.zeros((0, 3))


def expand_job(job, bed_wh, o):
    """작업 → 동작들 · 항목 요약 · 검사(판 밖 · 높이)."""
    items = job.get("items") or []
    if len(items) > 400:
        raise ValueError("작업 항목은 400 개까지")
    ops, info, warn, bad = [], [], [], []
    for i, it in enumerate(items):
        try:
            o_i, name, w = expand_item(it, i, o)
        except ValueError as e:
            bad.append(f"항목 {i + 1}: {e}")
            info.append({"name": f"(오류) {e}", "n": 0, "len": 0})
            continue
        warn += [f"항목 {i + 1}: {x}" for x in w]
        L = sum(plen(op["pts"]) for op in o_i if op["k"] == "stroke") + sum(plen(e[1]) for op in o_i if op["k"] == "prog" for e in op["els"] if e[0] == "feed")
        info.append({"name": name, "n": len(o_i), "len": round(L, 1)})
        ops += o_i
    W, H = bed_wh
    m = o["margin"]
    out_xy, low, high = [], [], []
    for j, op in enumerate(ops):
        P = op_pts(op)
        if not len(P):
            continue
        if W is not None and ((P[:, 0] < -m - 1e-6).any() or (P[:, 0] > W + m + 1e-6).any() or (P[:, 1] < -m - 1e-6).any() or (P[:, 1] > H + m + 1e-6).any()):
            out_xy.append(j)
        if (P[:, 2] < -o["z_below"] - 1e-6).any():
            low.append(j)
        if (P[:, 2] > o["z_max"] + 1e-6).any():
            high.append(j)
    if out_xy:
        bad.append(f"판 밖으로 나가는 동작 {len(out_xy)}개 (항목 " + ", ".join(sorted({str(ops[j]['item'] + 1) for j in out_xy})) + ")")
    if low:
        bad.append(f"판 아래로 {o['z_below']:g} mm 넘게 내려가는 동작 {len(low)}개 (항목 " + ", ".join(sorted({str(ops[j]['item'] + 1) for j in low})) + ")")
    if high:
        bad.append(f"판 위 {o['z_max']:g} mm 보다 높은 동작 {len(high)}개")
    if len(ops) > 5000:
        bad.append(f"동작이 {len(ops)}개 — 5000 개까지")
    return ops, info, warn, bad, {"out": out_xy, "low": low}


def ops_json(ops, step_mm=0.5):
    """미리 보기용 (판 좌표, 줄인 점)."""
    out = []
    for j, op in enumerate(ops):
        d = {"k": op["k"], "label": op.get("label", ""), "item": op.get("item")}
        if op["k"] == "stroke":
            P = op["pts"]
            if len(P) > 2 and plen(P) / max(1, len(P)) < step_mm:
                P = resample(P, step_mm)
            d["pts"] = np.round(P, 2).tolist()
        elif op["k"] == "point":
            d["p"] = np.round(op["p"], 2).tolist()
        elif op["k"] == "prog":
            d["els"] = [[e[0], np.round(resample(e[1], step_mm) if len(e[1]) > 2 else e[1], 2).tolist()] for e in op["els"] if e[0] in ("rapid", "feed")]
        elif op["k"] == "pause":
            d["msg"] = op["msg"]
        out.append(d)
    return out


# ── 계획 ────────────────────────────────────────────────────────────────
def _curv(P, ds, win=1.5):
    n = len(P)
    k = max(1, int(round(win / ds)))
    kap = np.zeros(n)
    if n > 2 * k:
        v1 = P[k:n - k] - P[:n - 2 * k]
        v2 = P[2 * k:] - P[k:n - k]
        c = np.sum(v1 * v2, 1) / np.maximum(np.linalg.norm(v1, axis=1) * np.linalg.norm(v2, axis=1), 1e-12)
        kap[k:n - k] = np.arccos(np.clip(c, -1, 1)) / (k * ds)
    return kap


def _timing(s, vv, ds, acc):
    """fr5path 0.1.0 과 같다: 속도 표본 → 50 Hz 길이 자리 (선 곳마다 끊어 가우스로 무르게)."""
    vm = vv[:-1] + vv[1:]
    dt = np.where(vm > 1e-6, 2 * ds / np.maximum(vm, 1e-6), math.sqrt(4 * ds / acc))
    t = np.r_[0.0, np.cumsum(dt)]
    sig = 0.08 * HZ
    m = int(round(3 * sig))
    ker = np.exp(-0.5 * (np.arange(-m, m + 1) / sig) ** 2)
    ker /= ker.sum()
    stops = np.where(vv <= 1e-9)[0]
    out = [np.array([s[0]])]
    for a, b in zip(stops[:-1], stops[1:]):
        if b <= a:
            continue
        tk = np.arange(t[a], t[b], 1.0 / HZ)
        seg = np.r_[np.interp(tk, t, s), s[b]]
        sm = np.convolve(np.r_[np.full(2 * m, s[a]), seg, np.full(2 * m, s[b])], ker, "valid")
        sm = np.clip(np.maximum.accumulate(sm), s[a], s[b])
        i0 = max(0, int(np.argmax(sm > s[a] + 1e-6)) - 1)
        i1 = len(sm) - max(0, int(np.argmax(sm[::-1] < s[b] - 1e-6)) - 1)
        sm = sm[i0:max(i1, i0 + 2)]
        sm[0], sm[-1] = s[a], s[b]
        out.append(sm[1:])
    return np.concatenate(out)


class Planner:
    """공구 끝(판 좌표) 꺾은선 → 관절 50 Hz. 손 방향 R 고정, 공구 끝 = TCP + R[:, 2] · L."""

    def __init__(self, q0, g0, R, bed, L, o):
        self.pl = PR.Plan(q0, g0)
        self.R, self.bed, self.L, self.o = R, bed, L, o
        self.toff = R[:, 2] * L
        self.cur = bed.bed(PR.fk(q0)[:3, 3] + self.toff)
        self.want = []                                    # (k0, k1, 원하는 공구 끝 베이스 점들)
        self.travel = []                                  # 판 좌표 옮기기 꺾은선
        self.len_draw = self.len_travel = 0.0
        self.stops = 0
        self.ik_err = 0.0

    @property
    def k(self):
        return len(self.pl.q)

    def move(self, pts_b, v, tag):
        """지금 자리에서 pts_b(판 좌표, 첫 점 = 지금 자리 근처)를 따라간다."""
        P = np.asarray(pts_b, float)
        if np.linalg.norm(P[0] - self.cur) > 1e-6:
            P = np.vstack([self.cur, P])
        L = plen(P)
        if L < 0.05:
            self.cur = P[-1].copy()
            return
        draw = tag in ("draw", "feed")
        ds = 0.5 if draw else 1.5
        ds = min(2.0, max(ds, L / 4000.0))
        Tb = self.bed.base(P) - self.toff                  # TCP 목표
        Pd = resample(Tb, ds)
        ds = float(np.linalg.norm(Pd[1] - Pd[0])) if len(Pd) > 1 else ds
        Qd = [np.array(self.pl.last, float)]
        errp = erra = 0.0
        q, ep, ea = PR.ik(Qd[0], Pd[0], self.R)
        Qd[0] = q
        errp, erra = max(errp, ep), max(erra, ea)
        for p in Pd[1:]:
            q, ep, ea = PR.ik(Qd[-1], p, self.R)
            Qd.append(q)
            errp, erra = max(errp, ep), max(erra, ea)
        Qd = np.array(Qd)
        o = self.o
        acc, vj = o["acc"], o["vj"]
        kap = _curv(Pd, ds, CURV_WIN if draw else 1.5)
        dq = np.max(np.abs(np.diff(Qd, axis=0)), axis=1) / ds
        jr = np.r_[dq[:1], np.maximum(dq[:-1], dq[1:]), dq[-1:]] if len(dq) > 1 else np.r_[dq, dq]
        vlim = np.minimum.reduce([np.full(len(Pd), v), np.sqrt(acc / np.maximum(kap, 1e-9)), vj / np.maximum(jr, 1e-9)])
        k1 = max(1, int(round(0.8 / ds)))
        if len(Pd) > 2 * k1:
            u1 = Pd[k1:-k1] - Pd[:-2 * k1]
            u2 = Pd[2 * k1:] - Pd[k1:-k1]
            cs = np.sum(u1 * u2, 1) / np.maximum(np.linalg.norm(u1, axis=1) * np.linalg.norm(u2, axis=1), 1e-12)
            sharp = np.where(cs < math.cos(math.radians(SHARP_DEG if draw else 40.0)))[0] + k1
            vlim[sharp] = 0.0
            self.stops += int(np.sum(np.diff(np.r_[-10, sharp]) > 2 * k1)) if len(sharp) else 0
        vv = vlim.copy()
        vv[0] = 0.0
        for i in range(1, len(vv)):
            vv[i] = min(vv[i], math.sqrt(vv[i - 1] ** 2 + 2 * acc * ds))
        vv[-1] = 0.0
        for i in range(len(vv) - 2, -1, -1):
            vv[i] = min(vv[i], math.sqrt(vv[i + 1] ** 2 + 2 * acc * ds))
        s = np.arange(len(Pd)) * ds
        sk = _timing(s, vv, ds, acc)
        Qk = np.column_stack([np.interp(sk, s, Qd[:, j]) for j in range(6)])
        k0 = self.k
        g = self.pl.g[-1]
        for qq in Qk:
            self.pl.q.append(qq)
            self.pl.g.append(g)
            self.pl.tag.append(tag)
            self.pl.cart.append(len(self.pl.q) - 1)
        Pk = np.column_stack([np.interp(sk, s, Pd[:, j]) for j in range(3)]) + self.toff
        self.want.append((k0, self.k, Pk))
        self.ik_err = max(self.ik_err, errp, erra / 10.0)
        self.pl.err = max(self.pl.err, errp, erra / 10.0)
        if draw:
            self.len_draw += L
        else:
            self.len_travel += L
            self.travel.append(np.round(P, 2).tolist())
        self.cur = P[-1].copy()

    def hold(self, sec, tag="hold"):
        if sec > 0:
            self.pl.hold(sec, None, tag)

    def ramp(self, g1, sec, tag="grip"):
        g0 = self.pl.g[-1]
        if abs(g1 - g0) < 1e-6:
            return
        n = max(1, int(round(sec * HZ)))
        for i in range(1, n + 1):
            self.pl.q.append(self.pl.last.copy())
            self.pl.g.append(g0 + (g1 - g0) * i / n)
            self.pl.tag.append(tag)

    def go_to(self, s_b, z_t):
        """지금 자리 → s_b: 옮기는 높이 이상으로 곧게 들고 · 그 높이로 옮기고 · 곧게 내린다(닿기 직전 천천히)."""
        o = self.o
        s_b = np.asarray(s_b, float)
        c = self.cur
        if np.linalg.norm(c - s_b) < 0.3:
            return
        h = max(z_t, c[2], s_b[2])
        if c[2] < h - 0.3:
            self.move([c, [c[0], c[1], h]], o["v_z"], "lift")
        if np.linalg.norm(self.cur[:2] - s_b[:2]) > 0.3:
            self.move([self.cur, [s_b[0], s_b[1], h]], o["v_travel"], "travel")
        if s_b[2] < self.cur[2] - 0.3:
            zt = s_b[2] + o["touch_mm"]
            if zt < self.cur[2] - 0.3:
                self.move([self.cur, [s_b[0], s_b[1], zt]], o["v_z"], "down")
            self.move([self.cur, s_b], o["v_touch"], "touch")
        elif np.linalg.norm(self.cur - s_b) > 1e-6:
            self.move([self.cur, s_b], o["v_z"], "down")

    def lift(self, z_t):
        if self.cur[2] < z_t - 0.3:
            self.move([self.cur, [self.cur[0], self.cur[1], z_t]], self.o["v_z"], "lift")


def plan_job(q0, g0, R, bed, L, ops, o, home, start_op=0):
    """동작들 → (계획, 동작별 표본 범위, 요약, 미리 보기). start_op 부터."""
    t_start = time.perf_counter()
    dz = o["air_dz"]
    z_t = o["z_travel"] + dz
    pn = Planner(q0, g0, R, bed, L, o)
    start_tip_b = pn.cur.copy()
    ranges = []                                            # (k0, k1, 동작 번호, 종류)
    for j in range(start_op, len(ops)):
        op = ops[j]
        k0 = pn.k
        if op["k"] == "pause":
            ranges.append((k0, k0, j, "pause"))
            continue
        if op["k"] == "grip":
            pn.ramp(op["v"] / 1000.0, o["ramp_s"])
        elif op["k"] == "dwell":
            pn.hold(op["s"], "dwell")
        elif op["k"] == "stroke":
            P = op["pts"] + np.array([0, 0, dz])
            pn.go_to(P[0], z_t)
            pn.move(P, op["v"] or o["v_draw"], "draw")
            pn.lift(z_t)
        elif op["k"] == "point":
            p = op["p"] + np.array([0, 0, dz])
            pn.go_to(p, z_t)
            a = op["act"]
            if a.get("g1") is not None:
                pn.ramp(a["g1"] / 1000.0, o["ramp_s"])
            pn.hold(a.get("dwell", 0.0), "dwell")
            if a.get("g2") is not None:
                pn.ramp(a["g2"] / 1000.0, o["ramp_s"])
            pn.lift(z_t)
        elif op["k"] == "prog":
            first = True
            for e in op["els"]:
                if e[0] in ("rapid", "feed"):
                    P = e[1] + np.array([0, 0, dz])
                    if first:                              # 프로그램 자리까지는 안전하게 (이미 거기면 안 움직임)
                        pn.go_to(P[0], z_t)
                    first = False
                    pn.move(P, o["v_travel"] if e[0] == "rapid" else min(e[2], LIM["v_draw"][1]), "rapid" if e[0] == "rapid" else "feed")
                elif e[0] == "grip":
                    pn.ramp((e[1] if e[1] is not None else g0 * 1000.0) / 1000.0, o["ramp_s"])
                elif e[0] == "dwell":
                    pn.hold(e[1], "dwell")
            if op.get("last", True):
                pn.lift(z_t)
        ranges.append((k0, pn.k, j, "op"))
    k_back = pn.k
    back_mode = "none"
    if o["back"] and start_tip_b[2] >= z_t - 0.5:          # 처음 자리가 옮기는 높이 이상일 때만 돌아간다
        pn.go_to(start_tip_b, z_t)
        pn.pl.joint(q0, 5.0, "settle")
        back_mode = "start"
    else:                                                   # 처음 자리가 판 가까이(멈춘 자리 등)거나 '돌아가기' 끔 → 마지막 자리 위로 들어 둔다
        pn.lift(z_t)
        back_mode = "lift"
    pl = pn.pl
    # ── 검사 ──
    out, bad = PR.check(pl, J.CHECK, home)
    Q = np.array(pl.q)
    T = fk_batch(Q)
    TCP = T[:, :3, 3]
    TIP = TCP + T[:, :3, 2] * L
    TB = bed.bed(TIP)
    zfloor = min(-o["z_below"], start_tip_b[2]) - 0.05
    if TB[:, 2].min() < zfloor:
        bad.append(f"공구 끝이 판보다 {-TB[:, 2].min():.1f} mm 아래로 간다 (허용 {o['z_below']:g} mm)")
    near = TB[:, 2] < z_t - 0.5
    mask = np.zeros(len(Q), bool)
    for k0, k1, j, kind in ranges:
        if kind == "op":
            mask[k0:k1] = True
    if ranges:
        first_op = next((r for r in ranges if r[3] == "op"), None)
        if first_op is not None:                          # 첫 동작 앞 '지금 자리에서 들기'는 뺀다
            k_lift_end = first_op[0]
            for t_i in range(first_op[0], first_op[1]):
                if pl.tag[t_i] != "lift":
                    break
                k_lift_end = t_i + 1
            mask[first_op[0]:k_lift_end] = False
    m = o["margin"] + 1.0
    outside = mask & near & ((TB[:, 0] < -m) | (TB[:, 0] > bed.W + m) | (TB[:, 1] < -m) | (TB[:, 1] > bed.H + m))
    if outside.any():
        bad.append(f"판 가까이(옮기는 높이 아래)에서 판 밖으로 나간다 — {int(outside.sum())} 표본")
    for nm, pts in (("TCP", TCP), ("공구 끝", TIP)):
        ox = (pts[:, 0].min() < BOX["x"][0]) or (pts[:, 0].max() > BOX["x"][1])
        oy = (pts[:, 1].min() < BOX["y"][0]) or (pts[:, 1].max() > BOX["y"][1])
        oz = pts[:, 2].max() > BOX["z_max"]
        if ox or oy or oz:
            bad.append(f"{nm}이 작업 상자 밖으로 나간다 (x {pts[:, 0].min():.0f}~{pts[:, 0].max():.0f} · y {pts[:, 1].min():.0f}~{pts[:, 1].max():.0f} · z 최고 {pts[:, 2].max():.0f})")
    if TIP[:, 2].min() < PR.TABLE_Z - 1.0:
        bad.append(f"공구 끝이 책상({PR.TABLE_Z:g} mm)보다 낮아진다 ({TIP[:, 2].min():.1f} mm)")
    dev = 0.0
    for k0, k1, Pk in pn.want:
        if k1 > k0:
            dev = max(dev, float(np.max(np.linalg.norm(TIP[k0:k1] - Pk, axis=1))))
    if dev > 0.5:
        bad.append(f"공구 끝이 정한 길에서 {dev:.2f} mm 벗어난다 (IK)")
    js = np.abs(np.diff(Q, axis=0)) * HZ if len(Q) > 1 else np.zeros((1, 6))
    jmax = float(js.max())
    if jmax > max(o["vj"], 30.0) * 1.2:
        bad.append(f"관절 속도 {jmax:.0f}°/s — 너무 빠르다")
    vt = np.linalg.norm(np.diff(TIP, axis=0), axis=1) * HZ if len(TIP) > 1 else np.zeros(1)
    sec = len(Q) / HZ
    if sec > MAX_SEC:
        bad.append(f"{sec / 60:.0f} 분 — {MAX_SEC / 60:.0f} 분 넘는 작업은 나눠서")
    if pn.len_draw + pn.len_travel > MAX_LEN:
        bad.append(f"길이 {(pn.len_draw + pn.len_travel) / 1000:.0f} m — {MAX_LEN / 1000:.0f} m 까지")
    touch = np.isin(np.array(pl.tag[1:]), ["touch"])
    v_touch_peak = float(vt[touch].max()) if touch.any() else 0.0
    zax = -R[:, 2]
    ang_n = math.degrees(math.acos(max(-1.0, min(1.0, float(zax @ bed.R[:, 2])))))
    n_kind = {}
    for op in ops[start_op:]:
        n_kind[op["k"]] = n_kind.get(op["k"], 0) + 1
    summ = {"ops": len(ops) - start_op, "start_op": start_op, "n_total": len(ops), "kinds": n_kind,
            "sec": round(sec, 1), "len_draw": round(pn.len_draw, 1), "len_travel": round(pn.len_travel, 1),
            "v_peak": round(float(vt.max()), 1), "v_touch_peak": round(v_touch_peak, 2), "j_peak": round(jmax, 1),
            "dev_mm": round(dev, 3), "ik_mm": round(pn.ik_err, 3), "stops": pn.stops,
            "z_bed": [round(float(TB[:, 2].min()), 2), round(float(TB[:, 2].max()), 1)],
            "tcp_z": [round(float(TCP[:, 2].min()), 1), round(float(TCP[:, 2].max()), 1)],
            "j5_min": None if out.get("j5_cart") is None else round(out["j5_cart"], 1),
            "j6": [round(out["j6"][0], 1), round(out["j6"][1], 1)], "tool_vs_normal": round(ang_n, 1),
            "air_dz": dz, "z_travel": o["z_travel"], "tool_len": L, "back": o["back"], "back_mode": back_mode,
            "plan_s": round(time.perf_counter() - t_start, 2), "bad": bad}
    stride = max(1, int(math.ceil(len(Q) / 20000)))
    prev = {"anim": np.round(TB[::stride], 1).tolist(), "anim_hz": HZ / stride, "stride": stride,
            "ranges": [[int(k0 // stride), int(k1 // stride), int(j), kind] for k0, k1, j, kind in ranges],
            "travel": pn.travel, "k_back": int(k_back // stride)}
    return pl, ranges, summ, prev


# ── 화면 · 실행 ─────────────────────────────────────────────────────────
class PrintApp(J.App):
    def __init__(self, a):
        super().__init__(a)
        self.dir = os.path.abspath(os.path.expanduser(a.dir))
        os.makedirs(os.path.join(self.dir, "jobs"), exist_ok=True)
        self.o = clean_opts(self._load("settings.json") or {})
        self.pivot = self.o["tool_len"]
        bd = self._load("bed.json")
        self.bed = Bed(bd) if bd else None
        self.marks = {}
        self.jp = None                                    # 마지막 계획
        self.jr = {"state": "idle"}                       # 실행 상태
        self.speed_r = 1.0
        self.pause_req = False
        self.jog8793 = None                               # 조그 화면(8793): None 꺼짐 · False 켜짐 · True 보내는 중
        threading.Thread(target=self._watch_jog, daemon=True).start()

    def _watch_jog(self):
        """조그 화면(8793)도 보내기를 켜면 이쪽 보내기를 끈다 — 보내는 곳은 하나만."""
        op = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        while True:
            time.sleep(0.5)
            try:
                st = json.loads(op.open("http://127.0.0.1:8793/api/status", timeout=0.4).read())
                self.jog8793 = bool(st.get("send"))
            except Exception:
                self.jog8793 = None
                continue
            if self.jog8793 and self.snd is not None:
                self.abort = True
                t0 = time.monotonic()
                while self.busy and time.monotonic() - t0 < 2.0:
                    time.sleep(0.05)
                with self.lock:
                    self.release("조그 화면(8793)도 보내기를 켰다 — fr5print 보내기를 껐다 (보내는 곳은 하나만)")

    # 파일
    def _load(self, name):
        p = os.path.join(self.dir, name)
        try:
            return json.load(open(p, encoding="utf-8")) if os.path.exists(p) else None
        except (OSError, ValueError):
            return None

    def _save(self, name, obj):
        p = os.path.join(self.dir, name)
        tmp = p + ".tmp"
        json.dump(obj, open(tmp, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        os.replace(tmp, p)

    def start_send(self):
        try:
            op = urllib.request.build_opener(urllib.request.ProxyHandler({}))
            st = json.loads(op.open("http://127.0.0.1:8793/api/status", timeout=0.4).read())
            if st.get("send"):
                return "조그 화면(8793)이 보내는 중이다 — 거기서 [보내기 끝] 을 누른 뒤"
        except Exception:
            pass
        return super().start_send()

    def set_opts(self, b):
        o = dict(self.o)
        o.update({k: v for k, v in b.items() if k in DEFAULTS})
        self.o = clean_opts(o)
        if not self.busy:
            self.pivot = self.o["tool_len"]
        self._save("settings.json", self.o)

    def tip_now(self):
        q, g, R = self._now()
        return tip_of(q, self.o["tool_len"]), q, R

    def _now(self):
        if self.snd is not None and self.q is not None:
            return np.array(self.q, float), float(self.g), (self.R0 if self.R0 is not None else PR.fk(self.q)[:3, :3])
        st = PG.get("/api/state", 1.0)
        q = np.array([float(x) for x in st["cmd"]])
        return q, float(st["gripper"]["pos"]) / 1000.0, PR.fk(q)[:3, :3]

    # 판
    def bed_mark(self, i):
        err = self.ready()
        if err:
            return err
        if i not in (1, 2, 3):
            return "점 번호는 1 · 2 · 3"
        p, q, R = self.tip_now()
        self.marks[i] = {"q": [round(float(x), 4) for x in q], "tip": [round(float(x), 2) for x in p], "L": self.o["tool_len"],
                         "R": [[round(float(x), 5) for x in r] for r in R]}
        self.say(f"판 점 {i}: 공구 끝 ({p[0]:.1f}, {p[1]:.1f}, {p[2]:.1f}) · 공구 길이 {self.o['tool_len']:g}")
        return None

    def bed_solve(self, b):
        mode = b.get("mode")
        L = self.o["tool_len"]
        if mode == "three":
            if any(i not in self.marks for i in (1, 2, 3)):
                return "점 1 · 2 · 3 을 다 기록한다", {}
            Ls = {self.marks[i]["L"] for i in (1, 2, 3)}
            if len(Ls) > 1:
                return "점마다 공구 길이가 다르다 — 같은 공구 길이로 다시 기록한다", {}
            Rs = [np.array(self.marks[i]["R"]) for i in (1, 2, 3)]
            dR = max(math.degrees(np.linalg.norm(PR.rotvec(Rs[0].T @ Ri))) for Ri in Rs[1:])
            try:
                d = bed_three(*(self.marks[i]["tip"] for i in (1, 2, 3)))
            except ValueError as e:
                return str(e), {}
            d["marks"] = [self.marks[i] for i in (1, 2, 3)]
            d["tool_dR"] = round(dR, 2)
            Lt = Ls.pop()
        elif mode == "one":
            err = self.ready()
            if err:
                return err, {}
            W, H = _num(b, "w", 200, 20, 600), _num(b, "h", 150, 20, 600)
            p, q, R = self.tip_now()
            d = bed_one(p, W, H)
            d["marks"] = [{"q": [round(float(x), 4) for x in q], "tip": [round(float(x), 2) for x in p], "L": L}]
            Lt = L
        else:
            return "mode 는 three · one", {}
        d.update({"mode": mode, "made": time.strftime("%Y-%m-%dT%H:%M:%S"), "tool_len": Lt, "W": round(d["W"], 2), "H": round(d["H"], 2)})
        old = os.path.join(self.dir, "bed.json")
        if os.path.exists(old):
            os.replace(old, os.path.join(self.dir, "bed.json.bak_" + time.strftime("%Y%m%d_%H%M%S")))
        self._save("bed.json", d)
        self.bed = Bed(d)
        self.jp = None
        self.say(f"판 저장: {d['W']:.1f} × {d['H']:.1f} mm · 기울기 {d['tilt']:.1f}°" + (f" · 세 점 손 방향 차 {d.get('tool_dR', 0):.1f}°" if mode == "three" else ""))
        return None, {"bed": self.bed_info()}

    def bed_info(self):
        if not self.bed:
            return None
        d = self.bed.d
        return {"mode": d.get("mode"), "W": d["W"], "H": d["H"], "tilt": round(d.get("tilt", 0.0), 2), "skew": round(d.get("skew", 0.0), 1),
                "made": d.get("made"), "tool_len": d.get("tool_len"), "tool_dR": d.get("tool_dR"), "O": [round(x, 1) for x in d["O"]],
                "R": d["R"]}

    # 작업
    def expand(self, job):
        W, H = (self.bed.W, self.bed.H) if self.bed else (None, None)
        ops, info, warn, bad, flags = expand_job(job, (W, H), self.o)
        return ops, {"ops": ops_json(ops), "items": info, "warn": warn[:30], "bad": bad, "n": len(ops), "flags": flags}

    def plan(self, b):
        if self.busy:
            return "움직이는 중", {}
        if not self.bed:
            return "작업판을 먼저 맞춘다", {}
        job = b.get("job") or {}
        start = int(_num(b, "start_op", 0, 0, 100000))
        try:
            q, g, R = self._now()
        except Exception as e:
            return f"제어판을 못 읽었다 ({repr(e)[:60]})", {}
        try:
            ops, ex = self.expand(job)
        except ValueError as e:
            return str(e), {}
        if not ops:
            return "작업이 비었다", {}
        if start >= len(ops):
            return f"시작 동작 {start + 1} 이 없다 (모두 {len(ops)}개)", {}
        if ex["bad"]:
            return "작업 검사: " + " · ".join(ex["bad"]), {"expand": ex}
        L = self.o["tool_len"]
        o = dict(self.o)
        if b.get("no_back"):                               # 바로 따라 그리기: 획마다 처음 자리로 돌아가지 않는다
            o["back"] = False
        pl, ranges, summ, prev = plan_job(q, g, R, self.bed, L, ops, o, self.home, start)
        bl = self.bed.d.get("tool_len")
        if bl is not None and abs(bl - L) > 0.05:
            summ["warn_tool"] = f"판을 맞출 때 공구 길이 {bl:g} mm · 지금 {L:g} mm — 공구를 바꿨으면 새 길이가 정확해야 하고, 아니면 판을 다시 맞춘다"
        pid = uuid.uuid4().hex[:8]
        self.jp = {"id": pid, "q": q, "R": R, "L": L, "bed": self.bed.key(), "pl": pl, "ranges": ranges, "ops": ops, "sum": summ,
                   "job": job, "name": str(job.get("name") or "작업"), "o": dict(self.o)}
        self.say(f"계획 {pid}: 동작 {summ['ops']}개 (시작 {start + 1}) · {summ['sec'] / 60:.1f} 분" + (" · 안 됨: " + " · ".join(summ["bad"]) if summ["bad"] else ""))
        return None, {"id": pid, "sum": summ, "prev": prev, "expand": ex}

    def run(self, pid):
        err = self.ready()
        if err:
            return err
        P = self.jp
        if not P or P["id"] != pid:
            return "계획이 없다 — [계획 보기]를 먼저"
        if P["sum"]["bad"]:
            return "보내지 않음: " + " · ".join(P["sum"]["bad"])
        if float(np.max(np.abs(np.array(self.q, float) - P["q"]))) > 0.05:
            return "계획한 뒤 로봇이 움직였다 — [계획 보기]를 다시"
        if abs(P["L"] - self.o["tool_len"]) > 1e-6 or not self.bed or P["bed"] != self.bed.key() or P["o"] != self.o:
            return "계획한 뒤 공구 길이 · 판 · 설정이 바뀌었다 — [계획 보기]를 다시"
        self.busy, self.abort, self.pause_req = True, False, False
        self.phase = "움직이는 중"
        n = len(P["pl"].q)
        self.jr = {"state": "running", "id": pid, "k": 0, "n": n, "op": P["sum"]["start_op"], "n_ops": P["sum"]["n_total"],
                   "start_op": P["sum"]["start_op"], "t0": time.time(), "t": 0.0, "sec": P["sum"]["sec"], "name": P["name"],
                   "resume_op": None, "msg": "", "lag": 0.0, "stride": max(1, int(math.ceil(n / 20000)))}
        threading.Thread(target=self._work, args=(P,), daemon=True).start()
        self.say(f"작업 시작: {P['name']} · 동작 {P['sum']['start_op'] + 1}–{P['sum']['n_total']} · {P['sum']['sec'] / 60:.1f} 분" + (f" · 허공 +{P['o']['air_dz']:g} mm" if P["o"]["air_dz"] else ""))
        return None

    def _op_at(self, ranges, k):
        for k0, k1, j, kind in ranges:
            if kind == "op" and k0 <= k < k1:
                return j
        prior = [j for k0, k1, j, kind in ranges if k1 <= k]
        return (prior[-1] + 1) if prior else (ranges[0][2] if ranges else 0)

    def _loop(self, P):
        pl, ranges = P["pl"], P["ranges"]
        Q, G = np.array(pl.q), np.array(pl.g)
        n = len(Q)
        ends = [(k1, j) for k0, k1, j, kind in ranges if kind == "op"]
        marks = [(k0, j, P["ops"][j]["msg"]) for k0, k1, j, kind in ranges if kind == "pause"]
        tv, last, chk, over, k_prev = 0.0, time.monotonic(), 0.0, 0, 0
        stop = None                                        # (k, 이어서 할 동작, 글)
        worst = 0.0
        while True:
            if self.abort:
                raise PR.Abort("멈춤 — 그 자리에 섰다")
            if self.snd is None:
                raise PR.Abort("보내기가 꺼졌다")
            now = time.monotonic()
            tv += (now - last) * self.speed_r
            last = now
            x = tv * HZ
            k = min(n - 1, int(x))
            f = x - int(x)
            if stop is None:
                for kp, j, msg in marks:
                    if k_prev <= kp <= k:
                        stop = (kp, j + 1, msg)
                        break
            if stop is None and self.pause_req:
                nxt = [(k1, j) for k1, j in ends if k1 >= k]
                stop = (nxt[0][0] - 1, nxt[0][1] + 1, "획 끝에서 멈춤") if nxt else None
                self.pause_req = False
            if stop is not None and k >= stop[0]:
                ks = max(0, min(n - 1, stop[0]))
                self.snd.q, self.snd.grip = list(Q[ks]), float(G[ks])
                self.q, self.g = Q[ks].copy(), float(G[ks])
                self.jr.update({"k": ks, "op": self._op_at(ranges, ks)})
                worst = max(worst, self._settle(Q[ks]))
                return "paused", stop[1], stop[2], worst
            if k >= n - 1:
                self.snd.q, self.snd.grip = list(Q[-1]), float(G[-1])
                self.q, self.g = Q[-1].copy(), float(G[-1])
                self.jr.update({"k": n - 1})
                worst = max(worst, self._settle(Q[-1]))
                return "done", None, "", worst
            q = Q[k] + (Q[k + 1] - Q[k]) * f
            g = float(G[k] + (G[k + 1] - G[k]) * f)
            self.snd.q, self.snd.grip = list(q), g
            self.q, self.g = q, g
            self.jr.update({"k": k, "op": self._op_at(ranges, k), "t": round(time.time() - self.jr["t0"], 1)})
            if now - chk >= 0.2:
                chk = now
                st = PG.get("/api/state", 1.0)
                if st.get("mode") != "engaged":
                    raise PR.Abort("제어판 조작 상태가 꺼졌다")
                js = [float(v) for v in st["joints"]]
                err = max(abs(a - b) for a, b in zip(js, self.snd.q)) if all(js) else 0.0
                worst = max(worst, err)
                self.jr["lag"] = round(err, 2)
                over = over + 1 if err > self.a.max_lag else 0
                if over >= 3:
                    raise PR.Abort(f"로봇이 명령보다 {err:.1f}° 뒤처진다")
            k_prev = k
            time.sleep(1.0 / HZ)

    def _settle(self, qf, tol=1.0, tmax=3.0):
        """끝 · 멈춤 자리: 로봇이 마지막 명령에 tol° 안으로 올 때까지 (못 오면 '끝'이라 하지 않는다)."""
        t0, err = time.monotonic(), 0.0
        while True:
            if self.abort:
                raise PR.Abort("멈춤 — 그 자리에 섰다")
            if self.snd is None:
                raise PR.Abort("보내기가 꺼졌다")
            st = PG.get("/api/state", 1.0)
            if st.get("mode") != "engaged":
                raise PR.Abort("제어판 조작 상태가 꺼졌다")
            js = [float(v) for v in st["joints"]]
            err = max(abs(a - b) for a, b in zip(js, qf)) if all(js) else 0.0
            self.jr["lag"] = round(err, 2)
            if err <= tol:
                return err
            if time.monotonic() - t0 > tmax:
                raise PR.Abort(f"끝 자리에서 로봇이 명령보다 {err:.1f}° 뒤처져 있다 ({tmax:g} s 기다림)")
            time.sleep(0.1)

    def _work(self, P):
        rec = {"start": time.strftime("%Y-%m-%dT%H:%M:%S"), "name": P["name"], "plan": P["id"], "start_op": P["sum"]["start_op"],
               "n_ops": P["sum"]["n_total"], "air_dz": P["o"]["air_dz"], "tool_len": P["L"]}
        try:
            st, nxt, msg, worst = self._loop(P)
            self.R0 = P["R"]
            if st == "done":
                self.jr.update({"state": "done", "resume_op": None, "msg": "끝"})
                self.say(f"작업 끝: {P['name']}")
            else:
                done = nxt >= P["sum"]["n_total"]
                self.jr.update({"state": "done" if done else "paused", "resume_op": None if done else nxt, "msg": msg})
                self.say(f"멈춤({msg}) — 다음은 동작 {nxt + 1}" if not done else "작업 끝 (마지막 동작 뒤 멈춤)")
            rec.update({"end": st, "worst_lag": round(worst, 2)})
        except PR.Abort as e:
            self.R0 = PR.fk(self.q)[:3, :3]
            op = self.jr.get("op") or P["sum"]["start_op"]
            self.jr.update({"state": "stopped", "resume_op": op, "msg": str(e)})
            self.say(f"작업 멈춤: {e} — [이어서] 는 동작 {op + 1} 처음부터")
            rec.update({"end": "stopped", "why": str(e)})
        except Exception as e:
            self.R0 = PR.fk(self.q)[:3, :3]
            op = self.jr.get("op") or P["sum"]["start_op"]
            self.jr.update({"state": "stopped", "resume_op": op, "msg": f"제어판 읽기 실패 {repr(e)[:60]}"})
            self.say(f"작업 멈춤 (제어판 읽기 실패 {repr(e)[:60]})")
            rec.update({"end": "error", "why": repr(e)[:120]})
        finally:
            self.busy, self.abort, self.pause_req = False, False, False
            self.phase = "준비" if self.snd is not None else "대기"
            rec.update({"stop": time.strftime("%Y-%m-%dT%H:%M:%S"), "op": self.jr.get("op"), "k": self.jr.get("k"), "n": self.jr.get("n")})
            try:
                with open(os.path.join(self.dir, "runs.jsonl"), "a", encoding="utf-8") as fo:
                    fo.write(json.dumps(rec, ensure_ascii=False) + "\n")
            except OSError:
                pass

    def corners(self, b):
        if not self.bed:
            return "작업판을 먼저 맞춘다", {}
        z = _num(b, "z", 3, 0.5, 50)
        W, H = self.bed.W, self.bed.H
        items = [{"type": "grid", "rows": 1, "cols": 1, "x": x, "y": y, "dz": z, "dwell": 1.0, "name": nm}
                 for nm, x, y in (("왼쪽 아래 (원점)", 0, 0), ("오른쪽 아래", W, 0), ("오른쪽 위", W, H), ("왼쪽 위", 0, H), ("가운데", W / 2, H / 2))]
        return self.plan({"job": {"name": f"모서리 확인 (판 위 {z:g} mm)", "items": items}})

    def jobs_list(self):
        out = []
        for p in sorted(glob.glob(os.path.join(self.dir, "jobs", "*.json")), key=os.path.getmtime, reverse=True):
            out.append({"name": os.path.basename(p)[:-5], "t": time.strftime("%m-%d %H:%M", time.localtime(os.path.getmtime(p)))})
        return out[:100]

    def job_save(self, name, job):
        name = re.sub(r"[^\w가-힣.-]+", "_", str(name or "").strip())[:60]
        if not name:
            return "이름이 비었다"
        self._save(os.path.join("jobs", name + ".json"), dict(job, name=name))
        self.say(f"작업 저장: {name}")
        return None

    def job_load(self, name):
        name = re.sub(r"[^\w가-힣.-]+", "_", str(name or ""))[:60]
        d = self._load(os.path.join("jobs", name + ".json"))
        return ("그 작업이 없다", None) if d is None else (None, d)

    def pstatus(self):
        st = self.status()
        try:
            tip = tip_of(self.q, self.o["tool_len"]) if self.q is not None else None
        except Exception:
            tip = None
        jr = dict(self.jr)
        jr.pop("t0", None)
        if jr.get("state") == "running":
            jr["remain"] = round((jr["n"] - jr["k"]) / HZ / max(self.speed_r, 1e-3), 1)
        return {"v": VERSION, "jog_v": J.VERSION, "jog": st, "o": self.o, "bed": self.bed_info(),
                "tip": None if tip is None else [round(float(x), 2) for x in tip],
                "tip_bed": None if (tip is None or not self.bed) else [round(float(x), 2) for x in self.bed.bed(tip)],
                "marks": {str(k): v["tip"] for k, v in self.marks.items()}, "marks_L": {str(k): v["L"] for k, v in self.marks.items()},
                "plan_id": self.jp["id"] if self.jp else None, "run": jr, "speed": self.speed_r, "pause_req": self.pause_req,
                "lim": LIM, "table_z": PR.TABLE_Z, "jog8793": self.jog8793}


class PrH(J.H):
    def do_GET(self):
        if self.path == "/" or self.path.startswith("/?"):
            return self._send(200, PAGE, "text/html")
        if self.path == "/jog" or self.path.startswith("/jog?"):
            return self._send(200, J.PAGE, "text/html")
        if self.path.startswith("/api/pstatus"):
            return self._send(200, json.dumps(self.app.pstatus()))
        if self.path.startswith("/api/jobs"):
            return self._send(200, json.dumps({"jobs": self.app.jobs_list()}))
        return super().do_GET()

    def do_POST(self):
        A = self.app
        p = self.path
        mine = ("/api/opts", "/api/bed/mark", "/api/bed/solve", "/api/expand", "/api/plan", "/api/run", "/api/pause", "/api/speed",
                "/api/corners", "/api/job/save", "/api/job/load", "/api/import")
        if p not in mine:
            return super().do_POST()
        try:
            n = int(self.headers.get("Content-Length") or 0)
            if n > 8_000_000:
                return self._send(200, json.dumps({"err": "요청이 너무 크다 (8 MB 까지)"}))
            b = json.loads(self.rfile.read(n) or b"{}")
            err, extra = None, {}
            if p == "/api/expand":                         # 미리 보기는 잠금 없이 (계획과 겹쳐도 됨)
                try:
                    _, extra = A.expand(b.get("job") or {})
                except ValueError as e:
                    err = str(e)
                return self._send(200, json.dumps({"err": err, **extra}))
            if p == "/api/pause":
                if A.busy and A.jr.get("state") == "running":
                    A.pause_req = True
                    A.say("획 끝에서 멈춤 — 지금 동작을 끝내고 들어 올린 뒤 선다")
                else:
                    err = "작업이 돌고 있지 않다"
                return self._send(200, json.dumps({"err": err}))
            if p == "/api/speed":
                A.speed_r = _num(b, "r", 1.0, 0.1, 1.0)
                return self._send(200, json.dumps({"err": None, "speed": A.speed_r}))
            with A.lock:
                if p == "/api/opts":
                    if A.busy:
                        err = "움직이는 중에는 설정을 바꾸지 않는다"
                    else:
                        A.set_opts(b)
                        extra = {"o": A.o}
                elif p == "/api/bed/mark":
                    err = A.bed_mark(int(_num(b, "i", 0, 0, 9)))
                elif p == "/api/bed/solve":
                    err, extra = A.bed_solve(b)
                elif p == "/api/plan":
                    err, extra = A.plan(b)
                elif p == "/api/run":
                    err = A.run(str(b.get("id", "")))
                elif p == "/api/corners":
                    err, extra = A.corners(b)
                elif p == "/api/job/save":
                    err = A.job_save(b.get("name"), b.get("job") or {})
                elif p == "/api/job/load":
                    err, d = A.job_load(b.get("name"))
                    extra = {"job": d}
                elif p == "/api/import":
                    err, extra = do_import(b, A.o)
            self._send(200, json.dumps({"err": err, **extra}))
        except Exception as e:
            self._send(200, json.dumps({"err": f"서버 오류: {repr(e)[:160]}"}))


def do_import(b, o):
    """파일 → 작업 항목 (검사만, 항목에는 원문을 담는다)."""
    try:
        return _import(b, o)
    except ValueError as e:
        return str(e), {}


def _import(b, o):
    fmt = str(b.get("fmt") or "").lower()
    text = str(b.get("text") or "")
    name = str(b.get("name") or "")[:40]
    if fmt in ("gcode", "nc", "ngc", "gc", "tap"):
        els, w = parse_gcode(text, o["z_travel"], o["v_draw"], o["v_travel"])
        n = sum(1 for e in els if e[0] in ("rapid", "feed"))
        return None, {"item": {"type": "gcode", "text": text, "name": name, "x": 0, "y": 0}, "info": f"G-code 이동 {n}묶음", "warn": w}
    if fmt == "svg":
        S, w = parse_svg(text)
        allp = np.vstack(S)
        return None, {"item": {"type": "svg", "text": text, "name": name, "x": 0, "y": 0, "scale": 1.0},
                      "info": f"SVG {len(S)}획 · {allp[:, 0].max():.1f} × {allp[:, 1].max():.1f} mm", "warn": w}
    if fmt in ("csv", "txt"):
        S = parse_csv(text)
        return None, {"item": {"type": "csv", "text": text, "name": name, "x": 0, "y": 0}, "info": f"좌표 {len(S)}획", "warn": []}
    return "파일 종류는 gcode · svg · csv", {}


PAGE = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>fr5print 프린터</title><style>
:root{--bg:#f4f3ef;--card:#fff;--ink:#1d1d1b;--mute:#6b6a64;--line:#dddbd3;--tcp:#cc79a7;--piv:#e69f00;--b:#0072b2;--warn:#d55e00;--ok:#009e73;--soft:#ecebe5;--sky:#56b4e9;--done:#009e73}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#1b1b1a;--card:#262624;--ink:#ecebe6;--mute:#a3a29b;--line:#3a3a37;--soft:#30302d}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.45 system-ui,"Noto Sans KR",sans-serif}
header{position:sticky;top:0;z-index:5;padding:10px 16px;background:var(--card);border-bottom:1px solid var(--line);display:flex;gap:10px;align-items:center;flex-wrap:wrap}
header b{font-size:17px}.chip{padding:2px 9px;border-radius:10px;border:1px solid var(--line);font-size:12px;color:var(--mute);text-decoration:none;white-space:nowrap}
.chip.on{color:#fff;background:var(--ok);border-color:var(--ok)}.chip.warn{color:#fff;background:var(--warn);border-color:var(--warn)}.chip.blue{color:#fff;background:var(--b);border-color:var(--b)}
#band{flex:1 1 240px;font-weight:600;min-width:180px}
main{display:grid;grid-template-columns:minmax(0,0.95fr) minmax(0,1.2fr);gap:14px;padding:14px 16px;max-width:1600px;margin:0 auto}
@media (max-width:1100px){main{grid-template-columns:minmax(0,1fr)}}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:14px;margin-bottom:14px}
h3{margin:0 0 10px;font-size:13px;color:var(--mute);font-weight:600;display:flex;gap:8px;align-items:center;flex-wrap:wrap}h3 .k{margin-left:auto;font-weight:400}
h3 .n{display:inline-flex;width:20px;height:20px;border-radius:50%;background:var(--ink);color:var(--card);align-items:center;justify-content:center;font-size:12px}
button{padding:7px 11px;border:1px solid var(--line);border-radius:8px;background:var(--bg);color:var(--ink);cursor:pointer;font:inherit;transition:background .12s,border-color .12s}
button:hover:not(:disabled){border-color:var(--mute)}button:disabled{opacity:.4;cursor:default}
button.go{background:var(--ok);color:#fff;border-color:var(--ok);font-weight:600}button.stop{background:var(--warn);color:#fff;border-color:var(--warn);font-weight:700;padding:8px 16px}
button.pri{background:var(--b);color:#fff;border-color:var(--b);font-weight:600}button.run{background:var(--warn);color:#fff;border-color:var(--warn);font-weight:700}
button.on{background:var(--ink);color:var(--card);border-color:var(--ink)}button.sm{padding:3px 8px;font-size:12px}
.chips{display:flex;gap:5px;flex-wrap:wrap}.chips button{padding:5px 10px;font-size:13px;border-radius:14px}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin:8px 0}.mute{color:var(--mute);font-size:12px}.sp{flex:1}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(118px,1fr));gap:8px 10px}
.fld{display:flex;flex-direction:column;gap:3px;font-size:12px;color:var(--mute)}
input[type=number],input[type=text],textarea,select{padding:6px 8px;border:1px solid var(--line);border-radius:6px;background:var(--bg);color:var(--ink);font:inherit;width:100%;min-width:0}
textarea{font:12px/1.4 ui-monospace,Menlo,monospace;height:90px}
.tabs{display:flex;gap:4px;flex-wrap:wrap;margin-bottom:10px;border-bottom:1px solid var(--line);padding-bottom:8px}.tabs button{border-radius:8px 8px 0 0;padding:6px 12px}
.pane{display:none}.pane.on{display:block}
#bed{width:100%;height:min(52vh,560px);min-height:300px;border-radius:8px;background:var(--soft);display:block;touch-action:none}
@media (min-width:1101px){#side{position:sticky;top:60px;align-self:start;max-height:calc(100vh - 68px);overflow:auto}}
@media (max-width:1100px){#side{order:-1}}
.legend{display:flex;gap:12px;flex-wrap:wrap;font-size:12px;color:var(--mute);margin-top:6px}.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:4px;vertical-align:middle}
.bar{display:inline-block;width:18px;height:3px;margin-right:4px;vertical-align:middle;border-radius:2px}
table{width:100%;border-collapse:collapse;font-size:13px}td{padding:5px 4px;border-top:1px solid var(--line);vertical-align:middle}td.n{color:var(--mute);width:34%}td.r{font-variant-numeric:tabular-nums}
.items td{padding:4px}.items input{width:64px;padding:3px 5px;font-size:12px}.items .nm{font-size:13px}.items tr.sel{background:color-mix(in srgb,var(--b) 8%,transparent)}
.bad{color:var(--warn);font-weight:600;margin:6px 0 0;padding-left:18px}.good{color:var(--ok);font-weight:600;margin-top:8px}.warnl{color:var(--piv);margin:6px 0 0;padding-left:18px;font-size:12px}
pre{white-space:pre-wrap;margin:0;font:12px/1.45 ui-monospace,Menlo,monospace;color:var(--mute);max-height:200px;overflow:auto}
.prog{position:relative;height:10px;background:var(--soft);border-radius:6px;overflow:hidden;margin-top:10px}.prog i{position:absolute;left:0;top:0;bottom:0;background:var(--b);width:0}
#modal{position:fixed;inset:0;background:rgba(0,0,0,.35);display:none;align-items:center;justify-content:center;z-index:9}#modal .box{background:var(--card);border-radius:12px;padding:18px;max-width:560px;width:92%;border:1px solid var(--line)}
.busy #bStop{animation:pulse 1s infinite}@keyframes pulse{50%{box-shadow:0 0 0 6px rgba(213,94,0,.25)}}
kbd{font:11px ui-monospace,Menlo,monospace;border:1px solid var(--line);border-bottom-width:2px;border-radius:4px;padding:0 4px;color:var(--mute)}
.pad{display:grid;grid-template-columns:repeat(3,52px);grid-template-rows:repeat(3,44px);gap:5px}.pad button,.zc button{padding:2px;font-size:17px;line-height:1;display:flex;flex-direction:column;align-items:center;justify-content:center}
.pad button small,.zc button small{font-size:10px;color:var(--mute);margin-top:2px}.zc{display:grid;grid-template-rows:repeat(2,68px);gap:5px;width:58px}
.jogwrap{display:flex;gap:14px;flex-wrap:wrap;align-items:center}
.mk{display:grid;grid-template-columns:auto 1fr auto;gap:4px 10px;align-items:center;font-size:13px}.mk .c{font:12px ui-monospace,Menlo,monospace;color:var(--mute)}
.stale{outline:2px dashed var(--piv);outline-offset:2px}
.air{color:#fff;background:var(--sky);border-color:var(--sky)}
.xyz{font:600 15px ui-monospace,Menlo,monospace}
</style></head><body>
<header><b>fr5print</b><span class="chip" id="cPanel">제어판</span><span class="chip" id="cFol">추종</span><span class="chip" id="cSend">보내기</span>
<span class="chip warn" id="cJog" style="display:none">8793 조그 화면이 보내는 중</span>
<span class="mute" id="ver"></span><a class="chip" href="/jog" target="_blank">조그 화면 ↗</a><div id="band">—</div>
<button class="go" id="bSend" onclick="post('/api/send',{on:true})">보내기 시작</button><button id="bRel" onclick="post('/api/send',{on:false})">보내기 끝</button>
<button class="stop" id="bStop" onclick="stopAll()">멈춤 <kbd style="color:#fff;border-color:#fff8">Space</kbd></button></header>
<main><section>
<div class="card"><h3><span class="n">1</span>공구 · 작업판 <span class="k mute" id="bedLine">—</span></h3>
<div class="row"><span>공구 길이</span><input type="number" id="toolLen" step="0.5" min="0" max="400" style="width:90px"> mm
<span class="chips"><button onclick="$('toolLen').value=0;saveTool()">TCP 0</button><button onclick="$('toolLen').value=225;saveTool()">스포이드 끝 225</button></span><button onclick="saveTool()">적용</button></div>
<div class="mute">TCP 에서 공구 축을 따라 공구 끝까지. 판을 맞출 때와 작업할 때 같은 공구 · 같은 손 방향(곧게)을 쓰면 길이 오차가 판 높이에서 지워진다.</div>
<div class="tabs" style="margin-top:12px" id="bedTabs"><button data-bt="three" class="on">3점으로 맞추기</button><button data-bt="one">1점 + 크기</button></div>
<div class="pane on" id="bt_three"><div class="mk" id="marks"></div>
<div class="row"><button class="pri" onclick="solve('three')">세 점으로 판 저장</button><span class="mute">1 → 2 가 판 X(오른쪽), 3 쪽이 Y(위). 위에서 볼 때 3 은 1→2 의 왼쪽.</span></div></div>
<div class="pane" id="bt_one"><div class="row"><span>폭 X</span><input type="number" id="bw" value="200" min="20" max="600" style="width:80px"> mm <span>높이 Y</span><input type="number" id="bh" value="150" min="20" max="600" style="width:80px"> mm</div>
<div class="row"><button class="pri" onclick="solve('one')">지금 공구 끝 = 판 왼쪽 아래 모서리로 저장</button></div><div class="mute">판은 수평으로 본다. X = 화면 오른쪽(베이스 −x), Y = 로봇 쪽(−y) — 조그 버튼 방향과 같다.</div></div>
<div class="row"><button id="bCorner" onclick="corners()">모서리 확인 ⚠</button><span class="mute">판 위</span><input type="number" id="cz" value="3" min="0.5" max="50" step="0.5" style="width:70px"><span class="mute">mm 에서 네 모서리 · 가운데를 1 초씩 — 판을 바르게 맞췄는지 눈으로 본다</span></div>
<div style="border-top:1px solid var(--line);margin-top:10px;padding-top:10px">
<div class="row" style="margin-top:0"><b style="font-size:13px">조그</b><span class="mute">공구 끝 <span class="xyz" id="tipB">—</span></span><span class="sp"></span><span class="mute"><kbd>←</kbd><kbd>→</kbd><kbd>↑</kbd><kbd>↓</kbd> <kbd>w</kbd><kbd>s</kbd> · 걸음 <kbd>1</kbd>–<kbd>7</kbd></span></div>
<div class="jogwrap"><div class="pad"><span></span><button data-j="0,-1,0" data-k="ArrowUp">↑<small>로봇 쪽</small></button><span></span>
<button data-j="1,0,0" data-k="ArrowLeft">←<small>+x</small></button><span class="mute" style="display:flex;align-items:center;justify-content:center;font-size:10px;text-align:center">베이스<br>방향</span><button data-j="-1,0,0" data-k="ArrowRight">→<small>−x</small></button>
<span></span><button data-j="0,1,0" data-k="ArrowDown">↓<small>카메라</small></button><span></span></div>
<div class="zc"><button data-j="0,0,1" data-k="w">▲<small>위</small></button><button data-j="0,0,-1" data-k="s">▼<small>아래</small></button></div>
<div style="flex:1;min-width:200px"><div class="mute">한 걸음 (mm)</div><div class="chips" id="steps" style="margin:4px 0 8px"></div>
<div class="row" style="margin:0"><button data-v="1">손 곧게</button><button onclick="goHome()">초기자세로</button></div>
<div class="row"><span class="mute">그리퍼</span><b id="gv">—</b><span class="chips"><button data-gd="-50">−50</button><button data-gd="-10">−10</button><button data-gd="10">+10</button><button data-gd="50">+50</button></span><input type="number" id="gIn" min="0" max="1000" style="width:70px"><button onclick="grip(+$('gIn').value)">적용</button></div></div></div></div></div>

<div class="card"><h3><span class="n">2</span>작업 만들기 <span class="k mute" id="placeHint">판 그림을 눌러 자리를 고른다</span></h3>
<div class="tabs" id="mkTabs"><button data-mt="draw" class="on">그리기</button><button data-mt="shape">도형</button><button data-mt="text">글씨</button><button data-mt="image">사진 · 그림</button><button data-mt="grid">배열 찍기</button><button data-mt="file">파일</button><button data-mt="act">동작</button></div>
<div class="pane on" id="mt_draw"><div class="mute">오른쪽 판 그림 위에서 끌어 그린다 — 한 번 끌기 = 한 획(펜을 들었다 내림).</div>
<div class="row"><span class="chips"><button id="dmFree" class="on" onclick="setDM('free')">자유</button><button id="dmPoly" onclick="setDM('poly')">꺾은선 (눌러 찍고 두 번 눌러 끝)</button></span></div>
<div class="grid"><div class="fld"><span>다듬기 mm</span><input type="number" id="dSmooth" value="1" min="0" max="20" step="0.5"></div><div class="fld"><span>높이 z (판 위 mm)</span><input type="number" id="dZ" value="0" min="-3" max="100" step="0.5"></div></div>
<div class="row"><button onclick="draft.pop();draftPoly=null;store();updDraft();redraw()">마지막 획 지우기</button><button onclick="draft=[];draftPoly=null;liveDone=[];liveCut=[];store();updDraft();redraw()">비우기</button><span class="sp"></span><span class="mute" id="draftInfo"></span><button class="go" onclick="addDraw()">작업에 넣기</button></div>
<div class="row" style="border-top:1px solid var(--line);padding-top:10px"><button class="run" id="bLive" onclick="liveToggle()">바로 따라 그리기 ⚠</button><span class="mute" id="liveInfo"></span></div></div>
<div class="pane" id="mt_image"><div class="row"><input type="file" id="iIn" accept="image/png,image/jpeg,image/webp,image/bmp,image/gif" style="max-width:100%"></div>
<div class="row"><span class="chips"><button data-im="center" class="on">중심선</button><button data-im="outline">윤곽선</button><button data-im="hatch">채우기 선</button><button data-im="both">둘 다</button></span><label><input type="checkbox" id="iInv"> 밝은 쪽을 그림</label></div>
<div class="grid"><div class="fld"><span>문턱 1–254 (빈칸 = 자동)</span><input type="number" id="iTh" min="1" max="254" placeholder="자동"></div><div class="fld"><span>폭 mm</span><input type="number" id="iW" value="80" min="5" max="600" step="1"></div>
<div class="fld"><span>채우기 간격 mm</span><input type="number" id="iS" value="1.5" min="0.3" max="20" step="0.1"></div><div class="fld"><span>다듬기 px</span><input type="number" id="iTol" value="0.8" min="0.2" max="5" step="0.1"></div>
<div class="fld"><span>짧은 선 버림 mm</span><input type="number" id="iMin" value="1" min="0" max="50" step="0.5"></div><div class="fld"><span>정밀도 mm/px</span><input type="number" id="iRes" value="0.1" min="0.03" max="1" step="0.01"></div><div class="fld"><span>티 지우기 mm²</span><input type="number" id="iSpeck" value="0.3" min="0" max="50" step="0.1"></div><div class="fld"><span>왼쪽 아래 x</span><input type="number" id="ix" value="10" step="0.5"></div>
<div class="fld"><span>왼쪽 아래 y</span><input type="number" id="iy" value="10" step="0.5"></div><div class="fld"><span>높이 z</span><input type="number" id="iz" value="0" step="0.5" min="-3" max="100"></div></div><div class="grid" id="iCenterOpts"><div class="fld"><span>펜 굵기 mm</span><input type="number" id="iPen" value="0.5" min="0" max="10" step="0.1"></div><div class="fld"><span>잔가지 버림 mm</span><input type="number" id="iSpur" value="1" min="0" max="20" step="0.1"></div><div class="fld"><span>이음 mm (펜 안 들고 잇기)</span><input type="number" id="iJoin" value="0.3" min="0" max="5" step="0.1"></div></div><div class="row"><label><input type="checkbox" id="iExt" checked> 선 끝까지 (펜 굵기 빼고)</label><label><input type="checkbox" id="iOrd" checked> 순서 정리 (옮기기 줄이기)</label><label><input type="checkbox" id="iDot" checked> 굵은 점 채우기 (나선)</label></div>
<div class="row" style="align-items:flex-start"><canvas id="iPrev" width="640" height="428" title="누르면 크게 · 작게" onclick="this.style.width=this.style.width==='100%'?'320px':'100%'" style="background:var(--soft);border-radius:6px;max-width:100%;width:320px;height:auto;cursor:zoom-in"></canvas><span class="mute" id="iInfo" style="flex:1;min-width:160px">그림 파일을 고른다 — 어두운 선 · 면을 그린다 (사진은 문턱으로 흑백). 왼쪽 작은 그림 = 그릴 곳.</span></div>
<div class="row"><button onclick="center('ix','iy',true)">판 가운데</button><span class="sp"></span><button class="go" onclick="addImage()">작업에 넣기</button></div></div>
<div class="pane" id="mt_shape"><div class="chips" id="shapeKinds"></div><div class="grid" id="shapeParams" style="margin-top:10px"></div>
<div class="grid" style="margin-top:8px"><div class="fld"><span>가운데 x</span><input type="number" id="sx" value="50" step="0.5"></div><div class="fld"><span>가운데 y</span><input type="number" id="sy" value="50" step="0.5"></div>
<div class="fld"><span>돌림 °</span><input type="number" id="srot" value="0" step="5"></div><div class="fld"><span>높이 z</span><input type="number" id="sz" value="0" step="0.5" min="-3" max="100"></div></div>
<div class="row"><button onclick="center('sx','sy')">판 가운데</button><label><input type="checkbox" id="scw"> 시계 방향</label><span class="sp"></span><button class="go" onclick="addShape()">작업에 넣기</button></div></div>
<div class="pane" id="mt_text"><div class="fld"><span>글씨 (영문 대문자 · 숫자 · - + = . , : / ( ) _ ! ? # * ' % · 줄바꿈 가능)</span><textarea id="tTxt" style="height:56px">FR5 PRINT</textarea></div>
<div class="grid" style="margin-top:8px"><div class="fld"><span>글자 높이 mm</span><input type="number" id="tH" value="10" min="1" max="300" step="0.5"></div><div class="fld"><span>글자 간격</span><input type="number" id="tSp" value="1" min="0" max="5" step="0.1"></div>
<div class="fld"><span>시작 x (기준선 왼쪽)</span><input type="number" id="tx" value="10" step="0.5"></div><div class="fld"><span>시작 y</span><input type="number" id="ty" value="10" step="0.5"></div>
<div class="fld"><span>돌림 °</span><input type="number" id="trot" value="0" step="5"></div><div class="fld"><span>높이 z</span><input type="number" id="tz" value="0" step="0.5" min="-3" max="100"></div></div>
<div class="row"><span class="sp"></span><button class="go" onclick="addText()">작업에 넣기</button></div></div>
<div class="pane" id="mt_grid"><div class="mute">칸마다: 옮기는 높이에서 칸 위로 → 곧게 내림 → 그리퍼 · 기다리기 → 들기. A1 이 첫 칸, 열은 +x, 행은 −y (위에서 본 판처럼).</div>
<div class="chips" style="margin:8px 0"><button onclick="gp(4,6,19.3,19.3)">24웰 4×6 · 19.3</button><button onclick="gp(8,12,9,9)">96웰 8×12 · 9</button><button onclick="gp(3,4,26,26)">12웰 3×4 · 26</button><button onclick="gp(2,3,39.1,39.1)">6웰 2×3 · 39.1</button></div>
<div class="grid"><div class="fld"><span>행 수</span><input type="number" id="gR" value="4" min="1" max="26"></div><div class="fld"><span>열 수</span><input type="number" id="gC" value="6" min="1" max="48"></div>
<div class="fld"><span>열 간격 x mm</span><input type="number" id="gPx" value="19.3" step="0.1"></div><div class="fld"><span>행 간격 y mm</span><input type="number" id="gPy" value="19.3" step="0.1"></div>
<div class="fld"><span>A1 x</span><input type="number" id="gx" value="20" step="0.5"></div><div class="fld"><span>A1 y</span><input type="number" id="gy" value="80" step="0.5"></div>
<div class="fld"><span>돌림 °</span><input type="number" id="grot" value="0" step="1"></div><div class="fld"><span>내릴 높이 (판 위 mm)</span><input type="number" id="gdz" value="5" step="0.5" min="-3" max="100"></div>
<div class="fld"><span>그리퍼 먼저 (빈칸 = 안 함)</span><input type="number" id="g1" min="0" max="1000" placeholder="예 267"></div><div class="fld"><span>기다리기 s</span><input type="number" id="gdw" value="0.5" min="0" max="60" step="0.1"></div>
<div class="fld"><span>그리퍼 나중 (빈칸 = 안 함)</span><input type="number" id="g2" min="0" max="1000" placeholder="예 300"></div><div class="fld"><span>순서</span><select id="gord"><option value="rows">행마다 같은 쪽</option><option value="snake">지그재그</option></select></div></div>
<div class="fld" style="margin-top:8px"><span>칸 고르기 (빈칸 = 전부 · 예: A1, B2-B5, C, 3)</span><input type="text" id="gW"></div>
<div class="row"><span class="sp"></span><button class="go" onclick="addGrid()">작업에 넣기</button></div></div>
<div class="pane" id="mt_file"><div class="row"><input type="file" id="fIn" accept=".nc,.gcode,.ngc,.gc,.tap,.svg,.csv,.txt" style="max-width:100%"></div>
<div class="row"><span class="mute">또는 붙여넣기</span><select id="fFmt" style="width:auto"><option value="gcode">G-code</option><option value="svg">SVG</option><option value="csv">좌표 CSV</option></select></div>
<textarea id="fTxt" placeholder="G0 X10 Y10&#10;G1 Z0 F300&#10;G1 X60&#10;G0 Z5"></textarea>
<div class="mute" style="margin-top:6px">G-code: 판 좌표 mm (원점 = 판 왼쪽 아래, Z = 판 위 높이). G0 = 옮기기 속도, G1 F = mm/분, G2 · G3 호, G4 P초, M3 S값 = 그리퍼, M5 = 처음 그리퍼, M0 = 멈춤.
SVG: 크기 그대로(mm), 왼쪽 아래를 x · y 에. 글자는 경로로 바꿔서. CSV: 줄마다 x, y[, z] · 빈 줄 = 새 획.</div>
<div class="row"><span class="sp"></span><button class="go" onclick="importFile()">가져와서 넣기</button></div><div class="mute" id="fInfo"></div></div>
<div class="pane" id="mt_act"><div class="row"><span style="width:110px">그리퍼 값</span><input type="number" id="aG" value="300" min="0" max="1000" style="width:90px"><button class="go" onclick="addAct({act:'grip',v:+$('aG').value})">넣기</button></div>
<div class="row"><span style="width:110px">기다리기 s</span><input type="number" id="aS" value="2" min="0" max="600" step="0.5" style="width:90px"><button class="go" onclick="addAct({act:'dwell',s:+$('aS').value})">넣기</button></div>
<div class="row"><span style="width:110px">멈춤 (확인 뒤 이어서)</span><input type="text" id="aM" value="펜 바꾸기" style="width:180px"><button class="go" onclick="addAct({act:'pause',msg:$('aM').value})">넣기</button></div>
<div class="mute">그리퍼가 바뀌는 시간은 설정의 '그리퍼 바꾸는 시간'. 멈춤에서는 로봇이 선 채 기다리고, [이어서]로 다음 동작부터.</div></div></div>

<div class="card"><h3><span class="n">3</span>작업 목록 <span class="k"><input type="text" id="jobName" value="작업" style="width:120px;padding:3px 6px"></span></h3>
<div style="overflow-x:auto"><table class="items" id="items"></table></div><div class="mute" id="itemsInfo" style="margin-top:6px"></div>
<div class="row"><button onclick="if(confirm('작업 목록을 다 지울까?')){job.items=[];jobChanged()}">모두 지우기</button><span class="sp"></span><button onclick="saveJob()">저장</button><select id="jobList" style="width:auto;max-width:180px"></select><button onclick="loadJob()">불러오기</button></div></div>

<div class="card"><h3><span class="n">4</span>설정 <span class="k mute">바꾸면 계획을 다시 본다</span></h3><div class="grid" id="opts"></div>
<div class="row"><label><input type="checkbox" id="oBack"> 끝나면 처음 자리로 (처음 자리가 옮기는 높이 위일 때)</label></div>
<div class="row"><label class="chip" id="airChip"><input type="checkbox" id="oAir" onchange="$('o_air_dz').value=this.checked?($('o_air_dz').value>0?$('o_air_dz').value:20):0"> 허공 시험</label><span class="mute">판 위</span><input type="number" id="o_air_dz" value="0" min="0" max="100" step="5" style="width:70px"><span class="mute">mm 띄워 같은 작업 — 닿지 않고 움직임만 본다</span><span class="sp"></span><button class="pri" onclick="saveOpts()">설정 저장</button></div></div>
<div class="card"><h3>기록</h3><pre id="log"></pre></div>
</section><section id="side">
<div class="card"><h3>판 <span class="k mute" id="hover">—</span></h3><canvas id="bed"></canvas>
<div class="legend"><span><span class="bar" style="background:var(--b)"></span>그리기 (펜 내림)</span><span><span class="bar" style="background:var(--mute);opacity:.7"></span>옮기기 (펜 들고)</span><span><span class="dot" style="background:var(--piv)"></span>점 찍기</span>
<span><span class="bar" style="background:var(--done)"></span>끝난 동작</span><span><span class="bar" style="background:var(--warn)"></span>판 밖 · 지금 동작</span><span><span class="dot" style="background:var(--tcp)"></span>공구 끝</span><span><span class="bar" style="background:var(--sky)"></span>넣기 전 모양</span></div></div>
<div class="card"><div class="row" style="margin:0"><button class="pri" id="bPlan" onclick="plan(0)">계획 보기 <small>(로봇 안 움직임)</small></button><button id="bAnim" onclick="anim()" disabled>미리 돌려보기</button><span class="sp"></span>
<button id="bPause" onclick="post('/api/pause',{})" disabled>획 끝에서 멈춤</button><button class="run" id="bResume" onclick="resume()" disabled>이어서 ⚠</button><button class="run" id="bRun" onclick="ask()" disabled>시작 ⚠</button></div>
<div class="row"><span class="mute">속도</span><span class="chips" id="speeds"></span><span class="mute">— 도는 중에도 바꿀 수 있다 (느리게만)</span></div>
<div class="prog"><i id="progI"></i></div><div class="mute" id="progT" style="margin-top:4px">작업을 만들고 [계획 보기] — 관절 · 높이 · 판 밖 · 속도를 미리 보고, 걸리면 보내지 않는다.</div>
<table id="sum" style="margin-top:8px"></table><div id="verdict"></div></div>
</section></main>
<div id="modal"><div class="box"><h3 style="font-size:15px;color:var(--ink)" id="mTitle">⚠ 로봇이 움직인다</h3><div id="mtxt" style="margin:8px 0 14px;white-space:pre-line"></div>
<div class="row"><button class="run" id="mGo">시작</button><button onclick="hide()">취소</button><span class="mute">멈춤은 <kbd>Space</kbd> — 그 자리에 선다</span></div></div></div>
<script>
const $=id=>document.getElementById(id);
const STEPS=[0.1,0.5,1,2,5,10,20];
const SHAPES={circle:['원',[['r','반지름',20,0.5,250,0.5]]],rect:['사각형',[['w','가로',40,1,600,1],['h','세로',30,1,600,1],['corner','모서리',0,0,100,0.5]]],
 polygon:['다각형',[['n','변',6,3,24,1],['r','반지름',20,1,250,0.5],['corner','모서리',0,0,100,0.5]]],star:['별',[['n','꼭지',5,3,24,1],['r','바깥 반지름',20,1,250,0.5],['inner','안쪽 비율',0.45,0.1,0.95,0.05]]],
 fig8:['8자',[['w','가로',60,2,600,1],['h','세로',30,1,600,1]]],line:['선',[['L','길이',40,0.5,600,0.5],['ang','방향 °',0,-360,360,5]]],
 spiral:['나선',[['r0','안 반지름',0,0,250,0.5],['r','바깥 반지름',20,0.5,250,0.5],['turns','바퀴',4,0.25,60,0.25]]],cross:['십자',[['L','길이',10,0.5,300,0.5]]],
 fill:['채우기',[['of','모양 (rect·circle)','rect'],['w','가로',30,1,600,1],['h','세로',20,1,600,1],['r','반지름 (원)',15,1,250,0.5],['s','간격',2,0.3,50,0.1],['ang','선 각 °',0,-360,360,5]]]};
const OPTS=[['v_draw','그리기 mm/s',1,60,1],['v_travel','옮기기 mm/s',5,60,1],['v_z','올리고 내리기 mm/s',2,30,1],['v_touch','닿기 직전 mm/s',0.5,10,0.5],['touch_mm','닿기 직전 구간 mm',0,20,0.5],
 ['z_travel','옮기는 높이 mm',2,100,1],['acc','가속 mm/s²',20,500,10],['vj','관절 최고 °/s',2,30,1],['z_below','판 아래 허용 mm',0,3,0.1],['margin','판 밖 허용 mm',0,50,1],['ramp_s','그리퍼 바꾸는 시간 s',0.2,10,0.1],['z_max','가장 높은 동작 mm',5,300,5]];
let S=null,job={name:'작업',items:[]},EXP=null,GHOST=null,PLAN=null,PLAN_KEY=null,draft=[],draftPoly=null,dmode='free',mkTab='draw',shapeKind='circle',sel=-1,animT=null,animP=null;
let live=false,liveQ=[],liveRun=[],liveDone=[],liveCut=[],liveRunId=null,liveBusy=false;          // 바로 따라 그리기
let IMG=null,IMGN='',imode='center',IMGS=null,IMGWH=[0,0],itT=null,IMGR=null;                     // 사진 · 그림 → 획 (판 mm, 왼쪽 아래 0,0)
// ── 저장 (이 브라우저에만) ──
function store(){try{localStorage.setItem('fr5print',JSON.stringify({job,draft,shapeKind,mkTab}))}catch(e){}}
function restore(){try{const o=JSON.parse(localStorage.getItem('fr5print')||'null');if(!o)return;if(o.job&&o.job.items)job=o.job;draft=o.draft||[];shapeKind=o.shapeKind||'circle';mkTab=o.mkTab||'draw'}catch(e){}}
async function post(u,b,quiet){try{const r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)});const j=await r.json();if(j.err&&!quiet)toast(j.err);return j}catch(e){if(!quiet)toast('fr5print 서버에 못 닿았다');return {err:'연결'}}}
function stopAll(){post('/api/stop',{});if(live)liveOff('멈춤 — 바로 따라 그리기 꺼짐',true)}
function toast(t){$('band').textContent=t;$('band').style.color='var(--warn)';lastToast=Date.now()}let lastToast=0;
function css(v){return getComputedStyle(document.documentElement).getPropertyValue(v).trim()}
function n(id){const v=parseFloat($(id).value);return isFinite(v)?v:0}
// ── 탭 ──
document.querySelectorAll('[data-bt]').forEach(b=>b.onclick=()=>{document.querySelectorAll('[data-bt]').forEach(x=>x.classList.toggle('on',x===b));['three','one'].forEach(k=>$('bt_'+k).classList.toggle('on',k===b.dataset.bt))});
function setMk(t){if(!$('mt_'+t))t='draw';mkTab=t;document.querySelectorAll('[data-mt]').forEach(x=>x.classList.toggle('on',x.dataset.mt===t));['draw','shape','text','image','grid','file','act'].forEach(k=>$('mt_'+k).classList.toggle('on',k===t));
 $('placeHint').textContent=t==='draw'?'판 그림 위에서 끌어 그린다':(['shape','text','grid','file','image'].includes(t)?'판 그림을 눌러 자리를 고른다':'');ghost();store();redraw()}
document.querySelectorAll('[data-mt]').forEach(b=>b.onclick=()=>setMk(b.dataset.mt));
function setDM(m){dmode=m;draftPoly=null;$('dmFree').classList.toggle('on',m==='free');$('dmPoly').classList.toggle('on',m==='poly');redraw()}
// ── 도형 폼 ──
function buildShapes(){$('shapeKinds').innerHTML=Object.entries(SHAPES).map(([k,v])=>`<button data-sk="${k}" onclick="setShape('${k}')">${v[0]}</button>`).join('');setShape(shapeKind)}
function setShape(k){shapeKind=k;document.querySelectorAll('[data-sk]').forEach(b=>b.classList.toggle('on',b.dataset.sk===k));
 $('shapeParams').innerHTML=SHAPES[k][1].map(p=>p[0]==='of'?`<div class="fld"><span>${p[1]}</span><select id="sp_of" onchange="ghost()"><option value="rect">사각형</option><option value="circle">원</option></select></div>`:`<div class="fld"><span>${p[1]}</span><input type="number" id="sp_${p[0]}" value="${p[2]}" min="${p[3]}" max="${p[4]}" step="${p[5]}" oninput="ghost()"></div>`).join('');store();ghost()}
['sx','sy','srot','sz','scw','tTxt','tH','tSp','tx','ty','trot','tz','gR','gC','gPx','gPy','gx','gy','grot','gdz','gW','gord'].forEach(id=>$(id).addEventListener('input',()=>ghost()));
function gp(r,c,px,py){$('gR').value=r;$('gC').value=c;$('gPx').value=px;$('gPy').value=py;ghost()}
function center(a,b,img){if(!S||!S.bed)return;const w=img?IMGWH[0]:0,h=img?IMGWH[1]:0;$(a).value=+(S.bed.W/2-w/2).toFixed(1);$(b).value=+(S.bed.H/2-h/2).toFixed(1);ghost()}
function shapeItem(){const it={type:'shape',kind:shapeKind,x:n('sx'),y:n('sy'),rot:n('srot'),z:n('sz'),cw:$('scw').checked};SHAPES[shapeKind][1].forEach(p=>{it[p[0]]=p[0]==='of'?$('sp_of').value:n('sp_'+p[0])});return it}
function textItem(){return {type:'text',text:$('tTxt').value,h:n('tH'),spacing:n('tSp'),x:n('tx'),y:n('ty'),rot:n('trot'),z:n('tz')}}
function gridItem(){const it={type:'grid',rows:n('gR'),cols:n('gC'),px:n('gPx'),py:n('gPy'),x:n('gx'),y:n('gy'),rot:n('grot'),dz:n('gdz'),dwell:n('gdw'),wells:$('gW').value,order:$('gord').value};
 it.g1=$('g1').value===''?null:n('g1');it.g2=$('g2').value===''?null:n('g2');return it}
function addItem(it){job.items.push(it);sel=job.items.length-1;jobChanged()}
function addShape(){addItem(shapeItem())}
function addText(){if(!$('tTxt').value.trim()){toast('글씨가 비었다');return}addItem(textItem())}
function addGrid(){addItem(gridItem())}
function addAct(a){addItem(Object.assign({type:'action'},a))}
function addDraw(){if(!draft.length){toast('그린 획이 없다 — 판 그림 위에서 끌어 그린다');return}addItem({type:'draw',strokes:draft.map(s=>s.map(p=>[+p[0].toFixed(2),+p[1].toFixed(2)])),smooth:n('dSmooth'),z:n('dZ')});draft=[];draftPoly=null;redraw()}
async function importFile(){let txt=$('fTxt').value,fmt=$('fFmt').value,name='';const f=$('fIn').files[0];
 if(f){if(f.size>4e6){toast('파일은 4 MB 까지');return}txt=await f.text();name=f.name;const ext=f.name.split('.').pop().toLowerCase();fmt=ext==='svg'?'svg':(['csv','txt'].includes(ext)?'csv':'gcode')}
 if(!txt.trim()){toast('파일을 고르거나 붙여 넣는다');return}
 const j=await post('/api/import',{fmt,text:txt,name});if(j.err){$('fInfo').textContent='못 가져옴: '+j.err;return}
 $('fInfo').textContent=j.info+(j.warn&&j.warn.length?' · '+j.warn.join(' · '):'');addItem(j.item);$('fIn').value=''}
// ── 작업 목록 ──
function placeable(it){return ['shape','text','grid','svg','csv','gcode','draw'].includes(it.type)}
function jobChanged(){job.name=$('jobName').value||'작업';store();renderItems();refresh()}
function renderItems(){const info=EXP?EXP.items:[];
 $('items').innerHTML=job.items.length?job.items.map((it,i)=>{const nm=(info[i]&&info[i].name)||it.type;const pl=placeable(it);
  return `<tr class="${i===sel?'sel':''}" onclick="sel=${i};renderItems();redraw()"><td class="mute">${i+1}</td><td class="nm">${esc(nm)}${info[i]&&info[i].len?` <span class="mute">${info[i].len.toFixed(0)} mm</span>`:''}</td>`+
  (pl?`<td><input type="number" step="0.5" value="${it.x??0}" title="x" onchange="setIt(${i},'x',this.value)"></td><td><input type="number" step="0.5" value="${it.y??0}" title="y" onchange="setIt(${i},'y',this.value)"></td><td><input type="number" step="5" value="${it.rot??0}" title="돌림 °" onchange="setIt(${i},'rot',this.value)"></td>`+(it.type==='svg'?`<td><input type="number" step="0.05" value="${it.scale??1}" title="크기 배율" onchange="setIt(${i},'scale',this.value)"></td>`:'<td></td>'):'<td colspan="4"></td>')+
  `<td style="white-space:nowrap">${pl?`<button class="sm" title="판 가운데로" onclick="event.stopPropagation();centerItem(${i})">⊕</button>`:''}<button class="sm" onclick="event.stopPropagation();mv(${i},-1)">↑</button><button class="sm" onclick="event.stopPropagation();mv(${i},1)">↓</button><button class="sm" onclick="event.stopPropagation();rm(${i})">×</button></td></tr>`}).join(''):'<tr><td class="mute">아직 없다 — 위에서 만들어 넣는다</td></tr>';
 $('itemsInfo').textContent=job.items.length?`x · y = 판 mm (도형은 가운데, 글씨는 시작, 배열은 A1, 파일은 왼쪽 아래) · 돌림 ° · SVG 는 크기 배율`:''}
function esc(s){return String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]))}
function setIt(i,k,v){job.items[i][k]=parseFloat(v)||0;jobChanged()}
function centerItem(i){if(!EXP||!S||!S.bed)return;let x0=1e9,y0=1e9,x1=-1e9,y1=-1e9;
 EXP.ops.forEach(op=>{if(op.item!==i)return;let P=[];if(op.k==='stroke')P=op.pts;else if(op.k==='point')P=[op.p];else if(op.k==='prog'){const f=op.els.filter(e=>e[0]==='feed');(f.length?f:op.els).forEach(e=>P=P.concat(e[1]))}
  P.forEach(p=>{x0=Math.min(x0,p[0]);y0=Math.min(y0,p[1]);x1=Math.max(x1,p[0]);y1=Math.max(y1,p[1])})});
 if(x0>x1)return;const it=job.items[i];it.x=+((it.x||0)+S.bed.W/2-(x0+x1)/2).toFixed(2);it.y=+((it.y||0)+S.bed.H/2-(y0+y1)/2).toFixed(2);jobChanged()}
function mv(i,d){const j=i+d;if(j<0||j>=job.items.length)return;[job.items[i],job.items[j]]=[job.items[j],job.items[i]];sel=j;jobChanged()}
function rm(i){job.items.splice(i,1);sel=-1;jobChanged()}
async function saveJob(){const nm=$('jobName').value.trim();if(!nm){toast('작업 이름을 쓴다');return}const j=await post('/api/job/save',{name:nm,job});if(!j.err)listJobs()}
async function listJobs(){try{const j=await (await fetch('/api/jobs')).json();$('jobList').innerHTML=j.jobs.map(x=>`<option value="${esc(x.name)}">${esc(x.name)} · ${x.t}</option>`).join('')||'<option value="">저장한 작업 없음</option>'}catch(e){}}
async function loadJob(){const nm=$('jobList').value;if(!nm)return;const j=await post('/api/job/load',{name:nm});if(j.err||!j.job)return;job={name:j.job.name||nm,items:j.job.items||[]};$('jobName').value=job.name;sel=-1;jobChanged()}
// ── 펼치기 · 넣기 전 모양 ──
let refT=null,ghT=null;
function refresh(){clearTimeout(refT);refT=setTimeout(async()=>{const j=await post('/api/expand',{job});if(!j.err){EXP=j;renderItems();showExpand()}redraw();buttons()},200)}
function ghost(){clearTimeout(ghT);ghT=setTimeout(async()=>{let it=null;
 if(mkTab==='image'){const ox=n('ix'),oy=n('iy');GHOST=IMGS?IMGS.map(s=>({k:'stroke',pts:s.map(p=>[p[0]+ox,p[1]+oy])})):null;redraw();return}
 if(mkTab==='shape')it=shapeItem();else if(mkTab==='text'&&$('tTxt').value.trim())it=textItem();else if(mkTab==='grid')it=gridItem();
 if(!it){GHOST=null;redraw();return}const r=await fetch('/api/expand',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({job:{items:[it]}})});const j=await r.json();GHOST=j.err?null:j.ops;redraw()},150)}
function showExpand(){if(!EXP)return;const v=$('verdict');if(PLAN&&planOK())return;
 v.innerHTML=(EXP.bad.length?`<ul class="bad">${EXP.bad.map(b=>'<li>'+esc(b)+'</li>').join('')}</ul>`:'')+(EXP.warn.length?`<ul class="warnl">${EXP.warn.map(b=>'<li>'+esc(b)+'</li>').join('')}</ul>`:'')}
// ── 설정 ──
function buildOpts(){$('opts').innerHTML=OPTS.map(([k,t,lo,hi,st])=>`<div class="fld"><span>${t} <b style="color:var(--ink)">(${lo}–${hi})</b></span><input type="number" id="o_${k}" min="${lo}" max="${hi}" step="${st}"></div>`).join('')}
function fillOpts(o){OPTS.forEach(([k])=>{const e=$('o_'+k);if(document.activeElement!==e)e.value=o[k]});if(document.activeElement!==$('o_air_dz'))$('o_air_dz').value=o.air_dz;$('oAir').checked=o.air_dz>0;$('oBack').checked=!!o.back;$('airChip').classList.toggle('air',o.air_dz>0)}
async function saveOpts(){const b={back:$('oBack').checked,air_dz:n('o_air_dz')};OPTS.forEach(([k])=>b[k]=n('o_'+k));const j=await post('/api/opts',b);if(!j.err){optsLoaded=false;refresh()}}
async function saveTool(){const j=await post('/api/opts',{tool_len:n('toolLen')});if(!j.err)refresh()}
let optsLoaded=false;
// ── 판 ──
async function solve(mode){const j=await post('/api/bed/solve',{mode,w:n('bw'),h:n('bh')});if(!j.err){PLAN=null;refresh()}}
function renderMarks(){const L=['원점 · 왼쪽 아래','+X 쪽 · 오른쪽 아래','+Y 쪽 · 왼쪽 위'];const m=(S&&S.marks)||{};
 $('marks').innerHTML=[1,2,3].map(i=>`<b>${i}</b><span>${L[i-1]} <span class="c">${m[i]?'('+m[i].map(v=>v.toFixed(1)).join(', ')+')'+(S.marks_L[i]!==S.o.tool_len?' 공구 '+S.marks_L[i]:''):'— 아직'}</span></span><button onclick="post('/api/bed/mark',{i:${i}})" ${ready()?'':'disabled'}>기록</button>`).join('')}
async function corners(){const j=await post('/api/corners',{z:n('cz')});if(j.err)return;setPlan(j,'corners');ask(true)}
// ── 조그 ──
function ready(){return S&&S.jog.phase==='준비'&&!S.jog.busy}
function grip(v){if(!isFinite(v))return;post('/api/cmd',{kind:'grip',g:Math.max(0,Math.min(1000,v))})}
async function goHome(){const j=await post('/api/plan_goto',{name:'home'});if(j.err)return;$('mTitle').textContent='⚠ 초기자세로 갈까?';$('mtxt').textContent=j.text;$('mGo').textContent='가기';$('mGo').onclick=()=>{hide();post('/api/cmd',{kind:'goto',name:'home'})};$('modal').style.display='flex'}
document.querySelectorAll('[data-j]').forEach(b=>b.onclick=()=>post('/api/cmd',{kind:'jog',d:b.dataset.j.split(',').map(Number)}));
document.querySelectorAll('[data-v]').forEach(b=>b.onclick=()=>post('/api/cmd',{kind:'vertical'}));
document.querySelectorAll('[data-gd]').forEach(b=>b.onclick=()=>S&&grip(Math.round(S.jog.g)+(+b.dataset.gd)));
$('steps').innerHTML=STEPS.map(s=>`<button data-s="${s}" onclick="post('/api/set',{step:${s}})">${s}</button>`).join('');
$('speeds').innerHTML=[0.25,0.5,0.75,1].map(r=>`<button data-r="${r}" onclick="post('/api/speed',{r:${r}})">${r*100}%</button>`).join('');
// ── 계획 ──
function planKey(){return JSON.stringify([job.items,S?S.o:null,S&&S.bed?S.bed.made:null])}
function planOK(){return PLAN&&PLAN.kind!=='corners'?PLAN_KEY===planKey():!!PLAN}
function setPlan(j,kind){PLAN=Object.assign(j,{kind:kind||'job'});PLAN_KEY=planKey();showPlan();redraw();buttons()}
async function plan(start){$('bPlan').disabled=true;$('progT').textContent='계획하는 중… (긴 작업은 몇 초)';
 try{job.name=$('jobName').value||'작업';const j=await post('/api/plan',{job,start_op:start||0});if(j.err){$('progT').textContent='계획 못 함: '+j.err;if(j.expand){EXP=j.expand;showExpand()}PLAN=null;redraw();return null}
  setPlan(j,'job');$('progT').textContent=j.sum.bad.length?'보내지 않음 — 아래 이유를 고친 뒤 다시':`준비되면 [시작] — ${fmtT(j.sum.sec)}`;return j}finally{$('bPlan').disabled=false;buttons()}}
function fmtT(s){s=Math.round(s);return s>=60?`${Math.floor(s/60)}분 ${s%60}초`:`${s}초`}
function f1(x){return (+x).toFixed(1)}
function showPlan(){const s=PLAN.sum;const kn={stroke:'획',point:'점',prog:'G-code',pause:'멈춤',grip:'그리퍼',dwell:'기다리기'};
 const rows=[['동작',`${s.ops}개 (${s.start_op+1}–${s.n_total})`+' · '+Object.entries(s.kinds).map(([k,v])=>`${kn[k]||k} ${v}`).join(' · ')],['시간',fmtT(s.sec)+(s.back_mode==='start'?' · 끝나면 처음 자리로':s.back_mode==='lift'?' · 끝나면 판 위로 들어 둠':'')],
  ['길이',`그리기 ${s.len_draw.toFixed(0)} mm · 옮기기 ${s.len_travel.toFixed(0)} mm`],['속도',`공구 끝 최고 ${f1(s.v_peak)} mm/s · 닿기 직전 최고 ${s.v_touch_peak.toFixed(1)} mm/s · 관절 최고 ${f1(s.j_peak)}°/s`+(s.stops?` · 서는 꼭짓점 ${s.stops}`:'')],
  ['높이',`공구 끝 판 위 ${s.z_bed[0]}~${s.z_bed[1]} mm · 옮기는 높이 ${s.z_travel} mm`+(s.air_dz?` · 허공 +${s.air_dz} mm`:'')+` · TCP z ${s.tcp_z[0]}~${s.tcp_z[1]}`],
  ['공구',`길이 ${s.tool_len} mm · 판 법선과 ${s.tool_vs_normal}° · J5 최소 ${s.j5_min??'—'}° · J6 ${s.j6[0]}~${s.j6[1]}°`],['정확도',`정한 길과 차 ${s.dev_mm.toFixed(3)} mm · IK ${s.ik_mm.toFixed(3)} mm · 계획 ${s.plan_s} s`]];
 $('sum').innerHTML=rows.map(r=>`<tr><td class="n">${r[0]}</td><td class="r">${r[1]}</td></tr>`).join('');
 $('verdict').innerHTML=(s.bad.length?`<div class="bad" style="padding:0">보내지 않음:</div><ul class="bad">${s.bad.map(b=>'<li>'+esc(b)+'</li>').join('')}</ul>`:'<div class="good">검사 통과 — 관절 한계 · J6 · 손목 특이점 · IK · 바닥 · 작업 상자 · 판 높이 · 판 밖</div>')+
  (s.warn_tool?`<ul class="warnl"><li>${esc(s.warn_tool)}</li></ul>`:'')+(s.tool_vs_normal>3?`<ul class="warnl"><li>공구가 판 법선과 ${s.tool_vs_normal}° 기울었다 — 공구 길이 오차가 자리 오차가 된다 ([손 곧게])</li></ul>`:'')+
  (EXP&&EXP.warn.length&&PLAN.kind==='job'?`<ul class="warnl">${EXP.warn.map(b=>'<li>'+esc(b)+'</li>').join('')}</ul>`:'')}
function buttons(){const rs=S?S.run:{};const running=rs.state==='running'&&S.jog.busy;const ok=ready();const pv=PLAN&&planOK()&&!PLAN.sum.bad.length;
 $('bRun').disabled=!(pv&&ok);$('bAnim').disabled=!PLAN;$('bPause').disabled=!running||(S&&S.pause_req);$('bPlan').disabled=S&&S.jog.busy;$('bCorner').disabled=!S||!S.bed||(S.jog.busy);
 const canRes=!!(rs&&['paused','stopped'].includes(rs.state)&&rs.resume_op!=null&&rs.name===($('jobName').value||'작업'));
 $('bResume').disabled=!(ok&&canRes);$('bResume').textContent=canRes?`이어서 ⚠ (동작 ${rs.resume_op+1}부터)`:'이어서 ⚠';
 $('bRun').title=!PLAN?'계획을 먼저':!planOK()?'바뀐 것이 있다 — 다시 [계획 보기]':PLAN.sum.bad.length?'검사에 걸렸다':!ok?'[보내기 시작] → 제어판 [리더 추종 시작] 뒤':'';
 if(PLAN&&!planOK())$('sum').classList.add('stale');else $('sum').classList.remove('stale')}
function ask(corner){const s=PLAN.sum;$('mTitle').textContent=corner?'⚠ 모서리 확인 — 로봇이 움직인다':'⚠ 이 작업을 할까? — 로봇이 움직인다';
 $('mtxt').textContent=`${corner?'판 위 '+n('cz')+' mm 에서 네 모서리 · 가운데':($('jobName').value||'작업')} · 동작 ${s.start_op+1}–${s.n_total}\n${fmtT(s.sec)} · 그리기 ${s.len_draw.toFixed(0)} mm · 옮기기 ${s.len_travel.toFixed(0)} mm\n공구 끝 판 위 ${s.z_bed[0]}~${s.z_bed[1]} mm`+(s.air_dz?` (허공 +${s.air_dz} mm — 닿지 않음)`:'')+`\n판 위에 공구 외에 부딪힐 것이 없는지, 공구 길이 ${s.tool_len} mm 가 맞는지 본다.`;
 $('mGo').textContent=corner?'모서리 확인':'시작';$('mGo').onclick=async()=>{hide();await post('/api/run',{id:PLAN.id})};$('modal').style.display='flex'}
async function resume(){const r=S.run;const j=await plan(r.resume_op);if(!j||j.sum.bad.length)return;ask(false)}
function hide(){$('modal').style.display='none'}
// ── 판 그림 ──
const cv=$('bed'),g=cv.getContext('2d');let VP=null;
function fit(){const d=window.devicePixelRatio||1,bw=S&&S.bed?S.bed.W:200,bh=S&&S.bed?S.bed.H:150,mx=Math.min(innerHeight*0.52,560);
 cv.style.height=Math.round(Math.max(260,Math.min(mx,cv.clientWidth*bh/bw+68)))+'px';const r=cv.getBoundingClientRect();cv.width=Math.max(10,r.width*d);cv.height=Math.max(10,r.height*d);g.setTransform(d,0,0,d,0,0);redraw()}
function view(){const W=cv.clientWidth,H=cv.clientHeight,bw=S&&S.bed?S.bed.W:200,bh=S&&S.bed?S.bed.H:150,m=34;const s=Math.min((W-2*m)/bw,(H-2*m)/bh);
 const ox=(W-bw*s)/2,oy=(H+bh*s)/2;return {s,bw,bh,X:x=>ox+x*s,Y:y=>oy-y*s,ix:px=>(px-ox)/s,iy:py=>(oy-py)/s,W,H}}
function line(P,col,w,dash){if(P.length<2){if(P.length===1)dot(P[0],w,col);return}g.strokeStyle=col;g.lineWidth=w;g.setLineDash(dash||[]);g.beginPath();P.forEach((p,i)=>{const x=VP.X(p[0]),y=VP.Y(p[1]);i?g.lineTo(x,y):g.moveTo(x,y)});g.stroke();g.setLineDash([])}
function dot(p,r,col,fill){g.beginPath();g.arc(VP.X(p[0]),VP.Y(p[1]),r,0,7);if(fill===false){g.strokeStyle=col;g.lineWidth=1.6;g.stroke()}else{g.fillStyle=col;g.fill()}}
function redraw(){if(!cv.clientWidth)return;VP=view();const W=VP.W,H=VP.H,ink=getComputedStyle(document.body).color;g.clearRect(0,0,W,H);
 const b={x0:VP.X(0),y0:VP.Y(VP.bh),x1:VP.X(VP.bw),y1:VP.Y(0)};
 g.fillStyle=css('--card');g.fillRect(b.x0,b.y0,b.x1-b.x0,b.y1-b.y0);
 for(let x=0;x<=VP.bw+1e-6;x+=10){const maj=Math.round(x)%50===0;g.strokeStyle=ink;g.globalAlpha=maj?.16:.06;g.lineWidth=1;g.beginPath();g.moveTo(VP.X(x),b.y0);g.lineTo(VP.X(x),b.y1);g.stroke()}
 for(let y=0;y<=VP.bh+1e-6;y+=10){const maj=Math.round(y)%50===0;g.strokeStyle=ink;g.globalAlpha=maj?.16:.06;g.beginPath();g.moveTo(b.x0,VP.Y(y));g.lineTo(b.x1,VP.Y(y));g.stroke()}
 g.globalAlpha=.55;g.strokeStyle=ink;g.lineWidth=1.5;g.strokeRect(b.x0,b.y0,b.x1-b.x0,b.y1-b.y0);g.globalAlpha=.6;g.fillStyle=ink;g.font='11px system-ui';
 g.fillText('0',b.x0-10,b.y1+12);g.fillText(`X ${VP.bw.toFixed(0)} →`,b.x1-46,b.y1+14);g.fillText(`↑ Y ${VP.bh.toFixed(0)}`,b.x0-30,b.y0-6);g.globalAlpha=1;
 if(!S||!S.bed){g.fillStyle=ink;g.globalAlpha=.6;g.font='13px system-ui';g.textAlign='center';g.fillText('작업판을 먼저 맞춘다 (①) — 지금은 200 × 150 mm 자리 표시',W/2,H/2);g.textAlign='left';g.globalAlpha=1}
 // 계획의 옮기기
 if(PLAN&&PLAN.prev)PLAN.prev.travel.forEach(t=>line(t,css('--mute'),1.1,[4,4]));
 // 동작
 const ops=EXP?EXP.ops:[],rs=S?S.run:null,running=rs&&rs.state==='running',cur=rs&&rs.op!=null?rs.op:-1,fl=EXP?EXP.flags:{out:[],low:[]};
 const sameRun=rs&&rs.name===($('jobName').value||'작업')&&['running','paused','stopped','done'].includes(rs.state);
 ops.forEach((op,i)=>{let col=css('--b');if(fl.out.includes(i)||fl.low.includes(i))col=css('--warn');else if(sameRun&&(rs.state==='done'||i<cur))col=css('--done');else if(sameRun&&running&&i===cur)col=css('--piv');
  const w=(sel>=0&&op.item===sel)?3.2:2;
  if(op.k==='stroke')line(op.pts,col,w);else if(op.k==='point'){dot(op.p,4.5,col===css('--b')?css('--piv'):col);g.fillStyle=ink;g.globalAlpha=.55;g.font='10px system-ui';g.fillText(op.label,VP.X(op.p[0])+6,VP.Y(op.p[1])-5);g.globalAlpha=1}
  else if(op.k==='prog')op.els.forEach(e=>line(e[1],e[0]==='rapid'?css('--mute'):col,e[0]==='rapid'?1.1:w,e[0]==='rapid'?[3,3]:null))});
 if(GHOST)GHOST.forEach(op=>{if(op.k==='stroke')line(op.pts,css('--sky'),1.6,[5,3]);else if(op.k==='point')dot(op.p,4,css('--sky'),false)});
 if(mkTab==='draw'){draft.forEach(s=>line(s,css('--sky'),2));if(draftPoly&&draftPoly.length){line(draftPoly,css('--sky'),2,[4,3]);draftPoly.forEach(p=>dot(p,2.5,css('--sky')))}}
 liveDone.forEach(s=>line(s,css('--done'),2));liveCut.forEach(s=>line(s,css('--warn'),2,[5,3]));liveRun.forEach(s=>line(s,css('--piv'),2.6));liveQ.forEach(s=>line(s,css('--sky'),2,[4,3]));
 if(animP){dot(animP,6,ink,false)}
 if(S&&S.tip_bed){const p=S.tip_bed,x=VP.X(p[0]),y=VP.Y(p[1]);g.strokeStyle=css('--tcp');g.lineWidth=2;g.beginPath();g.moveTo(x-8,y);g.lineTo(x+8,y);g.moveTo(x,y-8);g.lineTo(x,y+8);g.stroke();dot(p,3.5,css('--tcp'));
  g.fillStyle=css('--tcp');g.font='11px system-ui';g.fillText(`판 위 ${p[2].toFixed(1)}`,x+9,y+14)}
 g.fillStyle=ink;g.globalAlpha=.5;g.font='11px system-ui';g.fillText('칸 10 mm',8,H-8);g.globalAlpha=1}
function bpos(e){const r=cv.getBoundingClientRect();return [VP.ix(e.clientX-r.left),VP.iy(e.clientY-r.top)]}
let drawing=false;
cv.addEventListener('pointerdown',e=>{if(!VP)return;const p=bpos(e);
 if(mkTab==='draw'){cv.setPointerCapture(e.pointerId);if(dmode==='free'){draft.push([p]);drawing=true}else{draftPoly=draftPoly||[];draftPoly.push(p)}redraw();return}
 const r2=v=>Math.round(v*2)/2;if(mkTab==='shape'){$('sx').value=r2(p[0]);$('sy').value=r2(p[1])}else if(mkTab==='text'){$('tx').value=r2(p[0]);$('ty').value=r2(p[1])}else if(mkTab==='grid'){$('gx').value=r2(p[0]);$('gy').value=r2(p[1])}else if(mkTab==='image'){$('ix').value=r2(p[0]);$('iy').value=r2(p[1])}
 else if(mkTab==='file'&&sel>=0&&placeable(job.items[sel])){job.items[sel].x=r2(p[0]);job.items[sel].y=r2(p[1]);jobChanged()}ghost()});
cv.addEventListener('pointermove',e=>{if(!VP)return;const p=bpos(e);$('hover').textContent=`판 (${p[0].toFixed(1)}, ${p[1].toFixed(1)}) mm`;
 if(drawing){const s=draft[draft.length-1],l=s[s.length-1];if(Math.hypot(p[0]-l[0],p[1]-l[1])*VP.s>=2){s.push(p);redraw()}}});
cv.addEventListener('pointerup',()=>{if(drawing){drawing=false;const s=draft[draft.length-1];if(s.length<2)s.push([s[0][0]+0.01,s[0][1]]);
 if(live){liveQ.push(draft.pop());renderLive();liveStep()}store();updDraft();redraw()}});
cv.addEventListener('pointercancel',()=>{if(drawing){drawing=false;draft.pop();redraw()}});
cv.addEventListener('dblclick',()=>{if(mkTab==='draw'&&dmode==='poly'&&!live&&draftPoly&&draftPoly.length){const P=draftPoly.slice();if(P.length>1&&Math.hypot(P[P.length-1][0]-P[P.length-2][0],P[P.length-1][1]-P[P.length-2][1])<0.5)P.pop();if(P.length>1)draft.push(P);draftPoly=null;store();updDraft();redraw()}});
function updDraft(){let L=0;draft.forEach(s=>{for(let i=1;i<s.length;i++)L+=Math.hypot(s[i][0]-s[i-1][0],s[i][1]-s[i-1][1])});$('draftInfo').textContent=draft.length?`${draft.length}획 · ${L.toFixed(0)} mm`:''}
function anim(){if(animT){cancelAnimationFrame(animT);animT=null;animP=null;$('bAnim').textContent='미리 돌려보기';redraw();return}
 const A=PLAN.prev.anim,hz=PLAN.prev.anim_hz,t0=performance.now(),k=Math.max(1,Math.round(A.length/hz/30));$('bAnim').textContent='멈추기';
 const step=()=>{const t=(performance.now()-t0)/1000*k,i=Math.min(A.length-1,Math.floor(t*hz));animP=A[i];$('progI').style.width=(100*i/(A.length-1))+'%';$('progT').textContent=`미리 돌려보기 ×${k} — ${fmtT(i/hz)} / ${fmtT(A.length/hz)} (로봇 안 움직임)`;redraw();
  if(i<A.length-1)animT=requestAnimationFrame(step);else{animT=null;setTimeout(()=>{animP=null;$('bAnim').textContent='미리 돌려보기';redraw()},500)}};animT=requestAnimationFrame(step)}
// ── 바로 따라 그리기: 획을 다 그리면(손을 떼면) 그 획만 계획 · 검사 · 실행 ──
function liveToggle(){if(live){liveOff('바로 따라 그리기 끔');return}
 if(!S||!S.bed){toast('작업판을 먼저 맞춘다');return}if(!ready()){toast('[보내기 시작] → 제어판 [리더 추종 시작] 뒤에 켠다');return}
 const o=S.o;$('mTitle').textContent='⚠ 바로 따라 그리기를 켤까? — 로봇이 움직인다';
 $('mtxt').textContent=`켜면 판 그림에 획을 하나 그릴 때마다(손을 떼면) 로봇이 곧바로 그 획을 그린다.\n높이 z ${n('dZ')} mm · 다듬기 ${n('dSmooth')} mm · 옮기는 높이 ${o.z_travel} mm · 그리기 ${o.v_draw} mm/s · 공구 ${o.tool_len} mm`+(o.air_dz?`\n허공 시험 +${o.air_dz} mm — 닿지 않음`:'')+`\n획마다 검사(관절 · 판 밖 · 높이)를 하고, 걸리면 보내지 않고 꺼진다. 획 사이에서는 옮기는 높이로 들어 둔다.\nSpace = 그 자리에 멈춤 + 바로 따라 그리기 꺼짐.`;
 $('mGo').textContent='켜기';$('mGo').onclick=()=>{hide();live=true;liveQ=[];liveRun=[];liveRunId=null;PLAN=null;$('sum').innerHTML='';$('verdict').innerHTML='';setMk('draw');setDM('free');renderLive();redraw();buttons()};$('modal').style.display='flex'}
function liveOff(msg,cut){live=false;if(cut)liveCut=liveCut.concat(liveRun);else liveDone=liveDone.concat(liveRun);liveRun=[];liveRunId=null;if(liveQ.length){draft=draft.concat(liveQ);liveQ=[];store();updDraft()}renderLive();if(msg)toast(msg);redraw()}
function renderLive(){const b=$('bLive');b.textContent=live?'바로 따라 그리기 끄기':'바로 따라 그리기 ⚠';b.className=live?'on':'run';$('dmPoly').disabled=live;
 $('liveInfo').textContent=live?`켜짐 — 기다리는 획 ${liveQ.length} · 그리는 중 ${liveRun.length} · 다 그린 획 ${liveDone.length}`:'켜면 획을 그릴 때마다 로봇이 곧바로 그 획을 따라 그린다 (그리기 탭 · 자유 선)'}
async function liveStep(){if(!S)return;const r=S.run||{};
 if(liveRunId){if(r.id!==liveRunId||r.state==='running')return;const ok=r.state==='done';if(ok)liveDone=liveDone.concat(liveRun);else liveCut=liveCut.concat(liveRun);liveRun=[];liveRunId=null;if(!ok){liveOff('바로 따라 그리기 꺼짐 — '+(r.msg||r.state));return}}
 if(!live||liveBusy||!liveQ.length||!ready())return;liveBusy=true;const strokes=liveQ.splice(0);
 try{const it={type:'draw',strokes:strokes.map(s=>s.map(p=>[+p[0].toFixed(2),+p[1].toFixed(2)])),smooth:n('dSmooth'),z:n('dZ')};
  const j=await post('/api/plan',{job:{name:'바로 따라 그리기',items:[it]},no_back:true},true);
  if(j.err){liveQ=strokes.concat(liveQ);if(j.err!=='움직이는 중')liveOff('바로 따라 그리기 꺼짐 — '+j.err);return}
  if(j.sum.bad.length){liveQ=strokes.concat(liveQ);liveOff('바로 따라 그리기 꺼짐 — 검사: '+j.sum.bad.join(' · '));return}
  const r2=await post('/api/run',{id:j.id},true);if(r2.err){liveQ=strokes.concat(liveQ);liveOff('바로 따라 그리기 꺼짐 — '+r2.err);return}
  liveRun=strokes;liveRunId=j.id;renderLive()}finally{liveBusy=false;redraw()}}
// ── 사진 · 그림 → 획 (브라우저 안에서: 흑백 → 중심선(골격) · 윤곽선(마칭 스퀘어) · 채우기 선 → 줄이기 → 순서 정리) ──
document.querySelectorAll('[data-im]').forEach(b=>b.onclick=()=>{imode=b.dataset.im;document.querySelectorAll('[data-im]').forEach(x=>x.classList.toggle('on',x===b));$('iCenterOpts').style.display=imode==='center'?'':'none';traceImg()});
$('iIn').addEventListener('change',()=>{const f=$('iIn').files[0];if(!f)return;if(f.size>25e6){toast('그림은 25 MB 까지');return}const url=URL.createObjectURL(f);const im=new Image();
 im.onload=()=>{IMG=im;IMGN=f.name.replace(/\.[^.]+$/,'').slice(0,30);URL.revokeObjectURL(url);traceImg()};im.onerror=()=>{URL.revokeObjectURL(url);toast('그림을 못 읽었다')};im.src=url});
['iInv','iTh','iW','iS','iTol','iMin','iRes','iSpeck','iPen','iSpur','iJoin','iExt','iOrd','iDot'].forEach(id=>$(id).addEventListener('input',()=>{clearTimeout(itT);itT=setTimeout(traceImg,250)}));
['ix','iy','iz'].forEach(id=>$(id).addEventListener('input',()=>ghost()));
function otsu(H,N){let sum=0;for(let i=0;i<256;i++)sum+=i*H[i];let sB=0,wB=0,best=-1,t1=127,t2=127;for(let t=0;t<256;t++){wB+=H[t];if(!wB)continue;const wF=N-wB;if(!wF)break;sB+=t*H[t];const mB=sB/wB,mF=(sum-sB)/wF,v=wB*wF*(mB-mF)*(mB-mF);
 if(v>best*(1+1e-9)){best=v;t1=t2=t}else if(v>=best*(1-1e-9))t2=t}return (t1+t2)/2+0.5}                 // 같은 값이 이어지면 가운데 (두 봉우리 사이)
function binImg(){const iw=IMG.naturalWidth,ih=IMG.naturalHeight,Wmm=Math.max(5,n('iW')),res=Math.max(0.03,n('iRes')||0.1);        // 정밀도 mm/px → 화소 수 (200–1400, 160만 화소까지)
 let w=Math.max(200,Math.min(1400,Math.round(Wmm/res))),h=Math.max(2,Math.round(w*ih/iw));if(w*h>1.6e6){const f=Math.sqrt(1.6e6/(w*h));w=Math.max(2,Math.round(w*f));h=Math.max(2,Math.round(h*f))}
 const c=document.createElement('canvas');c.width=w;c.height=h;const x=c.getContext('2d');x.fillStyle='#fff';x.fillRect(0,0,w,h);x.imageSmoothingQuality='high';x.drawImage(IMG,0,0,w,h);const d=x.getImageData(0,0,w,h).data;
 const gr=new Uint8Array(w*h),hist=new Array(256).fill(0);for(let i=0;i<w*h;i++){const v=Math.round(0.299*d[4*i]+0.587*d[4*i+1]+0.114*d[4*i+2]);gr[i]=v;hist[v]++}
 let th=parseFloat($('iTh').value);const auto=!isFinite(th);if(auto)th=otsu(hist,w*h);const inv=$('iInv').checked,B=new Uint8Array(w*h);let nOn=0;
 for(let i=0;i<w*h;i++){B[i]=inv?(gr[i]>th?1:0):(gr[i]<th?1:0);nOn+=B[i]}return {B,w,h,th,auto,nOn}}
function contours(B,w,h){const W=w+2,H=h+2,P=new Uint8Array(W*H);for(let y=0;y<h;y++)for(let x=0;x<w;x++)P[(y+1)*W+x+1]=B[y*w+x];
 const sa=[],sb=[],adj=new Map();const add=(a,b)=>{const i=sa.length;sa.push(a);sb.push(b);let u=adj.get(a);if(!u)adj.set(a,u=[]);u.push(i);let v=adj.get(b);if(!v)adj.set(b,v=[]);v.push(i)};
 for(let y=0;y<H-1;y++)for(let x=0;x<W-1;x++){const c=P[y*W+x]*8+P[y*W+x+1]*4+P[(y+1)*W+x+1]*2+P[(y+1)*W+x];if(c===0||c===15)continue;
  const T=2*(y*W+x),Bt=2*((y+1)*W+x),L=2*(y*W+x)+1,R=2*(y*W+x+1)+1;
  switch(c){case 1:case 14:add(L,Bt);break;case 2:case 13:add(Bt,R);break;case 3:case 12:add(L,R);break;case 4:case 11:add(T,R);break;case 5:add(T,R);add(L,Bt);break;
   case 6:case 9:add(T,Bt);break;case 7:case 8:add(L,T);break;case 10:add(T,L);add(Bt,R);break}}
 const pt=id=>{const e=id>>1,x=e%W,y=(e-x)/W;return (id&1)?[x-1,y-0.5]:[x-0.5,y-1]};const used=new Uint8Array(sa.length),out=[];
 for(let s0=0;s0<sa.length;s0++){if(used[s0])continue;used[s0]=1;const ch=[sa[s0],sb[s0]];
  for(let pass=0;pass<2;pass++){for(;;){const end=ch[ch.length-1],l=adj.get(end);let nx=-1;for(const i of l)if(!used[i]){nx=i;break}if(nx<0)break;used[nx]=1;ch.push(sa[nx]===end?sb[nx]:sa[nx])}ch.reverse()}
  out.push(ch.map(pt))}return out}
function rdp(P,eps){if(P.length<3)return P;const keep=new Uint8Array(P.length);keep[0]=keep[P.length-1]=1;const st=[[0,P.length-1]];
 while(st.length){const [a,b]=st.pop();const ax=P[a][0],ay=P[a][1],dx=P[b][0]-ax,dy=P[b][1]-ay,L=Math.hypot(dx,dy);let dm=0,im=-1;
  for(let i=a+1;i<b;i++){const d=L>1e-9?Math.abs(dy*(P[i][0]-ax)-dx*(P[i][1]-ay))/L:Math.hypot(P[i][0]-ax,P[i][1]-ay);if(d>dm){dm=d;im=i}}
  if(dm>eps&&im>0){keep[im]=1;st.push([a,im],[im,b])}}return P.filter((_,i)=>keep[i])}
function hatchRuns(B,w,h,sp){const out=[];let row=0;for(let y=sp/2;y<h;y+=sp){const yi=Math.min(h-1,Math.round(y)),runs=[];let x=0;
 while(x<w){while(x<w&&!B[yi*w+x])x++;if(x>=w)break;const x0=x;while(x<w&&B[yi*w+x])x++;runs.push([x0-0.5,x-0.5])}
 if(!runs.length)continue;if(row%2)runs.reverse().forEach(r=>out.push([[r[1],yi],[r[0],yi]]));else runs.forEach(r=>out.push([[r[0],yi],[r[1],yi]]));row++}return out}
function plen2(P){let L=0;for(let i=1;i<P.length;i++)L+=Math.hypot(P[i][0]-P[i-1][0],P[i][1]-P[i-1][1]);return L}
// ── v0.2 중심선 · 정리 (브라우저 안): 티 지우기 · 구멍 메우기 → 골격(Zhang–Suen + 계단 지우기) → 그래프 → 잔가지 버림
//    → 갈림길에서 곧게 이어지는 것끼리 잇기 → 끝 늘리기 → 매끈 · 줄이기 → 순서 정리(가까운 것부터 · 2-opt · 작은 틈 잇기)
function ccLabel(B,w,h,val,c8){const lab=new Int32Array(w*h).fill(-1),area=[],touch=[],st=new Int32Array(w*h);let n=0;
 for(let i=0;i<w*h;i++){if(B[i]!==val||lab[i]>=0)continue;let sp=0,a=0,t=false;st[sp++]=i;lab[i]=n;
  while(sp){const j=st[--sp];a++;const x=j%w,y=(j-x)/w;if(!x||!y||x===w-1||y===h-1)t=true;
   for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){if((!dx&&!dy)||(!c8&&dx&&dy))continue;const X=x+dx,Y=y+dy;if(X<0||Y<0||X>=w||Y>=h)continue;const k=Y*w+X;if(B[k]===val&&lab[k]<0){lab[k]=n;st[sp++]=k}}}
  area.push(a);touch.push(t);n++}return {lab,n,area,touch}}
function cleanBin(B,w,h,minA,holeA){let nS=0,nH=0;
 if(minA>0){const L=ccLabel(B,w,h,1,true);for(let i=0;i<w*h;i++)if(B[i]&&L.area[L.lab[i]]<minA)B[i]=0;nS=L.area.filter(a=>a<minA).length}
 if(holeA>0){const L=ccLabel(B,w,h,0,false);for(let i=0;i<w*h;i++)if(!B[i]&&!L.touch[L.lab[i]]&&L.area[L.lab[i]]<holeA)B[i]=1;nH=L.area.filter((a,k)=>!L.touch[k]&&a<holeA).length}
 return {nS,nH}}
function distT(B,w,h){const D=new Float32Array(w*h),R2=Math.SQRT2;for(let i=0;i<w*h;i++)D[i]=B[i]?1e9:0;
 for(let y=0;y<h;y++)for(let x=0;x<w;x++){const i=y*w+x;if(!D[i])continue;let v=D[i];if(x>0)v=Math.min(v,D[i-1]+1);else v=Math.min(v,1);
  if(y>0){v=Math.min(v,D[i-w]+1);if(x>0)v=Math.min(v,D[i-w-1]+R2);if(x<w-1)v=Math.min(v,D[i-w+1]+R2)}else v=Math.min(v,1);D[i]=v}
 for(let y=h-1;y>=0;y--)for(let x=w-1;x>=0;x--){const i=y*w+x;if(!D[i])continue;let v=D[i];if(x<w-1)v=Math.min(v,D[i+1]+1);else v=Math.min(v,1);
  if(y<h-1){v=Math.min(v,D[i+w]+1);if(x<w-1)v=Math.min(v,D[i+w+1]+R2);if(x>0)v=Math.min(v,D[i+w-1]+R2)}else v=Math.min(v,1);D[i]=v}return D}
function thinZS(B,w,h){const S=Uint8Array.from(B),P=(x,y)=>(x<0||y<0||x>=w||y>=h)?0:S[y*w+x],del=[];let ch=true,it=0;
 while(ch&&it<1000){ch=false;it++;for(let ps=0;ps<2;ps++){del.length=0;
  for(let y=0;y<h;y++)for(let x=0;x<w;x++){const i=y*w+x;if(!S[i])continue;
   const p2=P(x,y-1),p3=P(x+1,y-1),p4=P(x+1,y),p5=P(x+1,y+1),p6=P(x,y+1),p7=P(x-1,y+1),p8=P(x-1,y),p9=P(x-1,y-1),b=p2+p3+p4+p5+p6+p7+p8+p9;if(b<2||b>6)continue;
   const a=(!p2&&p3)+(!p3&&p4)+(!p4&&p5)+(!p5&&p6)+(!p6&&p7)+(!p7&&p8)+(!p8&&p9)+(!p9&&p2);if(a!==1)continue;
   if(ps===0?(p2&&p4&&p6)||(p4&&p6&&p8):(p2&&p4&&p8)||(p2&&p6&&p8))continue;del.push(i)}
  if(del.length){ch=true;for(const i of del)S[i]=0}}}
 // 계단 · 겹친 화소: 이웃이 둘 이상이고 그 이웃끼리 p 없이도 한 덩어리면 지운다 (끝점 · 줄 가운데는 남음)
 const nb=new Int32Array(16);let chg=true,g=0;while(chg&&g<30){chg=false;g++;for(let y=0;y<h;y++)for(let x=0;x<w;x++){const i=y*w+x;if(!S[i])continue;let m=0;
  for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){if(!dx&&!dy)continue;const X=x+dx,Y=y+dy;if(X>=0&&Y>=0&&X<w&&Y<h&&S[Y*w+X]){nb[2*m]=X;nb[2*m+1]=Y;m++}}
  if(m<2)continue;let seen=1,grow=true;while(grow){grow=false;for(let k=0;k<m;k++){if(seen>>k&1)continue;for(let q=0;q<m;q++)if((seen>>q&1)&&Math.abs(nb[2*k]-nb[2*q])<=1&&Math.abs(nb[2*k+1]-nb[2*q+1])<=1){seen|=1<<k;grow=true;break}}}
  if(seen===(1<<m)-1){S[i]=0;chg=true}}}return S}
function skelGraph(S,w,h){const N8=[-w-1,-w,-w+1,-1,1,w-1,w,w+1],DX=[-1,0,1,-1,1,-1,0,1];
 const nbs=i=>{const x=i%w,o=[];for(let k=0;k<8;k++){const X=x+DX[k];if(X<0||X>=w)continue;const j=i+N8[k];if(j<0||j>=w*h||!S[j])continue;o.push(j)}return o};
 const deg=new Uint8Array(w*h),node=new Int32Array(w*h).fill(-1),vis=new Uint8Array(w*h),pix=[];for(let i=0;i<w*h;i++)if(S[i]){pix.push(i);deg[i]=nbs(i).length}
 const N=[];for(const i of pix){if(deg[i]===2||node[i]>=0)continue;const id=N.length,mem=[i];node[i]=id;
  if(deg[i]>2){const st=[i];while(st.length){const j=st.pop();for(const k of nbs(j))if(deg[k]>2&&node[k]<0){node[k]=id;mem.push(k);st.push(k)}}}
  let sx=0,sy=0;for(const j of mem){sx+=j%w;sy+=(j-j%w)/w}N.push({mem,x:sx/mem.length,y:sy/mem.length,ends:[]})}
 const E=[],pr=new Set(),xy=i=>[i%w,(i-i%w)/w];
 for(let a=0;a<N.length;a++)for(const s of N[a].mem)for(const t of nbs(s)){if(node[t]===a)continue;
  if(node[t]>=0){const b=node[t],k=Math.min(a,b)+'_'+Math.max(a,b)+'_'+Math.min(s,t)+'_'+Math.max(s,t);if(pr.has(k))continue;pr.add(k);
   if(N[a].mem.length>1&&N[b].mem.length>1)continue;E.push({a,b,p:[xy(s),xy(t)]});continue}
  if(vis[t])continue;const path=[xy(s),xy(t)];vis[t]=1;let prev=s,cur=t;
  for(;;){const nx=nbs(cur).filter(k=>k!==prev&&!(node[k]<0&&vis[k]));let k=nx.find(q=>node[q]>=0&&!(node[q]===a&&q===s&&path.length<3));if(k===undefined)k=nx.find(q=>node[q]<0);
   if(k===undefined)break;path.push(xy(k));if(node[k]>=0){cur=k;break}vis[k]=1;prev=cur;cur=k}
  E.push({a,b:node[cur]>=0?node[cur]:-1,p:path})}
 for(const i of pix){if(vis[i]||node[i]>=0)continue;const path=[xy(i)];vis[i]=1;let prev=-1,cur=i;   // 노드 없는 고리
  for(;;){const nx=nbs(cur).filter(k=>k!==prev&&!vis[k]);if(!nx.length)break;vis[nx[0]]=1;path.push(xy(nx[0]));prev=cur;cur=nx[0]}path.push(path[0].slice());E.push({a:-1,b:-1,p:path,loop:true})}
 return {N,E}}
function plenA(P){let L=0;for(let i=1;i<P.length;i++)L+=Math.hypot(P[i][0]-P[i-1][0],P[i][1]-P[i-1][1]);return L}
function graphStrokes(G,B,w,h,hw,spurPx,extend,penR){const {N,E}=G,cutR=hw+1.5,mergeL=Math.max(3,2*hw+2);
 const alive=E.map(()=>true),par=N.map((_,i)=>i),fd=i=>{while(par[i]!==i)i=par[i]=par[par[i]];return i},tips=N.map(()=>[]);
 const nd=(e,s)=>{const v=s?e.b:e.a;return v<0?-1:fd(v)};
 const degs=()=>{const d=new Array(N.length).fill(0);E.forEach((e,i)=>{if(!alive[i]||e.loop)return;const a=nd(e,0),b=nd(e,1);if(a>=0)d[a]++;if(b>=0)d[b]++});return d};
 let pruned=0;for(let r=0;r<4;r++){let any=false;let d=degs();
  E.forEach((e,i)=>{if(!alive[i]||e.loop)return;const a=nd(e,0),b=nd(e,1);if(a<0||b<0)return;if(a===b&&plenA(e.p)<mergeL){alive[i]=false;any=true;return}
   const ta=d[a]===1,tb=d[b]===1;if(((ta&&d[b]>=3)||(tb&&d[a]>=3))&&plenA(e.p)<spurPx){alive[i]=false;pruned++;any=true;d[a]--;d[b]--;const tp=ta?e.p[0]:e.p[e.p.length-1];tips[ta?b:a].push(tp.slice())}});
  d=degs();E.forEach((e,i)=>{if(!alive[i]||e.loop)return;const a=nd(e,0),b=nd(e,1);if(a<0||b<0||a===b)return;if(d[a]>=3&&d[b]>=3&&plenA(e.p)<mergeL){alive[i]=false;any=true;
   const na=N[a],nb=N[b],ma=na.mem.length,mb=nb.mem.length;nb.x=(na.x*ma+nb.x*mb)/(ma+mb);nb.y=(na.y*ma+nb.y*mb)/(ma+mb);nb.mem=nb.mem.concat(na.mem);tips[b]=tips[b].concat(tips[a]);par[a]=b}});
  if(!any)break}
 const d=degs(),Ed=[];E.forEach((e,i)=>{if(alive[i])Ed.push({a:e.loop?-1:nd(e,0),b:e.loop?-1:nd(e,1),p:e.p,loop:!!e.loop})});
 const ends=N.map(()=>[]);Ed.forEach((e,i)=>{if(e.loop)return;if(e.a>=0)ends[e.a].push([i,0]);if(e.b>=0)ends[e.b].push([i,1])});
 const K=(i,s)=>i*2+s,nodeOf=(i,s)=>s?Ed[i].b:Ed[i].a,ctr=n=>[N[n].x,N[n].y],dist=(p,q)=>Math.hypot(p[0]-q[0],p[1]-q[1]);
 const eDir=(e,s)=>{const P=s?e.p.slice().reverse():e.p,L0=Math.max(4,2*hw);let a=0,k=1;for(;k<P.length-1;k++){a+=Math.hypot(P[k][0]-P[k-1][0],P[k][1]-P[k-1][1]);if(a>=L0)break}const q=P[Math.min(k,P.length-1)],dx=q[0]-P[0][0],dy=q[1]-P[0][1],l=Math.hypot(dx,dy)||1;return [dx/l,dy/l]};
 const bent=n=>{const L=[];Ed.forEach(e=>{if(e.loop)return;if(e.a===n)L.push(eDir(e,0));if(e.b===n)L.push(eDir(e,1))});return L.length===2&&L[0][0]*L[1][0]+L[0][1]*L[1][1]>-0.85};   // 두 가지가 148° 보다 꺾임
 const kind=N.map((_,n)=>d[n]>=3?'J':(d[n]===2&&tips[n].length&&bent(n)?'C':(d[n]===1&&N[n].mem.length===1?'T':'P')));     // 갈림 · 모서리 · 끝 · 지남
 const cutOf=n=>(n>=0&&(kind[n]==='J'||kind[n]==='C'))?cutR:0;
 const ori=(i,s)=>s?Ed[i].p.slice().reverse():Ed[i].p.slice();                                           // s 쪽 노드에서 나가는 방향
 const trimmed=(i,s)=>{let P=ori(i,s);const n0=nodeOf(i,s),n1=nodeOf(i,1-s),c0=cutOf(n0),c1=cutOf(n1);
  if(c0>0){const C=ctr(n0);let k=0;while(k<P.length-2&&dist(P[k],C)<c0)k++;P=P.slice(k)}if(c1>0){const C=ctr(n1);let k=P.length-1;while(k>1&&dist(P[k],C)<c1)k--;P=P.slice(0,k+1)}return P};
 const outDir=(P,fromStart)=>{const Q=fromStart?P:P.slice().reverse(),L0=Math.max(3,hw*2);let s=0,k=1;for(;k<Q.length-1;k++){s+=dist(Q[k],Q[k-1]);if(s>=L0)break}
  const a=Q[Math.min(k,Q.length-1)],dx=Q[0][0]-a[0],dy=Q[0][1]-a[1],l=Math.hypot(dx,dy)||1;return [dx/l,dy/l]};          // 끝에서 밖으로
 const link=new Map(),bridge=new Map();
 ends.forEach((L,n)=>{if(kind[n]==='P'&&L.length===2){link.set(K(...L[0]),L[1]);link.set(K(...L[1]),L[0]);return}
  if(kind[n]==='C'&&L.length===2){const tp=tips[n].reduce((a,q)=>[a[0]+q[0]/tips[n].length,a[1]+q[1]/tips[n].length],[0,0]);link.set(K(...L[0]),L[1]);link.set(K(...L[1]),L[0]);bridge.set(K(...L[0]),[tp]);bridge.set(K(...L[1]),[tp]);return}
  if(kind[n]!=='J')return;const V=L.map(([i,s])=>{const P=trimmed(i,s);return outDir(P,true).map(v=>-v)}),free=L.map(()=>true);   // 노드에서 가지로 나가는 방향
  for(;;){let best=0.34,bi=-1,bj=-1;for(let a=0;a<L.length;a++)for(let b=a+1;b<L.length;b++){if(!free[a]||!free[b])continue;const c=-(V[a][0]*V[b][0]+V[a][1]*V[b][1]);if(c>best){best=c;bi=a;bj=b}}
   if(bi<0)break;free[bi]=free[bj]=false;link.set(K(...L[bi]),L[bj]);link.set(K(...L[bj]),L[bi]);const mid=best>0.94?[]:[ctr(n)];bridge.set(K(...L[bi]),mid);bridge.set(K(...L[bj]),mid)}});
 // 끝 처리: 갈림길에서 끝나는 가지는 같은 노드를 지나는 다리까지 곧게, 없으면 노드 가운데 · 줄 끝은 검은 곳 끝 − 펜 반지름까지
 const segsAt=n=>{const S=[];ends[n].forEach(([i,s])=>{const o=link.get(K(i,s));if(!o)return;const A=trimmed(i,s)[0],Bp=trimmed(o[0],o[1])[0],m=bridge.get(K(i,s))||[];const Q=[A,...m,Bp];for(let k=1;k<Q.length;k++)S.push([Q[k-1],Q[k]])});return S};
 const rayHit=(p,dv,S,maxT)=>{let best=null;for(const [a,b] of S){const ex=b[0]-a[0],ey=b[1]-a[1],den=dv[0]*ey-dv[1]*ex;if(Math.abs(den)<1e-9)continue;
   const t=((a[0]-p[0])*ey-(a[1]-p[1])*ex)/den,u=((a[0]-p[0])*dv[1]-(a[1]-p[1])*dv[0])/den;if(t>=-0.5&&t<=maxT&&u>=-0.05&&u<=1.05&&(!best||t<best[2]))best=[p[0]+t*dv[0],p[1]+t*dv[1],t]}return best};
 const march=(p,dv)=>{let t=0;for(;t<200;t+=0.5){const X=Math.round(p[0]+dv[0]*t),Y=Math.round(p[1]+dv[1]*t);if(X<0||Y<0||X>=w||Y>=h||!B[Y*w+X])break}return t};
 const endPt=(P,atStart,n)=>{if(n<0)return null;const dv=outDir(P,atStart),e=atStart?P[0]:P[P.length-1];
  if(kind[n]==='J'){const hit=rayHit(e,dv,segsAt(n),4*cutR);return hit?[hit[0],hit[1]]:ctr(n)}
  if(kind[n]==='T'&&extend){const ext=march(e,dv)-0.5-penR;return ext>0.3?[e[0]+dv[0]*ext,e[1]+dv[1]*ext]:null}return null};
 const used=new Uint8Array(Ed.length),out=[];
 const A=p=>[p[0],p[1],1];                                                                                 // 매끈하게 할 때 움직이지 않는 점(모서리 · 다리 끝)
 const walk=(i,s)=>{let P=trimmed(i,s),ci=i,cs=s;const first=[i,s];for(;;){used[ci]=1;const nx=link.get(K(ci,1-cs));if(!nx||used[nx[0]]&&!(nx[0]===first[0]&&nx[1]===first[1]))return {P,closed:false,last:[ci,1-cs]};
   const m=(bridge.get(K(ci,1-cs))||[]).map(A),pl=P.length-1,hasB=cutOf(nodeOf(ci,1-cs))>0;if(hasB)P[pl]=A(P[pl]);
   if(nx[0]===first[0]&&nx[1]===first[1]){if(hasB)P[0]=A(P[0]);P=P.concat(m,[P[0].slice()]);return {P,closed:true}}
   const Q=trimmed(nx[0],nx[1]);if(hasB)Q[0]=A(Q[0]);P=P.concat(m,Q);ci=nx[0];cs=nx[1]}};
 Ed.forEach((e,i)=>{if(used[i]||e.loop)return;for(const s of [0,1])if(!link.has(K(i,s))){const o=walk(i,s);const a=endPt(o.P,true,nodeOf(i,s)),b=endPt(o.P,false,nodeOf(...o.last));
   if(a)o.P.unshift(a);if(b)o.P.push(b);out.push(o.P);break}});
 Ed.forEach((e,i)=>{if(used[i])return;if(e.loop){used[i]=1;out.push(e.p)}else out.push(walk(i,0).P)});
 const had=new Uint8Array(N.length);E.forEach(e=>{if(e.a>=0)had[e.a]=1;if(e.b>=0)had[e.b]=1});N.forEach((nn,n)=>{if(had[n]||fd(n)!==n||nn.mem.length>64)return;out.push([[nn.x,nn.y]])});        // 점(골격이 한 점) — 한 번 찍기
 return {S:out,pruned}}
function smoothP(P,sig){if(sig<=0||P.length<4)return P;const n=P.length,closed=Math.hypot(P[0][0]-P[n-1][0],P[0][1]-P[n-1][1])<1e-6,R=Math.ceil(2*sig),W=[];
 for(let k=-R;k<=R;k++)W.push(Math.exp(-k*k/(2*sig*sig)));const anc=P.map((p,i)=>!!p[2]||(!closed&&(i===0||i===n-1)));
 if(closed&&!anc.some(Boolean)){const m=n-1,out=[];for(let i=0;i<m;i++){let sx=0,sy=0,sw=0;for(let k=-R;k<=R;k++){const j=((i+k)%m+m)%m,wt=W[k+R];sx+=P[j][0]*wt;sy+=P[j][1]*wt;sw+=wt}out.push([sx/sw,sy/sw])}out.push(out[0].slice());return out}
 if(closed)anc[0]=anc[n-1]=true;const out=P.map(p=>p.slice());
 for(let i=0;i<n;i++){if(anc[i])continue;let a=i,b=i;while(a>0&&!anc[a])a--;while(b<n-1&&!anc[b])b++;const lim=Math.min(i-a,b-i);let sx=0,sy=0,sw=0;
  for(let k=-Math.min(R,lim);k<=Math.min(R,lim);k++){const wt=W[k+R];sx+=P[i+k][0]*wt;sy+=P[i+k][1]*wt;sw+=wt}out[i]=[sx/sw,sy/sw]}return out}
function sharpen(P,hw){const n=P.length;if(n<5)return P;const closed=Math.hypot(P[0][0]-P[n-1][0],P[0][1]-P[n-1][1])<1e-6,S=[0];
 for(let i=1;i<n;i++)S.push(S[i-1]+Math.hypot(P[i][0]-P[i-1][0],P[i][1]-P[i-1][1]));const T=S[n-1],L1=Math.max(3,1.5*hw),La=Math.max(2,hw+1),Lb=La+Math.max(6,3*hw);
 const at=s=>{if(closed)s=((s%T)+T)%T;else s=Math.max(0,Math.min(T,s));let k=1;while(k<n-1&&S[k]<s)k++;const t=(s-S[k-1])/Math.max(1e-9,S[k]-S[k-1]);return [P[k-1][0]+t*(P[k][0]-P[k-1][0]),P[k-1][1]+t*(P[k][1]-P[k-1][1])]};
 const turn=(i,L)=>{if(!closed&&(S[i]<L||T-S[i]<L))return 0;const a=at(S[i]-L),b=at(S[i]+L),u=[P[i][0]-a[0],P[i][1]-a[1]],v=[b[0]-P[i][0],b[1]-P[i][1]],nu=Math.hypot(...u),nv=Math.hypot(...v);
  if(nu<1e-6||nv<1e-6)return 0;return Math.acos(Math.max(-1,Math.min(1,(u[0]*v[0]+u[1]*v[1])/(nu*nv))))};
 const th=P.map((_,i)=>turn(i,L1)),cand=[];for(let i=1;i<n-1;i++){if(th[i]<0.6)continue;let j=i,mx=th[i];while(j+1<n-1&&th[j+1]>=0.6){j++;mx=Math.max(mx,th[j])}     // 꺾임 구간 [i, j]
  if(mx>=0.7){const sm=(S[i]+S[j])/2;let c=i;while(c<j&&S[c]<sm)c++;const La2=Math.max(La,(S[j]-S[i])/2+2),Lb2=La2+Math.max(6,3*hw);if(turn(c,Lb2)>0.5)cand.push([c,La2,Lb2])}i=j}
 if(!cand.length)return P;const rep=[];
 for(const [i,La,Lb] of cand){const a0=at(S[i]-Lb),a1=at(S[i]-La),b0=at(S[i]+La),b1=at(S[i]+Lb),d1=[a1[0]-a0[0],a1[1]-a0[1]],d2=[b1[0]-b0[0],b1[1]-b0[1]],den=d1[0]*d2[1]-d1[1]*d2[0];
  if(Math.abs(den)<1e-9)continue;const res=(p0,p1,s0,s1)=>{const dx=p1[0]-p0[0],dy=p1[1]-p0[1],l=Math.hypot(dx,dy)||1;let m=0;for(let k=0;k<n;k++){let sk=S[k];if(closed){const dd=((sk-S[i])%T+1.5*T)%T-T/2;sk=S[i]+dd}if(sk<s0||sk>s1)continue;m=Math.max(m,Math.abs(dy*(P[k][0]-p0[0])-dx*(P[k][1]-p0[1]))/l)}return m};
  const tolR=Math.max(0.6,0.05*(Lb-La));if(res(a0,a1,S[i]-Lb,S[i]-La)>tolR||res(b0,b1,S[i]+La,S[i]+Lb)>tolR)continue;                                   // 다리가 곧아야 모서리
  const t=((b0[0]-a1[0])*d2[1]-(b0[1]-a1[1])*d2[0])/den,X=[a1[0]+t*d1[0],a1[1]+t*d1[1]];if(t<-0.2||Math.hypot(X[0]-P[i][0],X[1]-P[i][1])>2*hw+4)continue;
  rep.push({i,s0:S[i]-La,s1:S[i]+La,X})}
 if(!rep.length)return P;const out=[];let r=0;for(let i=0;i<n;i++){const c=rep[r];if(c&&S[i]>c.s0&&S[i]<c.s1){if(S[i]>=S[c.i]){out.push([c.X[0],c.X[1],1]);r++;while(i+1<n&&S[i+1]<c.s1)i++}continue}out.push(P[i])}
 return out}
function rdpA(P,eps){const idx=[0];for(let i=1;i<P.length-1;i++)if(P[i][2])idx.push(i);idx.push(P.length-1);let out=[];
 for(let k=1;k<idx.length;k++){const seg=rdp(P.slice(idx[k-1],idx[k]+1).map(p=>[p[0],p[1]]),eps);out=out.length?out.concat(seg.slice(1)):seg}return out}
function orderStrokes(S,start,join){const n=S.length,t0=Date.now();if(!n)return {S,tr0:0,tr1:0,lifts:0};
 const d=(a,b)=>Math.hypot(a[0]-b[0],a[1]-b[1]),cl=P=>P.length>2&&d(P[0],P[P.length-1])<1e-6;
 let tr0=d(start,S[0][0]);for(let i=1;i<n;i++)tr0+=d(S[i-1][S[i-1].length-1],S[i][0]);
 const left=new Set(S.map((_,i)=>i)),seq=[];let cur=start;
 while(left.size){let bi=-1,bd=1e18,bm=0;for(const i of left){const P=S[i];if(cl(P)){for(let k=0;k<P.length-1;k+=Math.max(1,Math.floor(P.length/64))){const v=d(cur,P[k]);if(v<bd){bd=v;bi=i;bm=2+k}}}
   else{const a=d(cur,P[0]),b=d(cur,P[P.length-1]);if(a<bd){bd=a;bi=i;bm=0}if(b<bd){bd=b;bi=i;bm=1}}}
  let P=S[bi];if(bm===1)P=P.slice().reverse();else if(bm>=2){const k=bm-2;P=P.slice(k,P.length-1).concat(P.slice(0,k+1))}
  seq.push(P);left.delete(bi);cur=P[P.length-1]}
 const st=i=>seq[i][0],en=i=>seq[i][seq[i].length-1];let imp=true,pass=0;
 while(imp&&pass<20&&Date.now()-t0<600&&seq.length<2500){imp=false;pass++;for(let i=0;i<seq.length-1;i++){const pe=i?en(i-1):start;
   for(let j=i+1;j<seq.length;j++){const nx=j+1<seq.length?st(j+1):null,dl=d(pe,en(j))+(nx?d(st(i),nx):0)-d(pe,st(i))-(nx?d(en(j),nx):0);
    if(dl<-1e-9){const r=seq.slice(i,j+1).reverse().map(P=>P.slice().reverse());seq.splice(i,j-i+1,...r);imp=true}}}}
 const out=[seq[0].slice()];for(let i=1;i<seq.length;i++){const L=out[out.length-1];if(d(L[L.length-1],seq[i][0])<=join)out[out.length-1]=L.concat(seq[i].slice(d(L[L.length-1],seq[i][0])<1e-9?1:0));else out.push(seq[i].slice())}
 let tr1=d(start,out[0][0]);for(let i=1;i<out.length;i++)tr1+=d(out[i-1][out[i-1].length-1],out[i][0]);return {S:out,tr0,tr1,lifts:out.length-1}}
function major3(B,w,h){const O=new Uint8Array(w*h);for(let y=0;y<h;y++)for(let x=0;x<w;x++){let c=0;for(let dy=-1;dy<=1;dy++){const Y=y+dy;if(Y<0||Y>=h)continue;for(let dx=-1;dx<=1;dx++){const X=x+dx;if(X>=0&&X<w)c+=B[Y*w+X]}}O[y*w+x]=c>=5?1:0}return O}
function keepBlobs(B,S,D,w,h){const lab=new Int32Array(w*h).fill(-1),st=[];let nAdd=0;          // 얇게 깎다 통째로 없어진 덩어리 → 가장 굵은 곳에 한 점
 for(let i0=0;i0<w*h;i0++){if(!B[i0]||lab[i0]>=0)continue;lab[i0]=i0;st.push(i0);let has=0,bi=i0;
  while(st.length){const i=st.pop(),x=i%w;if(S[i])has=1;if(D[i]>D[bi])bi=i;for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){const X=x+dx,j=i+dy*w+dx;if(X<0||X>=w||j<0||j>=w*h||!B[j]||lab[j]>=0)continue;lab[j]=i0;st.push(j)}}
  if(!has){S[bi]=1;nAdd++}}return nAdd}
function centerTrace(B0,w,h,o){let B=B0,D=distT(B,w,h),S=thinZS(B,w,h);keepBlobs(B,S,D,w,h);const med=()=>{const v=[];for(let i=0;i<w*h;i++)if(S[i]&&D[i]<1e8)v.push(D[i]-0.5);v.sort((a,b)=>a-b);return v.length?v[v.length>>1]:0};
 if(med()>=1.5){B=major3(B0,w,h);D=distT(B,w,h);S=thinZS(B,w,h);keepBlobs(B,S,D,w,h)}                                                 // 굵은 선: 가장자리 한 화소 혹 · 홈을 다수결로 지우고 다시
 const G=skelGraph(S,w,h),hwMed=med(),spur=Math.max(o.spurPx,2*hwMed+1),R=graphStrokes(G,B,w,h,hwMed,spur,o.extend,o.penRpx);
 const P=R.S.map(p=>p.length<2?[[p[0][0],p[0][1]]]:rdpA(smoothP(sharpen(p,hwMed),o.sig),o.tol)).filter(p=>p.length>=1);let nFill=0;
 for(let j=0;j<P.length;j++){const Q=P[j];if(Q.length<2)continue;const n=Q.length,L=plenA(Q);if(Math.hypot(Q[0][0]-Q[n-1][0],Q[0][1]-Q[n-1][1])<1e-6)continue;   // 짧고 굵은 덩어리(골격이 짧은 토막) → 점
  let a=0,c=Q[0];for(let k=1;k<n;k++){const l=Math.hypot(Q[k][0]-Q[k-1][0],Q[k][1]-Q[k-1][1]);if(a+l>=L/2){const t=(L/2-a)/Math.max(l,1e-9);c=[Q[k-1][0]+t*(Q[k][0]-Q[k-1][0]),Q[k-1][1]+t*(Q[k][1]-Q[k-1][1])];break}a+=l}
  const xi=Math.round(c[0]),yi=Math.round(c[1]);let r=0;for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){const X=xi+dx,Y=yi+dy;if(X>=0&&Y>=0&&X<w&&Y<h&&D[Y*w+X]<1e8)r=Math.max(r,D[Y*w+X]-0.5)}
  if(L<=2*r+1)P[j]=[c]}
 if(o.fillDots&&o.penRpx>0)for(let j=0;j<P.length;j++){if(P[j].length!==1)continue;const Q=dotSpiral(P[j][0],D,w,h,o.penRpx);if(Q){P[j]=Q;nFill++}}   // 굵은 점: 펜 굵기로 안쪽에서 바깥으로 나선
 return {S:P,hwMed,pruned:R.pruned,nodes:G.N.length,edges:G.E.length,nFill}}
function dotSpiral(c,D,w,h,pr){const xi=Math.round(c[0]),yi=Math.round(c[1]);let r=0;
 for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){const X=xi+dx,Y=yi+dy;if(X>=0&&Y>=0&&X<w&&Y<h&&D[Y*w+X]<1e8)r=Math.max(r,D[Y*w+X]-0.5)}
 const rr=r-pr;if(rr<0.75*pr)return null;const pitch=1.6*pr,Q=[[c[0],c[1]]],TW=2*Math.PI;let th=0,end=-1;      // 나선 간격 = 펜 굵기의 80 % (겹치게) · 끝에 한 바퀴 더
 for(let it=0;it<20000;it++){const rad=Math.min(rr,pitch*th/TW);if(rad>=rr&&end<0)end=th+TW;if(end>=0&&th>=end)break;th+=Math.min(0.5,1/Math.max(rad,1));const rd=Math.min(rr,pitch*th/TW);Q.push([c[0]+rd*Math.cos(th),c[1]+rd*Math.sin(th)])}
 return Q}
function traceImg(){if(!IMG)return;const R=binImg(),{B,w,h}=R,Wmm=Math.max(5,n('iW')),k=Wmm/w,minL=n('iMin'),tol=Math.max(0.2,n('iTol'));IMGR=R;
 if(R.nOn===0||R.nOn===w*h){IMGS=null;$('iInfo').textContent=`그릴 곳이 ${R.nOn?'전부':'없다'} — 문턱(지금 ${R.th.toFixed(0)}) · '밝은 쪽을 그림'을 바꾼다`;drawIPrev(R,[]);ghost();return}
 const sp=Math.max(0,n('iSpeck'))/(k*k),cl=cleanBin(B,w,h,sp,sp);let S1=[],extra='';
 if(imode==='center'){const C=centerTrace(B,w,h,{spurPx:Math.max(0,n('iSpur'))/k,extend:$('iExt').checked,penRpx:Math.max(0,n('iPen'))/2/k,sig:1.0,tol,fillDots:$('iDot').checked});
  C.S.forEach(P=>{if(P.length===1||plen2(P)*k>=minL)S1.push(P)});extra=` · 선 굵기 가운데 ${(2*C.hwMed*k).toFixed(2)} mm`+(C.pruned?` · 잔가지 ${C.pruned}`:'')+(C.nFill?` · 채운 점 ${C.nFill}`:'')}
 else{if(imode!=='hatch')contours(B,w,h).forEach(P=>{const Q=rdp(P,tol);if(plen2(Q)*k>=minL)S1.push(Q)});
  if(imode!=='outline'){const sp2=Math.max(0.3,n('iS'))/k;hatchRuns(B,w,h,sp2).forEach(P=>{if(plen2(P)*k>=Math.min(minL,1))S1.push(P)})}}
 let strokes=S1.map(P=>P.map(p=>[p[0]*k,(h-p[1])*k])),tr0=0,tr1=0,lifts=Math.max(0,strokes.length-1);
 const st=[-n('ix'),-n('iy')],trav=S=>{let t=S.length?Math.hypot(S[0][0][0]-st[0],S[0][0][1]-st[1]):0;for(let i=1;i<S.length;i++){const a=S[i-1][S[i-1].length-1],b=S[i][0];t+=Math.hypot(a[0]-b[0],a[1]-b[1])}return t};
 if($('iOrd').checked&&strokes.length>1){const o=orderStrokes(strokes,st,Math.max(0,n('iJoin')));strokes=o.S;tr0=o.tr0;tr1=o.tr1;lifts=o.lifts}else tr1=trav(strokes);
 strokes=strokes.map(P=>P.map(p=>[+p[0].toFixed(2),+p[1].toFixed(2)]));IMGS=strokes.length?strokes:null;IMGWH=[Wmm,h*k];
 const L=strokes.reduce((a,s)=>a+plen2(s),0),np=strokes.reduce((a,s)=>a+s.length,0),nd=strokes.filter(s=>s.length===1).length;
 $('iInfo').textContent=strokes.length?`${strokes.length}획${nd?` (점 ${nd})`:''} · 펜 들기 ${lifts} · 그리기 ${(L/1000).toFixed(2)} m · 옮기기 ${(tr1/1000).toFixed(2)} m${tr0?` (정리 전 ${(tr0/1000).toFixed(2)})`:''} · 점 ${np} · ${Wmm.toFixed(0)} × ${(h*k).toFixed(0)} mm · ${w}×${h} px (${k.toFixed(3)} mm/px) · 문턱 ${R.th.toFixed(0)}${R.auto?' (자동)':''}${extra}${cl.nS||cl.nH?` · 티 ${cl.nS} · 구멍 ${cl.nH}`:''} · 걸리는 시간은 넣은 뒤 [계획 보기]에서 (펜 들기 한 번에 몇 초)`+(strokes.length>3000?' — 획이 많다: 정밀도 · 짧은 선 버림 · 티 지우기를 키운다':''):'그릴 선이 없다 — 짧은 선 버림 · 티 지우기를 줄인다';
 drawIPrev(R,strokes);ghost()}
function drawIPrev(R,strokes){const c=$('iPrev'),x=c.getContext('2d'),{B,w,h}=R,s=Math.min(c.width/w,c.height/h);x.clearRect(0,0,c.width,c.height);
 const im=x.createImageData(w,h);for(let i=0;i<w*h;i++){const v=B[i]?200:250;im.data[4*i]=im.data[4*i+1]=im.data[4*i+2]=v;im.data[4*i+3]=255}
 const t=document.createElement('canvas');t.width=w;t.height=h;t.getContext('2d').putImageData(im,0,0);x.imageSmoothingEnabled=false;x.drawImage(t,0,0,w*s,h*s);
 if(!strokes.length)return;const k=IMGWH[0]/w,X=p=>p[0]/k*s,Y=p=>(h-p[1]/k)*s;
 x.setLineDash([3,4]);x.strokeStyle=css('--mute');x.lineWidth=1;x.beginPath();for(let i=1;i<strokes.length;i++){const a=strokes[i-1][strokes[i-1].length-1],b=strokes[i][0];x.moveTo(X(a),Y(a));x.lineTo(X(b),Y(b))}x.stroke();x.setLineDash([]);
 x.strokeStyle=css('--b');x.lineWidth=3;x.beginPath();strokes.forEach(P=>{if(P.length===1)return;P.forEach((p,i)=>i?x.lineTo(X(p),Y(p)):x.moveTo(X(p),Y(p)))});x.stroke();
 x.fillStyle=css('--b');strokes.forEach(P=>{if(P.length!==1)return;x.beginPath();x.arc(X(P[0]),Y(P[0]),3.5,0,6.283);x.fill()})}
function addImage(){if(!IMGS){toast('그림 파일을 먼저 고른다');return}addItem({type:'draw',name:IMGN||'그림',strokes:IMGS,smooth:0,x:n('ix'),y:n('iy'),z:n('iz')})}
// ── 상태 ──
function chip(id,t,c){const e=$(id);e.textContent=t;e.className='chip'+(c?' '+c:'')}
let lastLog='',lastJ=null;
async function tick(){try{S=await (await fetch('/api/pstatus')).json();const J=S.jog,p=J.panel;$('ver').textContent='v'+S.v+' · 조그 '+S.jog_v;
 chip('cPanel',p.ok?'제어판 '+(p.mode||'?'):'제어판 연결 안 됨',p.ok&&p.mode==='engaged'?'on':'warn');chip('cFol',p.following?'추종 중':'추종 꺼짐',p.following?'on':'');chip('cSend',J.send?'보내는 중':'안 보냄',J.send?'blue':'');
 $('cJog').style.display=S.jog8793?'':'none';
 document.body.classList.toggle('busy',J.busy);if(Date.now()-lastToast>4000){$('band').textContent=J.phase+(J.msg?' — '+J.msg:'');$('band').style.color=J.phase==='준비'?'var(--ok)':(J.phase==='움직이는 중'?'var(--piv)':'')}
 $('bSend').disabled=J.busy||J.send;$('bRel').disabled=!J.send||J.busy;const ok=ready();
 document.querySelectorAll('[data-j],[data-v],[data-gd]').forEach(b=>b.disabled=!ok);
 document.querySelectorAll('[data-s]').forEach(b=>b.classList.toggle('on',+b.dataset.s===J.step));document.querySelectorAll('[data-r]').forEach(b=>b.classList.toggle('on',+b.dataset.r===S.speed));
 $('gv').textContent=Math.round(J.g);if(document.activeElement!==$('toolLen'))$('toolLen').value=S.o.tool_len;if(!optsLoaded){fillOpts(S.o);optsLoaded=true}else{$('airChip').classList.toggle('air',S.o.air_dz>0)}
 const bd=S.bed;$('bedLine').textContent=bd?`판 ${bd.W.toFixed(1)} × ${bd.H.toFixed(1)} mm · 기울기 ${bd.tilt}° · ${bd.mode==='three'?'3점':'1점'} · ${bd.made.replace('T',' ')} · 공구 ${bd.tool_len} mm`+(Math.abs(bd.tool_len-S.o.tool_len)>0.05?' ⚠ 지금 공구와 다름':''):'아직 안 맞춤';
 $('tipB').textContent=S.tip_bed?`판 (${S.tip_bed.map(v=>v.toFixed(1)).join(', ')})`:(S.tip?`베이스 (${S.tip.map(v=>v.toFixed(0)).join(', ')})`:'보내기 전');
 renderMarks();
 const r=S.run;if(r&&r.state&&r.state!=='idle'){const tot=r.n||1;const fr=Math.min(1,r.k/tot);
  if(r.state==='running'){$('progI').style.width=(100*fr)+'%';$('progT').textContent=`${r.name} — 동작 ${r.op+1}/${r.n_ops} · ${fmtT(r.t)} 지남 · 남은 ${fmtT(r.remain||0)}`+(S.speed<1?` (속도 ${S.speed*100}%)`:'')+(S.pause_req?' · 획 끝에서 멈춤 기다리는 중':'')+` · 뒤처짐 ${r.lag}°`}
  else if(lastJ!==r.state+r.k){$('progI').style.width=(100*fr)+'%';$('progT').textContent=r.name==='바로 따라 그리기'?(r.state==='done'?'바로 따라 그리기 — 획 끝':`바로 따라 그리기 멈춤: ${r.msg}`):r.state==='done'?`끝 — ${r.name}`:(r.state==='paused'?`멈춤: ${r.msg} — [이어서]는 동작 ${r.resume_op+1}부터`:`멈춤: ${r.msg} — [이어서]는 동작 ${r.resume_op+1} 처음부터 (먼저 곧게 들어 올린다)`)}
  lastJ=r.state+r.k}
 const lg=J.log.slice().reverse().join('\n');if(lg!==lastLog){lastLog=lg;$('log').textContent=lg}
 buttons();liveStep();renderLive();const bk=S.bed?S.bed.made:'';if(bk!==lastBed){lastBed=bk;fit()}const sig=JSON.stringify([S.tip_bed,r&&r.k,r&&r.state,S.bed&&S.bed.made]);if(!animT&&sig!==lastSig){lastSig=sig;redraw()}}catch(e){$('band').textContent='fr5print 서버 연결 끊김';$('band').style.color='var(--warn)'}}
let lastSig='',lastBed=null;
// 단추를 누른 뒤 초점을 풀어 Space 가 그 단추를 다시 누르지 않게 (Space = 멈춤만)
document.addEventListener('click',e=>{const b=e.target.closest&&e.target.closest('button');if(b)b.blur()},true);
document.addEventListener('keydown',e=>{const k=e.key;if(k===' '||k==='Escape'){const tg=e.target.tagName;if(k===' '&&(tg==='TEXTAREA'||(tg==='INPUT'&&e.target.type==='text')))return;e.preventDefault();if(document.activeElement&&document.activeElement.blur)document.activeElement.blur();stopAll();return}
 const tg=e.target.tagName;if(tg==='INPUT'||tg==='TEXTAREA'||tg==='SELECT')return;if(/^[1-7]$/.test(k)){post('/api/set',{step:STEPS[+k-1]});return}
 const b=document.querySelector(`[data-k="${k}"]`);if(b&&!b.disabled){e.preventDefault();b.click()}});
$('jobName').addEventListener('change',()=>{job.name=$('jobName').value;store();buttons()});
window.addEventListener('resize',fit);
restore();$('jobName').value=job.name||'작업';buildShapes();buildOpts();setMk(mkTab);renderItems();updDraft();listJobs();fit();setInterval(tick,300);tick().then(()=>{refresh();fit()});
</script></body></html>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8796)
    ap.add_argument("--dir", default=HERE, help="판 · 설정 · 작업 파일 폴더 (기본: 이 파일이 있는 폴더)")
    ap.add_argument("--poses", default=os.path.join(HERE, "poses.json"), help="조그 자세 파일")
    ap.add_argument("--v", type=float, default=10.0, help="조그 곧게 속도 mm/s")
    ap.add_argument("--v-go", type=float, default=30.0)
    ap.add_argument("--safe-z", type=float, default=380.0)
    ap.add_argument("--max-lag", type=float, default=5.0)
    ap.add_argument("--udp-host", default="127.0.0.1")
    ap.add_argument("--udp-port", type=int, default=8097)
    a = ap.parse_args()
    jm = hashlib.md5(open(J.__file__, "rb").read()).hexdigest()[:8]
    if jm != "d60f8e6e":
        print(f"  주의: fr5jog_ui.py md5 {jm} — 시험한 판은 d60f8e6e (0.1.1)")
    try:
        srv = ThreadingHTTPServer(("127.0.0.1", a.port), PrH)
    except OSError:
        sys.exit(f"{a.port} 포트를 이미 쓰고 있다 — 이미 켜진 fr5print 를 끄거나 --port 8797 로 켠다")
    PrH.app = PrintApp(a)
    print(f"fr5print {VERSION} · http://localhost:{a.port} · 조그 화면 http://localhost:{a.port}/jog · 폴더 {PrH.app.dir} · 끝내기 Ctrl+C")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        PrH.app.release()
        print("끝 — 보내기를 멈췄다 (제어판이 0.3 s 뒤 그 자리에 선다)")


if __name__ == "__main__":
    main()
