#!/usr/bin/env python3
"""fr5jog 화면 — 버튼으로 FR5 손끝을 곧게 움직이고, 회전 중심(TCP · 스포이드 끝)을 고정한 채 돌리고, 그리퍼를 미세 조정한다.

    cd <작업 폴더> && fr5jogui            # 그 폴더의 poses.json · http://localhost:8793
    fr5jogui --poses ~/fr5cap/drop/poses.json --port 8793

순서: 제어판 [조작 시작] → 이 화면 [보내기 시작] → 제어판 /leader [리더 추종 시작] → '준비' 가 되면 버튼이 켜진다.
제어판 추종이 꺼지면 보내기를 스스로 멈춘다(다시 켤 때 지금 자세에서 새로 시작 — 튀지 않게).
움직이는 중에는 새 명령을 받지 않는다. [멈춤](Space)은 그 자리에 세운다.
"""
import argparse
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

sys.path.insert(0, os.path.expanduser("~/fr5cap/pour"))
import pose_go as PG  # noqa: E402
import pour_run as PR  # noqa: E402

VERSION = "0.1.1"
CHECK = {"z_pre": 1e4, "hang": 0.0}


def rmat(ax, deg):
    t = np.radians(deg)
    c, s = np.cos(t), np.sin(t)
    return {"x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]), "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[ax]


def vertical(R):
    x = R[:, 0].copy()
    x[2] = 0.0
    x /= np.linalg.norm(x)
    z = np.array([0.0, 0.0, -1.0])
    return np.column_stack([x, np.cross(z, x), z])


class App:
    def __init__(self, a):
        self.a = a
        self.poses_f = os.path.abspath(os.path.expanduser(a.poses))
        self.home = np.array(PG.DEFAULT_POSES["home"], float)
        self.q = None
        self.g = 0.0
        self.R0 = None
        self.snd = None
        self.phase, self.msg = "대기", "제어판 [조작 시작] 뒤 [보내기 시작]"
        self.busy, self.abort, self.seen_follow = False, False, False
        self.step, self.astep, self.pivot = 2.0, 1.0, 0.0
        self.panel, self.leader, self.panel_ok = {}, {}, False
        self.log = []
        self.lock = threading.Lock()
        threading.Thread(target=self._monitor, daemon=True).start()

    def say(self, s):
        self.log.append(time.strftime("%H:%M:%S ") + s)
        self.log = self.log[-40:]

    # ── 제어판 살피기 ──
    def _monitor(self):
        off = 0
        while True:
            try:
                self.panel = PG.get("/api/state", 0.8)
                self.leader = PG.get("/api/leader/state", 0.8)
                self.panel_ok = True
            except Exception:
                self.panel_ok = False
            fol = bool(self.leader.get("following")) and self.panel.get("mode") == "engaged" and self.panel_ok
            if self.snd is not None:
                if self.phase == "추종 기다림" and fol:
                    self.phase, self.msg, self.seen_follow = "준비", "", True
                    self.say("제어판이 따라온다 — 준비")
                elif self.seen_follow and not self.busy:
                    off = 0 if fol else off + 1
                    if off >= 2:
                        self.release("제어판 추종이 꺼져 보내기를 멈췄다 — 다시 [보내기 시작]")
            time.sleep(0.3)

    # ── 보내기 켜고 끄기 ──
    def start_send(self):
        if self.busy:
            return "움직이는 중"
        st = PG.get("/api/state")
        if st.get("mode") != "engaged":
            return "제어판이 조작 상태가 아니다 — 제어판에서 [조작 시작]"
        self.release()
        self.q = np.array([float(x) for x in st["cmd"]])
        self.g = float(st["gripper"]["pos"]) / 1000.0
        self.R0 = PR.fk(self.q)[:3, :3]
        self.snd = PG.Sender(self.a.udp_host, self.a.udp_port, self.g)
        self.snd.q = list(self.q)
        self.snd.start()
        self.seen_follow = False
        self.phase, self.msg = "추종 기다림", "제어판 /leader 에서 [리더 추종 시작]을 누른다"
        self.say("지금 자세를 보내기 시작")
        return None

    def release(self, why=None):
        if self.snd is not None:
            self.snd.on = False
            self.snd = None
            self.say(why or "보내기 끝 — 제어판이 0.3 s 뒤 그 자리에 선다")
        self.phase, self.msg, self.seen_follow = "대기", why or "", False

    # ── 움직이기 ──
    def _run(self, pl):
        t0, chk, over, n = time.monotonic(), 0.0, 0, len(pl.q)
        while True:
            if self.abort:
                raise PR.Abort("멈춤 — 그 자리에 섰다")
            k = min(n - 1, int((time.monotonic() - t0) * PR.HZ))
            self.snd.q, self.snd.grip = list(pl.q[k]), pl.g[k]
            self.q, self.g = np.array(pl.q[k]), pl.g[k]
            now = time.monotonic()
            if now - chk >= 0.2:
                chk = now
                st = PG.get("/api/state", 1.0)
                if st.get("mode") != "engaged":
                    raise PR.Abort("제어판 조작 상태가 꺼졌다")
                js = [float(x) for x in st["joints"]]
                err = max(abs(x - y) for x, y in zip(js, self.snd.q)) if all(js) else 0.0
                over = over + 1 if err > self.a.max_lag else 0
                if over >= 3:
                    raise PR.Abort(f"로봇이 명령보다 {err:.1f}° 뒤처진다")
            if k >= n - 1:
                return
            time.sleep(1.0 / PR.HZ)

    def _exec(self, pl, tag, R_after=None):
        out, bad = PR.check(pl, CHECK, self.home)
        if bad:
            self.say(f"{tag}: 안 움직임 — " + " · ".join(bad))
            return "안 움직임: " + " · ".join(bad)
        if len(pl.q) < 2:
            return "너무 작아 안 움직임"

        def work():
            try:
                self._run(pl)
                if R_after is not None:
                    self.R0 = R_after
                self.say(tag)
            except PR.Abort as e:
                self.R0 = PR.fk(self.q)[:3, :3]
                self.say(f"{tag}: {e}")
            except Exception as e:
                self.R0 = PR.fk(self.q)[:3, :3]
                self.say(f"{tag}: 멈춤 (제어판 읽기 실패 {repr(e)[:60]})")
            finally:
                self.busy, self.abort = False, False
                self.phase = "준비" if self.snd is not None else "대기"

        self.busy, self.abort, self.phase = True, False, "움직이는 중"
        threading.Thread(target=work, daemon=True).start()
        return None

    def ready(self):
        if self.snd is None or self.phase not in ("준비",):
            return "먼저 [보내기 시작] → 제어판 [리더 추종 시작]"
        if self.busy:
            return "움직이는 중"
        return None

    def plan_goto(self, name):
        P = self.poses()
        if name not in P:
            return None, f"'{name}' 자세가 없다"
        q_t = np.array(P[name], float)
        T0, Tt = PR.fk(self.q), PR.fk(q_t)
        zs = max(T0[2, 3], Tt[2, 3], self.a.safe_z)
        pl = PR.Plan(self.q, self.g)
        pl.line(np.r_[T0[:2, 3], zs], T0[:3, :3], self.a.v_go, 15.0, "up")
        pl.line(np.r_[Tt[:2, 3], zs], Tt[:3, :3], self.a.v_go, 15.0, "over")
        pl.line(Tt[:3, 3], Tt[:3, :3], self.a.v_go * 0.6, 15.0, "down")
        pl.joint(q_t, 5.0, "settle")
        out, bad = PR.check(pl, CHECK, self.home)
        t = Tt[:3, 3]
        txt = f"{name} · TCP ({t[0]:.1f}, {t[1]:.1f}, {t[2]:.1f}) · 곧게 위로 {zs:.0f} mm → 옆으로 → 아래로 · {out['sec']:.0f} s"
        return (pl, Tt[:3, :3], txt, bad), None

    def cmd(self, b):
        err = self.ready()
        if err:
            return err
        k = b.get("kind")
        p0 = PR.fk(self.q)[:3, 3]
        if k == "jog":
            d = np.array(b["d"], float) * self.step
            pl = PR.Plan(self.q, self.g)
            pl.line(p0 + d, self.R0, self.a.v if self.step > 1 else 5.0, 10.0, "jog")
            return self._exec(pl, f"곧게 ({d[0]:+g}, {d[1]:+g}, {d[2]:+g}) mm", self.R0)
        if k == "rot":
            piv = p0 + self.R0 @ np.array([0.0, 0.0, self.pivot])
            R1 = rmat(b["ax"], float(b["sg"]) * self.astep) @ self.R0
            pl = PR.Plan(self.q, self.g)
            pl.line(piv - R1 @ np.array([0.0, 0.0, self.pivot]), R1, self.a.v, 5.0, "rotate")
            where = "TCP" if self.pivot == 0 else f"TCP 아래 {self.pivot:g} mm"
            return self._exec(pl, f"회전 {b['ax']} {float(b['sg']) * self.astep:+g}° (중심 {where})", R1)
        if k == "vertical":                                  # 회전 중심(TCP · 스포이드 끝)은 그대로 두고 곧게
            piv = p0 + self.R0 @ np.array([0.0, 0.0, self.pivot])
            R1 = vertical(self.R0)
            pl = PR.Plan(self.q, self.g)
            pl.line(piv - R1 @ np.array([0.0, 0.0, self.pivot]), R1, self.a.v, 5.0, "vertical")
            return self._exec(pl, "손 곧게", R1)
        if k == "grip":
            g1 = min(1.0, max(0.0, float(b["g"]) / 1000.0))
            pl = PR.Plan(self.q, self.g)
            n = max(1, int(round(abs(g1 - self.g) / 0.005)))
            for i in range(1, n + 1):
                pl.hold(0.2, self.g + (g1 - self.g) * i / n, "grip")
            return self._exec(pl, f"그리퍼 {self.g * 1000:.0f} → {g1 * 1000:.0f}", self.R0)
        if k == "goto":
            r, e = self.plan_goto(b["name"])
            if e:
                return e
            pl, Rt, txt, bad = r
            if bad:
                return "못 감: " + " · ".join(bad)
            return self._exec(pl, f"{b['name']} 도착", Rt)
        return "모르는 명령"

    def poses(self):
        P = {"home": list(self.home)}
        if os.path.exists(self.poses_f):
            P.update(json.load(open(self.poses_f, encoding="utf-8")))
        return P

    def save(self, name):
        if self.q is None:
            return "보내기를 먼저 시작한다"
        name = name.strip()
        if not name or name == "home" or any(ch in name for ch in " /\\"):
            return "이름이 비었거나 쓸 수 없다 (home · 빈칸 · / 안 됨)"
        P = json.load(open(self.poses_f, encoding="utf-8")) if os.path.exists(self.poses_f) else {}
        P[name] = [round(float(x), 4) for x in self.q]
        json.dump(P, open(self.poses_f, "w", encoding="utf-8"), indent=1)
        t = PR.fk(self.q)[:3, 3]
        self.say(f"저장 {name}: TCP ({t[0]:.1f}, {t[1]:.1f}, {t[2]:.1f}) · 그리퍼 {self.g * 1000:.0f}")
        return None

    def status(self):
        T = PR.fk(self.q) if self.q is not None else None
        P = {}
        try:
            for k, v in self.poses().items():
                t = PR.fk(v)[:3, 3]
                P[k] = [round(float(x), 1) for x in t]
        except Exception as e:
            P = {"(읽기 실패)": [0, 0, 0]}
            self.msg = repr(e)[:80]
        return {"v": VERSION, "phase": self.phase, "msg": self.msg, "busy": self.busy, "log": self.log[-30:],
                "send": self.snd is not None, "v": self.a.v, "v_go": self.a.v_go, "step": self.step, "astep": self.astep, "pivot": self.pivot,
                "g": round(self.g * 1000, 1), "table_z": PR.TABLE_Z, "poses_f": self.poses_f, "poses": P,
                "T": None if T is None else {"p": [round(float(x), 2) for x in T[:3, 3]], "R": [[round(float(x), 5) for x in r] for r in T[:3, :3]]},
                "R0": None if self.R0 is None else [[round(float(x), 5) for x in r] for r in self.R0],
                "panel": {"ok": self.panel_ok, "mode": self.panel.get("mode"), "following": self.leader.get("following"),
                          "grip": (self.panel.get("gripper") or {}).get("pos")}}


PAGE = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>fr5jog 조그</title><style>
:root{--bg:#f4f3ef;--card:#fff;--ink:#1d1d1b;--mute:#6b6a64;--line:#dddbd3;--tcp:#cc79a7;--piv:#e69f00;--b:#0072b2;--warn:#d55e00;--ok:#009e73;--soft:#ecebe5}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#1b1b1a;--card:#262624;--ink:#ecebe6;--mute:#a3a29b;--line:#3a3a37;--soft:#30302d}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.45 system-ui,"Noto Sans KR",sans-serif}
header{position:sticky;top:0;z-index:3;padding:10px 16px;background:var(--card);border-bottom:1px solid var(--line);display:flex;gap:10px;align-items:center;flex-wrap:wrap}
header b{font-size:17px;letter-spacing:.2px}.chip{padding:2px 9px;border-radius:10px;border:1px solid var(--line);font-size:12px;color:var(--mute)}
.chip.on{color:#fff;background:var(--ok);border-color:var(--ok)}.chip.warn{color:#fff;background:var(--warn);border-color:var(--warn)}.chip.blue{color:#fff;background:var(--b);border-color:var(--b)}
#band{flex:1 1 260px;font-weight:600;min-width:200px}.sp{flex:1}
main{display:grid;grid-template-columns:minmax(0,1.25fr) minmax(0,1fr);gap:14px;padding:14px 16px;max-width:1500px;margin:0 auto}
@media (max-width:980px){main{grid-template-columns:1fr}}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:14px;margin-bottom:14px}
h3{margin:0 0 10px;font-size:13px;color:var(--mute);font-weight:600;display:flex;gap:8px;align-items:center}h3 .k{margin-left:auto;font-weight:400}
button{padding:8px 12px;border:1px solid var(--line);border-radius:8px;background:var(--bg);color:var(--ink);cursor:pointer;font:inherit;transition:background .12s,border-color .12s,transform .05s}
button:hover:not(:disabled){border-color:var(--mute)}button:active:not(:disabled){transform:translateY(1px)}button:disabled{opacity:.4;cursor:default}
button.go{background:var(--ok);color:#fff;border-color:var(--ok);font-weight:600}button.stop{background:var(--warn);color:#fff;border-color:var(--warn);font-weight:700;padding:9px 18px}
button.on{background:var(--ink);color:var(--card);border-color:var(--ink)}
.row{display:flex;gap:6px;align-items:center;flex-wrap:wrap;margin:8px 0}.mute{color:var(--mute);font-size:12px}
input[type=text],input[type=number]{width:90px;padding:6px 8px;border:1px solid var(--line);border-radius:6px;background:var(--bg);color:var(--ink);font:inherit}
.xyz{display:flex;gap:16px;font:600 22px/1.2 ui-monospace,Menlo,monospace;flex-wrap:wrap}.xyz span{display:flex;flex-direction:column}.xyz small{font:12px system-ui;color:var(--mute);font-weight:400}
.pads{display:flex;gap:22px;flex-wrap:wrap;align-items:center}
.pad{display:grid;grid-template-columns:repeat(3,64px);grid-template-rows:repeat(3,56px);gap:6px}
.pad button,.zcol button{display:flex;flex-direction:column;align-items:center;justify-content:center;padding:4px;font-size:20px;line-height:1}
.pad button small,.zcol button small{font-size:11px;color:var(--mute);margin-top:3px}.pad .mid{display:flex;align-items:center;justify-content:center;color:var(--mute);font-size:11px;text-align:center}
.zcol{display:grid;grid-template-rows:repeat(2,85px);gap:6px;width:72px}
.rot button{border-color:color-mix(in srgb,var(--piv) 55%,var(--line))}.rot button:hover:not(:disabled){border-color:var(--piv);background:color-mix(in srgb,var(--piv) 10%,var(--bg))}
.chips{display:flex;gap:4px;flex-wrap:wrap}.chips button{padding:5px 10px;font-size:13px;border-radius:14px}
.views{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}.view{background:var(--soft);border-radius:8px;position:relative}.view svg{width:100%;height:auto;display:block}
.view .lab{position:absolute;left:8px;top:6px;font-size:11px;color:var(--mute)}
.legend{display:flex;gap:12px;flex-wrap:wrap;font-size:12px;color:var(--mute);margin-top:6px}.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:4px;vertical-align:middle}
.gbig{font:700 30px/1 ui-monospace,Menlo,monospace}.gbar{position:relative;height:30px;background:var(--soft);border-radius:8px;margin:10px 0 22px;cursor:pointer}
.gbar .fill{position:absolute;left:0;top:0;bottom:0;background:var(--b);opacity:.85;border-radius:8px}.gbar .act{position:absolute;top:-4px;bottom:-4px;width:3px;background:var(--ink);border-radius:2px}
.gbar .tick{position:absolute;top:32px;font-size:11px;color:var(--mute);transform:translateX(-50%);white-space:nowrap}.gbar .tg{position:absolute;top:0;bottom:0;width:2px;background:var(--piv)}
table{width:100%;border-collapse:collapse;font-size:13px}td{padding:5px 4px;border-top:1px solid var(--line)}td.n{font-weight:600}td.c{font:12px ui-monospace,Menlo,monospace;color:var(--mute)}td.r{text-align:right}
pre{white-space:pre-wrap;margin:0;font:12px/1.45 ui-monospace,Menlo,monospace;color:var(--mute);max-height:220px;overflow:auto}
#modal{position:fixed;inset:0;background:rgba(0,0,0,.35);display:none;align-items:center;justify-content:center;z-index:9}#modal .box{background:var(--card);border-radius:12px;padding:18px;max-width:520px;width:92%;border:1px solid var(--line)}
.busy #bStop{animation:pulse 1s infinite}@keyframes pulse{50%{box-shadow:0 0 0 6px rgba(213,94,0,.25)}}
kbd{font:11px ui-monospace,Menlo,monospace;border:1px solid var(--line);border-bottom-width:2px;border-radius:4px;padding:0 4px;color:var(--mute)}
</style></head><body>
<header><b>fr5jog</b><span class="chip" id="cPanel">제어판</span><span class="chip" id="cFol">추종</span><span class="chip" id="cSend">보내기</span>
<span class="mute" id="ver"></span><div id="band">—</div><span class="sp"></span>
<button class="go" id="bSend" onclick="post('/api/send',{on:true})">보내기 시작</button><button id="bRel" onclick="post('/api/send',{on:false})">보내기 끝</button>
<button class="stop" id="bStop" onclick="post('/api/stop',{})">멈춤 <kbd style="color:#fff;border-color:#fff8">Space</kbd></button></header>
<main><section>
<div class="card"><h3>손끝 (TCP · 베이스 mm)<span class="k mute" id="tilt"></span></h3><div class="xyz"><span><small>x</small><b id="px">—</b></span><span><small>y</small><b id="py">—</b></span><span><small>z</small><b id="pz">—</b></span></div></div>
<div class="card"><h3>보기 <span class="k mute">3인칭 카메라에서 본 방향 · 점선 = 버튼을 누르면 갈 자리</span></h3>
<div class="views"><div class="view"><span class="lab">위에서 · ↑ 로봇 쪽</span><svg id="vTop" viewBox="0 0 240 240"></svg></div>
<div class="view"><span class="lab">앞에서 (카메라)</span><svg id="vFront" viewBox="0 0 240 240"></svg></div>
<div class="view"><span class="lab">옆에서 · 오른쪽 = 카메라 쪽</span><svg id="vSide" viewBox="0 0 240 240"></svg></div></div>
<div class="legend"><span><span class="dot" style="background:var(--tcp)"></span>TCP</span><span><span class="dot" style="background:var(--piv)"></span>회전 중심 (고정)</span><span><span class="dot" style="background:var(--ink)"></span>손 · 공구 축</span><span><span class="dot" style="background:var(--b)"></span>누르면 갈 자리</span></div></div>
<div class="card"><h3>곧게 움직이기 <span class="k"><kbd>←</kbd><kbd>→</kbd><kbd>↑</kbd><kbd>↓</kbd> <kbd>w</kbd><kbd>s</kbd></span></h3>
<div class="pads"><div class="pad"><span></span><button data-j="0,-1,0" data-k="ArrowUp">↑<small>로봇 쪽</small></button><span></span>
<button data-j="1,0,0" data-k="ArrowLeft">←<small>왼쪽 +x</small></button><span class="mid">화면<br>기준</span><button data-j="-1,0,0" data-k="ArrowRight">→<small>오른쪽 −x</small></button>
<span></span><button data-j="0,1,0" data-k="ArrowDown">↓<small>카메라 쪽</small></button><span></span></div>
<div class="zcol"><button data-j="0,0,1" data-k="w">▲<small>위</small></button><button data-j="0,0,-1" data-k="s">▼<small>아래</small></button></div>
<div><div class="mute">한 걸음 (mm) · <kbd>1</kbd>–<kbd>7</kbd></div><div class="chips" id="steps"></div><div class="row"><input type="number" id="stepIn" step="0.1" min="0.1" max="50"><button onclick="setv('step',$('stepIn').value)">적용</button></div></div></div><div class="row"><span class="mute">조그 속도</span><input type="number" id="vIn" step="1" min="1" max="60"> mm/s <button onclick="setv('v',$('vIn').value)">적용</button><span class="mute" style="margin-left:14px">저장 자세 · 초기자세로 가는 속도</span><input type="number" id="vgIn" step="1" min="5" max="90"> mm/s <button onclick="setv('v_go',$('vgIn').value)">적용</button><span class="mute">(1 mm 이하 걸음은 5 mm/s 고정)</span></div></div>
<div class="card"><h3>회전 — 중심 고정 <span class="k"><kbd>j</kbd><kbd>l</kbd> <kbd>i</kbd><kbd>k</kbd> <kbd>n</kbd><kbd>m</kbd> <kbd>[</kbd><kbd>]</kbd></span></h3>
<div class="row">회전 중심 <span class="chips"><button id="pv0" onclick="setv('pivot',0)">TCP</button><button id="pvT" onclick="setv('pivot',225)">스포이드 끝 225</button></span>
<input type="number" id="pivIn" step="1" min="0" max="400"> mm <button onclick="setv('pivot',$('pivIn').value)">적용</button><span class="mute">TCP 에서 공구 축으로 아래</span></div>
<div class="pads rot"><div class="pad"><span></span><button data-r="x,-1" data-k="i">↑<small>끝 로봇쪽</small></button><span></span>
<button data-r="y,-1" data-k="j">←<small>끝 +x</small></button><button data-v="1" data-k="v" title="손 곧게 (v)">⊥<small>곧게</small></button><button data-r="y,1" data-k="l">→<small>끝 −x</small></button>
<span></span><button data-r="x,1" data-k="k">↓<small>끝 카메라</small></button><span></span></div>
<div class="zcol"><button data-r="z,1" data-k="n">⟲<small>반시계</small></button><button data-r="z,-1" data-k="m">⟳<small>시계</small></button></div>
<div><div class="mute">한 번 회전 (°)</div><div class="chips" id="asteps"></div></div></div></div>
</section><section>
<div class="card"><h3>그리퍼 <span class="k"><kbd>o</kbd><kbd>c</kbd> ±10 · <kbd>O</kbd><kbd>C</kbd> ±50</span></h3>
<div class="row" style="align-items:flex-end;gap:18px"><div><div class="mute">보낸 값</div><div class="gbig" id="gv">—</div></div><div><div class="mute">실제 (제어판)</div><div class="gbig" id="ga" style="color:var(--mute)">—</div></div>
<svg id="gfig" viewBox="0 0 120 70" style="width:120px;margin-left:auto"></svg></div>
<div class="gbar" id="gbar"><div class="fill" id="gfill"></div><div class="tg" id="gtg" style="display:none"></div><div class="act" id="gact"></div></div>
<div class="row"><span class="chips"><button data-gd="-50">−50</button><button data-gd="-10">−10</button><button data-gd="-5">−5</button><button data-gd="-1">−1</button><button data-gd="1">+1</button><button data-gd="5">+5</button><button data-gd="10">+10</button><button data-gd="50">+50</button></span></div>
<div class="row"><span class="mute">바로 가기</span><span class="chips"><button data-gs="150">150 짜기</button><button data-gs="255">255 떨구기</button><button data-gs="285">285 쥐기</button><button data-gs="500">500 벌림</button></span>
<input type="number" id="gIn" min="0" max="1000"><button onclick="grip(+$('gIn').value)">적용</button></div>
<div class="mute">0 닫힘 – 1000 열림 · 5 단위마다 0.2 s 로 천천히 · 막대를 눌러도 그 값으로</div></div>
<div class="card"><h3>자세</h3><div class="mute" id="pf" style="margin:-4px 0 6px;word-break:break-all"></div>
<div class="row"><button class="go" id="bHome" onclick="goto('home')">초기자세로</button><button data-v="1">손 곧게</button><span class="sp"></span><input type="text" id="sName" placeholder="이름"><button onclick="post('/api/save',{name:$('sName').value})">지금 자세 저장</button></div>
<table id="ptab"></table></div>
<div class="card"><h3>기록</h3><pre id="log"></pre></div></section></main>
<div id="modal"><div class="box"><h3 style="font-size:15px;color:var(--ink)">이 자세로 갈까?</h3><div id="mtxt" style="margin:8px 0 14px"></div>
<div class="row"><button class="go" id="mGo">가기</button><button onclick="hide()">취소</button></div></div></div>
<script>
const $=id=>document.getElementById(id);let S=null,hover=null;
const STEPS=[0.5,1,2,5,10,20,50],ASTEPS=[0.5,1,2,5,10];
async function post(u,b){const r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)});const j=await r.json();if(j.err)toast(j.err);tick();return j}
function toast(t){$('band').textContent=t;$('band').style.color='var(--warn)'}
function setv(k,v){const o={};o[k]=Number(v);post('/api/set',o)}
function grip(v){if(isNaN(v))return;post('/api/cmd',{kind:'grip',g:Math.max(0,Math.min(1000,v))})}
async function goto(n){const j=await post('/api/plan_goto',{name:n});if(j.err)return;$('mtxt').textContent=j.text;$('mGo').onclick=()=>{hide();post('/api/cmd',{kind:'goto',name:n})};$('modal').style.display='flex'}
function hide(){$('modal').style.display='none'}
document.querySelectorAll('[data-j]').forEach(b=>{b.onclick=()=>post('/api/cmd',{kind:'jog',d:b.dataset.j.split(',').map(Number)});b.onmouseenter=()=>{hover={j:b.dataset.j.split(',').map(Number)};draw()};b.onmouseleave=()=>{hover=null;draw()}});
document.querySelectorAll('[data-r]').forEach(b=>{const [ax,sg]=b.dataset.r.split(',');b.onclick=()=>post('/api/cmd',{kind:'rot',ax,sg:+sg});b.onmouseenter=()=>{hover={r:[ax,+sg]};draw()};b.onmouseleave=()=>{hover=null;draw()}});
document.querySelectorAll('[data-v]').forEach(b=>{b.onclick=()=>post('/api/cmd',{kind:'vertical'});b.onmouseenter=()=>{hover={v:1};draw()};b.onmouseleave=()=>{hover=null;draw()}});
document.querySelectorAll('[data-gd]').forEach(b=>b.onclick=()=>S&&grip(Math.round(S.g)+(+b.dataset.gd)));
document.querySelectorAll('[data-gs]').forEach(b=>b.onclick=()=>grip(+b.dataset.gs));
$('gbar').onclick=e=>{const r=$('gbar').getBoundingClientRect();grip(Math.round((e.clientX-r.left)/r.width*1000))};
$('steps').innerHTML=STEPS.map((s,i)=>`<button onclick="setv('step',${s})" data-s="${s}">${s}</button>`).join('');
$('asteps').innerHTML=ASTEPS.map(s=>`<button onclick="setv('astep',${s})" data-a="${s}">${s}°</button>`).join('');
document.addEventListener('keydown',e=>{if(e.target.tagName==='INPUT')return;const k=e.key;
 if(k===' '||k==='Escape'){e.preventDefault();post('/api/stop',{});return}
 if(/^[1-7]$/.test(k)){setv('step',STEPS[+k-1]);return}
 if(k==='['||k===']'){if(!S)return;let i=ASTEPS.indexOf(S.astep);if(i<0)i=1;setv('astep',ASTEPS[Math.max(0,Math.min(4,i+(k===']'?1:-1)))]);return}
 if(k==='o'||k==='c'||k==='O'||k==='C'){if(S)grip(Math.round(S.g)+({o:10,c:-10,O:50,C:-50})[k]);return}
 const b=document.querySelector(`[data-k="${k}"]`);if(b&&!b.disabled){e.preventDefault();b.click()}});
// ── 3D 보기 ──
const mul=(A,B)=>A.map(r=>B[0].map((_,j)=>r.reduce((s,v,k)=>s+v*B[k][j],0))),mv=(A,v)=>A.map(r=>r[0]*v[0]+r[1]*v[1]+r[2]*v[2]),add=(a,b)=>a.map((x,i)=>x+b[i]),sc=(a,s)=>a.map(x=>x*s);
function rmat(ax,d){const t=d*Math.PI/180,c=Math.cos(t),s=Math.sin(t);return ax==='x'?[[1,0,0],[0,c,-s],[0,s,c]]:ax==='y'?[[c,0,s],[0,1,0],[-s,0,c]]:[[c,-s,0],[s,c,0],[0,0,1]]}
function vert(R){let x=[R[0][0],R[1][0],0];const n=Math.hypot(x[0],x[1])||1;x=[x[0]/n,x[1]/n,0];const z=[0,0,-1],y=[z[1]*x[2]-z[2]*x[1],z[2]*x[0]-z[0]*x[2],z[0]*x[1]-z[1]*x[0]];return [[x[0],y[0],z[0]],[x[1],y[1],z[1]],[x[2],y[2],z[2]]]}
function pose(R,p,piv){const z=[R[0][2],R[1][2],R[2][2]],Ld=Math.max(piv>0?piv+25:150,60);return {p,z,top:add(p,sc(z,-70)),tip:add(p,sc(z,Ld)),pv:add(p,sc(z,piv))}}
function preview(){if(!S||!S.T||!hover)return null;const R=S.R0,p=S.T.p,piv=S.pivot;
 if(hover.j)return pose(R,add(p,sc(hover.j,S.step)),piv);
 const R1=hover.v?vert(R):mul(rmat(hover.r[0],hover.r[1]*S.astep),R),c=add(p,mv(R,[0,0,piv]));return pose(R1,add(c,sc(mv(R1,[0,0,piv]),-1)),piv)}
const VIEWS={vTop:p=>[-p[0],p[1]],vFront:p=>[-p[0],-p[2]],vSide:p=>[p[1],-p[2]]};
function draw(){if(!S||!S.T){['vTop','vFront','vSide'].forEach(id=>$(id).innerHTML='<text x="120" y="125" text-anchor="middle" font-size="12" fill="currentColor" opacity=".5">보내기 시작 전</text>');return}
 const cur=pose(S.R0||S.T.R,S.T.p,S.pivot),pre=preview(),ink=getComputedStyle(document.body).color;
 for(const id in VIEWS){const P=VIEWS[id],top=id==='vTop';const L=Math.hypot(...cur.tip.map((v,i)=>v-cur.top[i]));
  const ctr=top?P(S.pivot>0?cur.pv:cur.p):P(sc(add(cur.top,cur.tip),0.5));
  const span=top?(S.pivot>0?80:60):L+70,s=200/span;
  const X=q=>{const u=P(q);return [120+(u[0]-ctr[0])*s,120+(u[1]-ctr[1])*s]};let h='';
  // 격자
  const gstep=top?10:50;for(let k=-12;k<=12;k++){const o=k*gstep*s;h+=`<line x1="${120+o}" y1="0" x2="${120+o}" y2="240" stroke="${ink}" stroke-opacity=".06"/><line x1="0" y1="${120+o}" x2="240" y2="${120+o}" stroke="${ink}" stroke-opacity=".06"/>`}
  if(!top){const ty=X([0,0,S.table_z])[1],tyy=X([0,0,S.table_z])[1];if(ty>0&&ty<240)h+=`<line x1="0" y1="${ty}" x2="240" y2="${tyy}" stroke="${ink}" stroke-opacity=".35" stroke-width="2"/><text x="236" y="${ty-4}" text-anchor="end" font-size="10" fill="${ink}" fill-opacity=".5">책상</text>`}
  // 회전 중심 기준 세로선
  const pv=X(cur.pv);if(!top)h+=`<line x1="${pv[0]}" y1="0" x2="${pv[0]}" y2="240" stroke="var(--piv)" stroke-opacity=".35" stroke-dasharray="3 4"/>`;
  const seg=(o,dash,col,w)=>{const a=X(o.top),b=X(o.p),c=X(o.tip);return `<line x1="${a[0]}" y1="${a[1]}" x2="${b[0]}" y2="${b[1]}" stroke="${col}" stroke-width="${w+4}" stroke-linecap="round" ${dash}/>`+`<line x1="${b[0]}" y1="${b[1]}" x2="${c[0]}" y2="${c[1]}" stroke="${col}" stroke-width="${w}" stroke-linecap="round" ${dash}/>`};
  if(pre){h+=seg(pre,'stroke-dasharray="5 4" opacity=".9"','var(--b)',1.6);const t=X(pre.p);h+=`<circle cx="${t[0]}" cy="${t[1]}" r="4" fill="none" stroke="var(--b)" stroke-width="1.5"/>`;
   if(hover&&(hover.r||hover.v)&&!top){const c=X(cur.pv),a=X(cur.top),b=X(pre.top),a0=Math.atan2(a[1]-c[1],a[0]-c[0]),a1=Math.atan2(b[1]-c[1],b[0]-c[0]);let da=a1-a0;da=Math.atan2(Math.sin(da),Math.cos(da));
    if(Math.abs(da)>0.004){const r=Math.max(30,Math.hypot(a[0]-c[0],a[1]-c[1])),sw=da>0?1:0,x0=c[0]+r*Math.cos(a0),y0=c[1]+r*Math.sin(a0),x1=c[0]+r*Math.cos(a1),y1=c[1]+r*Math.sin(a1),ah=a1+(da>0?Math.PI/2:-Math.PI/2);
     h+=`<path d="M${x0} ${y0} A${r} ${r} 0 0 ${sw} ${x1} ${y1}" fill="none" stroke="var(--piv)" stroke-width="2"/><path d="M${x1} ${y1} l${7*Math.cos(ah-2.6)} ${7*Math.sin(ah-2.6)} M${x1} ${y1} l${7*Math.cos(ah+2.6)} ${7*Math.sin(ah+2.6)}" stroke="var(--piv)" stroke-width="2" stroke-linecap="round"/>`}}}
  h+=seg(cur,'',ink,2);
  h+=`<circle cx="${pv[0]}" cy="${pv[1]}" r="7" fill="none" stroke="var(--piv)" stroke-width="2"/><line x1="${pv[0]-11}" y1="${pv[1]}" x2="${pv[0]+11}" y2="${pv[1]}" stroke="var(--piv)" stroke-width="1.5"/><line x1="${pv[0]}" y1="${pv[1]-11}" x2="${pv[0]}" y2="${pv[1]+11}" stroke="var(--piv)" stroke-width="1.5"/>`;
  const t=X(cur.p);h+=`<circle cx="${t[0]}" cy="${t[1]}" r="5" fill="var(--tcp)" stroke="${getComputedStyle(document.documentElement).getPropertyValue('--card')}" stroke-width="2"/>`;
  if(top)h+=`<text x="8" y="232" font-size="10" fill="${ink}" fill-opacity=".5">칸 ${gstep} mm</text>`;
  $(id).innerHTML=h}}
function gfig(v){const w=8+v/1000*84;return `<rect x="${60-w/2-10}" y="6" width="${w+20}" height="12" rx="3" fill="currentColor" opacity=".25"/><rect x="${60-w/2-10}" y="14" width="10" height="46" rx="3" fill="currentColor" opacity=".8"/><rect x="${60+w/2}" y="14" width="10" height="46" rx="3" fill="currentColor" opacity=".8"/><text x="60" y="68" text-anchor="middle" font-size="9" fill="currentColor" opacity=".6">${(v/1000*95).toFixed(1)} mm</text>`}
function chip(id,t,c){const e=$(id);e.textContent=t;e.className='chip'+(c?' '+c:'')}
let lastV=null,lastP=null;
async function tick(){try{S=await (await fetch('/api/status')).json();if(lastV&&lastV!==S.v)location.reload();lastV=S.v;$('ver').textContent='v'+S.v;
 const p=S.panel;chip('cPanel',p.ok?'제어판 '+(p.mode||'?'):'제어판 연결 안 됨',p.ok&&p.mode==='engaged'?'on':'warn');chip('cFol',p.following?'추종 중':'추종 꺼짐',p.following?'on':'');
 chip('cSend',S.send?'보내는 중':'안 보냄',S.send?'blue':'');document.body.classList.toggle('busy',S.busy);
 $('band').textContent=S.phase+(S.msg?' — '+S.msg:'');$('band').style.color=S.phase==='준비'?'var(--ok)':(S.phase==='추종 기다림'?'var(--b)':(S.phase==='움직이는 중'?'var(--piv)':''));
 const ok=S.phase==='준비'&&!S.busy;document.querySelectorAll('[data-j],[data-r],[data-v],[data-gd],[data-gs],#bHome,#ptab button').forEach(b=>b.disabled=!ok);
 $('bSend').disabled=S.busy;$('bRel').disabled=!S.send||S.busy;
 if(S.T){const q=S.T.p;$('px').textContent=q[0].toFixed(1);$('py').textContent=q[1].toFixed(1);$('pz').textContent=q[2].toFixed(1);const tz=Math.acos(Math.min(1,Math.abs(S.T.R[2][2])))*180/Math.PI;$('tilt').textContent=`손 기울기 ${tz.toFixed(1)}° · 회전 중심 ${S.pivot>0?'TCP 아래 '+S.pivot+' mm':'TCP'}`}
 document.querySelectorAll('[data-s]').forEach(b=>b.classList.toggle('on',+b.dataset.s===S.step));document.querySelectorAll('[data-a]').forEach(b=>b.classList.toggle('on',+b.dataset.a===S.astep));
 if(document.activeElement!==$('stepIn'))$('stepIn').value=S.step;if(document.activeElement!==$('pivIn'))$('pivIn').value=S.pivot;if(document.activeElement!==$('vIn'))$('vIn').value=S.v;if(document.activeElement!==$('vgIn'))$('vgIn').value=S.v_go;
 $('pv0').classList.toggle('on',S.pivot===0);$('pvT').classList.toggle('on',S.pivot===225);
 const ga=p.grip;$('gv').textContent=Math.round(S.g);$('ga').textContent=ga??'—';$('gfill').style.width=(S.g/10)+'%';$('gact').style.left=`calc(${(ga??0)/10}% - 1px)`;$('gfig').innerHTML=gfig(S.g);
 $('pf').textContent='자세 파일 '+S.poses_f.split('/').slice(-2).join('/');$('pf').title=S.poses_f;const pj=JSON.stringify(S.poses);if(pj!==lastP){lastP=pj;$('ptab').innerHTML=Object.entries(S.poses).map(([n,t])=>`<tr><td class="n">${n}</td><td class="c">(${t.join(', ')})</td><td class="r"><button onclick="goto('${n}')">가기</button></td></tr>`).join('')}
 document.querySelectorAll('#ptab button').forEach(b=>b.disabled=!ok);
 $('log').textContent=S.log.slice().reverse().join('\n');draw()}catch(e){$('band').textContent='fr5jog 서버 연결 끊김';$('band').style.color='var(--warn)'}}
const ticks=[0,150,285,500,1000];$('gbar').insertAdjacentHTML('beforeend',ticks.map(v=>`<span class="tick" style="left:${v/10}%">${v}</span>`).join(''));
setInterval(tick,200);tick();
</script></body></html>"""


class H(BaseHTTPRequestHandler):
    app = None

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json"):
        b = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype + ("; charset=utf-8" if ctype.startswith("text") or ctype.endswith("json") else ""))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/?"):
            return self._send(200, PAGE, "text/html")
        if self.path.startswith("/api/status"):
            return self._send(200, json.dumps(self.app.status()))
        self._send(404, "{}")

    def do_POST(self):
        A = self.app
        try:
            n = int(self.headers.get("Content-Length") or 0)
            b = json.loads(self.rfile.read(n) or b"{}")
            err, extra = None, {}
            with A.lock:
                if self.path == "/api/send":
                    err = A.start_send() if b.get("on") else (A.release() or None)
                elif self.path == "/api/stop":
                    A.abort = True
                elif self.path == "/api/set":
                    if "step" in b:
                        A.step = float(min(50.0, max(0.1, b["step"])))
                    if "astep" in b:
                        A.astep = float(min(15.0, max(0.1, b["astep"])))
                    if "v" in b:
                        A.a.v = float(min(60.0, max(1.0, b["v"])))
                    if "v_go" in b:
                        A.a.v_go = float(min(90.0, max(5.0, b["v_go"])))
                    if "pivot" in b:
                        A.pivot = float(min(400.0, max(0.0, b["pivot"])))
                elif self.path == "/api/save":
                    err = A.save(str(b.get("name", "")))
                elif self.path == "/api/plan_goto":
                    err = A.ready()
                    if not err:
                        r, err = A.plan_goto(str(b.get("name", "")))
                        if r:
                            pl, Rt, txt, bad = r
                            err = ("못 감: " + " · ".join(bad)) if bad else None
                            extra = {"text": txt}
                elif self.path == "/api/cmd":
                    err = A.cmd(b)
                else:
                    err = "모르는 요청"
            self._send(200, json.dumps({"err": err, **extra}))
        except Exception as e:
            self._send(200, json.dumps({"err": f"서버 오류: {repr(e)[:120]}"}))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--poses", default="poses.json", help="자세 파일 (기본: 지금 폴더의 poses.json)")
    ap.add_argument("--port", type=int, default=8793)
    ap.add_argument("--v", type=float, default=10.0, help="곧게 · 회전 속도 mm/s (1 mm 이하 걸음은 5)")
    ap.add_argument("--v-go", type=float, default=30.0, help="저장 자세 · 초기자세로 갈 때 mm/s (내릴 때 0.6 배)")
    ap.add_argument("--safe-z", type=float, default=380.0, help="저장 자세로 갈 때 옆으로 옮기는 최소 TCP 높이 mm")
    ap.add_argument("--max-lag", type=float, default=5.0)
    ap.add_argument("--udp-host", default="127.0.0.1")
    ap.add_argument("--udp-port", type=int, default=8097)
    a = ap.parse_args()
    H.app = App(a)
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), H)
    print(f"fr5jog 화면 {VERSION} · http://localhost:{a.port} · 자세 파일 {H.app.poses_f} · 끝내기 Ctrl+C")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        H.app.release()
        print("끝 — 보내기를 멈췄다 (제어판이 0.3 s 뒤 그 자리에 선다)")


if __name__ == "__main__":
    main()
