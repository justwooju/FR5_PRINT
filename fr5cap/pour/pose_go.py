#!/usr/bin/env python3
"""저장한 자세 사이를 관절 공간으로 천천히 옮긴다 (비커 붓기 1단계).

  python3 pose_go.py home            # 지금 자세 → 초기 자세
  python3 pose_go.py pour            # 지금 자세 → 붓기 자세
  python3 pose_go.py pour --dry      # 계획만 보기 (로봇 안 움직임)
  python3 pose_go.py save 이름       # 지금 제어판 명령 자세를 poses.json 에 저장
  python3 pose_go.py list            # 저장된 자세

제어판 UDP 8097 로 {sid, seq, q, grip} 를 50 Hz 로 보낸다. 그리퍼 값은 시작 때 값 그대로 둔다.
멈춤: Ctrl+C(보내기를 멈추면 제어판이 0.3 s 뒤 그 자리에 선다) · 제어판 [정지]/Esc · 비상정지.
"""
import argparse
import json
import math
import os
import random
import socket
import sys
import threading
import time
import urllib.request

PANEL = "http://127.0.0.1:8088"
POSES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "poses.json")
DEFAULT_POSES = {
    "home": [88.2542, -80.6386, -100.9385, -88.8735, 89.9876, -0.7449],   # fr5auto HOME_DEFAULT
    "pour": [108.09, -108.88, -112.14, -118.63, -12.73, -37.70],          # 2026-11-16 12:28 조그로 맞춘 붓기 자세
}
HZ = 50
TOOL_MM = 168.4      # 플랜지 → TCP(공구 1). 제어판 TCP 와 FK 로 맞춘 값
DH_A = [0, -425, -395, 0, 0, 0]
DH_D = [152, 0, 0, 102, 102, 100]
DH_AL = [math.pi / 2, 0, 0, math.pi / 2, -math.pi / 2, 0]


class Abort(Exception):
    pass


def get(path, timeout=2.0):
    with urllib.request.urlopen(PANEL + path, timeout=timeout) as r:
        return json.loads(r.read())


def fk_tcp(q):
    T = [[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]]
    for i in range(6):
        t = math.radians(q[i])
        ca, sa, ct, st = math.cos(DH_AL[i]), math.sin(DH_AL[i]), math.cos(t), math.sin(t)
        A = [[ct, -st * ca, st * sa, DH_A[i] * ct], [st, ct * ca, -ct * sa, DH_A[i] * st], [0, sa, ca, DH_D[i]], [0, 0, 0, 1]]
        T = [[sum(T[r][k] * A[k][c] for k in range(4)) for c in range(4)] for r in range(4)]
    return [T[r][3] + T[r][2] * TOOL_MM for r in range(3)]


def lerp(a, b, s):
    return [x + (y - x) * s for x, y in zip(a, b)]


def load():
    poses = dict(DEFAULT_POSES)
    if os.path.exists(POSES):
        with open(POSES, encoding="utf-8") as f:
            poses.update(json.load(f))
    return poses


class Sender(threading.Thread):
    def __init__(self, host, port, grip):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr, self.sid, self.seq = (host, port), random.randint(1, 99999), 0
        self.q, self.grip, self.on = None, grip, True

    def run(self):
        nxt = time.monotonic()
        while self.on:
            q = self.q
            if q is not None:
                self.seq += 1
                msg = {"sid": self.sid, "seq": self.seq, "q": [round(x, 4) for x in q], "grip": self.grip}
                self.sock.sendto(json.dumps(msg).encode(), self.addr)
            nxt += 1.0 / HZ
            time.sleep(max(0.0, nxt - time.monotonic()))


def move(name, a):
    poses = load()
    if name not in poses:
        sys.exit(f"'{name}' 자세가 없다. 있는 것: {', '.join(poses)}")
    st = get("/api/state")
    q0 = [float(x) for x in st["cmd"]]
    qm = [float(x) for x in st["joints"]]
    q1 = [float(x) for x in poses[name]]
    grip = round(float(st["gripper"]["pos"]) / 1000.0, 3)
    big = max(abs(y - x) for x, y in zip(q0, q1))
    T = max(1.0, 1.875 * big / a.vmax)                   # 최소 저크 최고 속도 = 평균 × 1.875
    zs = [fk_tcp(lerp(q0, q1, k / 50))[2] for k in range(51)]
    tcp0 = fk_tcp(q0)
    d_fk = math.dist(tcp0, [float(x) for x in st["tcp"][:3]])
    print(f"지금 {[round(x, 2) for x in q0]}\n목표 {name} {q1}")
    print(f"관절 변화 {[round(y - x, 1) for x, y in zip(q0, q1)]} · 가장 큰 {big:.1f}° · {T:.1f} s (최고 {a.vmax:g}°/s)")
    print(f"TCP 높이 시작 {zs[0]:.0f} · 끝 {zs[-1]:.0f} · 경로 최저 {min(zs):.0f} mm · 그리퍼 {grip * 1000:.0f} 유지")
    if d_fk > 5.0:
        print(f"  ⚠ FK 와 제어판 TCP 가 {d_fk:.1f} mm 다르다 — 공구 번호·길이 확인(높이 검사가 틀릴 수 있음)")
    if min(zs) < min(zs[0], zs[-1]) - 20.0:
        sys.exit("경로 중간에 TCP 가 끝점보다 20 mm 넘게 내려간다 — 보내지 않음")
    if big < 0.2:
        print("이미 그 자세다."); return
    if a.dry:
        print("(--dry: 보내지 않음)"); return
    if st.get("mode") != "engaged":
        sys.exit("제어판이 조작 상태가 아니다 — 제어판에서 [조작 시작]을 먼저 누른다")
    if max(abs(x - y) for x, y in zip(q0, qm)) > 2.0:
        sys.exit("명령과 실측이 2° 넘게 다르다 — 로봇이 선 뒤 다시")

    snd = Sender(a.udp_host, a.udp_port, grip)
    snd.q = q0
    snd.start()
    try:
        input("지금 자세를 보내는 중. 제어판 /leader 에서 [리더 추종 시작]을 누른 뒤 Enter (취소 Ctrl+C) > ")
        try:
            print("추종 상태:", json.dumps(get("/api/leader/state"), ensure_ascii=False)[:160])
        except Exception as e:                            # 상태 읽기는 참고용
            print("추종 상태 읽기 실패:", repr(e)[:80])
        t0, chk, worst, over = time.monotonic(), 0.0, 0.0, 0
        while True:
            tau = min(1.0, (time.monotonic() - t0) / T)
            snd.q = lerp(q0, q1, 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5)
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
                if over >= 3:                        # 0.6 s 연속으로 뒤처질 때만 멈춤
                    raise Abort(f"로봇이 명령보다 {err:.1f}° 뒤처진다 — 추종이 안 켜졌거나 제어판이 막았다")
            if tau >= 1.0:
                break
            time.sleep(1.0 / HZ)
        time.sleep(1.5)                                   # 끝 자세를 잠시 더 보낸다
        st = get("/api/state", 1.0)
        e = max(abs(float(x) - y) for x, y in zip(st["joints"], q1))
        print(f"도착: 관절 차 최대 {e:.2f}° · 이동 중 최대 뒤처짐 {worst:.1f}° · TCP {st['tcp']}")
    except KeyboardInterrupt:
        print("\nCtrl+C — 보내기를 멈춘다")
    except Abort as e:
        print("멈춤:", e)
    except Exception as e:
        print("멈춤(제어판 읽기 실패):", repr(e)[:120])
    finally:
        snd.on = False
        print("보내기 끝 — 제어판이 0.3 s 뒤 그 자리에 선다(다음 실행 때 [리더 추종 시작]을 다시 누른다)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", help="자세 이름(home·pour·저장한 이름) 또는 save · list")
    ap.add_argument("name", nargs="?", help="save 할 이름")
    ap.add_argument("--vmax", type=float, default=10.0, help="관절 최고 속도 °/s (기본 10)")
    ap.add_argument("--max-lag", type=float, default=8.0, help="이만큼(°) 뒤처지면 멈춤")
    ap.add_argument("--dry", action="store_true", help="계획만 보기")
    ap.add_argument("--udp-host", default="127.0.0.1")
    ap.add_argument("--udp-port", type=int, default=8097)
    a = ap.parse_args()
    if a.what == "list":
        for k, v in load().items():
            print(f"{k:10s} {v}  TCP {[round(x) for x in fk_tcp(v)]}")
    elif a.what == "save":
        if not a.name:
            sys.exit("save 할 이름을 준다: python3 pose_go.py save 이름")
        st = get("/api/state")
        saved = {}
        if os.path.exists(POSES):
            with open(POSES, encoding="utf-8") as f:
                saved = json.load(f)
        saved[a.name] = [round(float(x), 3) for x in st["cmd"]]
        with open(POSES, "w", encoding="utf-8") as f:
            json.dump(saved, f, ensure_ascii=False, indent=1)
        print(f"{a.name} = {saved[a.name]} · 제어판 TCP {st['tcp']} · 그리퍼 {st['gripper']['pos']} → {POSES}")
    else:
        if a.vmax <= 0 or a.vmax > 30:
            sys.exit("--vmax 는 0–30 °/s")
        move(a.what, a)


if __name__ == "__main__":
    main()
