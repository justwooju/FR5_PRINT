#!/usr/bin/env python3
"""fr5cam — FR5 작업 녹화 (단독 프로그램, 표준 라이브러리만).

- 카메라는 fr5rec 가 열어 둔 것을 읽기만 한다: GET /api/frame/<third|wrist>.jpg (fr5rec 녹화 · 데이터셋 · 설정은 건드리지 않음).
- 제어판 /api/state · fr5print /api/pstatus 도 읽기만 한다 (POST 없음).
- 쓰는 곳은 이 폴더 아래 videos/ 와 settings.json 뿐이다.
- 녹화 = 카메라마다 MJPEG AVI 한 개(1.9 GB 마다 다음 파일) + frames.csv(장면마다 시각 · 공구 자세) + meta.json.

켜기:  python3 ~/fr5cap/camrec/fr5cam.py   →  http://localhost:8798
"""
import argparse, csv, datetime as dt, io, json, os, re, shutil, struct, subprocess, sys, threading, time, urllib.request
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs, quote

VERSION = "0.1.0"
CAMS = ("third", "wrist")
CAM_KO = {"third": "3인칭", "wrist": "손목"}
ROLL_BYTES = 1_900_000_000          # AVI 한 파일 한도 (RIFF 2 GB 아래)
OP = urllib.request.build_opener(urllib.request.ProxyHandler({}))
DEFAULTS = {"fps": 10, "cams": ["third", "wrist"], "tcp": True, "auto": False, "tail_s": 3.0, "pause_wait_s": 120.0,
            "max_min": 30.0, "min_free_gb": 3.0, "label": ""}


def now_iso():
    return dt.datetime.now().strftime("%Y-%m-%dT%H:%M:%S")


def clean_label(s):
    s = re.sub(r"[^0-9A-Za-z가-힣_\-]+", "_", str(s or "")).strip("_")
    return s[:40]


def jpeg_size(b):
    """JPEG 의 SOF 에서 (가로, 세로). 못 찾으면 None."""
    i = 2
    n = len(b)
    while i + 9 < n:
        if b[i] != 0xFF:
            i += 1
            continue
        m = b[i + 1]
        if m in (0xD8, 0x01) or 0xD0 <= m <= 0xD7:
            i += 2
            continue
        L = struct.unpack(">H", b[i + 2:i + 4])[0]
        if m in (0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF):
            h, w = struct.unpack(">HH", b[i + 5:i + 9])
            return w, h
        i += 2 + L
    return None


# ── AVI (MJPEG) ────────────────────────────────────────────────────────────────────────
class AviWriter:
    """MJPEG AVI 1 스트림. 머리를 먼저 쓰고 닫을 때 크기 · 장면 수 · 색인(idx1)을 채운다."""

    def __init__(self, path, fps, w, h):
        self.path, self.fps, self.w, self.h = path, int(fps), int(w), int(h)
        self.f = open(path + ".tmp", "wb")
        self.idx = []                    # (movi 기준 자리, 크기)
        self.maxsz = 0
        self._head()

    def _head(self):
        f = self.f
        f.write(b"RIFF" + struct.pack("<I", 0) + b"AVI ")
        hdrl = io.BytesIO()
        avih = struct.pack("<IIIIIIIIII4I", int(1e6 / self.fps), 0, 0, 0x10, 0, 0, 1, 0, self.w, self.h, 0, 0, 0, 0)
        strh = struct.pack("<4s4sIHHIIIIIIiI4h", b"vids", b"MJPG", 0, 0, 0, 0, 1, self.fps, 0, 0, 0, -1, 0, 0, 0, self.w, self.h)
        strf = struct.pack("<IiiHH4sIiiII", 40, self.w, self.h, 1, 24, b"MJPG", self.w * self.h * 3, 0, 0, 0, 0)
        strl = b"strh" + struct.pack("<I", len(strh)) + strh + b"strf" + struct.pack("<I", len(strf)) + strf
        hdrl.write(b"avih" + struct.pack("<I", len(avih)) + avih)
        hdrl.write(b"LIST" + struct.pack("<I", 4 + len(strl)) + b"strl" + strl)
        hb = hdrl.getvalue()
        f.write(b"LIST" + struct.pack("<I", 4 + len(hb)) + b"hdrl")
        self.o_avih = f.tell() + 8
        f.write(hb)
        self.o_strh = self.o_avih + len(avih) + 8 + 12        # avih 뒤 LIST(12) + 'strh'(8)
        f.write(b"LIST" + struct.pack("<I", 0) + b"movi")
        self.o_movi = f.tell() - 4                             # 'movi' 글자 자리 = idx1 자리 기준
        f.flush()

    @property
    def size(self):
        return self.f.tell()

    def write(self, jpg):
        f = self.f
        off = f.tell() - self.o_movi
        f.write(b"00dc" + struct.pack("<I", len(jpg)) + jpg)
        if len(jpg) & 1:
            f.write(b"\0")
        self.idx.append((off, len(jpg)))
        self.maxsz = max(self.maxsz, len(jpg))

    def close(self):
        f = self.f
        end_movi = f.tell()
        f.write(b"idx1" + struct.pack("<I", 16 * len(self.idx)))
        f.write(b"".join(struct.pack("<4sIII", b"00dc", 0x10, o, s) for o, s in self.idx))
        end = f.tell()
        n = len(self.idx)
        f.seek(4); f.write(struct.pack("<I", end - 8))
        f.seek(self.o_movi - 4); f.write(struct.pack("<I", end_movi - self.o_movi))
        f.seek(self.o_avih); f.write(struct.pack("<II", int(1e6 / self.fps), int(self.maxsz * self.fps)))
        f.seek(self.o_avih + 16); f.write(struct.pack("<I", n))
        f.seek(self.o_avih + 28); f.write(struct.pack("<I", self.maxsz + 8))
        f.seek(self.o_strh + 32); f.write(struct.pack("<II", n, self.maxsz + 8))
        f.close()
        os.replace(self.path + ".tmp", self.path)
        return n


def avi_index(path):
    """AVI 의 장면 자리 [(파일 자리, 크기)]. idx1 이 없으면 movi 를 훑는다."""
    out = []
    with open(path, "rb") as f:
        data_len = os.path.getsize(path)
        if f.read(4) != b"RIFF":
            raise ValueError("AVI 아님")
        f.read(8)
        movi = None
        idx = None
        while f.tell() + 8 <= data_len:
            p = f.tell()
            ck = f.read(4)
            sz = struct.unpack("<I", f.read(4))[0]
            if ck == b"LIST":
                kind = f.read(4)
                if kind == b"movi":
                    movi = (p + 8, p + 8 + sz if sz else data_len)
                f.seek(p + 8 + sz + (sz & 1))
                if kind == b"movi" and sz == 0:
                    break
            elif ck == b"idx1":
                idx = (p + 8, sz)
                break
            else:
                f.seek(p + 8 + sz + (sz & 1))
        if movi is None:
            raise ValueError("movi 없음")
        if idx:
            f.seek(idx[0])
            raw = f.read(idx[1])
            ents = [struct.unpack("<4sIII", raw[k:k + 16]) for k in range(0, len(raw) - 15, 16)]
            base = movi[0] if ents and ents[0][2] < movi[0] else 0      # movi 기준이면 더함
            for ck, fl, o, s in ents:
                if ck[2:] == b"dc":
                    out.append((base + o + 8, s))
            return out
        p = movi[0] + 4
        end = min(movi[1], data_len)
        while p + 8 <= end:
            f.seek(p)
            ck = f.read(4)
            sz = struct.unpack("<I", f.read(4))[0]
            if p + 8 + sz > data_len:
                break
            if ck[2:] == b"dc":
                out.append((p + 8, sz))
            p += 8 + sz + (sz & 1)
    return out


def avi_repair(tmp_path, fps):
    """갑자기 꺼져 닫히지 않은 .avi.tmp → 남은 장면으로 .avi 를 다시 만든다 (원본 .tmp 는 .bak 으로 남김)."""
    with open(tmp_path, "rb") as f:
        head = f.read(1 << 16)
    k = head.find(b"movi")
    if k < 0:
        return 0
    bak = tmp_path + ".bak_" + time.strftime("%Y%m%d_%H%M%S")
    os.replace(tmp_path, bak)                       # 원본은 그대로 남긴다
    frames = []
    with open(bak, "rb") as f:
        n = os.path.getsize(bak)
        p = k + 4
        while p + 8 <= n:
            f.seek(p)
            ck = f.read(4)
            sz = struct.unpack("<I", f.read(4))[0]
            if ck != b"00dc" or p + 8 + sz > n:
                break
            frames.append((p + 8, sz))
            p += 8 + sz + (sz & 1)
        if not frames:
            return 0
        f.seek(frames[0][0])
        wh = jpeg_size(f.read(frames[0][1])) or (848, 480)
        out = tmp_path[:-4]
        if os.path.exists(out):
            out = out[:-4] + "_repaired.avi"
        W = AviWriter(out, fps, *wh)
        for o, s in frames:
            f.seek(o)
            W.write(f.read(s))
        W.close()
    return len(frames)


# ── 읽기 (fr5rec · 제어판 · fr5print) ─────────────────────────────────────────────────
def http_get(url, headers=None, timeout=0.6):
    r = OP.open(urllib.request.Request(url, headers=headers or {}), timeout=timeout)
    return r.read(), r.headers


class Sources:
    def __init__(self, a):
        self.rec, self.panel, self.prn = a.rec.rstrip("/"), a.panel.rstrip("/"), a.print_url.rstrip("/")
        self.lock = threading.Lock()
        self.st = {"panel": None, "panel_t": 0.0, "panel_err": "", "print": None, "print_t": 0.0, "print_err": "",
                   "rec": {}, "rec_err": "", "rec_state": None}
        self.want_panel = 0.0
        self.want_print = 0.0
        self.stop = False
        threading.Thread(target=self._loop, daemon=True).start()

    def frame(self, cam, timeout=0.6):
        b, h = http_get(f"{self.rec}/api/frame/{cam}.jpg", {"X-FR5REC": "1"}, timeout)
        if b[:2] != b"\xff\xd8":
            raise ValueError("JPEG 아님")
        num = h.get("X-Frame-Num")
        age = h.get("X-Frame-Age-Ms")
        info = {"num": int(num) if num and num.lstrip("-").isdigit() else None,
                "age": float(age) if age and re.match(r"^-?\d+(\.\d+)?$", age) else None, "t": time.time()}
        with self.lock:
            self.st["rec"][cam] = dict(info, ok=True)
            self.st["rec_err"] = ""
        return b, info

    def _loop(self):
        tp = tq = tr = 0.0
        while not self.stop:
            t = time.monotonic()
            if self.want_panel > 0 and t - tp >= 1.0 / self.want_panel:
                tp = t
                try:
                    b, _ = http_get(self.panel + "/api/state", timeout=0.4)
                    with self.lock:
                        self.st["panel"], self.st["panel_t"], self.st["panel_err"] = json.loads(b), time.time(), ""
                except Exception as e:
                    with self.lock:
                        self.st["panel_err"] = repr(e)[:80]
            if self.want_print > 0 and t - tq >= 1.0 / self.want_print:
                tq = t
                try:
                    b, _ = http_get(self.prn + "/api/pstatus", timeout=0.8)
                    with self.lock:
                        self.st["print"], self.st["print_t"], self.st["print_err"] = json.loads(b), time.time(), ""
                except Exception as e:
                    with self.lock:
                        self.st["print"], self.st["print_err"] = None, repr(e)[:80]
            if t - tr >= 2.0:
                tr = t
                try:
                    b, _ = http_get(self.rec + "/api/status", {"X-FR5REC": "1"}, timeout=0.6)
                    j = json.loads(b)
                    with self.lock:
                        self.st["rec_state"] = (j.get("rec") or {}).get("state")
                except Exception as e:
                    with self.lock:
                        self.st["rec_state"] = None
                        self.st["rec_err"] = repr(e)[:80]
            time.sleep(0.02)

    def snap(self):
        with self.lock:
            return json.loads(json.dumps(self.st))


# ── 녹화 ─────────────────────────────────────────────────────────────────────────────
class Recorder:
    def __init__(self, app, opts, why):
        self.app, self.o, self.why = app, dict(opts), why
        self.fps = int(opts["fps"])
        self.cams = [c for c in CAMS if c in opts["cams"]]
        stamp = time.strftime("%Y%m%d_%H%M%S")
        lab = clean_label(opts.get("label"))
        base = stamp + ("_" + lab if lab else "")
        self.name, k = base, 1
        while os.path.exists(os.path.join(app.vdir, self.name)):
            k += 1
            self.name = f"{base}_{k}"
        self.dir = os.path.join(app.vdir, self.name)
        os.makedirs(self.dir, exist_ok=False)
        self.stop_req = None
        self.t0 = None
        self.n = 0
        self.cs = {c: {"files": [], "frames": 0, "bytes": 0, "miss": 0, "dup": 0, "w": None, "h": None, "last": None, "lastnum": None, "part": 0, "W": None} for c in self.cams}
        self.late = 0
        self.runs = []                  # fr5print 실행 (plan id · 이름 · 상태 바뀜)
        self.end_reason = None
        self.meta = {"fr5cam": VERSION, "name": self.name, "label": lab, "start": now_iso(), "end": None, "fps": self.fps,
                     "cams": self.cams, "why_start": why, "source": "fr5rec /api/frame/<cam>.jpg (읽기만)", "tcp": bool(opts["tcp"]),
                     "state": "recording", "note": ""}
        self._meta()
        self.csvf = open(os.path.join(self.dir, "frames.csv"), "w", newline="")
        self.cw = csv.writer(self.csvf)
        hdr = ["i", "t_s", "wall"]
        for c in self.cams:
            hdr += [f"{c}_num", f"{c}_age_ms", f"{c}_src"]
        hdr += ["mode", "x", "y", "z", "rx", "ry", "rz", "j1", "j2", "j3", "j4", "j5", "j6", "grip", "print_state", "print_op", "print_k"]
        self.cw.writerow(hdr)
        self.pool = ThreadPoolExecutor(max_workers=max(1, len(self.cams)))
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def _meta(self):
        p = os.path.join(self.dir, "meta.json")
        with open(p + ".tmp", "w") as f:
            json.dump(self.meta, f, ensure_ascii=False, indent=1)
        os.replace(p + ".tmp", p)

    def _open(self, c, jpg):
        s = self.cs[c]
        wh = jpeg_size(jpg) or (848, 480)
        s["w"], s["h"] = wh
        s["part"] += 1
        fn = f"{c}.avi" if s["part"] == 1 else f"{c}_{s['part']:03d}.avi"
        s["files"].append(fn)
        s["W"] = AviWriter(os.path.join(self.dir, fn), self.fps, *wh)

    def _grab(self, c):
        try:
            return self.app.src.frame(c, timeout=min(0.8, 3.0 / self.fps))
        except Exception as e:
            return None, {"err": repr(e)[:60]}

    def _write(self, c, jpg, src):
        s = self.cs[c]
        if s["W"] is None or s["W"].size + len(jpg) > ROLL_BYTES:
            if s["W"] is not None:
                s["W"].close()
            self._open(c, jpg)
        s["W"].write(jpg)
        s["frames"] += 1
        s["bytes"] += len(jpg) + 8
        if src != "new":
            s["dup" if src == "same" else "miss"] += 1

    def _run(self):
        app = self.app
        app.src.want_panel = min(10.0, float(self.fps)) if self.o["tcp"] else 0.0
        app.src.want_print = 2.0
        fail_since = first_wait = None
        try:
            self.t0 = time.monotonic()
            per = 1.0 / self.fps
            maxs = float(self.o["max_min"]) * 60.0
            while True:
                tgt = self.t0 + self.n * per
                dtw = tgt - time.monotonic()
                if dtw > 0:
                    time.sleep(dtw)
                if self.stop_req:
                    self.end_reason = self.stop_req
                    break
                el = time.monotonic() - self.t0
                if el >= maxs:
                    self.end_reason = f"최대 길이 {self.o['max_min']:g} 분"
                    break
                if self.n % max(1, self.fps * 5) == 0:
                    free = shutil.disk_usage(self.dir).free / 1e9
                    if free < float(self.o["min_free_gb"]):
                        self.end_reason = f"디스크 여유 {free:.1f} GB < {self.o['min_free_gb']:g} GB"
                        break
                futs = {c: self.pool.submit(self._grab, c) for c in self.cams}
                got = {c: futs[c].result() for c in self.cams}
                # 늦었으면 놓친 박자만큼 앞 장면을 되풀이해 영상 시간을 맞춘다
                behind = int((time.monotonic() - self.t0) / per) - self.n
                anyok = False
                row_src = {}
                for c in self.cams:
                    jpg, info = got[c]
                    s = self.cs[c]
                    if jpg is not None:
                        src = "same" if (info.get("num") is not None and info.get("num") == s["lastnum"]) else "new"
                        s["last"], s["lastnum"] = jpg, info.get("num")
                        anyok = True
                    elif s["last"] is not None:
                        src = "miss"
                    else:
                        src = None
                    row_src[c] = (src, info)
                if not anyok:
                    fail_since = fail_since or time.monotonic()
                    if time.monotonic() - fail_since > 10:
                        self.end_reason = "fr5rec 프레임을 10 s 동안 못 받음"
                        break
                else:
                    fail_since = None
                if any(row_src[c][0] is None for c in self.cams):
                    # 첫 장면을 아직 못 받은 카메라가 있으면 이 박자는 건너뜀 (시작 직후)
                    if self.n == 0:
                        first_wait = first_wait or time.monotonic()
                        if time.monotonic() - first_wait > 5:
                            self.end_reason = "첫 장면을 5 s 동안 못 받은 카메라가 있음: " + ", ".join(CAM_KO[c] for c in self.cams if row_src[c][0] is None)
                            break
                        time.sleep(0.05)
                        self.t0 = time.monotonic()
                        continue
                    row_src = {c: (row_src[c][0] or "miss", row_src[c][1]) for c in self.cams}
                reps = 1 + max(0, min(behind, self.fps * 5))
                if behind > 0:
                    self.late += behind
                for r in range(reps):
                    for c in self.cams:
                        s = self.cs[c]
                        if s["last"] is None:
                            continue
                        self._write(c, s["last"], row_src[c][0] if r == 0 else "same")
                    self._row(row_src if r == 0 else {c: ("same", {}) for c in self.cams})
                    self.n += 1
                if self.n % max(1, self.fps) == 0:
                    self.csvf.flush()
                    self.meta["frames"] = self.n
                    self._meta()
        except Exception as e:
            self.end_reason = "오류 " + repr(e)[:120]
        finally:
            self._close()

    def _row(self, rs):
        st = self.app.src.snap()
        el = self.n / self.fps
        row = [self.n, f"{el:.3f}", dt.datetime.now().isoformat(timespec="milliseconds")]
        for c in self.cams:
            src, info = rs.get(c, (None, {}))
            row += [info.get("num", ""), "" if info.get("age") is None else f"{info['age']:.0f}", src or ""]
        p = st.get("panel") if (self.o["tcp"] and st.get("panel") and time.time() - st["panel_t"] < 1.0) else None
        if p:
            tcp = [float(x) for x in (p.get("tcp") or [])[:6]] + [None] * 6
            js = [float(x) for x in (p.get("joints") or [])[:6]] + [None] * 6
            g = (p.get("gripper") or {}).get("pos")
            row += [p.get("mode", "")] + ["" if v is None else f"{v:.2f}" for v in tcp[:6]] + ["" if v is None else f"{v:.3f}" for v in js[:6]] + ["" if g is None else g]
        else:
            row += [""] * 14
        pr = st.get("print")
        if pr and time.time() - st["print_t"] < 2.0:
            r = pr.get("run") or {}
            row += [r.get("state", ""), r.get("op", ""), r.get("k", "")]
            pid = pr.get("plan_id")
            if r.get("state") and (not self.runs or self.runs[-1].get("state") != r.get("state") or self.runs[-1].get("plan") != pid):
                self.runs.append({"t_s": round(el, 2), "state": r.get("state"), "plan": pid, "msg": r.get("msg")})
        else:
            row += ["", "", ""]
        self.cw.writerow(row)

    def _close(self):
        for c in self.cams:
            s = self.cs[c]
            if s["W"] is not None:
                try:
                    s["W"].close()
                except Exception:
                    pass
                s["W"] = None
        try:
            self.csvf.close()
        except Exception:
            pass
        self.pool.shutdown(wait=False)
        self.meta.update({"end": now_iso(), "state": "done", "end_reason": self.end_reason, "frames": self.n,
                          "seconds": round(self.n / self.fps, 2), "late_ticks": self.late, "print_runs": self.runs[:200],
                          "cam": {c: {k: self.cs[c][k] for k in ("files", "frames", "bytes", "miss", "dup", "w", "h")} for c in self.cams}})
        self._meta()
        self.app.src.want_panel = 0.0
        self.app.on_rec_end(self)

    def status(self):
        el = (self.n / self.fps) if self.t0 else 0.0
        return {"name": self.name, "seconds": round(el, 1), "frames": self.n, "fps": self.fps, "why": self.why, "late": self.late,
                "cams": {c: {k: self.cs[c][k] for k in ("frames", "bytes", "miss", "dup", "w", "h")} for c in self.cams},
                "bytes": sum(self.cs[c]["bytes"] for c in self.cams)}


# ── 앱 ───────────────────────────────────────────────────────────────────────────────
class App:
    def __init__(self, a):
        self.a = a
        self.dir = os.path.expanduser(a.dir)
        self.vdir = os.path.join(self.dir, "videos")
        os.makedirs(self.vdir, exist_ok=True)
        self.sp = os.path.join(self.dir, "settings.json")
        self.o = dict(DEFAULTS)
        try:
            self.o.update({k: v for k, v in json.load(open(self.sp)).items() if k in DEFAULTS})
        except Exception:
            pass
        self.o = self.clean(self.o)
        self.src = Sources(a)
        self.lock = threading.Lock()
        self.rec = None
        self.last = None              # 마지막 녹화 이름 · 끝난 까닭
        self.auto_state = {"armed": False, "since": None, "msg": ""}
        self.conv = {}                # 이름 → {"state", "msg"}
        self.idx_cache = {}
        self.ffmpeg = shutil.which("ffmpeg")
        self.repaired = self.repair_all()
        threading.Thread(target=self._auto_loop, daemon=True).start()

    def clean(self, o):
        r = dict(DEFAULTS)
        try:
            r["fps"] = int(o.get("fps", 10)) if int(o.get("fps", 10)) in (5, 10, 15) else 10
        except Exception:
            pass
        cams = [c for c in CAMS if c in (o.get("cams") or [])]
        r["cams"] = cams or ["third"]
        r["tcp"] = bool(o.get("tcp", True))
        r["auto"] = bool(o.get("auto", False))
        for k, lo, hi in (("tail_s", 0.0, 60.0), ("pause_wait_s", 0.0, 3600.0), ("max_min", 0.5, 240.0), ("min_free_gb", 0.5, 500.0)):
            try:
                r[k] = min(hi, max(lo, float(o.get(k, DEFAULTS[k]))))
            except Exception:
                pass
        r["label"] = clean_label(o.get("label", ""))
        return r

    def save(self):
        with open(self.sp + ".tmp", "w") as f:
            json.dump(self.o, f, ensure_ascii=False, indent=1)
        os.replace(self.sp + ".tmp", self.sp)

    def repair_all(self):
        """지난번에 갑자기 꺼져 닫히지 않은 녹화를 고친다 (지우지 않음)."""
        out = []
        for nm in sorted(os.listdir(self.vdir)):
            d = os.path.join(self.vdir, nm)
            mp = os.path.join(d, "meta.json")
            if not os.path.isfile(mp):
                continue
            try:
                m = json.load(open(mp))
            except Exception:
                continue
            if m.get("state") != "recording":
                continue
            fixed = {}
            for fn in sorted(os.listdir(d)):
                if fn.endswith(".avi.tmp"):
                    try:
                        fixed[fn[:-4]] = avi_repair(os.path.join(d, fn), m.get("fps", 10))
                    except Exception as e:
                        fixed[fn[:-4]] = "실패 " + repr(e)[:60]
            m.update({"state": "repaired", "end_reason": "프로그램이 녹화 중에 꺼짐 — 남은 장면으로 고침", "repaired": fixed, "end": m.get("end") or "?"})
            with open(mp, "w") as f:
                json.dump(m, f, ensure_ascii=False, indent=1)
            out.append({"name": nm, "fixed": fixed})
        return out

    def start(self, why="버튼"):
        with self.lock:
            if self.rec is not None:
                return "이미 녹화 중"
            free = shutil.disk_usage(self.vdir).free / 1e9
            if free < self.o["min_free_gb"]:
                return f"디스크 여유 {free:.1f} GB — {self.o['min_free_gb']:g} GB 아래라 시작하지 않음"
            bad = []
            for c in self.o["cams"]:
                try:
                    self.src.frame(c, timeout=1.0)
                except Exception as e:
                    bad.append(f"{CAM_KO[c]} ({repr(e)[:50]})")
            if bad:
                return "fr5rec 에서 프레임을 못 받음: " + ", ".join(bad) + " — fr5rec 가 켜져 있고 카메라가 열려 있는지 본다"
            self.rec = Recorder(self, self.o, why)
            return None

    def stop(self, why="버튼"):
        with self.lock:
            r = self.rec
        if r is None:
            return "녹화 중이 아님"
        r.stop_req = why
        r.th.join(timeout=10)
        return None

    def on_rec_end(self, r):
        with self.lock:
            self.last = {"name": r.name, "end_reason": r.end_reason, "seconds": round(r.n / r.fps, 1)}
            self.rec = None

    def _auto_loop(self):
        """fr5print 가 작업을 돌리는 동안 자동 녹화 (읽기만)."""
        while True:
            time.sleep(0.25)
            o = self.o
            if not o["auto"]:
                self.auto_state = {"armed": False, "since": None, "msg": ""}
                if self.rec is None:
                    self.src.want_print = 0.0
                continue
            self.src.want_print = 2.0
            st = self.src.snap()
            pr = st.get("print")
            if not pr or time.time() - st.get("print_t", 0) > 3:
                self.auto_state = {"armed": True, "since": None, "msg": "fr5print 응답 없음 — 기다림"}
                continue
            state = (pr.get("run") or {}).get("state")
            rec = self.rec
            if state == "running":
                self.auto_state = {"armed": True, "since": None, "msg": "fr5print 실행 중"}
                if rec is None:
                    err = self.start("자동 (fr5print 실행)")
                    self.auto_state["msg"] = err or "fr5print 실행 → 녹화 시작"
                continue
            if rec is None or not rec.why.startswith("자동"):
                self.auto_state = {"armed": True, "since": None, "msg": f"기다림 (fr5print {state or '?'})"}
                continue
            since = self.auto_state.get("since") or time.time()
            wait = o["pause_wait_s"] if state == "paused" else o["tail_s"]
            self.auto_state = {"armed": True, "since": since, "msg": f"fr5print {state} — {max(0, wait - (time.time() - since)):.0f} s 뒤 끝"}
            if time.time() - since >= wait:
                threading.Thread(target=self.stop, args=(f"자동 (fr5print {state})",), daemon=True).start()
                self.auto_state = {"armed": True, "since": None, "msg": "녹화 끝 (자동)"}
                time.sleep(1.0)

    def recs(self):
        out = []
        for nm in sorted(os.listdir(self.vdir), reverse=True):
            d = os.path.join(self.vdir, nm)
            mp = os.path.join(d, "meta.json")
            if not os.path.isfile(mp):
                continue
            try:
                m = json.load(open(mp))
            except Exception:
                continue
            files = sorted(fn for fn in os.listdir(d) if not fn.endswith(".tmp") and ".bak_" not in fn)
            size = sum(os.path.getsize(os.path.join(d, fn)) for fn in files)
            out.append({"name": nm, "label": m.get("label", ""), "start": m.get("start"), "end": m.get("end"), "state": m.get("state"),
                        "seconds": m.get("seconds"), "fps": m.get("fps"), "cams": m.get("cams"), "why": m.get("why_start"),
                        "end_reason": m.get("end_reason"), "note": m.get("note", ""), "files": files, "bytes": size,
                        "runs": len(m.get("print_runs") or []), "conv": self.conv.get(nm)})
            if len(out) >= 200:
                break
        return out

    def rec_dir(self, nm):
        if not re.match(r"^[0-9A-Za-z가-힣_\-]+$", nm or ""):
            return None
        d = os.path.join(self.vdir, nm)
        return d if os.path.isdir(d) else None

    def frame_of(self, nm, cam, i):
        d = self.rec_dir(nm)
        if d is None or cam not in CAMS:
            return None
        m = json.load(open(os.path.join(d, "meta.json")))
        files = ((m.get("cam") or {}).get(cam) or {}).get("files") or sorted(fn for fn in os.listdir(d) if re.match(rf"^{cam}(_\d{{3}})?\.avi$", fn))
        for fn in files:
            p = os.path.join(d, fn)
            key = (p, os.path.getmtime(p))
            ix = self.idx_cache.get(key)
            if ix is None:
                ix = avi_index(p)
                self.idx_cache[key] = ix
            if i < len(ix):
                o, s = ix[i]
                with open(p, "rb") as f:
                    f.seek(o)
                    return f.read(s)
            i -= len(ix)
        return None

    def info(self, nm):
        d = self.rec_dir(nm)
        if d is None:
            return None
        m = json.load(open(os.path.join(d, "meta.json")))
        rows = []
        cp = os.path.join(d, "frames.csv")
        if os.path.isfile(cp):
            with open(cp, newline="") as f:
                rd = csv.DictReader(f)
                for r in rd:
                    rows.append([float(r["t_s"] or 0), r.get("x", ""), r.get("y", ""), r.get("z", ""), r.get("print_state", ""), r.get("print_op", "")])
        n = {}
        for c in m.get("cams") or []:
            try:
                n[c] = sum(len(avi_index(os.path.join(d, fn))) for fn in ((m.get("cam") or {}).get(c) or {}).get("files", []))
            except Exception:
                n[c] = 0
        return {"meta": m, "rows": rows, "n": n}

    def convert(self, nm):
        d = self.rec_dir(nm)
        if d is None:
            return "녹화 없음"
        if not self.ffmpeg:
            return "ffmpeg 이 없다 — AVI 를 받아 VLC 로 본다"
        if self.rec is not None:
            return "녹화 중에는 MP4 를 만들지 않는다 (CPU)"
        try:
            b, _ = http_get(self.a.print_url.rstrip("/") + "/api/pstatus", timeout=0.8)
            if ((json.loads(b).get("run") or {}).get("state")) == "running":
                return "fr5print 가 작업 중이라 MP4 를 만들지 않는다 (로봇 제어 PC 부담) — 끝난 뒤 다시"
        except Exception:
            pass
        if (self.conv.get(nm) or {}).get("state") == "running":
            return "이미 만드는 중"
        m = json.load(open(os.path.join(d, "meta.json")))
        self.conv[nm] = {"state": "running", "msg": "만드는 중"}
        threading.Thread(target=self._convert, args=(nm, d, m), daemon=True).start()
        return None

    def _convert(self, nm, d, m):
        try:
            enc = subprocess.run([self.ffmpeg, "-hide_banner", "-encoders"], capture_output=True, text=True, timeout=20).stdout
            vc = ["-c:v", "libx264", "-preset", "veryfast", "-crf", "23", "-pix_fmt", "yuv420p", "-movflags", "+faststart"] if "libx264" in enc else ["-c:v", "mpeg4", "-q:v", "4"]
            fps = str(m.get("fps", 10))
            done = []
            nice = ["nice", "-n", "19"] if shutil.which("nice") else []
            ins = {}
            for c in m.get("cams") or []:
                files = ((m.get("cam") or {}).get(c) or {}).get("files") or []
                if not files:
                    continue
                if len(files) == 1:
                    ins[c] = ["-r", fps, "-i", os.path.join(d, files[0])]
                else:
                    lst = os.path.join(d, f".{c}_list.txt")
                    with open(lst, "w") as f:
                        for fn in files:
                            f.write(f"file '{fn}'\n")
                    ins[c] = ["-r", fps, "-f", "concat", "-safe", "0", "-i", lst]
                out = os.path.join(d, f"{c}.mp4")
                r = subprocess.run(nice + [self.ffmpeg, "-loglevel", "error", "-y"] + ins[c] + ["-an"] + vc + [out + ".part.mp4"], capture_output=True, text=True, timeout=3 * 3600)
                if r.returncode != 0:
                    raise RuntimeError(r.stderr[-200:])
                os.replace(out + ".part.mp4", out)
                done.append(f"{c}.mp4")
            if len(ins) == 2:
                out = os.path.join(d, "both.mp4")
                r = subprocess.run(nice + [self.ffmpeg, "-loglevel", "error", "-y"] + ins["third"] + ins["wrist"] +
                                   ["-filter_complex", "[0:v]scale=-2:480[a];[1:v]scale=-2:480[b];[a][b]hstack=inputs=2[v]", "-map", "[v]", "-an"] + vc + [out + ".part.mp4"],
                                   capture_output=True, text=True, timeout=3 * 3600)
                if r.returncode == 0:
                    os.replace(out + ".part.mp4", out)
                    done.append("both.mp4")
            self.conv[nm] = {"state": "done", "msg": "만듦: " + ", ".join(done)}
        except Exception as e:
            self.conv[nm] = {"state": "fail", "msg": "실패 " + repr(e)[:160]}

    def note(self, nm, text):
        d = self.rec_dir(nm)
        if d is None:
            return "녹화 없음"
        mp = os.path.join(d, "meta.json")
        m = json.load(open(mp))
        m["note"] = str(text or "")[:500]
        with open(mp + ".tmp", "w") as f:
            json.dump(m, f, ensure_ascii=False, indent=1)
        os.replace(mp + ".tmp", mp)
        return None

    def status(self):
        st = self.src.snap()
        r = self.rec
        pr = st.get("print")
        free = shutil.disk_usage(self.vdir).free / 1e9
        return {"v": VERSION, "o": self.o, "rec": r.status() if r else None, "last": self.last, "auto": self.auto_state,
                "src": {"rec": st.get("rec"), "rec_err": st.get("rec_err"), "rec_state": st.get("rec_state"),
                        "panel_ok": bool(st.get("panel")) and time.time() - st.get("panel_t", 0) < 2, "panel_err": st.get("panel_err"),
                        "print": None if not pr else {"state": (pr.get("run") or {}).get("state"), "v": pr.get("v")}, "print_err": st.get("print_err")},
                "free_gb": round(free, 1), "ffmpeg": bool(self.ffmpeg), "repaired": self.repaired, "now": time.time()}


# ── HTTP ─────────────────────────────────────────────────────────────────────────────
class H(BaseHTTPRequestHandler):
    app = None
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json; charset=utf-8", extra=None):
        if isinstance(body, (dict, list)):
            body = json.dumps(body, ensure_ascii=False).encode()
        elif isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _host_ok(self):
        h = (self.headers.get("Host") or "").split(":")[0].strip("[]").lower()
        return h in ("localhost", "127.0.0.1", "::1")

    def do_GET(self):
        A = self.app
        u = urlparse(self.path)
        q = parse_qs(u.query)
        p = u.path
        try:
            if p == "/":
                return self._send(200, PAGE.replace("__V__", VERSION), "text/html; charset=utf-8")
            if p == "/play":
                return self._send(200, PLAY.replace("__V__", VERSION), "text/html; charset=utf-8")
            if p == "/api/status":
                return self._send(200, A.status())
            if p == "/api/recs":
                return self._send(200, A.recs())
            m = re.match(r"^/api/live/(third|wrist)\.jpg$", p)
            if m:
                c = m.group(1)
                r = A.rec
                if r is not None and c in r.cs and r.cs[c]["last"] is not None:
                    return self._send(200, r.cs[c]["last"], "image/jpeg")
                b, _ = A.src.frame(c, timeout=1.0)
                return self._send(200, b, "image/jpeg")
            if p == "/api/info":
                j = A.info(q.get("rec", [""])[0])
                return self._send(200 if j else 404, j or {"err": "녹화 없음"})
            if p == "/api/frame":
                b = A.frame_of(q.get("rec", [""])[0], q.get("cam", [""])[0], int(q.get("i", ["0"])[0]))
                return self._send(200, b, "image/jpeg", {"Cache-Control": "max-age=3600"}) if b else self._send(404, {"err": "장면 없음"})
            m = re.match(r"^/file/([0-9A-Za-z가-힣_\-%]+)/([0-9A-Za-z_\-]+\.(avi|mp4|csv|json))$", p)
            if m:
                from urllib.parse import unquote
                nm = unquote(m.group(1))
                d = A.rec_dir(nm)
                fp = os.path.join(d, m.group(2)) if d else None
                if not fp or not os.path.isfile(fp):
                    return self._send(404, {"err": "파일 없음"})
                ct = {"avi": "video/x-msvideo", "mp4": "video/mp4", "csv": "text/csv; charset=utf-8", "json": "application/json; charset=utf-8"}[m.group(3)]
                return self._file(fp, ct, f"{nm}_{m.group(2)}")
            return self._send(404, {"err": "없음"})
        except (BrokenPipeError, ConnectionResetError):
            return
        except Exception as e:
            try:
                return self._send(500, {"err": repr(e)[:200]})
            except Exception:
                return

    def _file(self, fp, ct, dl):
        size = os.path.getsize(fp)
        rng = self.headers.get("Range")
        a, b = 0, size - 1
        code = 200
        if rng and rng.startswith("bytes="):
            s, _, e = rng[6:].split(",")[0].partition("-")
            if s:
                a = int(s)
                b = int(e) if e else size - 1
            else:
                a = max(0, size - int(e))
            b = min(b, size - 1)
            code = 206
        self.send_response(code)
        self.send_header("Content-Type", ct)
        self.send_header("Content-Length", str(b - a + 1))
        self.send_header("Accept-Ranges", "bytes")
        if code == 206:
            self.send_header("Content-Range", f"bytes {a}-{b}/{size}")
        if ct.startswith("video/x-msvideo") or ct.startswith("text/csv"):
            self.send_header("Content-Disposition", f"attachment; filename*=UTF-8''{quote(dl)}")
        self.end_headers()
        with open(fp, "rb") as f:
            f.seek(a)
            left = b - a + 1
            while left > 0:
                ch = f.read(min(1 << 20, left))
                if not ch:
                    break
                self.wfile.write(ch)
                left -= len(ch)

    def do_POST(self):
        A = self.app
        if not self._host_ok() or "application/json" not in (self.headers.get("Content-Type") or ""):
            self.close_connection = True
            return self._send(403, {"err": "거절"}, extra={"Connection": "close"})
        try:
            n = int(self.headers.get("Content-Length") or 0)
            b = json.loads(self.rfile.read(n) or b"{}")
            p = urlparse(self.path).path
            if p == "/api/start":
                if "o" in b:
                    A.o = A.clean(dict(A.o, **b["o"]))
                    A.save()
                err = A.start("버튼")
                return self._send(200, {"err": err})
            if p == "/api/stop":
                return self._send(200, {"err": A.stop("버튼")})
            if p == "/api/opts":
                if A.rec is not None and any(k in b for k in ("fps", "cams", "tcp")) and any(b.get(k) != A.o.get(k) for k in ("fps", "cams", "tcp") if k in b):
                    return self._send(200, {"err": "녹화 중에는 카메라 · 장면 수 · 자세 기록을 바꾸지 않는다", "o": A.o})
                A.o = A.clean(dict(A.o, **b))
                A.save()
                return self._send(200, {"err": None, "o": A.o})
            if p == "/api/convert":
                return self._send(200, {"err": A.convert(b.get("rec", ""))})
            if p == "/api/note":
                return self._send(200, {"err": A.note(b.get("rec", ""), b.get("note", ""))})
            return self._send(404, {"err": "없음"})
        except Exception as e:
            return self._send(500, {"err": repr(e)[:200]})


PAGE = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>fr5cam 녹화</title><style>
:root{--bg:#f4f3ef;--card:#fff;--ink:#1d1d1b;--mute:#6b6a64;--line:#dddbd3;--soft:#eceae4;--b:#0b6bcb;--rec:#c8102e;--ok:#1a7f4b;--warn:#b26a00}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#1b1b1a;--card:#262624;--ink:#ecebe6;--mute:#a3a29b;--line:#3a3a37;--soft:#30302d;--b:#5aa2f0;--rec:#ff5a6e;--ok:#4cc38a;--warn:#e0a040}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.5 system-ui,-apple-system,"Apple SD Gothic Neo","Noto Sans KR",sans-serif}
header{display:flex;gap:10px;align-items:center;flex-wrap:wrap;padding:10px 16px;border-bottom:1px solid var(--line);background:var(--card);position:sticky;top:0;z-index:2}
header b{font-size:17px}.chip{padding:2px 9px;border-radius:10px;border:1px solid var(--line);font-size:12px;color:var(--mute);white-space:nowrap}
.chip.ok{color:var(--ok);border-color:var(--ok)}.chip.bad{color:var(--rec);border-color:var(--rec)}.chip.warn{color:var(--warn);border-color:var(--warn)}
main{max-width:1200px;margin:0 auto;padding:14px 16px;display:grid;gap:14px;grid-template-columns:minmax(0,1fr)}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px}.card h3{margin:0 0 10px;font-size:15px}
.cams{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,300px),1fr));gap:10px}.cams figure{margin:0}.cams img{width:100%;aspect-ratio:848/480;object-fit:contain;background:var(--soft);border-radius:8px;display:block}
.cams figcaption{font-size:12px;color:var(--mute);margin-top:3px}
.row{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin:8px 0}.mute{color:var(--mute);font-size:12px}
button{font:inherit;padding:7px 14px;border-radius:8px;border:1px solid var(--line);background:var(--soft);color:var(--ink);cursor:pointer}button:disabled{opacity:.45;cursor:default}
button.big{font-size:17px;padding:12px 22px;font-weight:600}#bStart{background:var(--rec);border-color:var(--rec);color:#fff}#bStop{background:var(--ink);border-color:var(--ink);color:var(--card)}
.chips{display:flex;gap:5px;flex-wrap:wrap}.chips button{padding:5px 12px;border-radius:14px;font-size:13px}.chips button.on{background:var(--ink);color:var(--card);border-color:var(--ink)}
input[type=number],input[type=text]{font:inherit;padding:5px 8px;border:1px solid var(--line);border-radius:7px;background:var(--card);color:var(--ink);width:90px}
input[type=text]{width:200px;max-width:100%}
.recbar{display:flex;gap:14px;align-items:center;flex-wrap:wrap;font-variant-numeric:tabular-nums}.dot{width:12px;height:12px;border-radius:50%;background:var(--rec);display:inline-block;animation:bl 1s infinite}
@keyframes bl{50%{opacity:.2}}.big-t{font-size:22px;font-weight:600}
table{border-collapse:collapse;width:100%;font-size:13px}td,th{border-top:1px solid var(--line);padding:6px 6px;text-align:left;vertical-align:top}th{color:var(--mute);font-weight:500}
.tw{overflow-x:auto}a{color:var(--b)}.sm{padding:3px 9px;font-size:12px}
</style></head><body>
<header><b>fr5cam</b><span class="chip">v__V__ · 녹화 전용</span><span class="chip" id="cRec">fr5rec</span><span class="chip" id="cPanel">제어판</span><span class="chip" id="cPrint">fr5print</span><span class="chip" id="cDisk">디스크</span></header>
<main>
<section class="card"><h3>카메라 (fr5rec 에서 읽기만)</h3>
<div class="cams"><figure><img id="lv_third" alt="3인칭 카메라"><figcaption id="lc_third">3인칭</figcaption></figure><figure><img id="lv_wrist" alt="손목 카메라"><figcaption id="lc_wrist">손목</figcaption></figure></div>
<div class="mute" style="margin-top:6px">화면은 초당 2장으로만 보여 준다. 녹화는 아래 장면 수대로 따로 받는다.</div></section>
<section class="card"><h3>녹화</h3>
<div class="row"><span>카메라</span><span class="chips" id="camChips"><button data-cam="third">3인칭</button><button data-cam="wrist">손목</button></span>
<span>초당 장면</span><span class="chips" id="fpsChips"><button data-fps="5">5</button><button data-fps="10">10</button><button data-fps="15">15</button></span></div>
<div class="row"><label><input type="checkbox" id="oTcp"> 장면마다 로봇 자세 기록 (제어판 읽기)</label><span>이름</span><input type="text" id="oLabel" placeholder="예: mandala_pen05" maxlength="40"></div>
<div class="row"><button class="big" id="bStart">● 녹화 시작</button><button class="big" id="bStop" disabled>■ 녹화 끝</button><span id="msg" class="mute"></span></div>
<div class="recbar" id="recbar" style="display:none"><span class="dot"></span><span class="big-t" id="rT">0:00</span><span id="rInfo"></span></div>
<div class="row" style="margin-top:12px"><label><input type="checkbox" id="oAuto"> fr5print 가 작업을 돌릴 때 자동 녹화</label><span class="mute">끝난 뒤</span><input type="number" id="oTail" min="0" max="60" step="1"><span class="mute">s 더 · 멈춤(펜 바꾸기)은</span><input type="number" id="oPause" min="0" max="3600" step="10"><span class="mute">s 기다림</span></div>
<div class="row"><span class="mute">최대 길이</span><input type="number" id="oMax" min="0.5" max="240" step="1"><span class="mute">분 · 디스크 여유가</span><input type="number" id="oFree" min="0.5" max="500" step="0.5"><span class="mute">GB 아래면 멈춤</span></div>
<div class="mute" id="autoMsg"></div></section>
<section class="card"><h3>녹화한 것</h3><div class="mute" style="margin-bottom:6px">폴더 ~/fr5cap/camrec/videos/ · 지우기 버튼은 없다(지울 때는 폴더에서 직접). AVI 는 VLC 로 열린다.</div>
<div class="tw"><table><thead><tr><th>이름</th><th>길이</th><th>크기</th><th>끝난 까닭</th><th>받기</th></tr></thead><tbody id="recs"></tbody></table></div></section>
</main>
<script>
const $=id=>document.getElementById(id);let S=null,O=null,busy=false;
async function api(p,b){const r=await fetch(p,b===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)});return r.json()}
const fmt=s=>{s=Math.max(0,Math.round(s));const h=Math.floor(s/3600),m=Math.floor(s%3600/60),x=s%60;return (h?h+':'+String(m).padStart(2,'0'):m)+':'+String(x).padStart(2,'0')};
const mb=b=>b>=1e9?(b/1e9).toFixed(2)+' GB':(b/1e6).toFixed(1)+' MB';
const esc=s=>String(s??'').replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
function setO(o){O=o;document.querySelectorAll('[data-cam]').forEach(b=>b.classList.toggle('on',o.cams.includes(b.dataset.cam)));document.querySelectorAll('[data-fps]').forEach(b=>b.classList.toggle('on',+b.dataset.fps===o.fps));
 if(document.activeElement!==$('oLabel'))$('oLabel').value=o.label||'';$('oTcp').checked=o.tcp;$('oAuto').checked=o.auto;
 for(const [id,k] of [['oTail','tail_s'],['oPause','pause_wait_s'],['oMax','max_min'],['oFree','min_free_gb']])if(document.activeElement!==$(id))$(id).value=o[k]}
async function setOpt(p){const r=await api('/api/opts',p);if(r.err)$('msg').textContent=r.err;setO(r.o)}
document.querySelectorAll('[data-cam]').forEach(b=>b.onclick=()=>{const c=b.dataset.cam;let L=O.cams.includes(c)?O.cams.filter(x=>x!==c):O.cams.concat([c]);if(!L.length)return;setOpt({cams:L})});
document.querySelectorAll('[data-fps]').forEach(b=>b.onclick=()=>setOpt({fps:+b.dataset.fps}));
$('oTcp').onchange=()=>setOpt({tcp:$('oTcp').checked});$('oAuto').onchange=()=>setOpt({auto:$('oAuto').checked});
$('oLabel').onchange=()=>setOpt({label:$('oLabel').value});
for(const [id,k] of [['oTail','tail_s'],['oPause','pause_wait_s'],['oMax','max_min'],['oFree','min_free_gb']])$(id).onchange=()=>setOpt({[k]:+$(id).value});
$('bStart').onclick=async()=>{if(busy)return;busy=true;$('bStart').disabled=true;$('msg').textContent='시작하는 중…';const r=await api('/api/start',{o:{label:$('oLabel').value}});busy=false;$('msg').textContent=r.err||'';tick()};
$('bStop').onclick=async()=>{if(busy)return;busy=true;$('bStop').disabled=true;$('msg').textContent='닫는 중…';const r=await api('/api/stop',{});busy=false;$('msg').textContent=r.err||'';tick();loadRecs()};
function chip(id,cls,t){const e=$(id);e.className='chip '+cls;e.textContent=t}
let wasRec=false;
async function tick(){try{S=await api('/api/status')}catch(e){chip('cRec','bad','fr5cam 응답 없음');return}
 setO(S.o);const sr=S.src,now=S.now;
 const ages=['third','wrist'].map(c=>{const x=(sr.rec||{})[c];return x&&now-x.t<5?`${c==='third'?'3인칭':'손목'} ${x.age==null?'':Math.round(x.age)+' ms'}`:null}).filter(Boolean);
 chip('cRec',sr.rec_state?(sr.rec_state==='recording'?'warn':'ok'):'bad',sr.rec_state?`fr5rec ${sr.rec_state==='recording'?'데이터셋 기록 중':sr.rec_state}`:'fr5rec 없음');
 chip('cPanel',sr.panel_ok?'ok':'',sr.panel_ok?'제어판 읽음':'제어판 (녹화 때만 읽음)');
 chip('cPrint',sr.print?(sr.print.state==='running'?'warn':'ok'):'',sr.print?`fr5print ${sr.print.state||''}`:'fr5print (자동 녹화 때만 읽음)');
 chip('cDisk',S.free_gb<O.min_free_gb?'bad':'',`여유 ${S.free_gb} GB`);
 const R=S.rec;$('bStart').disabled=!!R||busy;$('bStop').disabled=!R||busy;$('recbar').style.display=R?'flex':'none';
 if(R){$('rT').textContent=fmt(R.seconds);$('rInfo').textContent=`${R.frames} 장면 · ${mb(R.bytes)} · ${R.fps} 장/s · `+Object.entries(R.cams).map(([c,x])=>`${c==='third'?'3인칭':'손목'} ${x.w||'?'}×${x.h||'?'} 놓침 ${x.miss} · 같은 장면 ${x.dup}`).join(' · ')+(R.late?` · 늦은 박자 ${R.late}`:'')+` · ${R.why}`}
 else if(S.last&&wasRec){$('msg').textContent=`녹화 끝: ${S.last.name} (${fmt(S.last.seconds)}) — ${S.last.end_reason||''}`;loadRecs()}
 wasRec=!!R;$('autoMsg').textContent=S.o.auto?('자동: '+(S.auto.msg||'')):'';
 if(S.repaired&&S.repaired.length&&!tick.rp){tick.rp=1;$('msg').textContent='지난번 꺼진 녹화를 고침: '+S.repaired.map(x=>x.name).join(', ')}}
async function loadRecs(){const L=await api('/api/recs');$('recs').innerHTML=L.length?L.map(r=>{const f=r.files,u=fn=>`/file/${encodeURIComponent(r.name)}/${fn}`;
 const links=f.filter(fn=>/\.(avi|mp4|csv)$/.test(fn)).map(fn=>`<a href="${u(fn)}" ${fn.endsWith('.mp4')?'target="_blank"':''}>${fn}</a>`).join(' · ');
 const cv=r.conv?`<div class="mute">MP4: ${esc(r.conv.msg)}</div>`:'';
 return `<tr><td><b>${esc(r.label||r.name)}</b><div class="mute">${esc(r.start||'')} · ${esc((r.cams||[]).map(c=>c==='third'?'3인칭':'손목').join('+'))} · ${r.fps||''} 장/s${r.runs?` · fr5print 기록 ${r.runs}`:''}</div>`+
  `<div class="row" style="margin:4px 0">${r.state==='recording'?'<span class="mute">녹화 중</span>':`<a href="/play?rec=${encodeURIComponent(r.name)}" target="_blank"><button class="sm">▶ 보기</button></a>`}<button class="sm" onclick="conv('${esc(r.name)}')" ${S&&S.ffmpeg?'':'disabled title="ffmpeg 없음"'}>MP4 만들기</button><button class="sm" onclick="note('${esc(r.name)}')">메모</button></div>${r.note?`<div class="mute">메모: ${esc(r.note)}</div>`:''}${cv}</td>`+
  `<td>${r.seconds!=null?fmt(r.seconds):'—'}</td><td>${mb(r.bytes)}</td><td class="mute">${esc(r.end_reason||r.state||'')}</td><td>${links}</td></tr>`}).join(''):'<tr><td class="mute" colspan="5">아직 없다</td></tr>'}
async function conv(nm){const r=await api('/api/convert',{rec:nm});$('msg').textContent=r.err||'MP4 만드는 중 — 끝나면 목록에 나온다';loadRecs()}
async function note(nm){const t=prompt('메모 (500자)');if(t===null)return;await api('/api/note',{rec:nm,note:t});loadRecs()}
function live(){if(document.hidden)return;for(const c of ['third','wrist']){const im=$('lv_'+c);if(im.dataset.busy==='1')continue;im.dataset.busy='1';const n=new Image();n.onload=()=>{im.src=n.src;im.dataset.busy='0';$('lc_'+c).textContent=c==='third'?'3인칭':'손목'};n.onerror=()=>{im.dataset.busy='0';$('lc_'+c).textContent=(c==='third'?'3인칭':'손목')+' — 못 받음'};n.src=`/api/live/${c}.jpg?t=${Date.now()}`}}
tick();loadRecs();setInterval(tick,700);setInterval(live,500);setInterval(()=>{if(!S||!S.rec)loadRecs()},5000);
</script></body></html>"""

PLAY = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>fr5cam 보기</title><style>
:root{--bg:#f4f3ef;--card:#fff;--ink:#1d1d1b;--mute:#6b6a64;--line:#dddbd3;--soft:#eceae4;--b:#0b6bcb}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#1b1b1a;--card:#262624;--ink:#ecebe6;--mute:#a3a29b;--line:#3a3a37;--soft:#30302d;--b:#5aa2f0}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.5 system-ui,-apple-system,"Apple SD Gothic Neo","Noto Sans KR",sans-serif}
main{max-width:1500px;margin:0 auto;padding:14px 16px}.cams{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,320px),1fr));gap:10px}
.cams img{width:100%;aspect-ratio:848/480;object-fit:contain;background:var(--soft);border-radius:8px;display:block}
.row{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin:10px 0;font-variant-numeric:tabular-nums}.mute{color:var(--mute);font-size:12px}
button{font:inherit;padding:6px 12px;border-radius:8px;border:1px solid var(--line);background:var(--soft);color:var(--ink);cursor:pointer}button.on{background:var(--ink);color:var(--card)}
input[type=range]{flex:1;min-width:200px}
</style></head><body><main><h3 id="ttl" style="margin:0 0 8px">fr5cam 보기</h3>
<div class="cams" id="cams"></div>
<div class="row"><button id="bp">▶</button><button id="bb" title="한 장 뒤">◀ 1</button><button id="bf" title="한 장 앞">1 ▶</button><input type="range" id="sl" min="0" value="0"><span id="tt">0:00</span></div>
<div class="row"><span class="mute">빠르기</span><span id="sp"></span><span class="mute" id="ln"></span></div>
<div class="mute" id="meta"></div></main>
<script>
const $=id=>document.getElementById(id),Q=new URLSearchParams(location.search),REC=Q.get('rec');let I=null,N=0,i=0,play=false,spd=1,cams=[],pend=0;
const fmt=s=>{s=Math.max(0,s);const m=Math.floor(s/60),x=(s%60).toFixed(1);return m+':'+x.padStart(4,'0')};
async function load(){I=await (await fetch('/api/info?rec='+encodeURIComponent(REC))).json();if(I.err){$('ttl').textContent=I.err;return}
 const m=I.meta;cams=m.cams||[];N=Math.min(...cams.map(c=>I.n[c]||0));$('ttl').textContent=(m.label||m.name)+' · '+(m.start||'');$('sl').max=Math.max(0,N-1);
 $('cams').innerHTML=cams.map(c=>`<figure style="margin:0"><img id="im_${c}" alt="${c}"><figcaption class="mute">${c==='third'?'3인칭':'손목'}</figcaption></figure>`).join('');
 $('meta').textContent=`${N} 장면 · ${m.fps} 장/s · ${m.seconds??'?'} s · 끝난 까닭: ${m.end_reason||m.state}`+(m.note?` · 메모: ${m.note}`:'');
 $('sp').innerHTML=[0.25,0.5,1,2,4].map(v=>`<button data-s="${v}" class="${v===1?'on':''}">${v}×</button>`).join(' ');
 document.querySelectorAll('[data-s]').forEach(b=>b.onclick=()=>{spd=+b.dataset.s;document.querySelectorAll('[data-s]').forEach(x=>x.classList.toggle('on',x===b))});show(0)}
function show(k){i=Math.max(0,Math.min(N-1,k));$('sl').value=i;const m=I.meta;$('tt').textContent=fmt(i/m.fps)+' / '+fmt(N/m.fps);pend=cams.length;
 const t0=performance.now();cams.forEach(c=>{const im=$('im_'+c),n=new Image();n.onload=n.onerror=()=>{im.src=n.src;if(--pend===0)next(t0)};n.src=`/api/frame?rec=${encodeURIComponent(REC)}&cam=${c}&i=${i}`});
 const r=I.rows[i];$('ln').textContent=r?(r[1]!==''?`TCP ${r[1]}, ${r[2]}, ${r[3]} mm`:'')+(r[4]?` · fr5print ${r[4]} 동작 ${r[5]}`:''):''}
function next(t0){if(!play)return;if(i>=N-1){play=false;$('bp').textContent='▶';return}const step=spd>2?Math.round(spd/2):1,wait=Math.max(0,1000*step/(I.meta.fps*spd)-(performance.now()-t0));setTimeout(()=>{if(play)show(i+step)},wait)}
$('bp').onclick=()=>{play=!play;$('bp').textContent=play?'❚❚':'▶';if(play){if(i>=N-1)i=0;show(i)}};$('bb').onclick=()=>{play=false;$('bp').textContent='▶';show(i-1)};$('bf').onclick=()=>{play=false;$('bp').textContent='▶';show(i+1)};
$('sl').oninput=()=>{play=false;$('bp').textContent='▶';show(+$('sl').value)};
document.addEventListener('keydown',e=>{if(e.key===' '){e.preventDefault();$('bp').click()}else if(e.key==='ArrowLeft')$('bb').click();else if(e.key==='ArrowRight')$('bf').click()});
load();
</script></body></html>"""


def main():
    ap = argparse.ArgumentParser(description="fr5cam — FR5 작업 녹화 (fr5rec 프레임 읽기만)")
    ap.add_argument("--port", type=int, default=8798)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--dir", default="~/fr5cap/camrec")
    ap.add_argument("--rec", default="http://127.0.0.1:8780", help="fr5rec 주소")
    ap.add_argument("--panel", default="http://127.0.0.1:8088", help="제어판 주소")
    ap.add_argument("--print-url", default="http://127.0.0.1:8796", help="fr5print 주소")
    a = ap.parse_args()
    srv = ThreadingHTTPServer((a.host, a.port), H)      # 먼저 자리를 잡는다 — 이미 켜져 있으면 여기서 멈춤 (고치기 전에)
    srv.daemon_threads = True
    app = App(a)
    H.app = app
    print(f"fr5cam {VERSION} · http://localhost:{a.port} · 폴더 {app.dir} · ffmpeg {'있음' if app.ffmpeg else '없음'} · 끝내기 Ctrl+C", flush=True)
    if app.repaired:
        print("지난번 꺼진 녹화를 고침:", ", ".join(x["name"] for x in app.repaired), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if app.rec is not None:
            print("녹화 닫는 중…", flush=True)
            app.stop("프로그램 끝")
        srv.server_close()


if __name__ == "__main__":
    main()
