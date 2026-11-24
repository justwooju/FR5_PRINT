# 01. 설치 · 켜기

## 필요한 것

| 항목 | 내용 |
|---|---|
| 로봇 | FAIRINO FR5 + DH AG-160-95 그리퍼 |
| 제어판 | FR5 제어판 프로그램 — HTTP `:8088`(`/api/state` · `/api/leader/state`), UDP `:8097`(`{sid, seq, q[6], grip}`) → 안전 필터 → ServoJ |
| 파이썬 | python3 + numpy (fr5print). fr5cam 은 표준 라이브러리만 |
| 녹화 (선택) | 수집 프로그램 fr5rec `:8780` — `/api/frame/<third\|wrist>.jpg` 를 제공해야 한다 |
| MP4 (선택) | ffmpeg (libx264 없으면 mpeg4 로 만든다) |

## 파일 자리

fr5print 는 `~/fr5cap/jog` 에서 `fr5jog_ui` 를, fr5jog_ui 는 `~/fr5cap/pour` 에서 `pose_go` · `pour_run` 을 읽는다. 저장소의 `fr5cap/` 를 그대로 `~/fr5cap/` 에 놓는다.

```bash
cp -r fr5cap/* ~/fr5cap/
md5sum ~/fr5cap/print/fr5print.py ~/fr5cap/jog/fr5jog_ui.py ~/fr5cap/camrec/fr5cam.py
```

`fr5jog_ui.py` md5 가 `d60f8e6e` 가 아니면 fr5print 가 켤 때 경고를 낸다(시험한 판은 0.1.1).

## 켜기

```bash
python3 ~/fr5cap/print/fr5print.py            # http://localhost:8796 · 조그 화면 /jog
python3 ~/fr5cap/camrec/fr5cam.py             # http://localhost:8798
```

| fr5print 옵션 | 기본 | 뜻 |
|---|---|---|
| `--port` | 8796 | 화면 포트 |
| `--dir` | 파일이 있는 폴더 | `bed.json` · `settings.json` · `jobs/` · `runs.jsonl` 을 두는 곳 |
| `--poses` | `<dir>/poses.json` | 조그 자세 파일 |
| `--v` · `--v-go` | 10 · 30 | 조그 곧게 · 자세 이동 속도 mm/s |
| `--safe-z` | 380 | 조그 자세 이동 때 높이 |
| `--max-lag` | 5.0 | 명령보다 이만큼(°) 0.6 s 넘게 뒤처지면 멈춤 |
| `--udp-host` · `--udp-port` | 127.0.0.1 · 8097 | 제어판 명령 입력 |

| fr5cam 옵션 | 기본 | 뜻 |
|---|---|---|
| `--port` · `--host` | 8798 · 127.0.0.1 | 화면 |
| `--dir` | `~/fr5cap/camrec` | 녹화 · 설정 폴더 |
| `--rec` | `http://127.0.0.1:8780` | fr5rec |
| `--panel` | `http://127.0.0.1:8088` | 제어판 |
| `--print-url` | `http://127.0.0.1:8796` | fr5print |

두 프로그램 모두 `127.0.0.1` 에만 열린다. 다른 컴퓨터에서는 ssh 터널로 본다.

```bash
ssh -N -L 8796:127.0.0.1:8796 -L 8798:127.0.0.1:8798 <user>@<control-pc>
```

## 보내기 순서

1. 제어판 [조작 시작]
2. fr5print [보내기 시작]
3. 제어판 리더 화면에서 [리더 추종 시작]
4. 머리띠가 '준비' → 버튼이 켜진다

## 다른 프로그램과 같이 쓰기

- 제어판 UDP 로 보내는 프로그램은 한 번에 하나다. fr5print · fr5jog(`:8793`) · 수집 실행기를 동시에 보내지 않는다.
- `:8793` 이 보내는 중이면 fr5print 의 [보내기 시작]을 거절한다. fr5print 가 보내는 도중 `:8793` 이 보내기를 켜면 fr5print 가 보내기를 끈다.
- 리더 팔 송신기도 같은 UDP 포트를 쓴다. fr5print 를 쓸 때는 리더 송신기를 끈다.
- fr5print 를 Ctrl+C 로 끄면 보내기가 멈추고, 제어판이 0.3 s 뒤 그 자리에 선다.

## 쓰는 파일 (fr5print 폴더)

| 파일 | 내용 |
|---|---|
| `bed.json` | 판(원점 · 축 · 크기 · 맞춘 방식 · 공구 길이). 새로 맞추면 앞 판은 `bed.json.bak_<시각>` |
| `settings.json` | 설정값 ([05_planning](05_planning.md)) |
| `jobs/<이름>.json` | 저장한 작업 |
| `runs.jsonl` | 실행 기록 한 줄씩 ([06_run](06_run.md)) |

fr5cam 은 `~/fr5cap/camrec/settings.json` · `videos/` 만 쓴다.
