# 08. HTTP API

모두 `127.0.0.1` 에만 열린다. 응답은 JSON, 실패는 `{"err": "<까닭>"}`.

## fr5print (`:8796`)

| 방법 | 경로 | 몸통 | 하는 일 |
|---|---|---|---|
| GET | `/` · `/jog` | — | 화면 · 조그 화면 |
| GET | `/api/pstatus` | — | 버전 · 조그 상태 · 설정 `o` · 판 `bed` · 공구 끝(베이스 · 판) · 기록한 점 · 계획 id · 실행 `run`{state, op, k, n, remain, msg} · 속도 · 한계 `lim` |
| GET | `/api/jobs` | — | 저장한 작업 목록 |
| POST | `/api/opts` | 설정 키들 | 설정 바꾸기 (움직이는 중 거절) |
| POST | `/api/bed/mark` | `{i: 1–3}` | 지금 공구 끝을 판 점 i 로 기록 |
| POST | `/api/bed/solve` | `{mode: "three"}` · `{mode: "one", w, h}` | 판 저장 |
| POST | `/api/expand` | `{job}` | 작업 → 동작 · 판 밖 검사 (로봇 안 움직임) |
| POST | `/api/plan` | `{job, start_op?, no_back?}` | 계획 · 검사 → `{id, sum, prev}` |
| POST | `/api/run` | `{id}` | 그 계획 실행 ⚠ |
| POST | `/api/pause` | — | 획 끝에서 멈춤 |
| POST | `/api/speed` | `{r: 0.1–1}` | 속도 비율 |
| POST | `/api/corners` | `{h}` | 모서리 확인 ⚠ |
| POST | `/api/job/save` · `/api/job/load` | `{name, job}` · `{name}` | 작업 저장 · 불러오기 |
| POST | `/api/import` | `{fmt, text, name}` | G-code · SVG · CSV → 작업 항목 |

조그 화면에서 이어받은 경로(`/api/status`, `/api/send`, `/api/stop`, `/api/cmd`{kind: jog · rot · vertical · grip · goto}, `/api/set`, `/api/save`, `/api/plan_goto`)도 같은 포트에 있다.

작업 `job` = `{name, items: [{type, x, y, z, rot, …}]}` — 항목 키는 [03_jobs](03_jobs.md).

## fr5cam (`:8798`)

POST 는 `Content-Type: application/json` 과 `Host: localhost | 127.0.0.1` 일 때만 받는다.

| 방법 | 경로 | 몸통 | 하는 일 |
|---|---|---|---|
| GET | `/` | — | 화면 |
| GET | `/play?rec=<이름>` | — | 돌려 보기 화면 |
| GET | `/api/status` | — | 설정 · 녹화 상태 · 자동 녹화 · 연결 상태 · 디스크 · ffmpeg 유무 |
| GET | `/api/recs` | — | 녹화 목록 (최근 200) |
| GET | `/api/live/<third\|wrist>.jpg` | — | 지금 장면 |
| GET | `/api/info?rec=<이름>` | — | meta · 장면 수 · CSV 행(t, x, y, z, fr5print 상태) |
| GET | `/api/frame?rec=&cam=&i=` | — | 녹화의 i 번째 장면 (JPEG) |
| GET | `/file/<이름>/<파일>` | — | avi · mp4 · csv · json 받기 (Range 지원) |
| POST | `/api/start` | `{o?: 설정}` | 녹화 시작 |
| POST | `/api/stop` | — | 녹화 끝 |
| POST | `/api/opts` | 설정 키들 | `fps` · `cams` · `tcp` · `auto` · `tail_s` · `pause_wait_s` · `max_min` · `min_free_gb` · `label` |
| POST | `/api/convert` | `{rec}` | MP4 만들기 |
| POST | `/api/note` | `{rec, note}` | 메모 |
