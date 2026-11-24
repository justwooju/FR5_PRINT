# 중심선 단위 시험

`center.js` · `rdp.js` 는 `fr5cap/print/fr5print.py` 0.2.0 안의 브라우저 코드와 같다. Node.js 로 돌린다.

| 파일 | 내용 |
|---|---|
| `t_center.js` | 합성 도형 14종 × 선 반굵기 2 · 4 · 7 px. 획 수 · 벗어남 · 빠짐을 잰다. `NOBIG=1` 큰 그림 시험 생략, `QUICK=1` 반굵기 4 만 |
| `t_fuzz.js` | 무작위 선 그림 (씨앗 고정). 멈춤 · NaN 이면 종료 코드 2 |

```bash
NOBIG=1 node t_center.js
for s in $(seq 1 150); do node t_fuzz.js $s > /dev/null || echo "seed $s"; done
```
