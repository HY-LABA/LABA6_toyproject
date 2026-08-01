# catcher — 파이5 캐치 파이프라인

낙하 물체를 인식해 궤적을 추정하고, 로봇이 갈 목표 속도를 만들어 피코로 보낸다.

설계 근거는 [`../docs/`](../docs/)에 있다. 이 문서는 **코드 지도**다.

> 기존 [`../pi5/`](../pi5/)는 구설계(2단계 확정→재보정) 코드다. 참고용으로 남겨두었고,
> 이 폴더가 새 아키텍처다.

---

## 큰 틀 — 4단계

```
  ①  객체 인식        YOLO11n        →  Detection(class, bbox, conf)
  ②  좌표 변환        핀홀 + 오도메트리 →  world (X, Y, Z)
  ③  궤적 추정        포물선 칼만필터   →  [X, Y, Z, VX, VY, VZ]
  ④  목표 지점        착지 예측        →  VelocityCommand(vx, vy, ttl)
```

**이 골격은 물리값 없이도 그대로 선다.** 물리는 각 단계 안의 파라미터와 보정으로 들어간다.

| 단계 | 물리가 정하는 것 | 문서 |
|---|---|---|
| ① 인식 | 추론 해상도 → σ_w, ROI 전략 | [vision-pipeline](../docs/vision-pipeline.md) |
| ② 좌표변환 | **어안 투영**, f_px, 주점, 카메라 오프셋 | [physics 7장](../docs/physics.md#7-광학--imx296-글로벌-셔터--28-mm-광각) |
| ③ 궤적추정 | g_eff(항력), 프로세스/측정 노이즈 | [physics 4장](../docs/physics.md#4-항력--포물선-모델이-양쪽-축에서-흔들린다) |
| ④ 목표 | V_MAX, A_MAX(견인한계), 도달가능성 | [physics 2·3장](../docs/physics.md#2-3륜-옴니의-힘-분배) |

---

## 파일 지도

```
catcher/
├── datatypes.py     공용 데이터 구조 (Detection, Pose, Landing, VelocityCommand, State)
├── config.py        모든 파라미터. 다른 파일에 상수 하드코딩 금지
│
│   ── 순수 함수 (하드웨어 없이 동작·테스트 가능) ──
├── frames.py     ②  픽셀 ↔ body ↔ world 좌표 변환
├── estimator.py  ③  포물선 칼만필터 (유일한 상태 보유자)
├── trajectory.py ④  착지점·착지시각 예측
├── control.py    ④  목표 속도 + 도달가능성 판정
├── protocol.py      프레임 인코딩/디코딩 (피코와의 계약)
│
│   ── 하드웨어 경계 (현재 스텁) ──
├── vision.py     ①  Picamera2 캡처 + Hailo YOLO 추론
├── link.py          pyserial 포트 + protocol.py 사용
│
├── main.py          SEARCH / TRACK 상태 기계
└── utils.py         로깅 + 타이밍
```

### 의존 방향

```
main ──▶ vision ──▶ frames ──▶ config
     ├─▶ estimator ─┤
     ├─▶ trajectory ┤
     ├─▶ control ───┘
     └─▶ link ──▶ protocol
```

**순환 의존이 없다.** `frames`, `trajectory`, `control`, `protocol`은 상태가 없는 순수
함수 모듈이라 단독으로 테스트할 수 있다. 상태를 가지는 것은 `estimator`(궤적),
`vision`(카메라 핸들), `link`(포트)뿐이다.

---

## 현재 구현 상태

| 파일 | 상태 |
|---|---|
| `datatypes.py` `config.py` | 완성 |
| `frames.py` | **동작** — 좌표 변환 전체 |
| `estimator.py` | **동작** — 6상태 칼만, 프레임별 R, ROI 중심 예측 |
| `trajectory.py` | **동작** — 이차방정식 착지 예측 |
| `control.py` | **동작** — 속도 명령, 도달가능성, 2구간 이동거리 |
| `protocol.py` | **동작** — 인코딩/디코딩 + 스트리밍 파서 |
| `vision.py` | 스텁 — `Camera` / `Detector` 인터페이스만. Picamera2·HailoRT 미연동 |
| `link.py` | 스텁 — pyserial 미연동 |
| `main.py` | **동작** — 상태 기계 (vision/link 구현체 주입 필요) |

`vision`과 `link`는 **의존성 주입**으로 갈아끼운다. 시뮬레이터나 테스트에서는 같은
인터페이스를 만족하는 가짜 객체를 넘기면 `main.run()`이 그대로 돈다.

```python
from catcher import main, vision, link, estimator
main.run(cam=MyFakeCamera(), detector=MyFakeDetector(), lnk=MyFakeLink())
```

---

## 다음 작업

1. `tests/` — 순수 함수 단위 테스트 (기구학 왕복, 좌표 왕복, 프로토콜 왕복)
2. `sim/` — 낙하·로봇 모델 + 가짜 카메라/링크 → 캐치 성공률 측정
3. `vision.py` 실연동 — Picamera2 + HailoRT
4. `link.py` 실연동 — pyserial

값을 채워야 하는 항목은 `config.py`의 `# 정해야함` 주석과
[pi5-algorithm.md 10장](../docs/pi5-algorithm.md#10-파라미터)에 정리되어 있다.
