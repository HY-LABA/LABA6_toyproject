# 움직이는 쓰레기통 (자율주행 캐치 로봇)

낙하하는 쓰레기(투척 물체)를 카메라로 인식하고, 낙하 궤적을 예측해 로봇이 물체 아래로 이동하여 받아내는 자율주행 캐치 로봇.

## 문서 구성

### 구현 기준 문서

- [`architecture.md`](architecture.md) — 파일 구조, 파일별 책임, 파이5↔피코 역할 분담
- [`algorithm.md`](algorithm.md) — 궤적 추정 수식, 관측 스팬, 제어 루프, 검증된 성능
- [`system_flow.md`](system_flow.md) — pi5↔pico 파이프라인 흐름 + 구현 상태
- [`pi5/prep/TROUBLESHOOTING.md`](pi5/prep/TROUBLESHOOTING.md) — 실기 작업 기록 (겪은 문제와 해결)

### 설계 근거 문서 (`docs/`)

숫자가 왜 그 값인지를 담는다. 위 문서가 "무엇을 어떻게"라면, 아래는 "왜 그 값인가"다.

| 문서 | 내용 |
|---|---|
| [physics.md](docs/physics.md) | **물리 계산** — 운동모델·마찰·캐치반경·통 크기·항력·광학 |
| [hardware.md](docs/hardware.md) | **하드웨어** — 부품 구성, 통 설계, 전원, 장착 규약, 구매 체크리스트 |
| [pico-control.md](docs/pico-control.md) | **피코** — 실시간 루프, 기구학, 오도메트리, 모터 제어 |
| [protocol.md](docs/protocol.md) | **통신** — 파이5 ↔ 피코 프레임 계약 |
| [open-questions.md](docs/open-questions.md) | **미결정 사항과 리스크** |
| [vision-pipeline.md](docs/vision-pipeline.md) | 비전 — 데이터셋·라벨링·학습·Hailo 컴파일 ⚠ |
| [pi5-algorithm.md](docs/pi5-algorithm.md) | 파이5 알고리즘 ⚠ |

> ⚠ 표시된 두 문서는 **깊이 추정을 bbox 폭 기반으로 서술**하고 있어 현재 구현
> ([`algorithm.md`](algorithm.md)의 중력 기반 최소제곱)과 다르다. 정리 대상이다.
> 자세한 내용은 [`docs/open-questions.md`](docs/open-questions.md).

## 하드웨어 구성

### 컴퓨팅

| 부품 | 역할 |
|---|---|
| 라즈베리파이 5 | 메인 컴퓨터, 카메라 영상 처리 + 궤적 계산 |
| AI HAT+ 13TOPS (Hailo-8L) | 물체 인식 AI 가속기, PCIe로 파이5 연결 |
| 라즈베리파이 피코 | 모터 실시간 제어 전담, 파이와 USB 시리얼 통신 |

### 센서

- 글로벌 셔터 카메라 (Sony IMX296, 최대 60fps, 위쪽 촬영)
- 모터 내장 엔코더 3개 (홀센서, A/B 2상)

### 구동부

- 12V DC 기어드모터 3개
- BTS7960 모터드라이버 3개
- 100mm 옴니휠 3개 (NEXUS 14049, 고무 롤러), 120도 삼각 배치 (3륜 홀로노믹 드라이브)

### 전원 (2계통 분리)

- 두뇌용: 18650 UPS 5V → 파이5 / AI HAT / 카메라 / 피코
- 모터용: 3S LiPo 11.1V → PDB → 모터드라이버 3개

> **⚠ 부품 사양이 문서 간에 어긋나 있다. 실물 기준으로 확정할 것.**
> 항목별 근거와 확인 방법은 [`docs/open-questions.md`](docs/open-questions.md) 참고.
>
> | 항목 | 확인 필요한 이유 |
> |---|---|
> | **렌즈 초점거리** ★ | `pi5/config.py`는 6mm(대각 55°, 핀홀) 가정. [physics.md 7.2장](docs/physics.md)은 번들 2.8mm(대각 140°, 어안)로 계산. **깊이가 직접 걸린다** ([`pi5/TODO.md`](pi5/TODO.md) 1번 항목) |
> | **모터 모델** | FIT0493(350RPM/34:1) vs FIT0186(251RPM/43.8:1). 가감속 계산이 갈린다 |
> | **엔코더 전원** | 3.3V vs 5V. **5V면 피코에 직결하면 안 된다** ([hardware.md 3.2장](docs/hardware.md)) |
> | **카메라 장착 위치** | 통 하단 vs 통 입구 림. 착지 기준면(`z_catch`)과 오프셋이 달라진다 |

## 개발 언어

| 보드 | 언어 | 선정 이유 |
|---|---|---|
| 라즈베리파이 5 | Python | Hailo NPU 공식 SDK가 Python API 우선 지원. OpenCV/NumPy 등 영상·수치 처리 생태계 풍부. 무거운 연산은 NPU 하드웨어 가속에 의존하므로 Python 오버헤드가 병목이 되지 않음 |
| 라즈베리파이 피코 | C/C++ (Pico SDK) | PIO 기반 엔코더 카운팅의 타이밍 정밀도, PID 제어 루프의 지터(jitter) 최소화 — 실시간 제어 전담이라는 역할에 부합 |
