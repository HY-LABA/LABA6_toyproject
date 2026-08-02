# prep — 구동 전 준비 단계

[`../catcher/`](../catcher/)가 실제 구동 코드라면, 여기는 **그걸 돌리기 위해 먼저 채워야 하는
값과 모델을 만드는 도구**다. 로봇을 움직이지 않는다.

| | 결과물 | 어디서 실행 |
|---|---|---|
| 카메라 캘리브레이션 | `f_px`, 주점, 왜곡계수 → `catcher/config.py` | 라파이 |
| 데이터 수집 | 라벨된 낙하 이미지 | 라파이 |
| YOLO 학습 | `best.pt` → (Hailo 변환) → `.hef` | PC |

---

## 실행 순서

```
0. python check_setup.py                                  # 환경·카메라 점검
1. python calibrate.py capture --camera csi                # 체커보드 촬영
   python calibrate.py solve   --camera csi                # -> config.py 값
2. python capture_dataset.py --label can --camera csi      # 낙하 이미지 수집 (반복)
3. python review_labels.py                                 # 자동 라벨 검수
   ── 여기서 PC로 dataset_raw/ 를 옮긴다 ──
4. python prepare_dataset.py                               # 세션 단위 분할 + yaml
5. python train_yolo.py --data dataset/dataset.yaml        # yolov8n 학습
6. python measure_sigma_w.py capture --label can --distance 2.0   # (라파이)
   python measure_sigma_w.py measure --weights .../best.pt        # (PC) 합격 판정
```

**6번이 진짜 합격선이다.** mAP가 아무리 좋아도 σ_w > 2 px면 거리 추정이 무너진다
([../docs/vision-pipeline.md 1장](../docs/vision-pipeline.md)).

---

## 파일

| 파일 | 하는 일 |
|---|---|
| `camera.py` | **카메라 추상화 — 교체 시 손대는 유일한 파일** |
| `check_setup.py` | 패키지·카메라·디스크 점검. 뭐가 없는지, 어느 단계에 필요한지 알려준다 |
| `calibrate.py` | 체커보드 촬영 + `cv2.calibrateCamera` / `cv2.fisheye.calibrate` |
| `capture_dataset.py` | MOG2 배경차분 → 자동 bbox → 자동 라벨 |
| `review_labels.py` | 자동 라벨 검수/수정/폐기 |
| `prepare_dataset.py` | **세션 단위** train/val/test 분할 + `dataset.yaml` |
| `train_yolo.py` | yolov8n 학습 |
| `measure_sigma_w.py` | σ_w 측정 (합격 기준 2 px) |

---

## 카메라를 바꾸면 뭐가 달라지나

**로직은 전부 동일하다.** `--camera` 프로파일만 바꾸면 된다. 다만 세 가지는 확인해야 한다.

### ① 캘리브레이션 모델 — 이게 가장 중요하다

| 카메라 | 렌즈 | 셔터 | 모델 |
|---|---|---|---|
| 기본 CSI (지금) | 고정 ~62° | 롤링 | `pinhole` |
| IMX296-GS + 2.8mm (나중) | 어안 140° | 글로벌 | **`fisheye`** |

화각 90°를 넘으면 핀홀 모델(`r = f·tanθ`)이 성립하지 않는다. 140° 렌즈를 pinhole로
캘리브레이션하면 가장자리에서 크게 어긋난다
([../docs/physics.md 7.2장](../docs/physics.md#72--이-렌즈는-핀홀이-아니다)).

`camera.py`의 `SPECS`에 프로파일별로 `calib_model`이 박혀 있어 자동으로 갈린다.

### ② 데이터셋은 재사용 불가

화각·왜곡·셔터가 다르면 같은 물체가 다른 크기·형태로 찍힌다. **GS 카메라가 오면 2~5번을
처음부터 다시 한다.**

> 그러면 지금 CSI로 하는 건 뭐냐 — **파이프라인과 절차를 검증하는 것**이다.
> MOG2 필터가 잘 도는지, 라벨 품질이 어떤지, 학습이 수렴하는지를 미리 확인해두면
> 실제 카메라가 왔을 때 데이터만 갈아끼우면 된다.

### ③ 롤링 셔터 데이터의 한계

CSI는 롤링 셔터라 빠르게 떨어지는 물체의 bbox가 기울고 늘어난다. 이 데이터로 학습한
모델은 GS 이미지에서 성능이 다르게 나온다. 재학습이 맞다.

---

## MOG2 자동 라벨링 — 맞는 방법인가

**맞다.** 조건이 잘 들어맞는다.

- 카메라가 **고정**이다 → 배경 모델이 성립한다
- 배경(천장)이 **정적**이다 → 움직이는 건 물체뿐
- 한 번에 **하나만** 떨어뜨린다 → 가장 큰 덩어리 = 물체
- 클래스를 **미리 안다**(`--label can`) → 라벨 자동 부여

다만 그냥 쓰면 안 되는 함정이 있어서 `capture_dataset.py`에 필터로 넣어뒀다.

| 함정 | 처리 |
|---|---|
| **던지는 손이 같이 잡힌다** | 유효 덩어리가 2개 이상인 프레임은 버린다 (`multi`) |
| 프레임 경계에 걸친 물체 | bbox 폭이 잘려 z가 틀어진다 — 버린다 (`edge`) |
| 그림자 | `detectShadows=False` |
| 형광등 깜빡임·노이즈 | 워밍업 + 면적 하한 + 모폴로지 |
| 길쭉한 노이즈 | 채움 비율(`min_fill`)·종횡비 필터 |
| 물체가 배경에 흡수됨 | 수집 중 `learningRate=0` 고정 |

**남는 한계 하나 — 모션 블러.** MOG2 마스크는 블러 꼬리를 포함해서 bbox가 실제보다
커진다. 다만 이건 **계통 오차**라 매번 같은 방향으로 생기고, `OBJECT_SIZE_M`을 그만큼
보정하면 흡수된다. 문서가 "일관성 > 절대 정확도"라고 한 게 이 얘기다
([../docs/vision-pipeline.md 4장](../docs/vision-pipeline.md)).
`measure_sigma_w.py`의 '편향' 열로 확인할 수 있다.

**그래서 자동 라벨은 초안이다.** `review_labels.py`로 반드시 훑을 것.

---

## 왜 yolov8n인가 (yolo11n이 아니라)

프로젝트 모델은 **yolov8n으로 확정**했다
([../docs/vision-pipeline.md 5장](../docs/vision-pipeline.md)).

- **Hailo Model Zoo 지원이 확실하다.** yolo11n은 지원 여부가 미확인 리스크로 남아 있었는데
  ([../docs/open-questions.md](../docs/open-questions.md)), yolov8n을 쓰면 그 리스크가 사라진다
- 정확도 차이는 이 용도에서 미미하다 — 클래스 3개에 물체도 단순하고,
  중요한 건 모델 크기가 아니라 **bbox 회귀 정밀도**이고 그건 데이터 품질이 좌우한다

`train_yolo.py --model yolo11n.pt` 로 언제든 바꿀 수 있다 (ultralytics가 동일하게 처리).

---

## 산출물을 어디에 넣나

캘리브레이션 결과는 `calib/<camera>/calibration.json`에 저장되고,
`calibrate.py solve`가 **붙여넣을 수 있는 형태로 출력**한다.

```python
# catcher/config.py
FOCAL_LENGTH_PX = ...
PRINCIPAL_POINT_PX = (..., ...)
DISTORTION_COEFFS = (...)
```

학습 가중치는 `runs/detect/catcher/weights/best.pt` →
ONNX → Hailo Dataflow Compiler → `.hef` → `catcher/config.py`의 `YOLO_MODEL_PATH`.
