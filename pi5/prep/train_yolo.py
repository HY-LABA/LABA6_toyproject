"""YOLOv8n 학습. (PC에서 실행 — 라파이는 학습용이 아니다)

    python train_yolo.py --data dataset/dataset.yaml
    python train_yolo.py --data dataset/dataset.yaml --model yolo11n.pt   # 모델 교체

왜 yolov8n인가:
  · n(nano)이라 Hailo-8L에서 60 fps를 노려볼 수 있다 (프레임 예산 16.7 ms, 추론 10 ms 목표)
  · **Hailo Model Zoo 지원이 yolo11n보다 확실하다** — 컴파일 단계에서 막히지 않는다
  · 클래스 3개에 물체도 단순해서 큰 모델이 필요 없다
  · 정확한 bbox 회귀가 목표인데, 그건 모델 크기보다 **데이터 품질**에 더 좌우된다

⚠ 이 시스템에서 중요한 지표는 mAP가 아니라 **bbox 폭 오차 σ_w**다.
   `z = f·W_real/w_px` 이므로 폭 오차가 그대로 거리 오차가 된다.
   학습이 끝나면 반드시 measure_sigma_w.py 로 확인할 것 (합격선 2 px).
   (../docs/vision-pipeline.md 1장, 7장)

┌─ 카메라 교체 시 ──────────────────────────────────────────────────────────┐
│ 학습 코드는 그대로다. 데이터셋만 새로 만들어 --data 를 바꾸면 된다.        │
│ 다만 CSI로 학습한 가중치를 GS 카메라에 그대로 쓰면 성능이 떨어진다 —       │
│ 화각·왜곡·셔터가 달라 물체가 다르게 찍히기 때문이다. 재학습이 맞다.        │
│ (CSI 라운드는 파이프라인과 절차를 검증하는 용도다)                         │
└──────────────────────────────────────────────────────────────────────────┘
"""

from __future__ import annotations

import argparse
import pathlib


def main() -> int:
    ap = argparse.ArgumentParser(description="YOLOv8n 학습")
    ap.add_argument("--data", default="dataset/dataset.yaml")
    ap.add_argument("--model", default="yolov8n.pt", help="yolo11n.pt 로도 교체 가능")
    ap.add_argument("--imgsz", type=int, default=640,
                    help="ROI 크롭 크기와 반드시 일치시킬 것 (catcher/config.py ROI_SIZE_PX)")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--patience", type=int, default=20)
    ap.add_argument("--device", default=None, help="'0' GPU, 'cpu'. 생략 시 자동")
    ap.add_argument("--name", default="catcher")
    ap.add_argument("--export-onnx", action="store_true", help="학습 후 ONNX 내보내기")
    args = ap.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print("ultralytics 가 없다:  pip install ultralytics")
        return 1

    data = pathlib.Path(args.data)
    if not data.exists():
        print(f"{data} 가 없다. prepare_dataset.py 를 먼저 돌려라.")
        return 1

    print(f"모델 {args.model}  데이터 {data}  imgsz {args.imgsz}\n")
    model = YOLO(args.model)          # COCO 사전학습 가중치에서 시작

    model.train(
        data=str(data),
        imgsz=args.imgsz,
        epochs=args.epochs,
        batch=args.batch,
        patience=args.patience,
        device=args.device,
        name=args.name,
        # ── 증강 조정 ────────────────────────────────────────────────────
        # 물체가 대칭이고 방향 의미가 없으므로 반전은 안전하다.
        fliplr=0.5,
        flipud=0.5,
        # bbox 크기가 곧 신호다. scale을 너무 넓히면 회귀 정밀도가 떨어질 수 있어
        # 실제 z 범위(0.3~2.5m)에 해당하는 정도로만 둔다.
        scale=0.3,
        # 실제 추론에는 모자이크가 없다. 마지막 10 에폭은 실제 분포로 마무리한다.
        close_mosaic=10,
        # 색·회전은 기본값 유지 (천장 조명 변화에 강해진다)
    )

    metrics = model.val()
    print("\n=== 검증 ===")
    try:
        print(f"  mAP50    = {metrics.box.map50:.3f}")
        print(f"  mAP50-95 = {metrics.box.map:.3f}")
        print(f"  recall   = {metrics.box.mr:.3f}   (목표 0.95 이상)")
    except Exception:  # noqa: BLE001
        print("  (지표 파싱 실패 — ultralytics 버전 차이. 출력 로그를 직접 확인)")

    best = pathlib.Path("runs/detect") / args.name / "weights/best.pt"
    print(f"\n가중치: {best}")

    if args.export_onnx and best.exists():
        YOLO(str(best)).export(format="onnx", imgsz=args.imgsz, opset=12)
        print("ONNX 내보내기 완료 — 다음은 Hailo Dataflow Compiler로 .hef 변환")

    print("\n⚠ mAP가 좋아도 끝난 게 아니다. 다음을 반드시 실행하라:")
    print(f"   python measure_sigma_w.py --weights {best}")
    print("   합격선: σ_w ≤ 2 px  (이게 거리 정확도를 결정한다)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
