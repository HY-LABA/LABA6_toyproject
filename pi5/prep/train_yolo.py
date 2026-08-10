"""YOLOv8n 학습. (PC에서 실행 — 라파이는 학습용이 아니다)

    python train_yolo.py --data dataset/dataset.yaml
    python train_yolo.py --data dataset/dataset.yaml --model yolo11n.pt   # 모델 교체

왜 yolov8n인가:
  · n(nano)이라 Hailo-8L에서 60 fps를 노려볼 수 있다 (프레임 예산 16.7 ms, 추론 10 ms 목표)
  · **Hailo Model Zoo 지원이 yolo11n보다 확실하다** — 컴파일 단계에서 막히지 않는다
  · 클래스가 하나(trash)고 물체도 단순해서 큰 모델이 필요 없다
  · 정확한 bbox 회귀가 목표인데, 그건 모델 크기보다 **데이터 품질**에 더 좌우된다

Colab에서 돌릴 때:
    !python train_yolo.py --data /content/dataset/dataset.yaml \
        --project /content/drive/MyDrive/yolo_runs
  · 데이터는 zip으로 Drive에 올려 /content(로컬 디스크)에 풀 것. Drive에 파일 수천
    개를 풀어놓고 마운트해서 읽으면 학습보다 파일 읽기가 느리다.
  · --project 를 Drive 안으로 주면 세션이 끊겨도 가중치가 남는다.
  · 실제로 끊겼으면 처음부터 다시 돌리지 말고 --resume 으로 이어간다:
        !python train_yolo.py --resume /content/drive/MyDrive/yolo_runs/catcher/weights/last.pt
    체크포인트에 저장된 data 경로를 그대로 다시 읽으므로, **데이터셋 zip은 먼저 같은
    위치에 다시 풀어놓아야 한다.**

⚠ 중요한 지표가 바뀌었다 (2026-08-10). 예전에는 bbox 폭 오차 σ_w가 합격 기준이었지만
   (`z = f·W_real/w_px` 로 거리를 뽑았으므로), 깊이 추정이 중력 기반 궤적 최소제곱
   (pi5/trajectory.py)으로 바뀌면서 **bbox 폭은 이제 안 쓴다.** 궤적 피팅의 유일한
   입력은 bbox **중심(u,v)** 이다. 그래서:
     · σ_w 합격선 2px → 폐기. 회전하는 물체에서 애초에 달성 불가능했다
     · 대신 **중심 좌표 정확도**가 중요하다 → 재투영 잔차(residual_px)로 확인
     · scale/degrees 증강을 좁게 잡던 이유도 약해졌지만, 중심 정확도에는 여전히
       도움이 되므로 유지한다

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
                    help="학습 입력 크기. 추론 때도 같은 값을 써야 한다 (pi5/vision.py)")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--patience", type=int, default=20)
    ap.add_argument("--device", default=None, help="'0' GPU, 'cpu'. 생략 시 자동")
    ap.add_argument("--project", default=None,
                    help="결과 저장 루트. Colab에서는 Drive 경로를 줄 것 "
                         "(예: /content/drive/MyDrive/yolo_runs) — 세션이 끊겨도 "
                         "가중치가 남는다. 생략하면 ultralytics 기본값 runs/detect.")
    ap.add_argument("--name", default="catcher")
    ap.add_argument("--resume", default=None, metavar="LAST_PT",
                    help="끊긴 학습을 이어서 돌린다. weights/last.pt 경로를 준다. 에폭·증강 "
                         "등 나머지 설정은 체크포인트에 저장된 값을 그대로 쓰므로 다른 "
                         "인자는 무시된다 (데이터셋은 저장된 경로에 다시 있어야 한다)")
    ap.add_argument("--export-onnx", action="store_true", help="학습 후 ONNX 내보내기")
    args = ap.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print("ultralytics 가 없다:  pip install ultralytics")
        return 1

    if args.resume:
        ckpt = pathlib.Path(args.resume)
        if not ckpt.exists():
            print(f"{ckpt} 가 없다. Drive에 남은 weights/last.pt 경로를 확인하라.")
            return 1
        print(f"이어하기 {ckpt}\n")
        model = YOLO(str(ckpt))
        # resume=True 면 ultralytics가 체크포인트에 저장된 인자(data, epochs, 증강 …)를
        # 전부 복원한다. 여기서 다시 넘기면 복원값과 충돌하므로 아무것도 주지 않는다.
        model.train(resume=True)
    else:
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
            project=args.project,
            name=args.name,
            # ── 증강 조정 ────────────────────────────────────────────────
            # 물체가 대칭이고 방향 의미가 없으므로 반전은 안전하다.
            fliplr=0.5,
            flipud=0.5,
            # 크기 증강을 넓게 주면 모델이 크기를 무시하도록 배우고, bbox 중심 회귀도
            # 같이 흐려진다. 실제 z 범위(0.3~2.5m)에 해당하는 만큼만 둔다.
            scale=0.3,
            # 회전 증강은 끈다. 회전시키면 축 정렬 bbox의 크기·중심이 물체와 무관하게
            # 변해서 라벨에 노이즈를 주입하는 셈이 된다. 공중에서 병이 회전하는 것은
            # 증강이 아니라 **실제 수집 데이터**로 담아야 한다.
            degrees=0.0,
            # 실제 추론에는 모자이크가 없다. 마지막 10 에폭은 실제 분포로 마무리한다.
            close_mosaic=10,
            # 색은 기본값 유지 (천장 조명 변화에 강해진다)
        )

    metrics = model.val()
    print("\n=== 검증 ===")
    try:
        print(f"  mAP50    = {metrics.box.map50:.3f}")
        print(f"  mAP50-95 = {metrics.box.map:.3f}")
        print(f"  recall   = {metrics.box.mr:.3f}   (목표 0.95 이상)")
    except Exception:  # noqa: BLE001
        print("  (지표 파싱 실패 — ultralytics 버전 차이. 출력 로그를 직접 확인)")

    # 저장 위치는 --name 으로 조립하면 안 된다. ultralytics 는 같은 이름이 이미 있으면
    # catcher2, catcher3 … 로 자동 증가시키므로, 재학습 때 조립한 경로는 **직전 실행의
    # 가중치**를 가리키게 된다 (그리고 아래 export 가 조용히 건너뛰어진다).
    # 실제로 쓴 디렉토리는 trainer 가 알고 있으니 그걸 받아온다.
    best = pathlib.Path(model.trainer.save_dir) / "weights" / "best.pt"
    print(f"\n가중치: {best}")
    if not best.exists():
        print("⚠ best.pt 를 찾지 못했다 — 학습이 중간에 끊겼는지 확인하라.")

    if args.export_onnx and best.exists():
        # NMS는 모델에 넣지 않는다 — Hailo는 NMS를 호스트(파이5)에서 돌리는 걸
        # 전제하고, 모델에 박혀 있으면 컴파일이 막힌다. ultralytics 기본값이
        # NMS 미포함이라 그대로 두면 된다.
        YOLO(str(best)).export(format="onnx", imgsz=args.imgsz, opset=12)
        print("ONNX 내보내기 완료 — 다음은 Hailo Dataflow Compiler로 .hef 변환")

    print("\n⚠ mAP가 좋아도 끝난 게 아니다. 이제 확인할 것:")
    print("   ① 실제로 던져서 궤적 피팅의 재투영 잔차(residual_px)를 볼 것")
    print("      2px 근처면 좋고, 4px 이상이면 config.MIN_TIME_SPAN_S를 늘려야 한다")
    print("   ② 예측 착지점과 실제 착지점의 거리 — 이게 최종 성능이다")
    print("   (σ_w 합격선 2px는 폐기됐다. bbox 폭을 더 이상 쓰지 않는다)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
