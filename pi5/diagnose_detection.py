"""검출이 왜 안 되는지 좁히는 진단 도구.

    python diagnose_detection.py                # 100프레임 진단
    python diagnose_detection.py --frames 300
    python diagnose_detection.py --save         # 박스 그린 이미지도 저장

무엇을 보나
-----------
① **채널 순서(RGB vs BGR)** — 가장 유력한 용의자다.
   데이터 수집은 `cv2.imwrite(프레임)`로 JPEG를 저장했다. cv2는 배열을 BGR로
   해석해서 쓰므로, 파일에 담긴 RGB는 **배열의 역순**이다. 그리고 Ultralytics는
   학습할 때 JPEG를 읽어 RGB로 바꿔 모델에 넣는다.

       학습 시 모델 입력  =  reverse(카메라 배열)
       추론 시 모델 입력  =  카메라 배열          ← vision.py 는 안 뒤집는다

   이게 맞다면 **채널이 뒤바뀐 채로 추론하고 있다.** 검출이 아예 0이 되진 않고
   (형태는 남으니까) "가끔 잡히는" 상태가 되는데, 로그에서 본 게 정확히 그 모습이다.

   여기서는 같은 프레임을 **원본 / 채널반전** 두 가지로 추론해 직접 비교한다.
   반전 쪽 검출이 뚜렷이 많으면 확정이다.

② **시간 배분** — 캡처 / 전처리 / 추론이 각각 몇 ms인지.
   로그에서 실측 32fps가 나왔는데 카메라는 60fps 설정이다. 어디서 먹는지 본다.

③ **검출률** — 프레임당 몇 개나 잡히는지, conf 분포는 어떤지.
"""

from __future__ import annotations

import argparse
import statistics
import time

import numpy as np

import config


def main() -> int:
    ap = argparse.ArgumentParser(description="검출 진단 (채널 순서 · 속도 · 검출률)")
    ap.add_argument("--frames", type=int, default=100)
    ap.add_argument("--save", action="store_true", help="박스 그린 이미지 몇 장 저장")
    args = ap.parse_args()

    import vision

    cam = vision.Camera(config.CAMERA_RESOLUTION, config.CAMERA_FPS)
    yolo = vision._get_yolo()
    print(f"모델: {yolo.model_path}")
    print(f"모델 입력: {yolo._in_info.shape if hasattr(yolo._in_info,'shape') else '?'}"
          f"  / config.YOLO_IMGSZ = {config.YOLO_IMGSZ}")
    print(f"프레임 {args.frames}장 수집 중… (움직이는 물체를 몇 번 던져줄 것)\n")

    t_cap, t_pre, t_inf = [], [], []
    n_orig, n_swap = [], []
    c_orig, c_swap = [], []
    saved = 0

    for i in range(args.frames):
        a = time.perf_counter()
        frame, _t = cam.capture()
        b = time.perf_counter()
        yolo._letterbox(frame)                      # 전처리만 따로 재본다
        c = time.perf_counter()
        hits_o = yolo.infer(frame)
        d = time.perf_counter()
        hits_s = yolo.infer(frame[:, :, ::-1].copy())   # 채널 반전
        e = time.perf_counter()

        t_cap.append(b - a)
        t_pre.append(c - b)
        t_inf.append(d - c)
        n_orig.append(len(hits_o))
        n_swap.append(len(hits_s))
        c_orig += [cf for _bb, cf in hits_o]
        c_swap += [cf for _bb, cf in hits_s]

        if args.save and saved < 5 and (hits_o or hits_s):
            _save(frame, hits_o, hits_s, saved)
            saved += 1
        if i % 20 == 0:
            print(f"\r  {i}/{args.frames}  원본 {sum(n_orig)}개 / 반전 {sum(n_swap)}개",
                  end="", flush=True)

    cam.close()
    print("\r" + " " * 60)

    def ms(v):
        return f"{statistics.median(v) * 1000:6.1f}ms"

    print("── ② 시간 배분 (중앙값) ──")
    print(f"   캡처      {ms(t_cap)}")
    print(f"   전처리    {ms(t_pre)}")
    print(f"   추론      {ms(t_inf)}   ← activate()를 매 프레임 하고 있어 여기가 클 수 있다")
    tot = statistics.median([x + y + z for x, y, z in zip(t_cap, t_pre, t_inf)])
    print(f"   합계      {tot * 1000:6.1f}ms  →  {1 / tot:4.1f} fps 상한")

    print("\n── ③ 검출률 ──")
    f = len(n_orig)
    print(f"   원본 채널: 검출 {sum(n_orig):4d}개, 검출된 프레임 {sum(1 for x in n_orig if x):3d}/{f}"
          f"  ({sum(1 for x in n_orig if x) / f * 100:.0f}%)")
    print(f"   반전 채널: 검출 {sum(n_swap):4d}개, 검출된 프레임 {sum(1 for x in n_swap if x):3d}/{f}"
          f"  ({sum(1 for x in n_swap if x) / f * 100:.0f}%)")
    if c_orig:
        print(f"   원본 conf: 중앙 {statistics.median(c_orig):.3f}  최대 {max(c_orig):.3f}")
    if c_swap:
        print(f"   반전 conf: 중앙 {statistics.median(c_swap):.3f}  최대 {max(c_swap):.3f}")

    print("\n── ① 판정 ──")
    o, s = sum(n_orig), sum(n_swap)
    if s > max(o * 1.5, o + 10):
        print("   ★ **채널 순서가 뒤바뀌어 있다.** 반전 쪽이 뚜렷이 많이 잡힌다.")
        print("     → vision.py 의 infer() 입력을 frame[:, :, ::-1] 로 바꿀 것.")
    elif o > max(s * 1.5, s + 10):
        print("   채널 순서는 지금이 맞다. 검출 문제의 원인은 다른 데 있다.")
        print("     → 학습 데이터와 지금 카메라/렌즈가 다른지(도메인 이동),")
        print("        .hef 의 imgsz 가 config.YOLO_IMGSZ 와 같은지 확인할 것.")
    else:
        print(f"   둘 다 비슷하다 (원본 {o} / 반전 {s}). 채널 문제가 아니다.")
        print("     → 모델 자체가 이 장면을 잘 못 맞추는 것이다. 학습 데이터와")
        print("        지금 환경(카메라·렌즈·조명)의 차이를 의심할 것.")
    return 0


def _save(frame, hits_o, hits_s, idx: int) -> None:
    """원본/반전 결과를 박스로 그려 나란히 저장한다. 눈으로 보는 게 제일 확실하다."""
    import cv2

    canvas = frame.copy()
    for bb, cf in hits_o:
        cx, cy, w, h = bb
        cv2.rectangle(canvas, (int(cx - w / 2), int(cy - h / 2)),
                      (int(cx + w / 2), int(cy + h / 2)), (0, 255, 0), 2)
        cv2.putText(canvas, f"orig {cf:.2f}", (int(cx - w / 2), int(cy - h / 2) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    for bb, cf in hits_s:
        cx, cy, w, h = bb
        cv2.rectangle(canvas, (int(cx - w / 2), int(cy - h / 2)),
                      (int(cx + w / 2), int(cy + h / 2)), (0, 0, 255), 2)
        cv2.putText(canvas, f"swap {cf:.2f}", (int(cx - w / 2), int(cy + h / 2) + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    import pathlib
    out = pathlib.Path(__file__).resolve().parent / "logs" / "diagnose"
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / f"frame{idx:02d}.jpg"), canvas)


if __name__ == "__main__":
    raise SystemExit(main())
