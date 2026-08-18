"""배경 대비 달라진 곳을 찾아 낙하 물체를 자동 검출 + 자동 라벨링한다. (라파이에서 실행)

    python capture_dataset.py --label trash
    python capture_dataset.py --label trash --session-note "복도 형광등"

    # SSH/VNC 로 접속했다면 반드시 이렇게. 창을 원격으로 보내는 비용이
    # 프레임률을 10분의 1로 떨어뜨린다 (TROUBLESHOOTING.md 3번)
    python capture_dataset.py --label trash --no-display

동작: 카메라를 고정하고 천장을 향하게 둔 뒤 물체를 떨어뜨린다. 배경(천장)은 정지해
있고 움직이는 건 물체뿐이므로, 배경과 달라진 영역을 찾으면 그게 물체다. 클래스는
--label 로 미리 알려주므로 사람이 라벨을 찍을 필요가 없다.

이 데이터셋의 목적은 **"쓰레기라는 물체를 인식하는 YOLO 학습"** 이다. 궤적 추정은
런타임에 별도로 하므로, 여기서 중요한 건 **잡힌 게 물체가 맞느냐**다.

┌─ 검출기 두 가지 (--detector) ────────────────────────────────────────────┐
│ ref  (기본)  기준 프레임 차분 + 밝기 정규화                                │
│              워밍업 때 빈 천장을 평균 내 '기준 이미지' 한 장을 만들고,      │
│              매 프레임 그것과 비교한다. 학습이 없어 배경이 오염될 수 없고,  │
│              밝기 정규화가 형광등 120Hz 맥동을 상쇄한다.                   │
│ mog2         기존 방식. 픽셀마다 가우시안 혼합모델을 유지한다. 무겁고        │
│              전 화면 밝기 변동을 '움직임'으로 오인하지만, 배경이 완전히     │
│              정적이지 않은 환경에서는 이쪽이 나을 수 있다.                 │
│                                                                          │
│ 실제 방에서 둘 다 돌려보고 기각 사유 통계를 비교할 것.                     │
└──────────────────────────────────────────────────────────────────────────┘

┌─ 조심할 것 (필터로 처리했다) ────────────────────────────────────────────┐
│ ① 던지는 손이 같이 잡힌다      — 덩어리가 2개 이상인 프레임은 버린다        │
│ ② 프레임 경계에 걸친 물체      — 잘린 부분의 중심은 진짜 중심이 아니다      │
│ ③ 형광등 깜빡임/노이즈         — 밝기 정규화 + 워밍업 + 면적 하한          │
│ ④ 배경과 색이 비슷하면 실패    — 흰 종이컵 + 흰 천장 조합은 피할 것         │
└──────────────────────────────────────────────────────────────────────────┘

**자동 라벨은 초안이다.** review_labels.py 로 검수한다.

카메라를 바꾸면 --camera 프로파일만 바꾸면 되지만, **수집한 데이터는 재사용할 수
없다** — 화각·왜곡·셔터가 달라 물체가 다른 크기/형태로 찍힌다.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import pathlib
import statistics
import sys
import time
import types

import camera as camlib
from frame_saver import FrameSaver

CLASSES = ["trash"]          # review_labels.py / prepare_dataset.py 와 동일해야 한다
ROOT = pathlib.Path("dataset_raw")


def beep() -> None:
    """저장될 때마다 소리로 알린다. --no-display 로 돌리면 이게 유일한 즉시 피드백이고,
    어차피 던지는 사람은 화면 앞이 아니라 방 건너편에 있다."""
    sys.stdout.write("\a")
    sys.stdout.flush()


# ── 입력원 ────────────────────────────────────────────────────────────────

class ReplaySource:
    """record_raw.py 로 녹화한 프레임을 **카메라처럼** 읽는다.

    tune_params.py 로 찾은 파라미터를 녹화본 전체에 적용해 라벨을 뽑을 때 쓴다.
    실시간 카메라와 같은 인터페이스라 아래 루프는 손댈 필요가 없다.
    """

    def __init__(self, cv2, session_dir: pathlib.Path) -> None:
        meta = json.loads((session_dir / "meta.json").read_text(encoding="utf-8"))
        self.cv2 = cv2
        self.dir = session_dir / "frames"
        self.frames = meta["frames"]
        self.n_warmup = sum(1 for f in self.frames if f["warmup"])
        self.spec = types.SimpleNamespace(fps=meta.get("fps_setting", 30),
                                          exposure_us=meta.get("exposure_us"),
                                          gain=meta.get("gain"),
                                          name=f"replay({session_dir.name})")
        self.i = 0

    def read(self):
        while self.i < len(self.frames):
            f = self.frames[self.i]
            self.i += 1
            img = self.cv2.imread(str(self.dir / f["name"]))
            if img is not None:
                return img, f["t"]
        raise EOFError("녹화본 끝")

    def close(self) -> None:
        pass


# ── 검출 ──────────────────────────────────────────────────────────────────

class BlobFinder:
    """배경 대비 전경 마스크에서 '물체 하나'를 찾는다.

    프레임 -> 전경 마스크 -> OPEN/CLOSE -> 윤곽 -> 면적/개수/경계/종횡비/채움 검사 -> bbox
    """

    def __init__(self, cv2, args, frame_area: int) -> None:
        self.cv2 = cv2
        self.a = args
        self.min_area = max(args.min_area, int(frame_area * 1e-5))
        self.max_area = int(frame_area * args.max_area_frac)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        self.ref = None          # 기준 이미지 (흑백). detector="ref" 일 때만 쓴다
        self.ref_mean = 0.0
        self._acc = []           # 워밍업 누적
        self.mog = None
        if args.detector == "mog2":
            # detectShadows=False: 그림자를 회색(127)으로 표시하는 기능. 천장 배경에는
            # 불필요하고 마스크만 지저분해진다.
            self.mog = cv2.createBackgroundSubtractorMOG2(
                history=args.history, varThreshold=args.var_threshold,
                detectShadows=False)

    # 배경 습득 ------------------------------------------------------------
    def learn(self, frame) -> None:
        """워밍업 프레임 하나를 배경에 반영한다."""
        cv2 = self.cv2
        if self.mog is not None:
            self.mog.apply(frame, learningRate=-1)     # -1: 자동 학습률
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self._acc.append(gray.astype("float32"))

    def finish_learning(self) -> None:
        """누적한 워밍업 프레임을 평균 내 기준 이미지를 만든다."""
        if self.mog is not None or not self._acc:
            return
        import numpy as np
        ref = np.mean(self._acc, axis=0)
        self.ref = ref.astype("uint8")
        self.ref_mean = float(ref.mean())
        self._acc.clear()

    # 전경 추출 ------------------------------------------------------------
    def _foreground(self, frame):
        cv2 = self.cv2
        if self.mog is not None:
            # 학습률 0: 물체가 배경으로 흡수되지 않게 고정한다.
            return self.mog.apply(frame, learningRate=0.0)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.a.normalize:
            # 형광등 맥동은 화면 전체 밝기를 곱셈으로 흔든다. 기준 이미지의 평균에
            # 맞춰 스케일을 되돌리면 그 성분이 거의 상쇄된다 — 배경차분이 깜빡임을
            # "화면 전체가 움직였다"로 오인하는 것을 막는 가장 직접적인 방법이다.
            m = float(gray.mean())
            if m > 1.0:
                gray = cv2.convertScaleAbs(gray, alpha=self.ref_mean / m)
        diff = cv2.absdiff(gray, self.ref)
        _, mask = cv2.threshold(diff, self.a.diff_threshold, 255, cv2.THRESH_BINARY)
        return mask

    def __call__(self, frame):
        cv2 = self.cv2
        mask = self._foreground(frame)
        # 열림 -> 점 노이즈 제거, 닫힘 -> 물체 내부 구멍 메우기.
        # 닫힘을 여러 번 하면 작은 물체(2m에서 약 26px)는 형태가 뭉개져 중심이 밀린다.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        if self.a.close_iters > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel,
                                    iterations=self.a.close_iters)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        big = [c for c in contours if self.min_area <= cv2.contourArea(c) <= self.max_area]
        if not big:
            return mask, None, "none"
        if len(big) > 1:
            # 손 + 물체처럼 덩어리가 여럿이면 어느 쪽이 물체인지 확신할 수 없다
            return mask, None, f"multi({len(big)})"

        x, y, w, h = cv2.boundingRect(big[0])
        H, W = mask.shape[:2]
        m = self.a.edge_margin
        if x <= m or y <= m or x + w >= W - m or y + h >= H - m:
            return mask, None, "edge"          # 잘린 부분의 중심은 진짜 중심이 아니다
        ar = w / h if h else 0
        if not (self.a.min_aspect <= ar <= self.a.max_aspect):
            return mask, None, f"aspect({ar:.2f})"
        if cv2.contourArea(big[0]) / (w * h) < self.a.min_fill:
            return mask, None, "sparse"        # 길쭉한 노이즈 제거
        return mask, (x, y, w, h), "ok"


# ── 메인 ──────────────────────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="배경차분 자동 라벨링 데이터 수집")
    camlib.add_profile_arg(ap)
    ap.add_argument("--label", default="trash", choices=CLASSES)
    ap.add_argument("--session-note", default="", help="배경/조명 메모 (다양성 추적용)")
    ap.add_argument("--replay", default=None, metavar="raw/<세션>",
                    help="카메라 대신 record_raw.py 녹화본을 재생한다. tune_params.py 로 "
                         "찾은 파라미터를 녹화본 전체에 적용해 라벨을 뽑을 때 쓴다 (PC에서)")
    ap.add_argument("--no-display", dest="display", action="store_false",
                    help="창을 띄우지 않는다. SSH/VNC 로 접속했다면 반드시 붙일 것 — "
                         "X11 로 프레임을 보내는 비용이 프레임률을 10분의 1로 떨어뜨린다")

    g = ap.add_argument_group("검출기")
    g.add_argument("--detector", choices=["ref", "mog2"], default="ref",
                   help="ref=기준 프레임 차분(가볍고 깜빡임에 강함) / mog2=기존 방식")
    g.add_argument("--no-normalize", dest="normalize", action="store_false",
                   help="밝기 정규화를 끈다 (ref 전용). 정규화가 오히려 해로울 때만")
    g.add_argument("--warmup", type=int, default=60, help="배경 학습 프레임 수")
    # ⚠ 두 검출기의 임계값은 **단위가 다르다.** 같은 이름을 쓰면 안 된다.
    #   ref  : 밝기 차이 그 자체 (0~255). "배경보다 25 이상 어둡거나 밝으면 전경"
    #   mog2 : 그 픽셀의 학습된 분산 대비 마할라노비스 거리의 제곱. 밝기 단위가 아니다
    g.add_argument("--diff-threshold", type=float, default=25.0,
                   help="[ref] 배경과 밝기가 이만큼 이상 다르면 전경 (0~255)")
    g.add_argument("--var-threshold", type=float, default=25.0,
                   help="[mog2] 배경 모델 대비 분산 기준 거리. 밝기 단위가 아니다")
    g.add_argument("--history", type=int, default=300, help="[mog2] 배경 학습 프레임 수")

    g = ap.add_argument_group("블롭 필터")
    g.add_argument("--min-area", type=int, default=80)
    g.add_argument("--max-area-frac", type=float, default=0.25,
                   help="프레임 대비 최대 면적. 넘으면 손/사람으로 본다")
    g.add_argument("--min-aspect", type=float, default=0.2)
    g.add_argument("--max-aspect", type=float, default=5.0)
    g.add_argument("--min-fill", type=float, default=0.3, help="bbox 대비 윤곽 채움 비율")
    g.add_argument("--edge-margin", type=int, default=4)
    g.add_argument("--close-iters", type=int, default=1,
                   help="닫힘 반복(0이면 안 함). 크면 구멍은 잘 메우지만 작은 물체의 "
                        "중심이 밀린다. tune_params.py 로 실측해서 정할 것")

    g = ap.add_argument_group("추적·저장")
    g.add_argument("--arm-frames", type=int, default=2,
                   help="연속 이 프레임 이상 유효해야 '확정'하고 추적을 시작한다. "
                        "확정 전까지 모아둔 프레임도 확정되는 순간 같이 저장된다")
    g.add_argument("--track-grace", type=int, default=3,
                   help="추적 중 연속으로 이 프레임까지는 놓쳐도 계속 따라가며 저장한다")
    g.add_argument("--jpeg-quality", type=int, default=85,
                   help="95는 인코딩이 무겁고 학습 품질 차이는 없다")
    g.add_argument("--max-frames", type=int, default=0, help="0이면 무제한")
    return ap


def main() -> int:
    import cv2

    args = build_parser().parse_args()

    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    session = f"{args.label}_{args.camera}_{stamp}"
    if args.replay:
        session += "_replay"
    out_img = ROOT / "images" / session
    out_lbl = ROOT / "labels" / session
    meta_dir = ROOT / "meta"
    for d in (out_img, out_lbl, meta_dir):
        d.mkdir(parents=True, exist_ok=True)
    meta_path = meta_dir / f"{session}.json"

    if args.replay:
        cam = ReplaySource(cv2, pathlib.Path(args.replay))
        args.warmup = cam.n_warmup          # 녹화 때 표시해둔 워밍업 구간을 그대로 쓴다
        args.display = False                # 수천 장을 창에 뿌릴 이유가 없다
        print(f"[replay] {cam.spec.name}  프레임 {len(cam.frames)}장 "
              f"(워밍업 {cam.n_warmup}장)")
    else:
        cam = camlib.open_camera(args.camera, exposure_us=args.exposure_us, gain=args.gain,
                                 auto_lock=args.auto_lock_exposure,
                                 max_exposure_us=args.max_exposure_us,
                                 max_gain=args.max_gain)
    frame, _ = cam.read()
    H, W = frame.shape[:2]
    finder = BlobFinder(cv2, args, W * H)
    saver = FrameSaver(cv2, out_img, out_lbl, args.jpeg_quality)
    cls_id = CLASSES.index(args.label)

    print(f"\n세션 {session}   클래스 {args.label}(id={cls_id})   검출기 {args.detector}")
    if args.display:
        print("[조작]  SPACE 일시정지/재개   R 배경 재학습   Q 종료")
    else:
        print("[조작]  Ctrl+C 종료   (창 없음 — 저장될 때마다 비프음이 난다)")
    print(f"[순서]  ① 카메라 고정 ② 워밍업 {args.warmup}프레임 동안 화면 비우기 "
          f"③ 물체 투척 반복\n")

    saved = 0
    streak = 0
    tracking = False
    miss = 0
    throw = 0                 # 투척 회차. 확정될 때마다 1씩 오른다
    pending: list = []        # 확정 전 보류 프레임
    frame_meta: list[dict] = []   # review_labels.py 의 자동 선별이 쓴다
    paused = False
    reasons: dict[str, int] = {}
    fps_log: list[float] = []

    def warm_up() -> None:
        for i in range(args.warmup):
            f, _ = cam.read()
            finder.learn(f)
            if i % 20 == 0:
                print(f"  워밍업 {i}/{args.warmup}")
        finder.finish_learning()
        print("  워밍업 완료 — 이제 던져도 된다\n")

    def save(save_frame, box, t_cap: float) -> None:
        """파일명·라벨·메타를 여기서 확정하고, 실제 쓰기는 스레드에 넘긴다."""
        nonlocal saved
        x, y, w, h = box
        name = f"{session}_{saved:05d}"
        # YOLO 포맷: class cx cy w h  (0~1 정규화)
        line = (f"{cls_id} {(x + w / 2) / W:.6f} {(y + h / 2) / H:.6f} "
                f"{w / W:.6f} {h / H:.6f}\n")
        if not saver.submit(name, save_frame, line):
            return
        frame_meta.append({"name": f"{name}.jpg", "t": round(t_cap, 6), "throw": throw})
        saved += 1
        beep()

    def flush_meta() -> None:
        """투척이 끝날 때마다 쓴다. 강제 종료돼도 자동 선별을 쓸 수 있게."""
        meta_path.write_text(json.dumps(frame_meta, indent=1), encoding="utf-8")

    try:
        warm_up()
        n = 0
        fps_t0 = time.monotonic()
        while args.max_frames == 0 or n < args.max_frames:
            frame, t_cap = cam.read()
            n += 1
            if paused:
                cv2.imshow("capture (paused)", frame)
                if (cv2.waitKey(30) & 0xFF) == ord(" "):
                    paused = False
                continue

            mask, box, why = finder(frame)
            reasons[why] = reasons.get(why, 0) + 1

            good = False      # 이번 프레임이 저장(또는 보류)됐는지 — 화면 표시용
            if not tracking:
                # 확정 전 — 연속 유효 프레임을 모으기만 한다. 손이 막 놓은 순간의
                # 우연한 블롭 하나로 오검출되는 걸 막는다.
                if box:
                    streak += 1
                    pending.append((frame, box, t_cap))
                    good = True
                    if streak >= args.arm_frames:
                        throw += 1
                        for pf, pb, pt in pending:
                            save(pf, pb, pt)
                        pending.clear()
                        tracking = True
                        miss = 0
                else:
                    streak = 0
                    pending.clear()
            elif box:
                miss = 0
                save(frame, box, t_cap)
                good = True
            else:
                miss += 1
                if miss > args.track_grace:
                    # 유예를 넘게 놓쳤다 — 물체가 진짜로 사라진 것으로 본다.
                    tracking, streak, miss = False, 0, 0
                    flush_meta()

            # 그리기와 창 전송은 --display 일 때만. 원격 접속에서는 이 블록 하나가
            # 나머지 전부를 합친 것보다 비싸다 (프레임당 6MB 넘게 나간다).
            if args.display:
                view = frame.copy()
                if box:
                    x, y, w, h = box
                    cv2.rectangle(view, (x, y), (x + w, y + h),
                                  (0, 255, 0) if good else (0, 200, 255), 2)
                else:
                    cv2.putText(view, why, (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (0, 0, 255), 2)
                cv2.putText(view, f"{args.label}  saved={saved}  throw={throw}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow("capture", view)
                cv2.imshow("mask", cv2.resize(mask, (W // 2, H // 2)))

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord(" "):
                    paused = True
                if key == ord("r"):
                    finder = BlobFinder(cv2, args, W * H)
                    print("  배경 재학습 — 화면을 비워라")
                    warm_up()

            # 실효 프레임률. 놓친 프레임은 그대로 투척당 수집 장수의 손실이다.
            if n % 30 == 0:
                now = time.monotonic()
                fps = 30.0 / max(1e-6, now - fps_t0)
                fps_t0 = now
                fps_log.append(fps)
                print(f"  {fps:5.1f} fps   saved={saved}   최근={why}")
    except EOFError:
        print("\n  녹화본 끝까지 재생했다.")
    except KeyboardInterrupt:
        print("\n  Ctrl+C — 중단한다. 저장된 사진은 그대로 남아 있다.")
    finally:
        cam.close()
        saver.close()
        flush_meta()
        if args.display:
            cv2.destroyAllWindows()

    manifest = ROOT / "sessions.json"
    data = json.loads(manifest.read_text(encoding="utf-8")) if manifest.exists() else {}
    data[session] = {"label": args.label, "class_id": cls_id, "camera": args.camera,
                     "detector": args.detector, "frames": saved, "throws": throw,
                     "note": args.session_note, "created": stamp, "image_size": [W, H]}
    manifest.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")

    # 절대경로로 찍는다. ROOT 가 상대경로라 실행 디렉토리에 따라 위치가 바뀌고,
    # 그 때문에 "폴더는 생겼는데 사진을 못 찾겠다"가 실제로 한 번 발생했다.
    print(f"\n{saved}장 / 투척 {throw}회 저장 -> {out_img.resolve()}")
    print(f"기각 사유: {dict(sorted(reasons.items(), key=lambda kv: -kv[1]))}")

    if saver.dropped:
        print(f"⚠ 저장 큐가 넘쳐 {saver.dropped}장을 버렸다 — 디스크가 못 따라온다.")
        print("   --jpeg-quality 를 낮추거나 더 빠른 저장매체를 쓸 것.")

    if fps_log:
        avg = statistics.mean(fps_log)
        print(f"실효 프레임률: 평균 {avg:.1f} fps  (카메라 설정 {cam.spec.fps} fps)")
        if avg < cam.spec.fps * 0.7:
            print("⚠ 프레임을 놓치고 있다 — 투척당 잡히는 장수가 그만큼 줄어든다.")
            if args.display:
                print("   원격 접속(SSH/VNC) 중이라면 --no-display 를 붙여라.")
            elif args.detector == "mog2":
                print("   --detector ref 로 바꿔볼 것 (훨씬 가볍다).")
            else:
                print("   camera.py 에서 더 낮은 해상도 프로파일을 검토할 것.")

    multi = sum(v for k, v in reasons.items() if k.startswith("multi"))
    if multi > saved:
        print("⚠ 'multi'가 많다 — 손이 오래 잡히거나 조명이 깜빡이고 있다.")
        if args.detector == "mog2":
            print("   --detector ref (밝기 정규화 포함) 로 바꿔보면 크게 줄어든다.")
        elif not args.normalize:
            print("   --no-normalize 를 뺀 채로 다시 돌려볼 것.")
        else:
            print("   조명을 자연광이나 DC LED 로 바꾸는 게 근본 해법이다.")
    if reasons.get("edge", 0) > saved:
        print("⚠ 'edge'가 많다 — 물체가 화면 가장자리로 지나간다. 카메라 정렬을 확인하라.")
    print("\n다음: python review_labels.py --session", session)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
