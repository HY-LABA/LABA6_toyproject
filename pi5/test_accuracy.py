"""궤적 예측 정확도 측정 — **로봇을 세워두고 예측만 기록한다.**

    python test_accuracy.py                    # 던지면서 실시간 측정 (기본)
    python test_accuracy.py --throws 10        # 10회 하고 종료
    python test_accuracy.py --no-measure       # 정답 입력 안 받고 수렴만 관찰
    python test_accuracy.py --video            # 검출 박스 그린 영상도 남김
    python test_accuracy.py --replay logs/*.json        # 저장된 관측으로 재실행
    python test_accuracy.py --replay logs/*.json --span 0.25   # 파라미터 바꿔서 재실행

왜 main.py를 그냥 돌리지 않나
-----------------------------
`main.py`는 피코에 명령을 보내고 **로봇이 실제로 움직인다.** 그러면 최종 오차에
예측 오차와 제어 오차가 섞여서 "궤적 예측이 얼마나 정확한가"를 분리할 수 없다.
여기서는 로봇을 세워두고(오도메트리 고정 (0,0)) **예측만** 기록한다.

쓰는 코드는 실제 구동과 완전히 같다 — `vision.observe` → `tracker.TrackerPool.step`
→ (그 안에서) `trajectory.fit_trajectory`. **`main.py`와 같은 함수를 부른다.**
달라지는 건 피코로 명령을 안 보낸다는 것과 오도메트리가 (0,0)이라는 것뿐이다.

실행하면 맨 위에 **실제로 로드된 모듈 파일 경로**를 찍는다 — 이 파일이
`trajectory`를 직접 import하지 않아서(`tracker.py`를 거쳐 부른다) 의심스러울 수
있는데, 그 출력으로 확인하면 된다.

★ 관측을 전부 저장한다
----------------------
한 번 던진 데이터를 `--replay`로 몇 번이든 다시 돌릴 수 있다. `MIN_TIME_SPAN_S`나
`DEPTH_STABILITY_RATIO`를 바꿔가며 비교할 때 **매번 다시 던질 필요가 없다.**
저장하는 건 프레임별 검출 후보 전체라, 가설 배정까지 그대로 재현된다.

★ 좌표 규약 — 부호를 여기서 검증한다
------------------------------------
코드가 내는 착지점 (x, y)는 카메라 좌표계다. 장착 규약(`docs/hardware.md`)에 따르면
**x = 로봇 전방, y = 로봇 우측**(= 이미지 +v 방향)이어야 한다. 이 규약이 실제 조립과
어긋나면 로봇이 반대쪽으로 간다. 그래서 정답을 **물리적인 말("전방 몇 cm, 좌측 몇 cm")**
로 입력받아, 부호가 계통적으로 뒤집혀 있으면 경고한다.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import pathlib
import statistics
import time
from dataclasses import dataclass

import config
import tracker as tracker_mod

LOG_DIR = pathlib.Path(__file__).resolve().parent / "logs" / "accuracy"


@dataclass
class Det:
    """검출 하나. `vision.Detection`과 같은 필드를 갖는 가벼운 대체물 —
    리플레이에서 카메라·YOLO 없이 만들어 쓰기 위함."""

    u: float
    v: float
    t: float
    confidence: float
    bbox: tuple = (0.0, 0.0, 0.0, 0.0)


# ── 검출 과정 영상 녹화 ────────────────────────────────────────────────────

class VideoRecorder:
    """검출 박스를 그려 영상으로 남긴다. **인코딩은 백그라운드 스레드에서** 한다.

    ⚠ 공짜가 아니다. 라즈베리파이5에는 하드웨어 인코더 가속이 없어서
    `cv2.VideoWriter`가 CPU를 쓴다. 다만 비용을 **캡처 루프 밖으로** 빼서, 루프는
    큐에 넣기만 하고 즉시 돌아온다. 큐가 차면 기다리지 않고 **버리고 센다** —
    측정 중에 프레임을 놓치느니 영상 몇 장을 잃는 게 낫다.

    그래도 부담되면 `--video-scale 0.5`(기본)로 줄여 쓴다. 화소가 1/4이면
    인코딩 비용도 대략 1/4이다. 검출 박스 위치를 눈으로 확인하는 용도라 충분하다.
    """

    def __init__(self, path: pathlib.Path, fps: int, scale: float) -> None:
        import queue
        import threading

        self.path = path
        self.fps = fps
        self.scale = scale
        self.q: queue.Queue = queue.Queue(maxsize=64)
        self.dropped = 0
        self.written = 0
        path.parent.mkdir(parents=True, exist_ok=True)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, frame, dets, label: str) -> None:
        """루프에서 부른다. 큐에 넣기만 하고 바로 돌아온다."""
        try:
            self.q.put_nowait((frame, [(d.u, d.v, d.confidence, d.bbox) for d in dets], label))
        except Exception:
            self.dropped += 1

    def _run(self) -> None:
        import cv2

        writer = None
        while True:
            item = self.q.get()
            if item is None:
                break
            frame, dets, label = item
            try:
                img = cv2.resize(frame, None, fx=self.scale, fy=self.scale) \
                    if self.scale != 1.0 else frame.copy()
                for _u, _v, conf, bbox in dets:
                    cx, cy, w, h = (v * self.scale for v in bbox)
                    cv2.rectangle(img, (int(cx - w / 2), int(cy - h / 2)),
                                  (int(cx + w / 2), int(cy + h / 2)), (0, 255, 0), 2)
                    cv2.putText(img, f"{conf:.2f}", (int(cx - w / 2), int(cy - h / 2) - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(img, label, (6, 18), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (0, 255, 255), 1)
                if writer is None:
                    h0, w0 = img.shape[:2]
                    writer = cv2.VideoWriter(str(self.path),
                                             cv2.VideoWriter_fourcc(*"mp4v"),
                                             self.fps, (w0, h0))
                writer.write(img)
                self.written += 1
            except Exception as exc:  # noqa: BLE001 - 영상 실패로 측정을 멈추지 않는다
                print(f"[video] {exc}")
            finally:
                self.q.task_done()
        if writer is not None:
            writer.release()

    def close(self) -> None:
        self.q.put(None)
        self._thread.join(timeout=30.0)
        if self.written:
            print(f"   영상: {self.path}  ({self.written}프레임"
                  + (f", 큐가 차서 버린 것 {self.dropped}장" if self.dropped else "") + ")")


# ── 한 번의 투척 ───────────────────────────────────────────────────────────

class Throw:
    """한 사이클 동안의 프레임·예측을 모아둔다."""

    def __init__(self, index: int) -> None:
        self.index = index
        self.frames: list[dict] = []       # [{t, dets:[{u,v,conf}]}]
        self.preds: list[dict] = []        # [{t_rel, x, y, remaining, z, residual, n, span}]
        self.t0: float | None = None
        self.gt: dict | None = None        # {forward_m, left_m}
        self.end_reason: str = ""

    def record_frame(self, t: float, dets) -> None:
        if self.t0 is None and dets:
            self.t0 = t
        self.frames.append({
            "t": t,
            "dets": [{"u": d.u, "v": d.v, "conf": d.confidence} for d in dets],
        })

    def record_pred(self, t: float, track, x: float, y: float, remaining: float) -> None:
        self.preds.append({
            "t_rel": t - (self.t0 if self.t0 is not None else t),
            "x": x, "y": y, "remaining": remaining,
            "z": float(track.fit.p0[2]),
            "residual": float(track.fit.residual_px),
            "n": track.n, "span": track.time_span,
        })

    # ── 오차 ──────────────────────────────────────────────────────────────
    def errors(self):
        """(문서 규약 기준 오차 리스트, 부호 뒤집힘 가설 기준 오차 리스트) — cm."""
        if self.gt is None:
            return None, None
        fwd, left = self.gt["forward_m"], self.gt["left_m"]
        # 문서 규약: code x = 전방, code y = 우측 = −좌측
        a = [math.hypot(p["x"] - fwd, p["y"] - (-left)) * 100 for p in self.preds]
        # 뒤집힘 가설: code y = 좌측
        b = [math.hypot(p["x"] - fwd, p["y"] - left) * 100 for p in self.preds]
        return a, b

    def to_json(self) -> dict:
        return {
            "index": self.index,
            "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "end_reason": self.end_reason,
            "params": {k: getattr(config, k) for k in (
                "MIN_OBSERVATIONS", "MIN_TIME_SPAN_S", "MAX_RESIDUAL_PX",
                "DEPTH_STABILITY_RATIO", "DEPTH_CHECK_FRACTION", "TRACK_MAX_GAP_S",
                "ASSOC_STEP_PX", "MAX_TRACKS", "CAMERA_OFFSET_M", "YOLO_CONF_THRESHOLD")},
            "frames": self.frames,
            "predictions": self.preds,
            "ground_truth": self.gt,
        }


# ── 한 투척을 파이프라인에 통과시킨다 ──────────────────────────────────────

def run_throw(throw: Throw) -> Throw:
    """저장된 프레임들을 `TrackerPool`에 흘려 예측을 다시 만든다.

    ★ **`main.py`와 완전히 같은 `pool.step()`을 쓴다.** 루프를 따로 베껴 쓰면
    `main.py`를 고쳤을 때 테스트가 조용히 다른 걸 재게 된다. 여기서 다른 건
    "명령을 피코로 보내지 않는다"와 "로봇이 안 움직이니 오도메트리가 (0,0)"뿐이다.
    """
    pool = tracker_mod.TrackerPool()
    throw.preds = []
    throw.t0 = next((f["t"] for f in throw.frames if f["dets"]), None)
    odom = (0.0, 0.0)                       # 로봇 정지 — 예측 오차만 보려는 것이므로

    for fr in throw.frames:
        t = fr["t"]
        dets = [Det(d["u"], d["v"], t, d["conf"]) for d in fr["dets"]]
        track, landing, reason = pool.step(dets, odom, t)
        if landing is not None:
            throw.record_pred(t, track, landing[0], landing[1], landing[2])
        if reason:
            throw.end_reason = reason
            break
    return throw


# ── 출력 ───────────────────────────────────────────────────────────────────

def print_throw(throw: Throw) -> None:
    print(f"\n── 투척 {throw.index} ──  프레임 {len(throw.frames)}장, "
          f"종료: {throw.end_reason or '(끝까지)'}")
    if not throw.preds:
        print("   ⚠ 예측이 한 번도 안 나왔다.")
        _diagnose(throw)
        return

    errs, _ = throw.errors()
    head = f"   {'t(s)':>6}{'관측':>5}{'스팬':>7}{'깊이(m)':>9}{'잔차(px)':>9}{'예측 착지 (cm)':>20}"
    print(head + (f"{'오차':>9}" if errs else ""))
    print("   " + "─" * (len(head) + (9 if errs else 0) - 3))
    for i, p in enumerate(throw.preds):
        # 처음 3개와 마지막 3개만 — 중간은 대개 단조 수렴이라 볼 게 없다
        if len(throw.preds) > 7 and 3 <= i < len(throw.preds) - 3:
            if i == 3:
                print(f"   {'':>6}{'':>5}{'':>7}{'⋮':>9}")
            continue
        xy = f"({p['x'] * 100:+.1f}, {p['y'] * 100:+.1f})"
        line = (f"   {p['t_rel']:>6.2f}{p['n']:>5}{p['span']:>7.2f}"
                f"{p['z']:>9.2f}{p['residual']:>9.2f}{xy:>20}")
        print(line + (f"{errs[i]:>8.1f}cm" if errs else ""))

    f = throw.preds[0]
    l = throw.preds[-1]
    print(f"   첫 예측 {f['t_rel']:.2f}s" + (f", 오차 {errs[0]:.1f}cm" if errs else ""))
    print(f"   최종 예측 ({l['x']*100:+.1f}, {l['y']*100:+.1f}) cm"
          + (f", 오차 {errs[-1]:.1f}cm" if errs else ""))


def _diagnose(throw: Throw) -> None:
    """예측이 안 나왔을 때 어디서 막혔는지 짚어준다."""
    n_det = sum(len(f["dets"]) for f in throw.frames)
    n_frames_with = sum(1 for f in throw.frames if f["dets"])
    print(f"   검출 {n_det}개 / 검출된 프레임 {n_frames_with}장")
    if n_det == 0:
        print("   → YOLO가 아무것도 못 찾았다. 조명·노출·YOLO_CONF_THRESHOLD를 볼 것.")
        return
    span = throw.frames[-1]["t"] - throw.frames[0]["t"] if throw.frames else 0.0
    fps = len(throw.frames) / span if span > 0 else 0.0
    print(f"   프레임 스팬 {span:.2f}s, 실측 {fps:.1f}fps "
          f"(MIN_TIME_SPAN_S={config.MIN_TIME_SPAN_S})")

    # ★ 검출이 움직였는가 — 지금 가장 흔한 실패 원인이 "정적 오탐만 봤다"이다.
    spots: list[list] = []
    for f in throw.frames:
        for d in f["dets"]:
            for s in spots:
                if math.hypot(d["u"] - s[0], d["v"] - s[1]) <= 40:
                    s[2] += 1
                    break
            else:
                spots.append([d["u"], d["v"], 1])
    moving = sum(1 for s in spots if s[2] <= 3)
    print(f"   검출 위치 군집 {len(spots)}개 "
          f"(고정 {len(spots) - moving}개, 스쳐간 것 {moving}개)")

    if moving == 0:
        print("   → **움직이는 검출이 하나도 없다.** 전부 같은 자리에 계속 나타나는")
        print("      정적 오탐(에어컨·조명·공유기)이다. 던진 물체가 YOLO에 안 잡혔다는 뜻.")
        print("      확인할 것: 물체가 화면 안에 들어왔는지, 노출/모션블러, YOLO_CONF_THRESHOLD,")
        print("      그리고 .hef 의 imgsz 가 config.YOLO_IMGSZ 와 같은지.")
    elif span < config.MIN_TIME_SPAN_S:
        print("   → 물체가 화면에 있던 시간이 최소 스팬보다 짧다. 더 높이 던지거나 "
              "MIN_TIME_SPAN_S를 낮출 것.")
    elif fps < config.CAMERA_FPS * 0.5:
        print(f"   → **프레임레이트가 설정({config.CAMERA_FPS})의 절반도 안 된다.** "
              f"YOLO 추론이 느려 관측이 성기다. 이러면 궤적을 못 푼다.")
    else:
        print("   → 움직이는 검출은 있었다. 중간에 끊겨(TRACK_MAX_GAP_S "
              f"{config.TRACK_MAX_GAP_S}s 초과) 가설이 죽었거나, fit.ok(잔차·조건수·"
              "깊이범위)를 못 넘었을 가능성이 크다. --span 을 낮춰 리플레이해볼 것.")


def print_summary(throws: list[Throw]) -> None:
    ok = [t for t in throws if t.preds]
    print("\n" + "=" * 62)
    print(f"투척 {len(throws)}회 요약  (예측 성공 {len(ok)}회, 실패 {len(throws) - len(ok)}회)")
    print("=" * 62)
    if not ok:
        return

    def stat(vals, unit, name):
        if not vals:
            return
        print(f"  {name:<16} 평균 {statistics.mean(vals):6.2f}{unit}"
              f"   중앙 {statistics.median(vals):6.2f}{unit}"
              f"   최소 {min(vals):6.2f}{unit}   최대 {max(vals):6.2f}{unit}")

    stat([t.preds[0]["t_rel"] for t in ok], "s", "첫 예측 시각")
    stat([t.preds[-1]["z"] for t in ok], "m", "최종 깊이")
    stat([t.preds[-1]["residual"] for t in ok], "px", "최종 잔차")
    stat([len(t.preds) for t in ok], "회", "예측 횟수")

    measured = [t for t in ok if t.gt is not None]
    if not measured:
        print("\n  (정답을 입력하지 않아 절대 정확도는 계산하지 않았다)")
        return

    first_e, final_e, flip_wins = [], [], 0
    for t in measured:
        a, b = t.errors()
        first_e.append(a[0])
        final_e.append(a[-1])
        if b[-1] < a[-1]:
            flip_wins += 1
    print()
    stat(first_e, "cm", "첫 예측 오차")
    stat(final_e, "cm", "최종 예측 오차")

    # ★ 좌표 부호 검증
    if flip_wins >= max(2, int(len(measured) * 0.7)):
        print(f"\n  ⚠ **좌우 부호가 뒤집혀 있을 수 있다** — {len(measured)}회 중 {flip_wins}회에서")
        print("     'code y = 좌측' 가설이 문서 규약('code y = 우측')보다 오차가 작았다.")
        print("     카메라 장착 방향(docs/hardware.md)이나 control의 y 부호를 확인할 것.")
        print("     그대로 두면 로봇이 좌우 반대로 간다.")
    elif len(measured) >= 3:
        print(f"\n  좌표 부호: 문서 규약(code y = 로봇 우측)과 일치 "
              f"({len(measured)}회 중 {len(measured) - flip_wins}회)")


# ── 정답 입력 ──────────────────────────────────────────────────────────────

def ask_ground_truth(throw: Throw) -> None:
    print("\n   실제 착지점을 로봇 중심 기준으로 재서 입력 (cm, 공백으로 구분)")
    print("   형식: <전방> <좌측>     예: `55 -20` = 앞으로 55cm, 오른쪽으로 20cm")
    print("   그냥 Enter = 이번 투척은 정답 없이 넘어감")
    try:
        raw = input("   > ").strip()
    except (EOFError, KeyboardInterrupt):
        raise
    if not raw:
        return
    try:
        fwd, left = (float(x) / 100.0 for x in raw.split()[:2])
    except ValueError:
        print("   숫자 두 개를 넣어야 한다. 이번 투척은 건너뛴다.")
        return
    throw.gt = {"forward_m": fwd, "left_m": left}
    errs, _ = throw.errors()
    if errs:
        print(f"   → 첫 예측 오차 {errs[0]:.1f}cm, 최종 예측 오차 {errs[-1]:.1f}cm")


# ── 실시간 수집 ────────────────────────────────────────────────────────────

def save_throw(throw: Throw, path: pathlib.Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(throw.to_json(), indent=1, ensure_ascii=False),
                    encoding="utf-8")
    print(f"   저장: {path}")


def collect_live(args) -> list[Throw]:
    import vision                                   # 여기서 import — PC에서도 --replay는 되게
    import trajectory

    # ★ **어떤 파일을 실제로 쓰고 있는지 찍는다.**
    #   test_accuracy.py 는 trajectory를 직접 import하지 않는다 — tracker.py 를 거쳐
    #   traj.fit_trajectory / predict_landing / project 를 부른다. 눈에 안 보이니
    #   여기서 경로를 출력해 "내 코드가 맞는지" 의심할 여지를 없앤다.
    print("사용 중인 모듈:")
    for m in (config, vision, tracker_mod, trajectory):
        print(f"   {m.__name__:<12} {m.__file__}")
    print(f"   모델        {config.YOLO_MODEL_PATH}")
    print(f"   주요 값     conf>={config.YOLO_CONF_THRESHOLD}  imgsz={config.YOLO_IMGSZ}  "
          f"스팬>={config.MIN_TIME_SPAN_S}s  잔차<={config.MAX_RESIDUAL_PX}px  "
          f"수렴비<={config.DEPTH_STABILITY_RATIO}")
    print()

    cam = vision.Camera(config.CAMERA_RESOLUTION, config.CAMERA_FPS)
    throws: list[Throw] = []
    stamp = time.strftime("%Y%m%d_%H%M%S")
    # 채택 전에는 최근 것만 들고 있는다. Ctrl-C로 끊어도 이 버퍼가 통째로 저장되므로
    # **"계속 탐지만 하고 아무 일도 안 일어날 때" 원인을 오프라인으로 뜯어볼 수 있다.**
    ring = int(config.CAMERA_FPS * args.buffer_s)
    rec = (VideoRecorder(LOG_DIR / f"{stamp}.mp4", config.CAMERA_FPS, args.video_scale)
           if args.video else None)

    print(f"카메라 열림 {config.CAMERA_RESOLUTION[0]}x{config.CAMERA_RESOLUTION[1]}"
          f"@{config.CAMERA_FPS}. 로봇은 **세워둘 것** (예측 오차만 재는 중).")
    print(f"던지면 자동으로 감지한다. Ctrl-C로 끊으면 **직전 {args.buffer_s:.0f}초가 저장**된다.\n")

    interrupted = False
    while not interrupted and (args.throws is None or len(throws) < args.throws):
        throw = Throw(len(throws) + 1)
        pool = tracker_mod.TrackerPool()
        odom = (0.0, 0.0)
        armed = False
        t_status = 0.0
        n_frames = 0
        t_first = None

        print(f"── 투척 {throw.index} 대기 중… ──")
        try:
            while True:
                dets, t, frame = vision.observe(cam)     # main.py와 같은 함수
                track, landing, reason = pool.step(dets, odom, t)
                if landing is not None:
                    armed = True
                if rec is not None:
                    lab = (f"t={t - (throw.frames[0]['t'] if throw.frames else t):6.2f}s  "
                           f"det={len(dets)}  trk={pool.n_tracks}  {pool.state}")
                    if landing is not None:
                        lab += f"  -> 착지({landing[0] * 100:+.0f},{landing[1] * 100:+.0f})cm"
                    rec.submit(frame, dets, lab)

                throw.record_frame(t, dets)
                if not armed and len(throw.frames) > ring:
                    throw.frames.pop(0)

                # ── 상태 표시 — 뭐가 되고 있는지 보여야 원인을 안다 ──────
                n_frames += 1
                if t_first is None:
                    t_first = t
                if t - t_status > 0.5:
                    t_status = t
                    fps = n_frames / max(t - t_first, 1e-6)
                    top = max((d.confidence for d in dets), default=0.0)
                    if armed and track is not None and track.fit is not None:
                        msg = (f"[추적] 관측 {track.n:3d}  스팬 {track.time_span:.2f}s  "
                               f"깊이 {track.fit.p0[2]:.2f}m  잔차 {track.fit.residual_px:.2f}px")
                    else:
                        msg = (f"[대기] {fps:4.1f}fps  검출 {len(dets)}개(최고 {top:.2f})  "
                               f"가설 {pool.n_tracks}개  {pool.state}  버퍼 {len(throw.frames)}")
                    print(f"\r   {msg:<78}", end="", flush=True)

                if reason and armed:
                    throw.end_reason = reason
                    break
        except KeyboardInterrupt:
            interrupted = True
            throw.end_reason = "사용자 중단 (Ctrl-C)"
            print("\r" + " " * 80 + "\r   중단됨 — 버퍼를 저장하고 분석한다.")

        print()
        if not throw.frames:
            print("   프레임이 하나도 없다. 카메라를 확인할 것.")
            break

        run_throw(throw)                             # 저장된 프레임으로 다시 풀어 일관성 확보
        print_throw(throw)
        if not args.no_measure and throw.preds:
            try:
                ask_ground_truth(throw)
            except (EOFError, KeyboardInterrupt):
                interrupted = True

        suffix = "interrupted" if interrupted else f"throw{throw.index:02d}"
        save_throw(throw, LOG_DIR / f"{stamp}_{suffix}.json")
        throws.append(throw)

    cam.close()
    if rec is not None:
        rec.close()
    return throws


# ── 리플레이 ───────────────────────────────────────────────────────────────

def collect_replay(patterns: list[str]) -> list[Throw]:
    paths: list[pathlib.Path] = []
    for pat in patterns:
        paths.extend(pathlib.Path(p) for p in sorted(glob.glob(pat)))
    if not paths:
        print(f"파일을 못 찾았다: {patterns}")
        return []

    throws = []
    for p in paths:
        data = json.loads(p.read_text(encoding="utf-8"))
        th = Throw(data.get("index", len(throws) + 1))
        th.frames = data["frames"]
        th.gt = data.get("ground_truth")
        th.t0 = next((f["t"] for f in th.frames if f["dets"]), None)
        run_throw(th)
        print_throw(th)
        throws.append(th)
    return throws


def main() -> int:
    ap = argparse.ArgumentParser(description="궤적 예측 정확도 측정 (로봇 정지 상태)")
    ap.add_argument("--throws", type=int, default=None, help="이 횟수만큼 하고 종료")
    ap.add_argument("--no-measure", action="store_true", help="정답 입력 안 받음")
    ap.add_argument("--video", action="store_true",
                    help="검출 박스를 그린 영상을 남긴다. 인코딩은 백그라운드 스레드라 "
                         "루프는 거의 안 느려지지만, CPU를 쓰므로 fps가 조금 떨어질 수 있다")
    ap.add_argument("--video-scale", type=float, default=0.5,
                    help="영상 축소 배율. 0.5면 화소 1/4 = 인코딩 비용 약 1/4 (기본 0.5)")
    ap.add_argument("--buffer-s", type=float, default=10.0,
                    help="투척 감지 전에 들고 있을 프레임 길이(초). Ctrl-C로 끊으면 이만큼 저장된다")
    ap.add_argument("--replay", nargs="+", metavar="JSON", help="저장된 관측으로 재실행")
    ap.add_argument("--span", type=float, default=None,
                    help="MIN_TIME_SPAN_S를 덮어써서 실행 (리플레이 비교용)")
    ap.add_argument("--ratio", type=float, default=None,
                    help="DEPTH_STABILITY_RATIO를 덮어써서 실행")
    args = ap.parse_args()

    if args.span is not None:
        config.MIN_TIME_SPAN_S = args.span
        print(f"[덮어씀] MIN_TIME_SPAN_S = {args.span}")
    if args.ratio is not None:
        config.DEPTH_STABILITY_RATIO = args.ratio
        print(f"[덮어씀] DEPTH_STABILITY_RATIO = {args.ratio}")

    throws = collect_replay(args.replay) if args.replay else collect_live(args)
    if throws:
        print_summary(throws)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
