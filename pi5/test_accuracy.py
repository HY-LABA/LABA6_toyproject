"""궤적 예측 정확도 측정 — **로봇을 세워두고 예측만 기록한다.**

    python test_accuracy.py                    # 던지면서 실시간 측정 (기본)
    python test_accuracy.py --throws 10        # 10회 하고 종료
    python test_accuracy.py --no-measure       # 정답 입력 안 받고 수렴만 관찰
    python test_accuracy.py --replay logs/*.json        # 저장된 관측으로 재실행
    python test_accuracy.py --replay logs/*.json --span 0.25   # 파라미터 바꿔서 재실행

왜 main.py를 그냥 돌리지 않나
-----------------------------
`main.py`는 피코에 명령을 보내고 **로봇이 실제로 움직인다.** 그러면 최종 오차에
예측 오차와 제어 오차가 섞여서 "궤적 예측이 얼마나 정확한가"를 분리할 수 없다.
여기서는 로봇을 세워두고(오도메트리 고정 (0,0)) **예측만** 기록한다.

쓰는 코드는 실제 구동과 완전히 같다 — `vision.detect_all` → `tracker.TrackerPool`
→ `trajectory.fit_trajectory`. 달라지는 건 피코로 명령을 안 보낸다는 것뿐이다.

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
    print(f"   프레임 전체 시간 스팬 {span:.2f}s (MIN_TIME_SPAN_S={config.MIN_TIME_SPAN_S})")
    if span < config.MIN_TIME_SPAN_S:
        print("   → 물체가 화면에 있던 시간이 최소 스팬보다 짧다. 더 높이 던지거나 "
              "MIN_TIME_SPAN_S를 낮출 것.")
    else:
        print("   → 스팬은 충분하다. 검출이 끊겨(TRACK_MAX_GAP_S 초과) 가설이 죽었거나, "
              "fit.ok(잔차·조건수·깊이범위)를 못 넘었을 가능성이 크다.")


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

def collect_live(args) -> list[Throw]:
    import vision                                   # 여기서 import — PC에서도 --replay는 되게

    cam = vision.Camera(config.CAMERA_RESOLUTION, config.CAMERA_FPS)
    throws: list[Throw] = []
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")

    print(f"카메라 열림 {config.CAMERA_RESOLUTION[0]}x{config.CAMERA_RESOLUTION[1]}"
          f"@{config.CAMERA_FPS}. 로봇은 **세워둘 것** (예측 오차만 재는 중).")
    print("던지면 자동으로 감지해서 기록한다. Ctrl-C로 종료.\n")

    try:
        while args.throws is None or len(throws) < args.throws:
            throw = Throw(len(throws) + 1)
            pool = tracker_mod.TrackerPool()
            odom = (0.0, 0.0)
            armed = False                            # 물체를 실제로 잡기 시작했나

            print(f"── 투척 {throw.index} 대기 중… ──")
            while True:
                dets, t = vision.observe(cam)
                _track, landing, reason = pool.step(dets, odom, t)
                if landing is not None:
                    armed = True

                # 채택 전 프레임도 들고 있어야 리플레이가 같은 결과를 낸다.
                # 다만 대기가 길어지면 무한정 쌓이므로 채택 전에는 최근 것만 남긴다.
                throw.record_frame(t, dets)
                if not armed and len(throw.frames) > config.TRACK_WINDOW * 3:
                    throw.frames.pop(0)

                if reason and armed:
                    throw.end_reason = reason
                    break

            run_throw(throw)                         # 저장된 프레임으로 다시 풀어 일관성 확보
            print_throw(throw)
            if not args.no_measure:
                ask_ground_truth(throw)

            path = LOG_DIR / f"{stamp}_throw{throw.index:02d}.json"
            path.write_text(json.dumps(throw.to_json(), indent=1, ensure_ascii=False),
                            encoding="utf-8")
            print(f"   저장: {path}")
            throws.append(throw)
    except KeyboardInterrupt:
        print("\n중단됨.")
    finally:
        cam.close()
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
