"""로봇을 굴리면서 **투척(positive) 학습 데이터를 자동 수집한다.** (라파이에서 실행)

    python collect_throws.py --tag lab_a                     # 로밍하며 계속 던지기
    python collect_throws.py --tag lab_a --no-drive          # 로봇 세워두고 수집만
    python collect_throws.py --tag lab_a --duration 300

무엇을 만드는가
---------------
`collect_live.py`(배경/하드네거티브)의 짝이다. 여기는 **양성(진짜 쓰레기) 라벨**을 만든다.

    dataset_raw/images/throw_<tag>_<시각>/*.jpg
    dataset_raw/labels/throw_<tag>_<시각>/*.txt      ← "0 cx cy w h" (원본 픽셀 정규화)
    dataset_raw/meta/throw_<tag>_<시각>.jsonl        ← 트랙별 요약(잔차·깊이·|v0|·궤적점)

`extract_from_video.py`와 완전히 같은 출력 규약이라 뒷단(`prepare_dataset.py` ->
`review_labels.py` -> `train_yolo.py`)은 안 건드려도 이어진다.

★ 라벨을 만드는 게 MOG2가 아니라 **물리 게이트**다
----------------------------------------------------
`extract_from_video.py`는 "이 픽셀 덩어리가 물체인가"를 MOG2로 판정했다. 그건 카메라가
고정이어야만 성립한다. 지금은 로봇이 계속 움직이니 MOG2를 못 쓴다.

대신 이미 구현돼 있는 훨씬 강한 판정자를 그대로 쓴다 — `tracker.TrackerPool`의 중력
제약 포물선 피팅. 정지한 전등은 어떤 포물선으로도 설명이 안 돼서 저절로 탈락하고,
카메라가 움직여도 `Tracker._cam_xy()`가 오도메트리로 보정하므로 성립한다
(`algorithm.md` 2장). **이 스크립트는 판정 로직을 한 줄도 새로 안 짠다** — `main.py`가
쓰는 것과 완전히 같은 `pool.step()`을 그대로 부른다.

★ 왜 로봇을 굴리면서 던지나
---------------------------
배경 다양성이 목적이다(`collect_live.py`와 같은 이유). 같은 자리에서만 던지면 배경이
매번 같은 천장 한 조각이라 데이터 다양성이 안 늘어난다. `collect_live.Roamer`를 그대로
가져다 써서 일정 속도로 배회하고, 판정만 물리에 맡긴다.

⚠ **이 스크립트는 로봇을 표적 쪽으로 몰지 않는다.** `main.py`처럼 착지점을 계산해
피코에 목표점(16B)을 보내는 게 아니라, `Roamer`가 정하는 방향으로 **계속** 굴러간다
(12B 속도 명령). 궤적 예측은 라벨을 만드는 데만 쓰이지 로봇을 조종하지 않는다 —
그래서 "잡을 수 있는가"(`tracker._reach`)는 여기서 아예 안 본다.

★ 왜 링 버퍼 + 소급 저장인가
-----------------------------
어떤 트랙이 "진짜"라고 확정되는 시점은 물체가 나타나고 최소 `MIN_TIME_SPAN_S`(스팬)가
지난 뒤다. 그때는 이미 초반 프레임이 지나가버렸으므로, 프레임을 미리 원형 버퍼에
쌓아두고 트랙이 확정되면 **그 트랙의 시간 구간을 소급해서** 꺼낸다.

버퍼는 원본 해상도 numpy 배열을 그대로 든다(인코딩 없음) — 매 프레임 JPEG로 인코딩하면
그 비용이 캡처 루프를 막아 관측을 놓친다(`vision-pipeline.md`가 지적하는 그 문제).
인코딩은 실제로 저장하기로 정해진 소수의 프레임에 대해서만, `collect_live.Writer`가
백그라운드 스레드에서 한다.

버퍼 길이는 `--buffer-s`(기본 `config.CYCLE_TIMEOUT_S`와 동일, 2.0초)만큼 최근 프레임을
든다. 60fps 기준 프레임당 약 4.75MB(1456x1088x3)이므로 2.0초 = 약 120장 = **약 570MB**다.
파이5 메모리가 빠듯하면 `--buffer-s`를 줄일 것 — 실제 비행시간은 대개 1초 미만이라
(`algorithm.md` 4장 실측 표) 1.2~1.5초로 줄여도 대부분의 투척은 커버된다. 부족하면
`--buffer-s`가 실제로 부족했던 트랙 수를 종료 시 요약에서 알려준다.

★ 확정 판정과 "깜빡임" 문제
----------------------------
`tracker.Tracker.add()`는 매 프레임 `self.fit`을 **일단 None으로 리셋한 뒤** 그 프레임의
게이트를 통과해야만 다시 채운다(`tracker.py` `add()` 본문). 그 말은 한 번 유효했던
트랙도 다음 프레임에 노이즈로 깊이 수렴 판정이 잠깐 실패하면 `tr.fit`이 다시 `None`으로
돌아갈 수 있다는 뜻이다 — 트랙이 그 프레임 직후 유실돼 사라지면 "확정된 적 있었다"는
사실을 놓친다.

그래서 순간값(`tr.fit is not None`)이 아니라 **"이 트랙이 살아있는 동안 한 번이라도
`fit`이 나온 적 있는가"**를 별도로 기록한다(`confirmed` 집합). 소멸 시점엔 이 기록을
본다.

무엇을 저장하는가
------------------
확정된 트랙마다, 그 트랙에 배정됐던 **모든 관측 프레임**(fit 통과 여부와 무관하게 —
확정 전 초반 프레임도 진짜 물체를 본 게 맞으므로 라벨로 유효하다) 중에서 균등하게
`--per-track`장(기본 12, `vision-pipeline.md` 3장 권장 10~15장과 일치)을 뽑는다.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import random
import sys
import time
from collections import deque
from datetime import datetime

_PI5 = pathlib.Path(__file__).resolve().parent.parent
if str(_PI5) not in sys.path:
    sys.path.insert(0, str(_PI5))
_PREP = pathlib.Path(__file__).resolve().parent
if str(_PREP) not in sys.path:
    sys.path.insert(0, str(_PREP))

import config              # noqa: E402
import control              # noqa: E402
from collect_live import Roamer, Writer   # noqa: E402  — 배회 로직·저장 로직을 공유한다

ROOT = pathlib.Path("dataset_raw")


# ── 링 버퍼 ─────────────────────────────────────────────────────────────

class FrameBuffer:
    """최근 `window_s`초의 (t, frame)을 든다. 인코딩 없이 원본 배열 그대로.

    조회는 시각으로 한다 — `Tracker.times`에 저장된 값과 **같은 실수**가 그대로
    들어오므로(별도로 반올림·재계산하지 않는다) 사전(dict) 정확 일치로 충분하다.
    `vision.Camera.capture()`가 매 호출 `.copy()`된 새 배열을 주므로(`prep/camera.py`
    `read()`) 여기서 또 복사할 필요가 없다.
    """

    def __init__(self, window_s: float) -> None:
        self.window_s = window_s
        self._order: deque = deque()      # [t, ...] 오래된 것이 왼쪽
        self._frames: dict = {}           # t -> frame

    def push(self, t: float, frame) -> None:
        self._order.append(t)
        self._frames[t] = frame
        cutoff = t - self.window_s
        while self._order and self._order[0] < cutoff:
            old = self._order.popleft()
            self._frames.pop(old, None)

    def get(self, t: float):
        return self._frames.get(t)

    @property
    def span_s(self) -> float:
        return (self._order[-1] - self._order[0]) if self._order else 0.0


# ── 균등 샘플링 ─────────────────────────────────────────────────────────

def pick_evenly(n_total: int, n_pick: int) -> list[int]:
    """0..n_total-1 중 n_pick개를 최대한 고르게 뽑은 인덱스(정렬됨, 중복 없음).

    처음·마지막을 포함해서 궤적 전체를 커버한다(낙하 구간 균등 샘플 — `vision-pipeline.md`
    3장).
    """
    if n_total <= n_pick:
        return list(range(n_total))
    if n_pick <= 1:
        return [0]
    return sorted({round(i * (n_total - 1) / (n_pick - 1)) for i in range(n_pick)})


# ── 트랙 하나를 라벨로 확정 ─────────────────────────────────────────────

class TrackRecorder:
    """살아있는 트랙마다 "배정된 관측(시각, 원본 bbox)"을 곁에서 따로 쌓는다.

    `tracker.Tracker`는 왜곡 보정 **후** 중심(u,v)만 갖고 있어서 YOLO 라벨(원본 픽셀
    기준)을 못 만든다. 그래서 매 프레임 이 프레임의 원본 검출(`dets`) 중 어느 것이
    이 트랙에 배정됐는지를 `tr.uvs[-1]`(방금 막 붙은 보정후 좌표)과 값 비교로 되짚어
    찾는다 — `Tracker.add()`가 `det.u`/`det.v`를 그대로(재계산 없이) 저장하므로 실수
    정확 일치가 성립한다.
    """

    def __init__(self) -> None:
        self.assigned: dict = {}     # id(tr) -> [(t, bbox), ...]
        self.confirmed: set = set()  # id(tr) — 살아있는 동안 fit이 한 번이라도 나온 것들
        self.last_fit: dict = {}     # id(tr) -> tr.fit (마지막으로 성공했을 때 값)
        self.known: dict = {}        # id(tr) -> tr (소멸 후에도 참조하려고 들고 있음)

    def observe(self, pool, dets, t: float) -> None:
        for tr in pool.tracks:
            key = id(tr)
            self.known[key] = tr
            if tr.times and tr.times[-1] == t and tr.uvs:
                u, v = tr.uvs[-1]
                for d in dets:
                    if d.u == u and d.v == v:
                        self.assigned.setdefault(key, []).append((t, d.bbox))
                        break
            if tr.fit is not None:
                self.confirmed.add(key)
                self.last_fit[key] = tr.fit

    def pop_finalizable(self, ids) -> list:
        """넘겨준 id들 중 확정된 것만 (tr, frames, fit) 목록으로 꺼내고 내부 기록은 지운다."""
        out = []
        for key in ids:
            tr = self.known.pop(key, None)
            frames = self.assigned.pop(key, [])
            fit = self.last_fit.pop(key, None)
            was_confirmed = key in self.confirmed
            self.confirmed.discard(key)
            if tr is not None and was_confirmed and frames:
                out.append((tr, frames, fit))
        return out

    def pop_all(self) -> list:
        """지금 알고 있는 트랙 전부(사이클 종료 시 — `pool.reset()`이 다 지우기 전에 부를 것)."""
        return self.pop_finalizable(list(self.known.keys()))


# ── 본체 ───────────────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tag", required=True,
                    help="장소 이름. 세션 폴더 이름이 되고 prepare_dataset.py 의 "
                         "분할 단위가 된다")

    g = ap.add_argument_group("주행 — collect_live.py 와 동일 (Roamer 공유)")
    g.add_argument("--speed", type=float, default=0.25)
    g.add_argument("--leg", type=float, default=4.0)
    g.add_argument("--radius", type=float, default=1.2,
                   help="실제 최대 반경은 radius + speed^2/(2*accel) — collect_live.py 참고")
    g.add_argument("--accel", type=float, default=0.5)
    g.add_argument("--no-drive", action="store_true")
    g.add_argument("--port", default=config.SERIAL_PORT)

    c = ap.add_argument_group("수집")
    c.add_argument("--duration", type=float, default=300.0, help="최대 수집 시간(초)")
    c.add_argument("--buffer-s", type=float, default=None,
                   help="링 버퍼 길이(초). 기본은 config.CYCLE_TIMEOUT_S "
                        f"({config.CYCLE_TIMEOUT_S}s) — 트랙이 최대로 살아있을 수 있는 "
                        "시간과 맞춘 것. 메모리가 부족하면 줄일 것 (60fps*1456x1088x3 "
                        "≈ 4.75MB/프레임)")
    c.add_argument("--per-track", type=int, default=12,
                   help="트랙 하나당 저장할 장수 (vision-pipeline.md 권장 10~15)")
    c.add_argument("--min-observations", type=int, default=None,
                   help="이 미만으로 관측된 트랙은 확정돼도 저장 안 함 (노이즈성 짧은 "
                        "트랙 배제). 기본은 config.MIN_OBSERVATIONS")
    c.add_argument("--quality", type=int, default=95)
    args = ap.parse_args()

    buffer_s = args.buffer_s if args.buffer_s is not None else config.CYCLE_TIMEOUT_S
    min_obs = args.min_observations if args.min_observations is not None else config.MIN_OBSERVATIONS
    est_mb = buffer_s * config.CAMERA_FPS * (config.CAMERA_RESOLUTION[0]
                                             * config.CAMERA_RESOLUTION[1] * 3) / 1e6
    print(f"링 버퍼 {buffer_s:.1f}s ≈ {est_mb:.0f}MB 예상 (부족하면 --buffer-s로 줄일 것)")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session = f"throw_{args.tag}_{stamp}"
    writer = Writer(ROOT / "images" / session, ROOT / "labels" / session,
                    ROOT / "meta" / f"{session}.jsonl", args.quality)
    writer.write_header({
        "header": True, "session": session,
        "created": datetime.now().isoformat(timespec="seconds"),
        "kind": "throw",
        "args": vars(args),
        "config": {k: getattr(config, k) for k in (
            "CAMERA_RESOLUTION", "CAMERA_FPS", "YOLO_CONF_THRESHOLD", "YOLO_IMGSZ",
            "MIN_OBSERVATIONS", "MIN_TIME_SPAN_S", "MAX_RESIDUAL_PX",
            "DEPTH_STABILITY_RATIO", "MAX_LAUNCH_SPEED_MPS", "MIN_TRACK_DISPLACEMENT_PX",
            "ASSOC_STEP_PX", "CYCLE_TIMEOUT_S", "TRACK_MAX_GAP_S")},
    })
    print(f"세션: {(ROOT / 'images' / session).resolve()}")

    link = None
    if not args.no_drive:
        try:
            import communication
            link = communication.SerialLink(port=args.port)
            print(f"피코 연결: {args.port}")
        except Exception as exc:  # noqa: BLE001
            print(f"⚠ 시리얼을 못 열었다 ({exc}) — 로봇을 세워둔 채로 수집만 한다.")
            link = None

    import tracker as tracker_mod
    import vision

    cam = vision.Camera(config.CAMERA_RESOLUTION, config.CAMERA_FPS)
    pool = tracker_mod.TrackerPool()
    roamer = Roamer(args.speed, args.leg, args.radius, args.accel, random.Random())
    buf = FrameBuffer(buffer_s)
    rec = TrackRecorder()

    odom_origin: tuple[float, float] | None = None
    odom_xy = (0.0, 0.0)
    track_seq = 0
    n_saved_tracks = 0
    n_saved_frames = 0
    n_buffer_misses = 0
    n_frames = 0
    started = time.monotonic()

    def finalize(tr, frames: list, fit) -> None:
        nonlocal track_seq, n_saved_tracks, n_saved_frames, n_buffer_misses
        if len(frames) < min_obs:
            return  # 너무 짧게 스쳐간 것 — 노이즈일 가능성이 커서 배제
        track_seq += 1
        track_id = f"{track_seq:04d}"
        idxs = pick_evenly(len(frames), args.per_track)
        centers, files = [], []
        H, W = None, None
        missing = 0
        for k, i in enumerate(idxs):
            t_i, bbox_i = frames[i]
            frame = buf.get(t_i)
            if frame is None:
                missing += 1
                continue
            if H is None:
                H, W = frame.shape[:2]
            cx, cy, w, h = bbox_i
            name = f"{session}_trk{track_id}_{k:02d}"
            writer.submit(name, frame, {
                "file": f"{name}.jpg", "kind": "throw_frame", "track_id": track_id,
                "t": t_i, "bbox_px": [float(v) for v in bbox_i],
            }, label=(cx / W, cy / H, w / W, h / H))
            centers.append([cx / W, cy / H])
            files.append(f"{name}.jpg")
        if not files:
            return
        n_saved_tracks += 1
        n_saved_frames += len(files)
        n_buffer_misses += missing
        span = frames[-1][0] - frames[0][0]
        writer.write_header({
            "kind": "track", "track_id": track_id, "session": session,
            "n_observed": len(frames), "n_saved": len(files), "n_buffer_missed": missing,
            "span_s": span, "files": files, "centers": centers,
            "fit": None if fit is None else {
                "n": fit.n, "residual_px": fit.residual_px, "z_m": float(fit.p0[2]),
                "v0_mps": float((fit.v0[0] ** 2 + fit.v0[1] ** 2 + fit.v0[2] ** 2) ** 0.5),
                "condition": fit.condition,
            },
        })
        if missing:
            print(f"  트랙 {track_id}: {missing}/{len(idxs)}장이 버퍼에서 이미 밀려나 "
                  f"못 건짐 — 필요하면 --buffer-s를 늘릴 것 (이 트랙 관측 스팬 {span:.2f}s)")

    print(f"수집 시작 — 최대 {args.duration:.0f}초. 로봇이 굴러가는 동안 던질 것. Ctrl+C로 정지.")
    if link is not None:
        print("⚠ 로봇이 움직인다. 주변을 치우고 지켜볼 것 (장애물 감지 없음).")

    try:
        while True:
            now_wall = time.monotonic()
            if now_wall - started >= args.duration:
                print("\n시간 종료.")
                break

            dets, t, frame = vision.observe(cam)
            buf.push(t, frame)
            n_frames += 1

            if link is not None:
                odom = link.try_receive_odometry()
                if odom is not None:
                    if odom_origin is None:
                        odom_origin = odom.xy
                    odom_xy = odom.xy
            if link is not None:
                offset = None if odom_origin is None else (
                    odom_xy[0] - odom_origin[0], odom_xy[1] - odom_origin[1])
                vx, vy = roamer.step(now_wall, offset)
                link.send_command(control.DriveCommand(
                    target_vx=vx, target_vy=vy, timeout_s=config.DRIVE_TIMEOUT_S))

            pre_ids = {id(tr) for tr in pool.tracks}
            _track, _landing, reason = pool.step(dets, odom_xy, t)
            rec.observe(pool, dets, t)

            # ── 개별 트랙 소멸(유실 등) — pool이 조용히 지운 것들 ────────
            post_ids = {id(tr) for tr in pool.tracks}
            for tr, frames, fit in rec.pop_finalizable(pre_ids - post_ids):
                finalize(tr, frames, fit)

            # ── 사이클 종료 — reset() 이 전부 지우기 전에 먼저 거둔다 ────
            if reason:
                for tr, frames, fit in rec.pop_all():
                    finalize(tr, frames, fit)
                # 명령을 보낸 적이 없으므로(로밍만 함) COOLDOWN 불필요.
                pool.reset(t, reason, cooldown=False)

            if n_frames % 120 == 0:
                elapsed = now_wall - started
                fps = n_frames / elapsed if elapsed > 0 else 0.0
                print(f"  {elapsed:5.0f}s  트랙 {n_saved_tracks}개 / 프레임 {n_saved_frames}장 저장  "
                      f"{fps:4.1f}fps  버퍼 {buf.span_s:.1f}s", flush=True)
    except KeyboardInterrupt:
        print("\n중단됨.")
    finally:
        # 진행 중이던 트랙도 거둔다 — 중간에 멈춰도 데이터가 버려지지 않게.
        for tr, frames, fit in rec.pop_all():
            finalize(tr, frames, fit)
        if link is not None:
            for _ in range(2):
                try:
                    link.send_command(control.STOP)
                except Exception as exc:  # noqa: BLE001
                    print(f"정지 명령 실패: {exc}")
                time.sleep(0.05)
            link.close()
        cam.close()
        writer.close()

    print(f"\n트랙 {n_saved_tracks}개 / 프레임 {n_saved_frames}장 저장"
          + (f", 버퍼 부족으로 놓친 것 {n_buffer_misses}장" if n_buffer_misses else "")
          + f"  -> dataset_raw/images/{session}/")
    print("다음: python review_labels.py --by-track --session " + session)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
