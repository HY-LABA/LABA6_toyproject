"""로봇을 굴리면서 **배경(negative) 학습 데이터를 자동 수집한다.** (라파이에서 실행)

    python collect_live.py --tag lab_ceiling                 # 기본: 0.25m/s로 배회하며 수집
    python collect_live.py --tag hall --speed 0.3 --radius 1.5 --duration 240
    python collect_live.py --tag lab_ceiling --no-drive      # 로봇 세워두고 수집만

무엇을 만드는가
---------------
`dataset_raw/images/bg_<tag>_<시각>/*.jpg` 와 같은 이름의 **빈 .txt** 를
`dataset_raw/labels/bg_<tag>_<시각>/` 에 만든다. `extract_background.py` 와 **완전히
같은 규약**이라 `prepare_dataset.py` -> `review_labels.py` -> `train_yolo.py` 가
그대로 이어진다. 수집 근거는 `dataset_raw/meta/bg_<tag>_<시각>.jsonl` 에 따로 남는다
(프레임마다 그때 YOLO가 뭘 어디서 잘못 잡았는지 — 나중에 오탐 분석용).

빈 라벨 = "이 사진에는 물체가 없다" 는 뜻이고, YOLO 는 이걸로 **오검출을 줄이는 법**을
배운다. 조명·에어컨·공유기에 "이건 쓰레기가 아니다" 라는 박스를 칠 필요가 없다 —
클래스가 `trash` 하나뿐이라 그런 박스는 존재할 수 없다.

★ 왜 로봇을 굴리는가
--------------------
배경 다양성이 목적이다. 카메라가 한 자리에 고정돼 있으면 천장 사진 300장이 사실상
같은 사진 한 장이고, 그러면 학습에 아무 신호도 안 준다. 바퀴로 위치를 옮겨가며 찍으면
같은 천장에서도 조명·환기구·배관이 화면 안에서 다른 위치·다른 각도로 들어온다.

주행은 `communication.SerialLink.send_command()` (12B 속도 명령) 를 쓴다 —
`teleop_test.py` 와 같은 경로다. 착지점 명령(16B)이 아니다: 여기서는 "어디로 가라"가
아니라 "그냥 이 속도로 굴러라"가 필요하기 때문이다.

★ 하드 네거티브가 진짜 목적이다
-------------------------------
무작위 천장 사진보다 **실제로 모델을 속인 프레임**이 훨씬 값지다. 그래서 두 종류를
나눠 모은다:

    hard_negative   YOLO가 뭔가를 검출한 프레임 (= 전부 오탐. 이게 핵심)
    background      아무것도 검출 안 된 프레임 (분포를 채워주는 역할)

meta.jsonl 에 어느 쪽인지, 그때 검출 좌표가 어디였는지가 남는다.

⚠ 몇 장을 모을 것인가 — **전체 데이터셋의 0~10%**
--------------------------------------------------
Ultralytics 권장치다(COCO는 1%). 그 이상 넣으면 물체를 배우는 신호가 묽어진다.
지금 데이터셋이 약 1,200장이므로 **한 번에 100~150장이면 충분하다** — `--max-frames`
기본값 150은 그 근거로 잡았다. "많이 넣을수록 오탐이 준다"가 아니다.

천장/장소마다 `--tag` 를 다르게 줄 것. `prepare_dataset.py` 가 **세션 단위로**
train/val/test 를 나누므로, 배경 세션이 하나뿐이면 그 전부가 한쪽 split 에만 들어간다.

⚠ 진짜 쓰레기가 섞여 들어가면 데이터가 오염된다
------------------------------------------------
낙하 중인 물체가 있는 프레임을 빈 라벨로 저장하면 **"쓰레기는 배경이다"** 라고 가르치는
셈이다. 그래서 후보를 바로 저장하지 않고 `--hold` 초만큼 붙들고 있다가, 그 앞뒤로
**화면에서 움직인 검출이 하나도 없었을 때만** 확정 저장한다. 판정 기준은
`config.MIN_TRACK_DISPLACEMENT_PX` — `tracker.py` 가 정적 오탐을 거를 때 쓰는 바로
그 값이고, **이미지 좌표 기준이라 오도메트리가 없어도 성립한다.**

그래도 수집 중에는 아무도 던지지 말 것. 위 장치는 실수를 막는 안전망이지 면허가 아니다.

⚠ 안전 — 장애물 감지가 없다
---------------------------
`docs/open-questions.md` 대로 이 시스템은 바닥이 비어 있다고 가정한다. 이 스크립트도
마찬가지다. 주변을 치우고, 사람이 옆에서 지켜보는 상태에서만 돌릴 것.

  · `--radius` 안에 머무른다 (오도메트리가 올 때만. 넘어가면 원점 쪽으로 되돌아온다)
  · `--duration` 초가 지나면 자동 종료
  · Ctrl+C 로 언제든 정지. 어느 경로로 끝나도 STOP 을 두 번 보낸다
  · 스크립트가 죽어도 피코 워치독(`config.DRIVE_TIMEOUT_S`)이 알아서 세운다
  · 가감속은 `--accel` 로 완만하게 준다 — 급가속은 바퀴를 미끄러뜨리고, 슬립은
    오도메트리를 오염시킨다 (`docs/open-questions.md`)

아직 안 하는 것
---------------
**투척(positive) 자동 수집은 여기 없다.** 그건 링 버퍼로 프레임을 붙들고 있다가
`TrackerPool` 이 궤적을 채택한 뒤 소급해서 저장하는 구조라 별개의 작업이고, 이 파일에
반쯤 만들어 두면 검증 안 된 코드가 섞인다. 지금은 배경 수집만 확실히 한다.
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import queue
import random
import sys
import threading
import time
from collections import deque
from datetime import datetime

# pi5/ 를 import 경로에 넣는다 — 이 파일은 pi5/prep/ 에 있고 config·vision·tracker 는
# 그 상위에 있다. (vision.py 가 prep/camera.py 를 import 하는 것과 반대 방향이다.)
_PI5 = pathlib.Path(__file__).resolve().parent.parent
if str(_PI5) not in sys.path:
    sys.path.insert(0, str(_PI5))

import config              # noqa: E402
import control             # noqa: E402

ROOT = pathlib.Path("dataset_raw")     # 다른 prep 도구와 같은 cwd 기준 상대경로


# ── 저장 (별도 스레드) ─────────────────────────────────────────────────────

class Writer:
    """이미지·라벨 쓰기를 **캡처 루프 밖으로** 뺀다.

    `capture_video.py` 가 "매 프레임 검출+저장을 캡처 루프 안에서 동기로 하면 그 처리
    시간만큼 프레임을 놓친다"고 지적한 그 문제다. 루프는 큐에 넣기만 하고 즉시 돌아온다.
    큐가 차면 기다리지 않고 **버리고 센다** — 배경 사진 몇 장 잃는 게 낫다.
    """

    def __init__(self, img_dir: pathlib.Path, lbl_dir: pathlib.Path,
                 meta_path: pathlib.Path, quality: int = 95) -> None:
        self.img_dir = img_dir
        self.lbl_dir = lbl_dir
        self.quality = quality
        img_dir.mkdir(parents=True, exist_ok=True)
        lbl_dir.mkdir(parents=True, exist_ok=True)
        meta_path.parent.mkdir(parents=True, exist_ok=True)
        self._meta = meta_path.open("a", encoding="utf-8")
        self.q: queue.Queue = queue.Queue(maxsize=8)
        self.written = 0
        self.dropped = 0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, name: str, frame, record: dict,
              label: tuple[float, float, float, float] | None = None) -> bool:
        """`label=None`이면 빈 라벨(배경). `(cx,cy,w,h)`(0~1 정규화)를 주면 그 한 줄을 쓴다
        (`collect_throws.py`가 투척 라벨을 쓸 때 씀)."""
        try:
            self.q.put_nowait((name, frame, record, label))
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def write_header(self, record: dict) -> None:
        self._meta.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._meta.flush()

    def _run(self) -> None:
        import cv2

        while True:
            item = self.q.get()
            if item is None:
                break
            name, frame, record, label = item
            try:
                cv2.imwrite(str(self.img_dir / f"{name}.jpg"), frame,
                            [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                # ⚠ 빈 파일이어야 하고, 파일이 없으면 안 된다 — prepare_dataset.py 가
                #   라벨 없는 이미지를 학습에서 제외하므로 배경/투척이 조용히 사라진다.
                if label is None:
                    text = ""
                else:
                    cx, cy, w, h = label
                    text = f"0 {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}\n"
                (self.lbl_dir / f"{name}.txt").write_text(text, encoding="utf-8")
                self._meta.write(json.dumps(record, ensure_ascii=False) + "\n")
                self._meta.flush()
                self.written += 1
            except Exception as exc:  # noqa: BLE001 - 저장 실패로 수집을 멈추지 않는다
                print(f"[writer] {exc}")
            finally:
                self.q.task_done()

    def close(self) -> None:
        self.q.put(None)
        self._thread.join(timeout=30.0)
        self._meta.close()


# ── 주행 ───────────────────────────────────────────────────────────────────

class Roamer:
    """일정 속도로 배회한다. `--leg` 초마다 방향을 바꾸고, `--radius` 를 넘으면 돌아온다.

    회전(omega)은 프로토콜에 없으므로 **평행이동만** 한다 — 천장 그림이 화면 안에서
    미끄러지듯 옮겨갈 뿐 돌지는 않는다. 그것만으로도 조명·환기구가 화면의 다른 위치에
    걸리므로 배경 다양성 목적에는 충분하다.

    속도를 계단처럼 바꾸지 않고 `accel` 로 기울여 올린다. 급가속은 견인 한계를 넘어
    바퀴를 헛돌게 하고, 그 슬립이 오도메트리를 오염시킨다 (`docs/open-questions.md`).
    """

    def __init__(self, speed: float, leg_s: float, radius: float,
                 accel: float, rng: random.Random) -> None:
        self.speed = speed
        self.leg_s = leg_s
        self.radius = radius
        self.accel = accel
        self.rng = rng
        angle = rng.uniform(0.0, 2.0 * math.pi)
        self._dir = (math.cos(angle), math.sin(angle))
        self._leg_until = 0.0
        self._returning = False
        self._v = (0.0, 0.0)          # 지금 명령 중인 속도 (램프 적용된 값)
        self._last_t: float | None = None

    def step(self, now: float, offset_xy: tuple[float, float] | None) -> tuple[float, float]:
        """이번 틱에 보낼 (vx, vy). `offset_xy` 는 시작점 기준 이동량(없으면 None)."""
        dt = 0.0 if self._last_t is None else max(0.0, now - self._last_t)
        self._last_t = now

        if offset_xy is not None:
            dist = math.hypot(*offset_xy)
            if dist > self.radius:
                self._returning = True
            elif dist < self.radius * 0.6:
                self._returning = False
        else:
            self._returning = False

        if self._returning and offset_xy is not None:
            # 원점 쪽으로. 경계에서 왔다갔다 하지 않게 반경 60% 안에 들어와야 푼다.
            # 정확히 원점을 겨누면 매번 같은 방사선을 되짚어서 배경이 겹치므로,
            # ±50도 안에서 흔들어 준다 (cos50도=0.64 이라 안쪽 성분은 충분히 남는다).
            home = math.atan2(-offset_xy[1], -offset_xy[0])
            home += math.radians(self.rng.uniform(-50.0, 50.0))
            self._dir = (math.cos(home), math.sin(home))
            self._leg_until = now + self.leg_s
        elif now >= self._leg_until:
            # 방향 전환. 100~260도 안에서만 고르면 "거의 같은 방향"이 안 나와서
            # 한쪽으로 계속 흘러가지 않는다.
            base = math.atan2(self._dir[1], self._dir[0])
            turn = math.radians(self.rng.uniform(100.0, 260.0))
            self._dir = (math.cos(base + turn), math.sin(base + turn))
            self._leg_until = now + self.leg_s

        want = (self._dir[0] * self.speed, self._dir[1] * self.speed)
        if dt <= 0.0:
            return self._v
        limit = self.accel * dt
        dvx, dvy = want[0] - self._v[0], want[1] - self._v[1]
        mag = math.hypot(dvx, dvy)
        if mag > limit and mag > 0.0:
            dvx, dvy = dvx * limit / mag, dvy * limit / mag
        self._v = (self._v[0] + dvx, self._v[1] + dvy)
        return self._v


# ── 오염 방지 ──────────────────────────────────────────────────────────────

def moving_detection_seen(pool) -> bool:
    """지금 화면에 **움직이는 검출**이 있나 — 즉 뭔가 날아가고 있나.

    ★ 이미지 좌표만 본다. 오도메트리가 없어도, 궤적 피팅이 실패해도 성립한다.
      기준값은 `tracker.Tracker.add()` 가 정적 오탐을 거를 때 쓰는 것과 같은
      `config.MIN_TRACK_DISPLACEMENT_PX` 다 — 실측 대조가 이미 되어 있는 값이라
      (오탐 2~7px vs 실제 낙하물 107~292px) 여유가 크다.
    """
    if pool.committed is not None:
        return True
    for tr in pool.tracks:
        if len(tr.uvs) < 2:
            continue
        us = [uv[0] for uv in tr.uvs]
        vs = [uv[1] for uv in tr.uvs]
        if math.hypot(max(us) - min(us), max(vs) - min(vs)) >= config.MIN_TRACK_DISPLACEMENT_PX:
            return True
    return False


# ── 본체 ───────────────────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tag", required=True,
                    help="천장/장소 이름. 세션 폴더 이름이 되고 prepare_dataset.py 의 "
                         "분할 단위가 된다. 장소마다 다르게 줄 것")

    g = ap.add_argument_group("주행")
    g.add_argument("--speed", type=float, default=0.25,
                   help=f"주행 속력(m/s). 로봇 이론상한은 {config.ROBOT_MAX_SPEED_MPS} "
                        f"이고 아직 실측 전이니 낮게 시작할 것 (기본 0.25)")
    g.add_argument("--leg", type=float, default=4.0, help="방향을 유지하는 시간(초)")
    g.add_argument("--radius", type=float, default=1.2,
                   help="시작점에서 벗어날 수 있는 최대 거리(m). 오도메트리가 올 때만 동작. "
                        "⚠ 경계에서 즉시 서는 게 아니라 --accel 로 되돌아오므로 실제 최대 "
                        "반경은 radius + v^2/(2*accel) 이다 (기본값이면 1.2+0.06=1.26m)")
    g.add_argument("--accel", type=float, default=0.5,
                   help="속도 변화율 상한(m/s^2). 급가속은 슬립을 만든다")
    g.add_argument("--no-drive", action="store_true",
                   help="로봇을 안 움직이고 수집만 (시리얼도 안 연다)")
    g.add_argument("--port", default=config.SERIAL_PORT, help="피코 시리얼 포트")

    c = ap.add_argument_group("수집")
    c.add_argument("--duration", type=float, default=180.0, help="최대 수집 시간(초)")
    c.add_argument("--max-frames", type=int, default=150,
                   help="이 장수를 채우면 종료. 전체 데이터셋의 10%%를 넘기지 말 것")
    c.add_argument("--interval", type=float, default=3.0,
                   help="검출이 없어도 이 간격으로 한 장씩 (배경 분포 채우기)")
    c.add_argument("--min-gap", type=float, default=1.0,
                   help="저장 사이 최소 간격(초). 같은 사진이 연달아 쌓이는 걸 막는다")
    c.add_argument("--min-move", type=float, default=0.15,
                   help="직전 저장 이후 이만큼(m) 움직였어야 또 저장한다 "
                        "(오도메트리가 올 때만. 0이면 끔)")
    c.add_argument("--hold", type=float, default=1.5,
                   help="후보를 붙들고 있다가 이 시간 뒤에 확정 저장한다. 그 사이 "
                        "움직이는 검출이 보이면 버린다 (오염 방지)")
    c.add_argument("--quality", type=int, default=95, help="JPEG 품질")
    args = ap.parse_args()

    if args.speed <= 0 and not args.no_drive:
        ap.error("--speed 는 양수여야 한다 (안 움직이려면 --no-drive)")
    if args.speed > config.ROBOT_MAX_SPEED_MPS:
        print(f"⚠ --speed {args.speed} 가 이론 상한 {config.ROBOT_MAX_SPEED_MPS} 보다 크다.")

    # ── 세션 준비 ─────────────────────────────────────────────────────────
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session = f"bg_{args.tag}_{stamp}"
    writer = Writer(ROOT / "images" / session, ROOT / "labels" / session,
                    ROOT / "meta" / f"{session}.jsonl", args.quality)
    writer.write_header({
        "header": True,
        "session": session,
        "created": datetime.now().isoformat(timespec="seconds"),
        "kind": "background",
        "args": vars(args),
        "config": {k: getattr(config, k) for k in (
            "CAMERA_RESOLUTION", "CAMERA_FPS", "YOLO_CONF_THRESHOLD", "YOLO_IMGSZ",
            "MIN_TRACK_DISPLACEMENT_PX", "DRIVE_TIMEOUT_S")},
    })
    print(f"세션: {(ROOT / 'images' / session).resolve()}")

    # ── 시리얼 (주행 + 오도메트리) ────────────────────────────────────────
    link = None
    if not args.no_drive:
        try:
            import communication
            link = communication.SerialLink(port=args.port)
            print(f"피코 연결: {args.port}")
        except Exception as exc:  # noqa: BLE001 - 시리얼이 없어도 수집은 계속한다
            print(f"⚠ 시리얼을 못 열었다 ({exc}) — 로봇을 세워둔 채로 수집만 한다.")
            link = None

    # ── 카메라 · 추적기 ───────────────────────────────────────────────────
    import tracker as tracker_mod
    import vision

    cam = vision.Camera(config.CAMERA_RESOLUTION, config.CAMERA_FPS)
    pool = tracker_mod.TrackerPool()
    roamer = Roamer(args.speed, args.leg, args.radius, args.accel, random.Random())

    odom_origin: tuple[float, float] | None = None
    odom_xy = (0.0, 0.0)
    pending: deque = deque()          # [(t, frame, kind, dets, offset)]
    last_saved_t = -1e9
    last_saved_at: tuple[float, float] | None = None
    last_bg_t = -1e9
    last_moving_t = -1e9
    idx = 0
    n_frames = 0
    slow_warned = False
    started = time.monotonic()

    print(f"수집 시작 — 최대 {args.duration:.0f}초 / {args.max_frames}장. Ctrl+C 로 정지.")
    if link is not None:
        print("⚠ 로봇이 움직인다. 주변을 치우고 지켜볼 것 (장애물 감지 없음).")

    try:
        while True:
            now_wall = time.monotonic()
            if now_wall - started >= args.duration:
                print("\n시간 종료.")
                break
            if writer.written >= args.max_frames:
                print("\n목표 장수 도달.")
                break

            # ── 캡처 + 검출 (구동 루프와 같은 경로) ──────────────────────
            dets, t, frame = vision.observe(cam)
            n_frames += 1

            # ── 오도메트리 ───────────────────────────────────────────────
            if link is not None:
                odom = link.try_receive_odometry()
                if odom is not None:
                    if odom_origin is None:
                        odom_origin = odom.xy
                    odom_xy = odom.xy
            offset = None if odom_origin is None else (
                odom_xy[0] - odom_origin[0], odom_xy[1] - odom_origin[1])

            # ── 주행 명령 ────────────────────────────────────────────────
            if link is not None:
                vx, vy = roamer.step(now_wall, offset)
                link.send_command(control.DriveCommand(
                    target_vx=vx, target_vy=vy, timeout_s=config.DRIVE_TIMEOUT_S))

            # ── 움직이는 검출 감시 (오염 방지용) ─────────────────────────
            #   ⚠ `reason` 이 왔는데 reset 을 안 하면 채택된 가설이 그대로 남아
            #     `moving_detection_seen` 이 영원히 True 가 되고, 그러면 아무것도
            #     저장되지 않는다. main.py 와 같은 방식으로 끊어준다 — 다만 여기선
            #     로봇이 표적을 쫓은 게 아니므로 COOLDOWN 은 건너뛴다.
            _track, _landing, reason = pool.step(dets, odom_xy, t)
            if moving_detection_seen(pool):
                last_moving_t = t
            if reason:
                pool.reset(t, reason, cooldown=False)

            # ── 후보 선정 ────────────────────────────────────────────────
            #   hard_negative 가 목적이고, background 는 분포를 채우는 보조다.
            kind = None
            if t - last_saved_t >= args.min_gap:
                if dets:
                    kind = "hard_negative"
                elif t - last_bg_t >= args.interval:
                    kind = "background"
            if kind is not None:
                moved_enough = True
                if args.min_move > 0 and offset is not None and last_saved_at is not None:
                    moved = math.hypot(offset[0] - last_saved_at[0],
                                       offset[1] - last_saved_at[1])
                    # 오래 기다렸으면 안 움직였어도 한 장은 받는다 — 벽에 붙어
                    # 제자리걸음일 때 아무것도 안 모으는 것보단 낫다.
                    moved_enough = moved >= args.min_move or (t - last_saved_t) >= args.interval * 3
                if moved_enough:
                    pending.append((t, frame.copy(), kind,
                                    [{"u": d.u, "v": d.v, "conf": d.confidence,
                                      "bbox": [float(v) for v in d.bbox]} for d in dets],
                                    offset))
                    last_saved_t = t
                    if kind == "background":
                        last_bg_t = t
                    if offset is not None:
                        last_saved_at = offset

            # ── 확정 저장 — hold 만큼 지났고, 그 앞뒤로 움직임이 없었을 때만 ──
            while pending and (t - pending[0][0]) >= args.hold:
                p_t, p_frame, p_kind, p_dets, p_off = pending.popleft()
                if abs(last_moving_t - p_t) <= args.hold:
                    # 그 시각 근처에 뭔가 날아갔다 — 진짜 쓰레기일 수 있으니 버린다.
                    continue
                idx += 1
                name = f"{session}_{idx:05d}"
                writer.submit(name, p_frame, {
                    "file": f"{name}.jpg", "t": p_t, "kind": p_kind,
                    "odom_offset": list(p_off) if p_off is not None else None,
                    "n_dets": len(p_dets), "dets": p_dets,
                })

            # ── 상태 표시 ────────────────────────────────────────────────
            if n_frames % 60 == 0:
                elapsed = now_wall - started
                fps = n_frames / elapsed if elapsed > 0 else 0.0
                where = f"{math.hypot(*offset):.2f}m" if offset is not None else "오도메트리 없음"
                print(f"  {elapsed:5.0f}s  {writer.written:4d}장 저장  "
                      f"{fps:4.1f}fps  이동 {where}  대기 {len(pending)}", flush=True)
                if link is not None and not slow_warned and fps > 0 and \
                        1.0 / fps > config.DRIVE_TIMEOUT_S * 0.8:
                    slow_warned = True
                    print(f"  ⚠ 루프가 느려 명령 간격이 워치독"
                          f"({config.DRIVE_TIMEOUT_S * 1000:.0f}ms)에 가깝다 — 로봇이 끊길 수 있다.")
    except KeyboardInterrupt:
        print("\n중단됨.")
    finally:
        # 어떤 경로로 빠져나가도 로봇은 세운다.
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

    print(f"\n저장 {writer.written}장"
          + (f" (큐가 차서 버린 것 {writer.dropped}장)" if writer.dropped else "")
          + f"  -> dataset_raw/images/{session}/")
    print("다음: python review_labels.py --session " + session
          + "   (섞여 들어간 물체가 없는지 훑어볼 것)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
