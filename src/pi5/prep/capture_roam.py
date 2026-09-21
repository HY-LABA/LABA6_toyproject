"""로봇을 굴리면서 물체 던지는 걸 영상으로만 녹화한다 — 검출/라벨링은 나중에 따로.

    python capture_roam.py --seconds 120                       # 기본 0.25m/s로 배회하며 120초 녹화
    python capture_roam.py --seconds 180 --speed 0.3 --radius 1.5
    python capture_roam.py --seconds 60 --no-drive             # 로봇 세워두고 녹화만 (capture_video.py --continuous와 동일 효과)

★ 왜 이 파일이 필요한가 — collect_throws.py 의 한계
----------------------------------------------------
`collect_throws.py`는 `TrackerPool`의 **물리 피팅(궤적 fit)이 성공해야만** 그 던지기를
저장한다(`trajectory.Fit.ok`). 이건 정확한 라벨을 자동으로 얻으려는 대가로 **꽤 많은
진짜 투척을 버린다** — 궤적 피팅 조건(`Z_RANGE_M`, `MAX_CONDITION`, `MAX_RESIDUAL_PX`,
`MAX_LAUNCH_SPEED_MPS`)을 하나라도 못 채우면(예: 손 근처 프레임 오염, 관측 수 부족,
잔차 초과) 실제로는 멀쩡히 던져서 화면에 다 찍혔어도 통째로 사라진다. 데이터 수집
속도가 "물리 게이트를 통과한 것"에 의존해서, 컷당하는(=버려지는) 던지기가 많으면
수집이 느려진다는 게 원래 문제였다.

이 파일은 그 문제를 **캡처 단계에서 물리 게이트를 아예 없애서** 우회한다:
YOLO도 TrackerPool도 안 쓰고, `capture_video.py`처럼 Picamera2 하드웨어 인코더로
**무처리 녹화**만 한다 — 그래서 물리적으로 화면에 찍히기만 하면 100% 남는다.
대신 로봇은 `collect_live.py`의 `Roamer`로 계속 배회시킨다(피코에 보내는 명령은
지금 `collect_throws.py`/`collect_live.py`와 완전히 같은 12B 속도 명령이다 — 이
파일이 바뀐다고 피코 쪽 프로토콜이나 로봇 동작이 달라지지 않는다).

라벨은 나중에 `label_throws.py`로 이 영상을 오프라인(PC 권장 — `extract_from_video.py`와
같은 이유: 라파이 GUI가 불안정하다)으로 훑으면서 **사람이 직접** 시작점(구간 시작)을
찍고 프레임마다 바운딩 박스를 그리고 끝점(물체가 사라지는 시점)을 찍어서 만든다.
자동 물리 게이트가 없으니 "던지긴 했는데 피팅이 안 돼서 버려지는" 일 자체가 없다 —
대가는 라벨링이 자동이 아니라 수동이라는 것뿐이다.

⚠ `collect_throws.py`를 대체하는 게 아니라 **보완**이다
--------------------------------------------------------
`collect_throws.py`의 물리 게이트 자동 라벨링은 여전히 유효하고 더 빠르다(사람 손이
안 든다) — 게이트를 통과하는 "쉬운" 투척(궤적이 깨끗한 경우)은 그쪽으로 계속 모으는
게 낫다. 이 파일은 **게이트에 자주 걸리는 어려운 케이스**(카메라 경계 근처, 빠른
투척, 관측이 짧은 경우 등)를 놓치지 않고 데이터셋에 넣기 위한 우회로다. 두 도구를
같이 쓰면 된다.

★ 왜 로봇 주행과 영상 녹화가 서로 안 부딪히는가
------------------------------------------------
`cam.start_recording()`은 Picamera2 하드웨어 인코더 경로라 Python 루프와 완전히
별개로 돌아간다(`camera.py` 참고) — `capture_video.py`가 버저용 저해상도 스트림을
녹화 중에도 동시에 뽑아 쓸 수 있는 것과 같은 이유다. 그래서 이 파일은 녹화를 켜놓은
채로 그 위에서 `Roamer.step()` + 오도메트리 폴링 + 주행 명령 전송을 별도 루프로
그냥 돌리면 된다 — 녹화 프레임 유실 위험이 없다.

⚠ 코덱 기본값 mjpeg인 이유는 capture_video.py와 동일: 파이5는 하드웨어 H.264
  인코더가 없다. ffmpeg 설치 필요 (`sudo apt install ffmpeg`).
⚠ 안전 — 장애물 감지가 없다 (collect_live.py와 동일한 주의사항):
  · `--radius` 안에 머무른다 (오도메트리가 올 때만)
  · Ctrl+C 로 언제든 정지, 어느 경로로 끝나도 STOP 을 두 번 보낸다
  · 스크립트가 죽어도 피코 워치독(`config.DRIVE_TIMEOUT_S`)이 알아서 세운다
  · 주변을 치우고 사람이 옆에서 지켜보는 상태에서만 돌릴 것

출력: capture_sessions/session_YYYYMMDD_HHMMSS/clips/roam.mp4 + session.json
      (session.json에 카메라 설정 + 이번 배회 파라미터가 같이 남는다)
다음 단계: python label_throws.py capture_sessions/session_YYYYMMDD_HHMMSS
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import random
import sys
import time
from datetime import datetime

import numpy as np

# pi5/ 를 import 경로에 넣는다 — collect_live.py 와 같은 이유
# (config·control·communication 은 이 파일의 상위 폴더에 있다).
_PI5 = pathlib.Path(__file__).resolve().parent.parent
if str(_PI5) not in sys.path:
    sys.path.insert(0, str(_PI5))

import config              # noqa: E402
import control             # noqa: E402

import camera as camlib               # noqa: E402  (같은 prep/ 폴더, capture_video.py와 동일)
from collect_live import Roamer       # noqa: E402  (재사용 — 새로 안 만든다)
from capture_video import beep, _check_fps  # noqa: E402  (재사용)

ROOT = pathlib.Path("capture_sessions")   # capture_video.py 와 같은 출력 루트


def _roam_and_record(cam, link, roamer: Roamer | None, seconds: float,
                     buzzer: bool, buzzer_threshold: float,
                     drive_hz: float = 50.0, check_hz: float = 15.0) -> dict:
    """녹화 시간(`seconds`)만큼 배회 명령을 계속 보내면서, buzzer가 켜져 있으면
    저해상도 스트림으로 움직임을 체크해 터미널 벨을 울린다.

    ★ 녹화 자체(`cam.start_recording()`)는 호출자가 이미 켜놓은 상태로 들어온다 —
    여기서는 그 위에서 도는 "주행 + 버저" 루프만 담당한다(capture_video.py의
    `_record_with_buzzer`가 하던 일 + collect_live.py의 메인 루프에서 주행 부분만
    뽑아 합친 것).
    """
    drive_period = 1.0 / drive_hz
    buzzer_period = 1.0 / check_hz
    t_end = time.monotonic() + seconds

    odom_origin: tuple[float, float] | None = None
    odom_xy = (0.0, 0.0)
    offset: tuple[float, float] | None = None
    bg = None
    was_moving = False
    last_buzzer_t = 0.0
    n_ticks = 0
    slow_warned = False

    while time.monotonic() < t_end:
        tick0 = time.monotonic()
        now = tick0

        if link is not None and roamer is not None:
            odom = link.try_receive_odometry()
            if odom is not None:
                if odom_origin is None:
                    odom_origin = odom.xy
                odom_xy = odom.xy
            offset = None if odom_origin is None else (
                odom_xy[0] - odom_origin[0], odom_xy[1] - odom_origin[1])
            vx, vy = roamer.step(now, offset)
            link.send_command(control.DriveCommand(
                target_vx=vx, target_vy=vy, timeout_s=config.DRIVE_TIMEOUT_S))

        if buzzer and (now - last_buzzer_t) >= buzzer_period:
            last_buzzer_t = now
            gray = cam.capture_lores().astype(np.float32)
            if bg is None:
                bg = gray
            diff = float(np.abs(gray - bg).mean())
            moving = diff > buzzer_threshold
            if moving and not was_moving:
                beep()
            was_moving = moving
            bg = bg * 0.9 + gray * 0.1

        n_ticks += 1
        elapsed = time.monotonic() - tick0
        if elapsed < drive_period:
            time.sleep(drive_period - elapsed)
        elif link is not None and not slow_warned and elapsed > config.DRIVE_TIMEOUT_S * 0.8:
            slow_warned = True
            print(f"  ⚠ 루프가 느려(한 틱 {elapsed*1000:.0f}ms) 명령 간격이 워치독"
                  f"({config.DRIVE_TIMEOUT_S*1000:.0f}ms)에 가깝다 — 로봇이 끊길 수 있다.")

    return {"n_ticks": n_ticks, "final_offset": list(offset) if offset is not None else None}


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    camlib.add_profile_arg(ap)
    ap.add_argument("--seconds", type=float, default=120.0,
                    help="녹화(=배회) 시간(초). 기본 120초")
    ap.add_argument("--tag", default="roam", help="클립 파일 이름에 붙는 태그")
    ap.add_argument("--bitrate", type=int, default=20_000_000)
    ap.add_argument("--codec", choices=["h264", "mjpeg"], default="mjpeg",
                    help="기본 mjpeg — capture_video.py와 같은 이유(파이5는 하드웨어 "
                         "H.264 인코더가 없다)")
    ap.add_argument("--buzzer", action=argparse.BooleanOptionalAction, default=True,
                    help="물체가 지나가는 동안 터미널 벨을 울린다. --no-buzzer로 끔")
    ap.add_argument("--buzzer-threshold", type=float, default=6.0)

    g = ap.add_argument_group("주행 (collect_live.py의 Roamer와 동일)")
    g.add_argument("--speed", type=float, default=0.25,
                   help=f"주행 속력(m/s). 이론상한 {config.ROBOT_MAX_SPEED_MPS} — 아직 "
                        f"실측 전이니 낮게 시작할 것 (기본 0.25)")
    g.add_argument("--leg", type=float, default=4.0, help="방향을 유지하는 시간(초)")
    g.add_argument("--radius", type=float, default=1.2,
                   help="시작점에서 벗어날 수 있는 최대 거리(m). 오도메트리가 올 때만 동작")
    g.add_argument("--accel", type=float, default=0.5, help="속도 변화율 상한(m/s^2)")
    g.add_argument("--no-drive", action="store_true",
                   help="로봇을 안 움직이고 녹화만 (capture_video.py --continuous와 동일 효과)")
    g.add_argument("--port", default=config.SERIAL_PORT, help="피코 시리얼 포트")
    args = ap.parse_args()

    if args.speed <= 0 and not args.no_drive:
        ap.error("--speed 는 양수여야 한다 (안 움직이려면 --no-drive)")
    if args.speed > config.ROBOT_MAX_SPEED_MPS:
        print(f"⚠ --speed {args.speed} 가 이론 상한 {config.ROBOT_MAX_SPEED_MPS} 보다 크다.")

    cam = camlib.open_camera(args.camera, exposure_us=args.exposure_us, gain=args.gain,
                              auto_lock=args.auto_lock_exposure,
                              max_exposure_us=args.max_exposure_us, max_gain=args.max_gain,
                              want_lores=args.buzzer)
    if cam.backend != "picamera2":
        print("[에러] 영상 녹화는 Picamera2 전용이다 (webcam 프로파일로는 안 됨).")
        cam.close()
        return 1

    link = None
    roamer = None
    if not args.no_drive:
        try:
            import communication
            link = communication.SerialLink(port=args.port)
            roamer = Roamer(args.speed, args.leg, args.radius, args.accel, random.Random())
            print(f"피코 연결: {args.port}  (속도 {args.speed}m/s, 반경 {args.radius}m)")
        except Exception as exc:  # noqa: BLE001 - 시리얼이 없어도 녹화는 계속한다
            print(f"⚠ 시리얼을 못 열었다 ({exc}) — 로봇을 세워둔 채로 녹화만 한다.")
            link = None

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session = f"session_{stamp}"
    outdir = ROOT / session
    clipdir = outdir / "clips"
    clipdir.mkdir(parents=True, exist_ok=True)
    name = f"{args.tag}.mp4"

    meta = {
        "created": datetime.now().isoformat(timespec="seconds"),
        "camera": args.camera,
        "exposure_us": cam.spec.exposure_us,
        "gain": cam.spec.gain,
        "resolution": [cam.spec.width, cam.spec.height],
        "fps": cam.spec.fps,
        "codec": args.codec,
        "clips": [{"file": name, "label": "trash", "mode": "roam", "seconds": args.seconds}],
        "drive": {
            "driven": link is not None,
            "speed_mps": args.speed, "leg_s": args.leg,
            "radius_m": args.radius, "accel_mps2": args.accel,
        },
        "labeling": "manual (label_throws.py) — 자동 물리 게이트 없음",
    }

    print(f"\n{args.seconds:.0f}초 녹화 시작" +
          (" — 로봇이 배회한다. 주변을 치우고 지켜볼 것 (장애물 감지 없음)." if link else " — 로봇은 세워둔 채.") +
          " 자유롭게 던지세요.\n")
    try:
        cam.start_recording(str(clipdir / name), bitrate=args.bitrate, codec=args.codec)
        stats = _roam_and_record(cam, link, roamer, args.seconds, args.buzzer,
                                 args.buzzer_threshold)
        cam.stop_recording()
        meta["clips"][0]["stats"] = stats
        print(f"저장: {name}")
        _check_fps(clipdir / name, args.seconds, cam.spec.fps, args.codec)
    except KeyboardInterrupt:
        print("\n중단 — 지금까지 녹화된 부분은 그대로 남아있다.")
        try:
            cam.stop_recording()
        except Exception:  # noqa: BLE001
            pass
    finally:
        if link is not None:
            for _ in range(2):
                try:
                    link.send_command(control.STOP)
                except Exception as exc:  # noqa: BLE001
                    print(f"정지 명령 실패: {exc}")
                time.sleep(0.05)
            link.close()
        (outdir / "session.json").write_text(
            json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
        cam.close()
        print(f"\n-> {outdir.resolve()}")
        print(f"다음: python label_throws.py {outdir}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
