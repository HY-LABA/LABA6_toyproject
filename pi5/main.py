"""전체 실행 순서 오케스트레이션.

    프레임마다:
      ① 카메라 캡처 → YOLO → **검출 후보 전체**            [vision.observe]
      ② 후보마다 가설을 돌린다 (미리 고르지 않는다)         [pool.update]
      ③ 물리를 통과한 가설을 채택                          [pool.best]
      ④ 착지점·남은시간                                    [track.landing]
      ⑤ 착지점을 월드 좌표로 옮겨 피코에 전송              [control]
      ⑥ 사이클 종료 판단 → 리셋                            [pool.cycle_end_reason]

★ 왜 ②에서 하나를 고르지 않나
------------------------------
YOLO가 에어컨(conf 0.88)을 실제 쓰레기(0.62)보다 높게 준다. 여기서 하나를 고르는
휴리스틱을 쓰면 틀렸을 때 회복이 안 된다 — 오탐이 트랙에 들어가는 순간 재투영 잔차가
무한대로 튀어 사이클 전체가 죽는다.

대신 **후보마다 가설을 돌리고 물리가 판정하게 한다.** 정적 오탐은 화소가 고정돼
포물선 제약을 만족할 수 없으므로 착지해를 영영 못 낸다. 에어컨 가설은 저절로 탈락한다.

★ 오도메트리를 두 곳에 쓴다 — 역할이 다르다
-------------------------------------------
  피팅 입력  카메라가 움직이면 관측 (u,v)에 물체의 운동과 카메라의 운동이 섞인다.
             보정 없이 풀면 **등속 주행에서는 잔차가 작은 채로 답만 틀린다**
             (합성 검증: 1.2 m/s에서 잔차 1.48px인데 착지 57cm 오차).
             `Tracker`가 각 관측과 함께 카메라 위치를 쌓는다.
  도달 판정  "남은 거리를 남은 시간 안에 갈 수 있나"로 표적을 고른다. `tracker._reach`.
             (실제 구동에서 남은 거리를 빼는 건 파이가 아니라 피코다 — 아래 참고)

**둘 다 가설이 시작된 시점을 원점으로 한 상대 이동량이다.** 피코가 주는 값은 부팅 이후
누적이라, 사이클마다 원점을 다시 잡지 않으면 두 번째 투척부터 좌표가 어긋난다.

★ 남은 거리는 파이가 빼지 않는다
--------------------------------
피코에는 **월드 좌표 도착 지점**만 보내고, "이미 간 만큼"을 빼는 건 피코가 자기
pose 로 한다. 파이가 미리 빼서 보내면 **두 번 빠진다** — 예전 프로토콜이 정확히 그
반대 방향으로 터졌었다(피코가 아무도 안 빼서 매번 처음부터 다시 갔다).
여기서 `moved_since_start()` 를 여전히 쓰는 곳은 **로그용 예측 속도**뿐이다.

★ 단발 모드 (`--once`) — 첫 예측만으로 가 보는 테스트
-----------------------------------------------------
평소에는 프레임이 쌓일 때마다 다시 풀어서 목표점을 갱신한다. 깊이는 스팬이 짧을수록
과소추정되는 계통 편향이 있어서(errors-in-variables) **첫 예측은 대개 가깝게 나오고,
관측이 쌓이며 참값 쪽으로 수렴한다.** `--once` 는 그 갱신을 끄고 첫 예측을 고정해서
"첫 예측만으로 얼마나 갈 수 있나"를 재는 모드다. 고정한 뒤에도 **재계산은 계속 하고
로그에 남긴다** — 첫 예측과 나중 예측의 차이가 곧 그 편향의 크기다.

  --once            첫 목표점을 고정하고 **계속 재전송**한다. 워치독·정지 동작은
                    프로덕션과 완전히 같다. 기본값으로 이걸 쓸 것.
  --once --hold N   **딱 한 프레임만 전송**하고 워치독을 N초로 늘린다. 펌웨어의
                    단발 처리 자체를 보는 모드다. 사이클 끝 STOP도 보내지 않으므로
                    **로봇을 세우는 건 워치독 N초뿐이다.**

⚠ `--hold N` 을 남은시간보다 훨씬 크게 주지 말 것
--------------------------------------------------
피코는 새 명령이 안 오는 동안 `time_remaining_s` 를 스스로 깎고, **0 이하가 되면
"이미 늦었다"고 보고 상한 속도로 붙는다**(`pico/main.c` DRIVE_TARGET 블록). 평소에는
그 시점에 사이클이 끝나 파이가 STOP 을 보내므로 한 프레임 이상 그 상태에 있지 않다.
그런데 `--hold` 는 그 STOP 을 생략하므로, 남은시간이 다 지나고도 워치독이 살아 있으면
**로봇이 목표점 주변을 최대속도로 왕복한다.** N 은 예상 이동시간 정도로만 줄 것.
"""

from __future__ import annotations

import math

import config
import communication
import control
import tracker as tracker_mod
import utils


def run(once: bool = False, hold_s: float | None = None) -> None:
    """`once=True` 면 사이클마다 **첫 목표점만** 쓴다 (모듈 docstring 참고).

    `hold_s` 가 있으면 그 한 프레임만 전송하고 워치독을 `hold_s` 로 늘린다.
    """
    cam = vision_open()
    link = communication.SerialLink()
    pool = tracker_mod.TrackerPool()

    odom_xy = (0.0, 0.0)
    commanded = False
    first_target = None      # --once: 이 사이클에서 처음 보낸 목표점
    first_target_at = 0.0

    single_packet = once and hold_s is not None

    utils.log("catch loop start" + (
        f" [단발 모드: 첫 목표점 고정, 1회 전송, 워치독 {hold_s:.2f}s]" if single_packet
        else " [단발 모드: 첫 목표점 고정, 재전송 유지]" if once else ""))
    try:
        while True:
            # ── ① 캡처 + 검출 (후보 전체) ────────────────────────────────
            #    시각은 캡처 시점 것을 쓴다 — 피팅 입력과 같은 시계여야 한다.
            candidates, now, _frame = vision.observe(cam)

            # ── 오도메트리 갱신 (논블로킹) ───────────────────────────────
            odom = link.try_receive_odometry()
            if odom is not None:
                odom_xy = odom.xy

            # ── ②③④ 가설 갱신 → 채택 → 예측 ──────────────────────────
            #    test_accuracy.py가 **이 함수를 그대로 쓴다.**
            track, landing, reason = pool.step(candidates, odom_xy, now)

            if landing is not None:
                landing_x, landing_y, time_remaining = landing

                # ── ⑤ 명령 — 도착 지점(월드 좌표)만 보낸다 ──────────────
                #    매 프레임 다시 푼다. --once 여도 계산은 계속한다 —
                #    첫 예측과의 차이가 곧 깊이 편향의 크기이고, 그게 이 테스트로
                #    보려는 값이다. 다만 **보내는 건 첫 것으로 고정**한다.
                target = control.to_target_command(
                    landing_xy=(landing_x, landing_y),
                    origin_odom=track.origin_odom,
                    time_remaining=time_remaining,
                )

                if once and first_target is not None:
                    dx = target.target_x - first_target.target_x
                    dy = target.target_y - first_target.target_y
                    utils.log(f"hold  첫목표=({first_target.target_x:+.3f},"
                              f"{first_target.target_y:+.3f}) "
                              f"재계산=({target.target_x:+.3f},{target.target_y:+.3f}) "
                              f"차이={math.hypot(dx, dy) * 100:.1f}cm"
                              + ("  [전송 안 함]" if single_packet else "  [고정값 재전송]"))
                    target = control.held_target(first_target, now - first_target_at)
                    if not single_packet:
                        link.send_target(target)
                else:
                    if single_packet:
                        # 워치독을 늘려서 **딱 이 한 장**으로 끝까지 가게 한다.
                        target = control.held_target(target, 0.0, timeout_s=hold_s)
                        if hold_s > time_remaining + 0.3:
                            utils.log(
                                f"⚠ --hold {hold_s:.2f}s 가 남은시간 {time_remaining:.2f}s "
                                f"보다 길다. 남은시간이 다 지나면 피코가 상한 속도로 붙으므로 "
                                f"목표점 주변을 왕복할 수 있다 (STOP 을 생략하는 모드다)")
                    link.send_target(target)
                    commanded = True
                    if once:
                        first_target = target
                        first_target_at = now

                # 속도는 보내지 않지만(피코가 계산한다) **예측치를 로그에 남긴다** —
                # 어느 바퀴가 한계에 붙었는지 봐야 튜닝이 되고, 실기에서 피코가 실제로
                # 낸 속도(오도메트리 회신)와 이 예측을 비교하면 양쪽 식이 갈렸는지
                # 바로 보인다. 붙은 바퀴엔 ! 표시.
                predicted = control.to_drive_command(
                    landing_xy=(landing_x, landing_y),
                    time_remaining=time_remaining,
                    odometry_xy=pool.moved_since_start(odom_xy),
                )
                utils.log_cycle(track.fit, (landing_x, landing_y), time_remaining,
                                odom_xy, predicted, target)
                utils.log(control.describe(predicted))

            # ── ⑥ 사이클 종료 ───────────────────────────────────────────
            if reason:
                utils.log(f"cycle end — {reason} (관측 {pool.n}, "
                          f"스팬 {pool.time_span:.2f}s, 가설 {pool.n_tracks}개)")
                # ★ 움직이지 않았으면 COOLDOWN 없이 바로 다음 궤적을 기다린다.
                #   COOLDOWN은 "움직인 뒤 관성/튐"을 위한 것이라, 가만히 있었으면
                #   0.8초를 쉬는 건 그 사이 날아오는 걸 놓치는 것뿐이다.
                pool.reset(now, reason, cooldown=commanded)
                first_target = None
                if commanded:
                    # 단발 모드에서는 세우지 않는다 — 워치독 hold_s 가 끝까지
                    # 가게 두는 게 이 모드의 목적이다. 정지는 워치독이 한다.
                    if single_packet:
                        utils.log("단발 모드 — 사이클 끝 STOP 생략 (워치독이 세운다)")
                    else:
                        link.send_command(control.STOP)
                    commanded = False
    except KeyboardInterrupt:
        utils.log("interrupted")
    finally:
        # 어떤 경로로 빠져나가도 로봇은 세운다.
        try:
            link.send_command(control.STOP)
        except Exception as exc:  # noqa: BLE001 - 정지 시도는 실패해도 계속 정리한다
            utils.log(f"stop command failed: {exc}")
        link.close()
        cam.close()
        utils.log("catch loop end")


def vision_open():
    """vision을 늦게 import한다 — Picamera2/HailoRT가 없는 PC에서도 이 파일을
    읽고 문법 검사할 수 있게 하기 위함."""
    global vision
    import vision as _vision

    vision = _vision
    return vision.Camera(config.CAMERA_RESOLUTION, config.CAMERA_FPS)


def _parse_args():
    import argparse

    # ⚠ 도움말 문자열에 em-dash 같은 비 cp949 문자를 쓰지 말 것. 파이(UTF-8)에서는
    #   괜찮지만 윈도우 콘솔에서 `--help` 가 UnicodeEncodeError 로 죽는다.
    p = argparse.ArgumentParser(description="쓰레기 받는 로봇 - 구동 루프")
    p.add_argument("--once", action="store_true",
                   help="사이클마다 첫 예측만 쓴다. 이후 프레임의 재계산은 로그만 남기고 "
                        "보내지 않는다 (기본: 고정한 목표점을 계속 재전송)")
    p.add_argument("--hold", type=float, default=None, metavar="초",
                   help="--once 와 함께: 딱 한 번만 전송하고 워치독을 이 시간으로 늘린다. "
                        "사이클 끝 STOP도 생략되므로 로봇을 세우는 건 이 시간뿐이다")

    # ── config 덮어쓰기 ──────────────────────────────────────────────────
    # 없으면 config.py 값을 그대로 쓴다. test_accuracy.py --span/--ratio/--set 와
    # 같은 방식이다 — 리플레이·실기 튜닝할 때 config.py 를 매번 고치지 않아도 된다.
    g = p.add_argument_group("트래킹 파라미터 (기본값: config.py)")
    g.add_argument("--min-time-span", type=float, default=None, metavar="초",
                   help="config.MIN_TIME_SPAN_S 덮어쓰기 — 최소 관측 스팬")
    g.add_argument("--max-residual-px", type=float, default=None, metavar="px",
                   help="config.MAX_RESIDUAL_PX 덮어쓰기 — 재투영 잔차 상한(탄도 게이트)")
    g.add_argument("--z-range", type=float, nargs=2, default=None, metavar=("MIN", "MAX"),
                   help="config.Z_RANGE_M 덮어쓰기 — 물리적으로 말이 되는 깊이 범위(m)")
    g.add_argument("--assoc-step-px", type=float, default=None, metavar="px",
                   help="config.ASSOC_STEP_PX 덮어쓰기 — 검출을 가설에 붙이는 연관 반경")
    g.add_argument("--set", action="append", default=[], metavar="KEY=VAL",
                   help="config 의 아무 값이나 덮어쓴다. 여러 번 쓸 수 있다. "
                        "예: --set MIN_OBSERVATIONS=6 --set DEPTH_STABILITY_RATIO=1.5")

    args = p.parse_args()
    if args.hold is not None and not args.once:
        p.error("--hold 는 --once 와 함께 써야 한다")
    if args.hold is not None and args.hold <= 0:
        p.error("--hold 는 양수여야 한다")
    if args.z_range is not None and args.z_range[0] >= args.z_range[1]:
        p.error(f"--z-range 는 MIN < MAX 여야 한다: {args.z_range}")
    return args


def _apply_config_overrides(args) -> None:
    """CLI 로 받은 트래킹 파라미터를 config 모듈에 그대로 덮어쓴다.

    모든 모듈이 `import config` 후 `config.ATTR` 로 매번 읽으므로(값을 지역
    상수로 캐시하지 않는다), `run()` 을 부르기 전에 여기서 한 번 덮어쓰면 그
    이후의 모든 접근에 반영된다.
    """
    overrides = {
        "MIN_TIME_SPAN_S": args.min_time_span,
        "MAX_RESIDUAL_PX": args.max_residual_px,
        "Z_RANGE_M": tuple(args.z_range) if args.z_range is not None else None,
        "ASSOC_STEP_PX": args.assoc_step_px,
    }
    for name, value in overrides.items():
        if value is not None:
            setattr(config, name, value)
            utils.log(f"[덮어씀] config.{name} = {value}")

    for kv in args.set:
        if "=" not in kv:
            raise SystemExit(f"--set 은 KEY=VALUE 형식이어야 한다: {kv!r}")
        k, v = kv.split("=", 1)
        k = k.strip()
        if not hasattr(config, k):
            raise SystemExit(f"--set: config 에 없는 이름이다: {k}")
        try:
            val = float(v) if "." in v or "e" in v.lower() else int(v)
        except ValueError:
            val = v
        setattr(config, k, val)
        utils.log(f"[덮어씀] config.{k} = {val}")


if __name__ == "__main__":
    _args = _parse_args()
    _apply_config_overrides(_args)
    run(once=_args.once, hold_s=_args.hold)
