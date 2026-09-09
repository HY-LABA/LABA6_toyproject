"""가설 추적 · 사이클 관리 — **상태를 갖는 부분 전부.**

순수 계산(포물선 풀이, 착지 예측)은 `trajectory.py`에 있다. 여기는 그걸 언제 어떤
입력으로 부를지 결정한다.

★ 왜 "미리 고르지" 않고 "다 돌려보나"
--------------------------------------
YOLO가 에어컨·공유기·조명을 쓰레기로 검출하고, 심지어 실제 쓰레기(conf 0.6)보다
높은 점수(0.88)를 준다. 그래서 "후보 중 진짜를 미리 고르는" 휴리스틱이 필요해 보이지만,
그 휴리스틱이 틀리면 **회복할 방법이 없다** — 오탐이 트랙에 들어가는 순간 재투영 잔차가
무한대로 튀어 사이클 전체가 죽는다(합성 검증: 25프레임 중 2프레임 오염으로 예측 소멸).

그런데 **정적 오탐은 애초에 물리를 만족시킬 수 없다.** 화소가 고정되면 a = u−cx 가
상수라, 중력항이 있는 상태에서 a·Z − fx·X = 0 을 만족하는 해가 없다. 조건수가 1e17로
치솟아 `fit.ok`에서 자동으로 걸린다 — **단, 이건 화소가 정말로 고정됐을 때 얘기다.**

⚠ 2026-09-04 실기 로그에서 이 가정이 짧은 관측 구간에서는 안 통한다는 걸 확인했다.
YOLO bbox 중심은 완전히 고정되지 않고 몇 px씩 흔들리는데(검출 노이즈), 그 흔들림이
행렬을 "완전한 특이"에서 "잘 안 풀리지만 풀리긴 하는" 상태로 바꿔놓는다. 실측: 배경
오탐 3건이 관측 4~8개(스팬 0.3~0.56s) 구간에서 조건수 3천~1.2만(MAX_CONDITION=1e9에
한참 못 미침), 잔차 3.3~5.8px(MAX_RESIDUAL_PX=6.0 통과)로 fit.ok를 다 통과했다. 대신
풀린 속도가 10.9~202.4 m/s로 튀었다 — 노이즈가 짧은 구간 안에서는 그럴듯한 잔차를
만들어주지만, 그 구간 밖으로 외삽하면(착지 예측) 폭발한다. 조건수·잔차는 관측 구간
**안에서의** 적합도만 재지, 외삽 결과의 물리적 타당성은 안 보기 때문이다.

그래서 `Tracker.add()`에 값싼 사전 필터를 하나 더 뒀다 — 관측 윈도우 전체에서 화면상
이동량(`MIN_TRACK_DISPLACEMENT_PX`)이 너무 작으면 최소제곱을 시도하지도 않는다. 실제
낙하물의 프레임간 이동량(107~292px)과 오탐의 관측 구간 전체 이동량(2~7px)이 10배 넘게
차이 나므로 안전한 여유가 있다. `Fit.ok`에도 풀린 속도 크기 상한(`MAX_LAUNCH_SPEED_MPS`)을
추가했다 — 짧은 구간이 아니라 **긴** 구간에서 노이즈가 우연히 만들어내는 궤적까지 잡기
위한 이중 방어다.

그래서 **미리 고르지 않는다.** 후보마다 가설(`Tracker`)을 하나씩 돌리고, 물리(+ 위 두
사전/사후 필터)를 통과한 가설만 채택한다(`TrackerPool.best`). 에어컨 가설은 영원히
착지해를 못 내므로 저절로 탈락한다. 판정자가 휴리스틱이 아니라 중력이다.

비용은 무시할 수준이다 — lstsq 한 번이 마이크로초라, 후보 3~8개를 병렬로 돌려도
프레임당 0.3~0.4 ms (60fps 예산 16.7 ms의 2.5%)다.

사이클
------
    IDLE ──(첫 관측)──▶ TRACKING ──(착지/유실/타임아웃)──▶ COOLDOWN ──▶ IDLE

**COOLDOWN이 필요한 이유:** 착지 직후 물체가 통 안에서 튀거나 로봇이 관성으로
미끄러지는 동안 새 트랙을 시작하면 안 된다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

import config
import control
import trajectory as traj

IDLE, TRACKING, COOLDOWN = "IDLE", "TRACKING", "COOLDOWN"


# ── 좌표계 경계 ────────────────────────────────────────────────────────────
# `trajectory.py` 는 **카메라 이미지 축**(X=화면 우, Y=화면 아래)으로 푼다.
# 오도메트리·역기구학·`control` 은 **로봇 body 축**(+X=우측, +Y=전방)을 쓴다.
# 그 사이의 회전이 `config.CAMERA_YAW_RAD` 이고, 두 축이 만나는 지점은
# 이 파일의 딱 두 곳뿐이다:
#
#   robot -> cam :  Tracker._cam_xy()   피팅 입력(`cams`)을 만들 때
#   cam -> robot :  Tracker.landing()   착지 예측을 내보낼 때
#
# 이 두 곳만 지키면 `tracker` 바깥(`control`, `main`)은 전부 로봇 축이고,
# `trajectory` 안은 전부 카메라 축이라 섞일 수가 없다.


def _rot(x: float, y: float, ang: float) -> tuple[float, float]:
    c, s = math.cos(ang), math.sin(ang)
    return (c * x - s * y, s * x + c * y)


def cam_to_robot(x: float, y: float) -> tuple[float, float]:
    """카메라 이미지 축 -> 로봇 body 축."""
    return _rot(x, y, config.CAMERA_YAW_RAD)


def robot_to_cam(x: float, y: float) -> tuple[float, float]:
    """로봇 body 축 -> 카메라 이미지 축."""
    return _rot(x, y, -config.CAMERA_YAW_RAD)


# ── 가설 하나 ──────────────────────────────────────────────────────────────

@dataclass
class Tracker:
    """관측을 누적하며 착지 예측을 계속 정밀화하는 **가설 하나**.

    매 프레임 **처음부터 다시 피팅**한다. 이 규모에선 최소제곱이 마이크로초라 아낄
    게 없고, 배치 피팅은 물리를 매 프레임 정확히 강제하므로 프로세스 노이즈가
    포물선을 서서히 밀어내는 일이 없다.

    프레임을 몇 장 모아야 하는가는 **개수가 아니라 시간 스팬**의 문제다. 궤적의
    처짐량은 ⅛·g·T²이라 T에 제곱으로 붙는 반면, 같은 시간에 프레임만 늘리면 노이즈
    평균 효과로 √N밖에 못 얻는다.

    사이클 상태(IDLE/TRACKING/COOLDOWN)는 여기가 아니라 `TrackerPool`이 갖는다.
    가설 하나는 "관측을 쌓고 푼다"만 한다.
    """

    times: list[float] = field(default_factory=list)
    uvs: list[tuple[float, float]] = field(default_factory=list)
    cams: list[tuple[float, float]] = field(default_factory=list)
    fit: traj.Fit | None = None

    started_at: float | None = None       # 이 가설의 첫 관측 시각
    last_seen: float | None = None        # 마지막으로 관측이 붙은 시각
    fit_at: float | None = None           # self.fit 을 마지막으로 **갱신**한 관측 시각
    origin_odom: tuple[float, float] = (0.0, 0.0)   # ★ 첫 관측 시점의 오도메트리
    _first_shown: bool = field(default=False, repr=False)

    # ── 좌표 ──────────────────────────────────────────────────────────────
    def _cam_xy(self, odom_xy) -> tuple[float, float]:
        """이 가설의 원점 기준 **카메라 광학중심** 위치 — **카메라 축**으로 돌려서.

        로봇 회전중심이 아니라 카메라 위치여야 한다 — 관측 (u,v)를 만든 게 카메라이기
        때문이다. `CAMERA_OFFSET_M`은 통 입구 림에 단 카메라가 회전중심에서 떨어진 양.

        ★ 오도메트리도 CAMERA_OFFSET_M 도 **로봇 body 축**이다. 그런데 이 값이
          들어가는 `trajectory.fit_trajectory(cams=...)` 는 **카메라 축**으로 푼다.
          그래서 로봇 축에서 더한 뒤 마지막에 한 번 회전시킨다 (`robot_to_cam`).
          예전에는 이 회전이 없어서 두 축을 그냥 섞고 있었다.
        """
        ox, oy = config.CAMERA_OFFSET_M
        rx = odom_xy[0] - self.origin_odom[0] + ox      # 로봇 body 축
        ry = odom_xy[1] - self.origin_odom[1] + oy
        return robot_to_cam(rx, ry)                     # -> 카메라 축

    def moved_since_start(self, odom_xy) -> tuple[float, float]:
        """이 가설이 시작된 뒤 **로봇 중심**이 이동한 양.

        피코가 주는 값은 부팅 이후 누적이라 그대로 쓰면 두 번째 투척부터 어긋난다.
        `control`이 "남은 거리 = 착지점 − 이만큼"을 계산할 때 쓴다.
        """
        return (odom_xy[0] - self.origin_odom[0],
                odom_xy[1] - self.origin_odom[1])

    # ── 누적 ──────────────────────────────────────────────────────────────
    def add(self, det, odom_xy, now: float) -> traj.Fit | None:
        """관측 하나를 누적하고 **새로 푼** fit을 돌려준다. 못 믿으면 None.

        ★ 반환이 None이어도 `self.fit`은 남아 있을 수 있다 (`config.FIT_HOLD_S`).
          예전에는 맨 앞에서 `self.fit = None`을 해서, 게이트를 한 프레임만 못 넘겨도
          그 가설이 통째로 `viable`이 아니게 되고 그 프레임엔 피코로 명령이 아예
          안 나갔다. 워치독이 0.15s라 몇 번 겹치면 로봇이 가다 선다.
          그래서 직전 해를 짧게 들고 간다 — 자세한 근거는 config.FIT_HOLD_S 주석.
        """
        if self.started_at is None:
            self.started_at = det.t
            self.origin_odom = tuple(odom_xy)      # ★ 이 가설의 좌표 원점
        self.last_seen = det.t

        self.times.append(float(det.t))
        self.uvs.append((float(det.u), float(det.v)))
        self.cams.append(self._cam_xy(odom_xy))
        if len(self.times) > config.TRACK_WINDOW:
            del self.times[0], self.uvs[0], self.cams[0]

        fit = self._solve()
        if fit is not None:
            self.fit = fit
            self.fit_at = det.t
            return fit

        # 이번 프레임은 못 풀었다. 직전 해를 FIT_HOLD_S 까지만 들고 가고,
        # 그걸 넘겼으면 버린다 — 노이즈 한 번이 아니라 뭔가 진짜 잘못된 것이다.
        if (self.fit is not None and self.fit_at is not None
                and det.t - self.fit_at > config.FIT_HOLD_S):
            self.fit = None
            self.fit_at = None
        return None

    def _solve(self) -> traj.Fit | None:
        """지금 쌓인 관측으로 푼다. 게이트를 하나라도 못 넘기면 None.

        `add()`에서 갈라낸 이유는 "푸는 것"과 "직전 해를 얼마나 들고 갈 것인가"가
        서로 다른 결정이기 때문이다.
        """
        if self.n < config.MIN_OBSERVATIONS:
            return None
        if self.time_span < config.MIN_TIME_SPAN_S:
            # 시간 스팬이 짧으면 곡률을 못 재고 깊이가 발산한다.
            # 자신 있게 틀린 숫자를 주느니 아무것도 안 주는 게 낫다.
            return None

        # ★ 정지 오탐 사전 차단 (config.MIN_TRACK_DISPLACEMENT_PX 참고).
        # 최소제곱을 돌리기 전에 값싸게 거른다 — 시간 스팬은 채웠는데 화면에서
        # 거의 안 움직인 트랙(형광등·화재감지기 등)은 짧은 구간 안에서는 잔차·
        # 조건수 게이트를 우연히 통과할 수 있다는 걸 실기 로그로 확인했다.
        us = [uv[0] for uv in self.uvs]
        vs = [uv[1] for uv in self.uvs]
        displacement = math.hypot(max(us) - min(us), max(vs) - min(vs))
        if displacement < config.MIN_TRACK_DISPLACEMENT_PX:
            return None

        fit = traj.fit_trajectory(self.times, self.uvs, self.cams)
        if not fit.ok:
            return None

        # 첫 번째로 fit.ok를 통과한 순간만 깊이 수렴 검사를 건너뛰고 바로 내보낸다.
        # 초반 깊이는 항상 과소추정 방향으로만 편향되므로(errors-in-variables)
        # 로봇이 목표보다 덜 가는 쪽으로만 틀린다 — 오버슈트 위험이 없고 다음
        # 프레임들이 이어서 보정한다.
        if not self._first_shown:
            self._first_shown = True
            return fit

        if not self._depth_converged(fit):
            return None
        return fit

    def _depth_converged(self, fit: traj.Fit) -> bool:
        """앞쪽 관측만으로 다시 풀어서 깊이가 안정됐는지 본다.

        **잔차로는 이 판정을 할 수 없다.** 스케일이 틀린 궤적도 화면에는 잘 맞게
        투영되기 때문이다 — 그게 단안 카메라의 원래 모호성이고, 중력이 그 모호성을
        깨주지만 관측 시간이 짧으면 충분히 못 깬다. 합성 검증에서 스팬 0.5초일 때
        잔차 1.58px(아주 좋음)인데 깊이는 참값의 69%였다.

        관측이 짧을수록 깊이가 **작게** 나오는 계통 편향이 있으므로, 관측을 덜 쓴
        해와 전부 쓴 해가 가까워졌다면 수렴한 것이다. 노이즈 크기를 미리 몰라도
        되는 자기 교정식 판정이다.
        """
        k = int(len(self.times) * config.DEPTH_CHECK_FRACTION)
        if k < config.MIN_OBSERVATIONS:
            return False
        sub = traj.fit_trajectory(self.times[:k], self.uvs[:k], self.cams[:k])
        z_full, z_sub = fit.p0[2], sub.p0[2]
        if not np.isfinite(z_sub) or z_sub <= 0.0:
            return False
        ratio = z_full / z_sub
        limit = config.DEPTH_STABILITY_RATIO
        return (1.0 / limit) <= ratio <= limit

    # ── 연관용 ────────────────────────────────────────────────────────────
    def predict_uv(self, t: float, odom_xy) -> tuple[float, float] | None:
        """다음 관측이 화면 어디에 나타날지. 연관(association)에 쓴다.

        fit이 있으면 궤적으로 투영하고, 없으면 직전 관측 자리를 쓴다.
        """
        if self.fit is not None:
            uv = traj.project(self.fit, t, self._cam_xy(odom_xy))
            if uv is not None:
                return uv
        return self.uvs[-1] if self.uvs else None

    # ── 결과 ──────────────────────────────────────────────────────────────
    def landing(self, now: float | None = None):
        """(x, y, 남은시간) 또는 None.

        x, y는 **이 가설이 시작된 시점의 로봇 중심**을 원점으로 한, **로봇 body 축**
        좌표다. 카메라 오프셋은 `_cam_xy`에서 이미 반영됐으므로 여기서 또 빼면 안 된다.

        ★ `trajectory` 가 돌려주는 건 **카메라 축**이므로 여기서 로봇 축으로 돌린다
          (`cam_to_robot`). 이 함수 바깥(`_reach`, `control.to_target_command`,
          `control.to_drive_command`)은 전부 로봇 축 세계다 — 거기서 오도메트리와
          더하고 빼는 게 그래서 성립한다.

        ⚠ 남은 시간 기준 시각 (2026-09-07 수정)
        ------------------------------------------
        `now`를 주면 **그 시각 기준** 남은시간을 돌려준다 — 실시간 구동은 항상 이걸
        써야 한다. 안 주면(레거시) 마지막 관측 시각(`self.times[-1]`) 기준인데,
        물체가 화면 밖으로 나가 관측이 끊기면 `times[-1]`이 그 순간에 멈춰버려서
        **남은 시간이 실제로는 줄고 있는데 그대로 얼어붙는다.** 그 얼어붙은 값을
        피코에 보내면(`TargetCommand.time_remaining_s`) 피코가 실제보다 여유
        있다고 믿고 느리게 간다 — 가장 서둘러야 할 마지막 구간에서 오히려 늦어지는
        구조적 버그였다. `viable` 프로퍼티처럼 "존재 여부"만 보는 곳은 `now` 없이
        불러도 안전하다(x, y 자체는 `now`와 무관하다 — `fit.t0`와 `dt_from_t0`로만
        정해진다).
        """
        if self.fit is None:
            return None
        out = traj.predict_landing(self.fit)
        if out is None:
            return None
        x, y, dt_from_t0 = out
        x, y = cam_to_robot(x, y)           # 카메라 축 -> 로봇 body 축
        ref_t = now if now is not None else self.times[-1]
        return x, y, self.fit.t0 + dt_from_t0 - ref_t

    @property
    def viable(self) -> bool:
        """물리를 만족하는가 — 채택 후보가 될 자격."""
        return self.fit is not None and self.landing() is not None

    @property
    def n(self) -> int:
        return len(self.times)

    @property
    def time_span(self) -> float:
        return (self.times[-1] - self.times[0]) if self.times else 0.0

    def __repr__(self) -> str:
        r = f"{self.fit.residual_px:.2f}px" if self.fit else "—"
        return f"Track(n={self.n} span={self.time_span:.2f}s resid={r})"


# ── 가설 묶음 (다중 가설 추적) ─────────────────────────────────────────────

class TrackerPool:
    """후보마다 가설을 하나씩 돌리고, **물리를 통과한 가설만 채택**한다.

    매 프레임:
      ① 검출 ↔ 가설을 **1:1 배타 배정** (가까운 쌍부터 그리디)
      ② 배정 못 받은 검출은 새 가설로 스폰
      ③ 오래 못 본 가설은 소멸, 개수가 `MAX_TRACKS`를 넘으면 오래된 것부터 폐기
      ④ `viable`한 가설 중 재투영 잔차 최소인 것을 채택

    **한번 채택하면 계속 따라간다**(`committed`). 매 프레임 최소 잔차로 새로 고르면
    가설끼리 목표가 흔들려서 로봇이 갈팡질팡하기 때문이다. 채택한 가설이 죽으면
    그때 다시 고른다.
    """

    def __init__(self) -> None:
        self.tracks: list[Tracker] = []
        self.committed: Tracker | None = None
        self.state: str = IDLE
        self.started_at: float | None = None
        self.ended_at: float | None = None
        # ★ 가설이 처음 생긴 시각 — **채택 여부와 무관**하다.
        #   `started_at` 은 물리를 통과한 궤적이 채택된 시각이라, 오탐만 잡고 있으면
        #   영원히 None 이고 그래서 타임아웃이 안 돈다. 그 구멍을 메우는 값이다.
        self.tracks_since: float | None = None

    # ── 사이클 ────────────────────────────────────────────────────────────
    def tick(self, now: float) -> None:
        """COOLDOWN이 끝났으면 IDLE로 되돌린다. 매 프레임 호출."""
        if self.state == COOLDOWN and self.ended_at is not None:
            if now - self.ended_at >= config.COOLDOWN_S:
                self.state = IDLE
                self.ended_at = None

    def reset(self, now: float | None = None, reason: str = "",
              cooldown: bool = True) -> str:
        """모든 가설을 버린다. `cooldown=True`면 COOLDOWN을 거치고, 아니면 바로 IDLE.

        ★ **명령을 한 번도 안 보냈으면 COOLDOWN이 필요 없다.** COOLDOWN은 "로봇이
        움직인 뒤 관성으로 미끄러지거나 물체가 통 안에서 튀는 동안 새 트랙을 시작하지
        않기" 위한 것이다. 오탐을 붙잡고 타임아웃으로 끝난 경우처럼 로봇이 가만히
        있었다면, 0.8초를 쉬는 건 그 사이에 날아오는 진짜 쓰레기를 놓치는 것뿐이다.
        """
        self.tracks.clear()
        self.committed = None
        self.started_at = None
        self.tracks_since = None
        self.ended_at = now
        self.state = COOLDOWN if (now is not None and cooldown) else IDLE
        return reason

    def cycle_end_reason(self, now: float, time_remaining: float | None) -> str | None:
        """지금 사이클을 끝내야 하나. 끝내야 하면 사유 문자열, 아니면 None.

        **채택한 가설을 기준으로 판정한다.** 채택 전이라면 아직 아무것도 시작 안 한
        것이므로 타임아웃만 본다 — 오탐만 잔뜩 잡고 있는 상태에서 빠져나오는 길이다.
        """
        if self.state != TRACKING:
            # ★ 채택 전에도 타임아웃을 본다 (2026-09-07).
            #   예전에는 여기서 바로 None 을 돌려줘서, **천장 오탐만 잡고 있는 상태는
            #   영원히 타임아웃되지 않았다** — state 가 IDLE 이라 `started_at` 이 None 이고
            #   아래 CYCLE_TIMEOUT_S 검사에 도달하지도 못했다. 그래서 사이클이 끝났다는
            #   로그도 안 찍혀서 터미널이 조용했다.
            #   이제는 가설이 생긴 지 CYCLE_TIMEOUT_S 를 넘기면 통째로 리셋한다.
            if (self.tracks_since is not None
                    and now - self.tracks_since > config.CYCLE_TIMEOUT_S):
                return f"미채택 타임아웃 ({now - self.tracks_since:.2f}s, 채택된 궤적 없음)"
            return None
        if time_remaining is not None and time_remaining <= 0.0:
            return "착지 시각 경과"
        c = self.committed
        # ⚠ 2026-09-07 수정: 확정된 물리 해(fit)가 있는 트랙은 화면 밖으로 나가도
        # "유실"로 끝내지 않는다. 천장 카메라 특성상 물체는 **착지 직전 반드시
        # 화면을 벗어난다**(u=fx·X/Z가 Z→0에서 발산) — 실측 어안 계수 기준
        # 착지 0.05~0.19초 전. TRACK_MAX_GAP_S(0.15s)가 그 구간과 겹쳐서, 정상적으로
        # 잘 던진 진짜 낙하물도 착지 직전에 "유실"로 사이클이 끝나고 STOP이 나갔다.
        # 게다가 그 사이 YOLO가 몇 프레임만 놓쳐도(32fps에서 5프레임=0.15s) 물체가
        # 화면 한복판에 있어도 같은 일이 벌어졌다. fit이 있다는 건 물리로 이미 확정된
        # 궤적이 있다는 뜻이므로, 그 경우엔 "착지 시각 경과"(위 분기, now 기준으로
        # 고쳐졌으므로 이제 정확하다)와 CYCLE_TIMEOUT_S(2.0s, 아래)만으로 끝낸다.
        # 아직 fit이 없는 트랙(오탐 후보가 물리를 못 만족해 계속 붙잡고 있는 경우)은
        # 기존대로 유실 처리한다 — 그런 트랙은 화면에 계속 보여야 다음 프레임에
        # 다시 풀릴 여지가 있으므로, 안 보이면 놓아주는 게 맞다.
        if c is not None and c.fit is None and c.last_seen is not None:
            gap = now - c.last_seen
            if gap > config.TRACK_MAX_GAP_S:
                return f"물체 유실 ({gap:.2f}s)"
        if self.started_at is not None and now - self.started_at > config.CYCLE_TIMEOUT_S:
            return f"사이클 타임아웃 ({now - self.started_at:.2f}s)"
        return None

    # ── 갱신 ──────────────────────────────────────────────────────────────
    def update(self, detections, odom_xy, now: float) -> None:
        """이번 프레임 검출들을 가설에 배정하고, 필요하면 새 가설을 만든다."""
        if self.state == COOLDOWN:
            return

        # ① 1:1 배타 배정 — 가까운 쌍부터 그리디.
        #    한 가설이 같은 프레임에서 검출을 두 개 먹으면 시간축이 깨진다.
        pairs = []
        for di, d in enumerate(detections):
            for ti, tr in enumerate(self.tracks):
                uv = tr.predict_uv(now, odom_xy)
                if uv is None:
                    continue
                dist = math.hypot(d.u - uv[0], d.v - uv[1])
                if dist <= config.ASSOC_STEP_PX:
                    pairs.append((dist, di, ti))
        pairs.sort()

        used_d: set[int] = set()
        used_t: set[int] = set()
        for _dist, di, ti in pairs:
            if di in used_d or ti in used_t:
                continue
            self.tracks[ti].add(detections[di], odom_xy, now)
            used_d.add(di)
            used_t.add(ti)

        # ② 배정 못 받은 검출 → 새 가설
        for di, d in enumerate(detections):
            if di not in used_d:
                tr = Tracker()
                tr.add(d, odom_xy, now)
                self.tracks.append(tr)

        # ③ 소멸 — 오래 못 본 가설. 채택한 가설은 사이클 종료 판정이 따로 하므로 남긴다.
        self.tracks = [tr for tr in self.tracks
                       if tr is self.committed
                       or (tr.last_seen is not None
                           and now - tr.last_seen <= config.TRACK_MAX_GAP_S)]

        # ④ 개수 상한 — 오탐이 깜빡이면 가설이 무한정 늘 수 있다.
        #    채택한 가설과 관측이 많은 가설을 우선 남긴다.
        if len(self.tracks) > config.MAX_TRACKS:
            self.tracks.sort(key=lambda tr: (tr is not self.committed, -tr.n))
            self.tracks = self.tracks[:config.MAX_TRACKS]

        # ④' 가설이 언제부터 있었나 — 채택 전 타임아웃 판정용
        if self.tracks:
            if self.tracks_since is None:
                self.tracks_since = now
        else:
            self.tracks_since = None

        # ⑤ 채택
        self._commit(now, odom_xy)

    # ── 채택 ──────────────────────────────────────────────────────────────
    def _commit(self, now: float, odom_xy) -> None:
        """물리를 통과한 가설 중 하나를 고른다.

        ★ **여기서만 TRACKING으로 들어간다.** "검출이 있으면 TRACKING"으로 하면
        에어컨만 보이는 대기 상태에서도 사이클이 시작돼, `CYCLE_TIMEOUT_S`마다
        리셋→COOLDOWN이 반복된다. 그 COOLDOWN 동안에는 가설을 아예 안 만들기
        때문에, 하필 그때 던지면 초반 관측을 통째로 놓친다.

        대기 중에도 가설은 계속 돌아간다(싸다). 사이클은 **물리를 통과한 궤적이
        나타난 순간**부터다.
        """
        cand = []
        for tr in self.tracks:
            if not tr.viable:
                continue
            r = self._reach(tr, odom_xy, now)
            if r is None:
                continue
            ok, need, remaining = r
            # 정렬 키: ① 도달 가능한 것 먼저 ② 급한 것(남은시간 짧은 것) 먼저
            cand.append((0 if ok else 1, remaining, need, tr))
        if not cand:
            self.committed = None
            return
        cand.sort(key=lambda c: (c[0], c[1]))
        best = cand[0]

        # 이미 채택한 게 아직 후보에 있으면 **웬만하면 유지한다.** 매 프레임 새로
        # 고르면 목표가 가설끼리 오가면서 로봇이 갈팡질팡한다.
        # 예외: 내가 도달 불가가 됐는데 도달 가능한 대안이 있으면 갈아탄다 —
        #       못 잡을 걸 쫓느니 잡을 수 있는 걸 잡는 게 낫다.
        for c in cand:
            if c[3] is self.committed:
                if c[0] == 0 or best[0] == 1:
                    return
                break

        self.committed = best[3]
        if self.state == IDLE:
            self.state = TRACKING
            self.started_at = now

    def _reach(self, track: Tracker, odom_xy, now: float) -> tuple[bool, float, float] | None:
        """`(도달 가능한가, 필요 속력, 남은 시간)`. 착지해가 없으면 None.

        "도달 가능"은 **남은 거리를 남은 시간 안에 갈 수 있는가**다. 이걸 봐야
        동시에 두 개가 날아올 때 **잡을 수 있는 쪽**을 고를 수 있다. 잔차가 제일
        작은 걸 고르는 건 "가장 확실한 궤적"이지 "가장 잡기 쉬운 것"이 아니다.

        ⚠ `now`를 반드시 넘겨야 한다 — `Tracker.landing()` 참고. 안 넘기면 물체가
          안 보이는 동안 남은 시간이 얼어붙어 도달 판정이 실제보다 낙관해진다.
        """
        out = track.landing(now)
        if out is None:
            return None
        lx, ly, remaining = out
        mx, my = track.moved_since_start(odom_xy)
        rx, ry = lx - mx, ly - my
        dist = math.hypot(rx, ry)
        if remaining <= 1e-3:
            return (False, math.inf, remaining)
        need = dist / remaining
        limit = control.max_body_speed(rx, ry) * config.REACH_MARGIN
        return (need <= limit, need, remaining)

    def early_bearing(self) -> tuple[float, float] | None:
        """아직 fit 이 없을 때 **"일단 갈 방향"** 만 준다. 로봇 body 축 단위벡터.

        깊이는 몰라도 방향은 안다 — 카메라가 하늘을 보므로 화면상 위치가 곧 방위각
        이고, 그건 깊이와 무관하게 정확하다. 그래서 물체가 화면에서 흘러가는 방향
        (첫 관측 -> 마지막 관측)을 로봇 축으로 돌려주면 그게 갈 방향이다.

        ★ 현재 위치가 아니라 **이동 방향**을 쓴다. 물체는 처음엔 카메라 반대편
          하늘에 있다가 머리 위를 지나 착지점 쪽으로 넘어가므로, 초반의 "현재 방위각"
          은 오히려 정반대를 가리킨다. 실기 로그에서도 v 가 182(뒤) -> 951(앞) 로
          흘렀고 실제 착지는 앞이었다 — 이동 방향이 맞고 현재 위치는 틀린다.

        ⚠ 게이트는 `MIN_TRACK_DISPLACEMENT_PX` 하나뿐이다. 이 단계엔 fit 이 없어서
          물리 게이트를 못 쓰고, 그건 깊이가 필요 없는 유일한 판정이기 때문이다.
          실제 낙하물은 프레임당 100px 넘게 움직여 3프레임이면 여유로 넘고, 정지
          오탐은 관측 구간 전체에서 2~7px 라 못 넘는다.
        """
        best = None
        for tr in self.tracks:
            if tr.n < config.EARLY_START_OBS:
                continue
            us = [uv[0] for uv in tr.uvs]
            vs = [uv[1] for uv in tr.uvs]
            disp = math.hypot(max(us) - min(us), max(vs) - min(vs))
            if disp < config.MIN_TRACK_DISPLACEMENT_PX:
                continue
            du = tr.uvs[-1][0] - tr.uvs[0][0]
            dv = tr.uvs[-1][1] - tr.uvs[0][1]
            norm = math.hypot(du, dv)
            if norm < 1e-6:
                continue
            # 가장 크게 움직인 가설을 고른다 — 가장 확실히 "날아가는 것"이다.
            if best is None or disp > best[0]:
                best = (disp, cam_to_robot(du / norm, dv / norm))
        return None if best is None else best[1]

    def best(self) -> Tracker | None:
        """채택된 가설. `update()`가 이미 정해뒀다."""
        return self.committed

    # ── 한 프레임 전체 ────────────────────────────────────────────────────
    def step(self, detections, odom_xy, now: float):
        """한 프레임을 처리한다. → `(채택된 가설, 착지예측, 사이클 종료사유)`

        ★ **`main.py`와 `test_accuracy.py`가 이 함수 하나를 공유한다.**
        각자 루프를 따로 쓰면 한쪽만 고쳤을 때 테스트가 조용히 다른 걸 재게 된다.
        여기에 모아두면 그럴 수가 없다.

        착지예측은 `(x, y, 남은시간)` 또는 None. 종료사유는 문자열 또는 None —
        호출부가 `reset()`을 부를지 정한다(테스트는 로그만 남기고, 구동 루프는
        피코에 STOP도 보낸다).
        """
        self.tick(now)
        self.update(detections, odom_xy, now)
        track = self.best()
        landing = track.landing(now) if track is not None else None
        remaining = landing[2] if landing is not None else None
        return track, landing, self.cycle_end_reason(now, remaining)

    # ── 조회 ──────────────────────────────────────────────────────────────
    def moved_since_start(self, odom_xy) -> tuple[float, float]:
        """채택한 가설 기준 로봇 이동량. 채택 전이면 (0,0)."""
        c = self.committed
        return c.moved_since_start(odom_xy) if c is not None else (0.0, 0.0)

    @property
    def n(self) -> int:
        return self.committed.n if self.committed is not None else 0

    @property
    def time_span(self) -> float:
        return self.committed.time_span if self.committed is not None else 0.0

    @property
    def n_tracks(self) -> int:
        return len(self.tracks)

    @property
    def n_viable(self) -> int:
        """물리를 통과한 가설 수. 2 이상이면 동시에 여러 개가 날아온 것이다."""
        return sum(1 for tr in self.tracks if tr.viable)
