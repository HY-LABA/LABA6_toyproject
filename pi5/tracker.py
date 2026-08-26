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
치솟아 `fit.ok`에서 자동으로 걸린다.

그래서 **미리 고르지 않는다.** 후보마다 가설(`Tracker`)을 하나씩 돌리고, 물리를 통과한
가설만 채택한다(`TrackerPool.best`). 에어컨 가설은 영원히 착지해를 못 내므로 저절로
탈락한다. 판정자가 휴리스틱이 아니라 중력이다.

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
import trajectory as traj

IDLE, TRACKING, COOLDOWN = "IDLE", "TRACKING", "COOLDOWN"


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
    origin_odom: tuple[float, float] = (0.0, 0.0)   # ★ 첫 관측 시점의 오도메트리
    _first_shown: bool = field(default=False, repr=False)

    # ── 좌표 ──────────────────────────────────────────────────────────────
    def _cam_xy(self, odom_xy) -> tuple[float, float]:
        """이 가설의 원점 기준 **카메라 광학중심** 위치.

        로봇 회전중심이 아니라 카메라 위치여야 한다 — 관측 (u,v)를 만든 게 카메라이기
        때문이다. `CAMERA_OFFSET_M`은 통 입구 림에 단 카메라가 회전중심에서 떨어진 양.
        """
        ox, oy = config.CAMERA_OFFSET_M
        return (odom_xy[0] - self.origin_odom[0] + ox,
                odom_xy[1] - self.origin_odom[1] + oy)

    def moved_since_start(self, odom_xy) -> tuple[float, float]:
        """이 가설이 시작된 뒤 **로봇 중심**이 이동한 양.

        피코가 주는 값은 부팅 이후 누적이라 그대로 쓰면 두 번째 투척부터 어긋난다.
        `control`이 "남은 거리 = 착지점 − 이만큼"을 계산할 때 쓴다.
        """
        return (odom_xy[0] - self.origin_odom[0],
                odom_xy[1] - self.origin_odom[1])

    # ── 누적 ──────────────────────────────────────────────────────────────
    def add(self, det, odom_xy, now: float) -> traj.Fit | None:
        """관측 하나를 누적하고 갱신된 fit을 돌려준다. 아직 못 믿으면 None."""
        if self.started_at is None:
            self.started_at = det.t
            self.origin_odom = tuple(odom_xy)      # ★ 이 가설의 좌표 원점
        self.last_seen = det.t

        self.times.append(float(det.t))
        self.uvs.append((float(det.u), float(det.v)))
        self.cams.append(self._cam_xy(odom_xy))
        if len(self.times) > config.TRACK_WINDOW:
            del self.times[0], self.uvs[0], self.cams[0]

        self.fit = None
        if self.n < config.MIN_OBSERVATIONS:
            return None
        if self.time_span < config.MIN_TIME_SPAN_S:
            # 시간 스팬이 짧으면 곡률을 못 재고 깊이가 발산한다.
            # 자신 있게 틀린 숫자를 주느니 아무것도 안 주는 게 낫다.
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
            self.fit = fit
            return fit

        if not self._depth_converged(fit):
            return None
        self.fit = fit
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
    def landing(self):
        """(x, y, 남은시간) 또는 None.

        x, y는 **이 가설이 시작된 시점의 로봇 중심**을 원점으로 한 월드 좌표다.
        카메라 오프셋은 `_cam_xy`에서 이미 반영됐으므로 여기서 또 빼면 안 된다.
        """
        if self.fit is None:
            return None
        out = traj.predict_landing(self.fit)
        if out is None:
            return None
        x, y, dt_from_t0 = out
        # 남은 시간은 마지막 관측 시각 기준으로 환산해 준다 —
        # 호출부는 "지금부터 몇 초 남았나"로 판단해야 하기 때문.
        return x, y, self.fit.t0 + dt_from_t0 - self.times[-1]

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

    # ── 사이클 ────────────────────────────────────────────────────────────
    def tick(self, now: float) -> None:
        """COOLDOWN이 끝났으면 IDLE로 되돌린다. 매 프레임 호출."""
        if self.state == COOLDOWN and self.ended_at is not None:
            if now - self.ended_at >= config.COOLDOWN_S:
                self.state = IDLE
                self.ended_at = None

    def reset(self, now: float | None = None, reason: str = "") -> str:
        """모든 가설을 버리고 COOLDOWN으로 보낸다."""
        self.tracks.clear()
        self.committed = None
        self.started_at = None
        self.ended_at = now
        self.state = COOLDOWN if now is not None else IDLE
        return reason

    def cycle_end_reason(self, now: float, time_remaining: float | None) -> str | None:
        """지금 사이클을 끝내야 하나. 끝내야 하면 사유 문자열, 아니면 None.

        **채택한 가설을 기준으로 판정한다.** 채택 전이라면 아직 아무것도 시작 안 한
        것이므로 타임아웃만 본다 — 오탐만 잔뜩 잡고 있는 상태에서 빠져나오는 길이다.
        """
        if self.state != TRACKING:
            return None
        if time_remaining is not None and time_remaining <= 0.0:
            return "착지 시각 경과"
        c = self.committed
        if c is not None and c.last_seen is not None:
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

        # ⑤ 채택
        self._commit(now)

    # ── 채택 ──────────────────────────────────────────────────────────────
    def _commit(self, now: float) -> None:
        """물리를 통과한 가설 중 하나를 고른다.

        ★ **여기서만 TRACKING으로 들어간다.** "검출이 있으면 TRACKING"으로 하면
        에어컨만 보이는 대기 상태에서도 사이클이 시작돼, `CYCLE_TIMEOUT_S`마다
        리셋→COOLDOWN이 반복된다. 그 COOLDOWN 동안에는 가설을 아예 안 만들기
        때문에, 하필 그때 던지면 초반 관측을 통째로 놓친다.

        대기 중에도 가설은 계속 돌아간다(싸다). 사이클은 **물리를 통과한 궤적이
        나타난 순간**부터다.
        """
        if self.committed is not None:
            if self.committed in self.tracks and self.committed.viable:
                return
            self.committed = None          # 죽었으면 놓아준다

        viable = [tr for tr in self.tracks if tr.viable]
        if not viable:
            return
        self.committed = min(viable, key=lambda tr: tr.fit.residual_px)
        if self.state == IDLE:
            self.state = TRACKING
            self.started_at = now

    def best(self) -> Tracker | None:
        """채택된 가설. `update()`가 이미 정해뒀다."""
        return self.committed

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
