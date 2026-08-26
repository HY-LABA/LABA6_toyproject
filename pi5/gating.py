"""검출 후보 중에서 **지금 쫓는 물체 하나**를 고른다 (데이터 연관).

왜 필요한가
-----------
`vision.detect()` 는 신뢰도 최고 하나만 돌려주고, `Tracker.add()` 는 그게 아까 그
물체인지 묻지 않는다. 그래서 에어컨·공유기·조명이 한 프레임이라도 쓰레기보다 높은
점수를 받으면 그 좌표가 트랙에 섞인다.

**섞이면 예측이 안 나온다.** 25프레임 궤적에 오탐 2프레임만 들어가도 재투영 잔차가
무한대로 튀어 `Fit.ok` 가 거짓이 된다(실측). 즉 증상은 "틀린 예측"이 아니라
"아무 예측도 안 나옴"이다.

정적 오탐 자체는 궤적 단계가 이미 완벽히 거른다 — 화면에서 안 움직이는 점은 어떤
포물선으로도 설명이 안 되므로 잔차가 발산한다. 그러니 **포물선 게이트를 또 만들
필요는 없고**, 오탐이 트랙에 **들어가기 전에** 막으면 된다.

두 단계로 막는다
----------------
1. `StaticSuppressor` — 같은 자리에 계속 나타나는 검출을 기억해뒀다가 무시한다.
   에어컨·공유기·조명은 정의상 안 움직인다. 던지기 전에 몇 초 돌려두면 알아서 등록된다.
2. `choose()` — 남은 후보 중, 트랙이 이미 있으면 **예측 위치에 가장 가까운 것**을
   고른다. 트랙이 없으면 신뢰도 최고를 고른다.
"""

from __future__ import annotations

import math

import numpy as np

import config

# 이 반경 안에 다시 나타나면 "같은 자리"로 본다. 검출 지터보다 넉넉히 크게.
STATIC_RADIUS_PX = getattr(config, "STATIC_RADIUS_PX", 25.0)
# 같은 자리에서 이 횟수 이상 나오면 정적 오탐으로 등록한다.
STATIC_MIN_HITS = getattr(config, "STATIC_MIN_HITS", 8)
# 이 시간 동안 안 보이면 기억에서 지운다 (조명을 껐거나 카메라를 옮긴 경우).
STATIC_TTL_S = getattr(config, "STATIC_TTL_S", 5.0)
# 트랙이 있을 때, 예측 위치에서 이보다 멀면 다른 물체로 본다.
ASSOC_RADIUS_PX = getattr(config, "ASSOC_RADIUS_PX", 200.0)


def predict_uv(fit, t: float) -> tuple[float, float] | None:
    """fit 이 예측하는 시각 t 의 화면 좌표. 카메라 뒤로 가면 None."""
    p = fit.position_at(t - fit.t0)
    if p[2] <= 1e-6:
        return None
    return (config.CAMERA_FX * p[0] / p[2] + config.CAMERA_CX,
            config.CAMERA_FY * p[1] / p[2] + config.CAMERA_CY)


class StaticSuppressor:
    """같은 자리에 반복해서 나타나는 검출을 기억했다가 무시한다.

    낙하 물체는 매 프레임 다른 자리에 나타나므로 hits 가 안 쌓인다. 반면 에어컨은
    같은 자리에서 계속 나오므로 몇 프레임 만에 등록된다.

    ⚠ 시작 직후 STATIC_MIN_HITS 프레임 동안은 아무것도 억제되지 않는다.
      **던지기 전에 빈 화면으로 1~2초 돌려두면** 그동안 정적 오탐이 전부 등록된다.
    """

    def __init__(self, radius_px: float = STATIC_RADIUS_PX,
                 min_hits: int = STATIC_MIN_HITS, ttl_s: float = STATIC_TTL_S) -> None:
        self.radius = radius_px
        self.min_hits = min_hits
        self.ttl = ttl_s
        self._spots: list[list] = []      # [u, v, hits, last_t]

    def update(self, uvs, t: float) -> None:
        """이번 프레임의 **모든** 후보를 보고 기억을 갱신한다.

        고른 것 하나만 넣으면 안 된다 — 억제하려는 대상은 애초에 안 고른 것들이다.
        """
        self._spots = [s for s in self._spots if t - s[3] <= self.ttl]
        for u, v in uvs:
            for s in self._spots:
                if math.hypot(u - s[0], v - s[1]) <= self.radius:
                    # 위치를 조금씩 따라가며 평균낸다 (검출 지터 흡수)
                    s[0] += (u - s[0]) * 0.2
                    s[1] += (v - s[1]) * 0.2
                    s[2] += 1
                    s[3] = t
                    break
            else:
                self._spots.append([u, v, 1, t])

    def is_static(self, u: float, v: float) -> bool:
        return any(s[2] >= self.min_hits and math.hypot(u - s[0], v - s[1]) <= self.radius
                   for s in self._spots)

    @property
    def known(self) -> list[tuple[float, float, int]]:
        """등록된 정적 지점 목록 (진단용)."""
        return [(s[0], s[1], s[2]) for s in self._spots if s[2] >= self.min_hits]


def choose(detections, tracker, t: float, suppressor: StaticSuppressor):
    """후보 중 하나를 고른다. 없으면 None.

    detections: [Detection, ...]  (vision.detect_all 결과)
    tracker   : trajectory.Tracker (fit 이 있으면 예측 위치로 연관)
    """
    if not detections:
        return None

    suppressor.update([(d.u, d.v) for d in detections], t)
    live = [d for d in detections if not suppressor.is_static(d.u, d.v)]
    if not live:
        return None

    fit = getattr(tracker, "fit", None)
    if fit is None:
        # 아직 트랙이 없다 — 신뢰도로 고른다. 틀려도 궤적 잔차가 걸러준다.
        return max(live, key=lambda d: d.confidence)

    uv = predict_uv(fit, t)
    if uv is None:
        return max(live, key=lambda d: d.confidence)

    best = min(live, key=lambda d: math.hypot(d.u - uv[0], d.v - uv[1]))
    if math.hypot(best.u - uv[0], best.v - uv[1]) > ASSOC_RADIUS_PX:
        # 예측 위치 근처에 아무것도 없다. 억지로 붙이면 트랙이 오염된다 —
        # 이번 프레임은 그냥 놓친 것으로 둔다 (TRACK_MAX_GAP_S 가 처리한다).
        return None
    return best
