"""어안 픽셀 -> 핀홀 등가 픽셀 변환.

**렌즈가 좁은 화각(대각 90° 미만)으로 바뀌면 이 파일은 필요 없다.**
지금 렌즈(번들 ZH3019-14, **실측 대각 133.5°**)에서는 **없으면 안 된다.**

왜 있는가
---------
`trajectory.py`의 선형 해법은 핀홀 투영을 전제한다:

    a·Z − fx·X = 0        (a = u − cx)
    b·Z − fy·Y = 0        (b = v − cy)

이게 선형인 이유는 `u = fx·X/Z + cx`, 즉 **r = f·tanθ** 이기 때문이다.
그런데 화각이 90°를 넘는 어안 렌즈는 이 식을 따르지 않는다 (tanθ가 발산한다).
번들 ZH3019-14는 실측 대각 133.5°로 이 범위에 한참 들어간다. 캘리브레이션 계수로
계산한 θ_d/θ 는 중심 0.996에서 모서리(66.7°) 0.857까지 **단조 감소**한다 —
중심부는 등거리(`r = f·θ`)에 가깝고 가장자리는 14% 압축된다.
cv2.fisheye 가 쓰는 모델이 이걸 그대로 기술한다.  근거: ../../docs/design/physics.md 7.2장

⚠ **표기 화각에서 f 를 역산하지 말 것.** 압축을 빼먹게 되어 25% 틀린다.
  압축이 있으면 f_px 가 크면서도 화각이 넓다 — 973 과 133.5° 는 모순이 아니다.

어안 픽셀을 그대로 넣으면 **관측 자체가 틀려서** 최소제곱이 조용히 잘못된
거리를 뱉는다. 핀홀 기준 입사각 42°에서 z가 79%, 화면 끝에서 165% 어긋난다.

해법은 간단하다 — **최소제곱을 고칠 필요가 없다.** 관측 uv를 핀홀 등가로 편 뒤
기존 해법에 그대로 넣으면 된다. 선형성도 그대로 유지된다.

    uv = fisheye.to_pinhole_px(uv)      # ← 이 한 줄만 추가
    fit = trajectory.fit_trajectory(times, uv)

어느 쪽을 쓰나
--------------
① **캘리브레이션을 돌렸다면** (`prep/calibrate.py solve --model fisheye`)
   `cv2.fisheye.undistortPoints`가 정답이다. K와 D가 실측이라 모델 가정이 없다.
② **아직 안 돌렸다면** 사양에서 역산한 등거리 근사를 쓴다. 임시방편이다.

①이 가능하면 언제나 ①을 쓴다. ②는 "렌즈가 어안인 건 아는데 캘리브레이션은
아직"인 구간을 메우는 용도다.
"""

from __future__ import annotations

import math

import numpy as np

import config


def _principal() -> tuple[float, float]:
    return float(config.CAMERA_CX), float(config.CAMERA_CY)


def _focal() -> float:
    return (float(config.CAMERA_FX) + float(config.CAMERA_FY)) / 2.0


def theta_from_radius(r_px: float, f_px: float, model: str = "equidistant") -> float:
    """주점에서의 픽셀 거리 -> 입사각(rad)."""
    if model == "pinhole":
        return math.atan(r_px / f_px)
    if model == "equisolid":              # r = 2f·sin(θ/2)
        return 2.0 * math.asin(min(1.0, r_px / (2.0 * f_px)))
    # equidistant (기본): r = f·θ — 이 렌즈이자 cv2.fisheye 가 쓰는 모델
    return r_px / f_px


def ray_from_pixel(u: float, v: float, model: str = "equidistant") -> tuple[float, float]:
    """픽셀 -> (입사각 theta, 방위각 phi). 광축이 theta=0이다."""
    cx, cy = _principal()
    du, dv = u - cx, -(v - cy)            # 이미지 y축은 아래로 증가하므로 뒤집는다
    return theta_from_radius(math.hypot(du, dv), _focal(), model), math.atan2(dv, du)


def to_pinhole_px(uv, model: str | None = None):
    """어안 픽셀 (N,2) -> 같은 f를 쓰는 **핀홀 등가** 픽셀 (N,2).

    반환값을 그대로 trajectory.fit_trajectory 에 넣으면 된다.
    캘리브레이션 계수(config.CAMERA_DISTORTION)가 있으면 그걸 쓰고,
    없으면 사양 기반 해석 근사로 떨어진다.
    """
    uv = np.asarray(uv, dtype=float).reshape(-1, 2)
    if config.CAMERA_MODEL == "pinhole":
        return uv                          # 이미 핀홀이면 할 일 없다

    _warn_if_outside_frame(uv)
    if config.CAMERA_DISTORTION is not None:
        return _undistort_cv2(uv)
    return _undistort_analytic(uv, model or "equidistant")


_warned_outside = False


def _warn_if_outside_frame(uv: np.ndarray) -> None:
    """프레임 밖 좌표가 들어오면 한 번만 경고한다.

    왜곡 다항식은 **캘리브레이션에서 관측된 범위 안에서만** 검증된 것이다.
    화면 최대 입사각은 모서리의 66.7°이고, 그 바깥은 외삽이라 보장이 없다.
    (검출 결과는 항상 프레임 안이라 정상 동작에서는 걸릴 일이 없다.)
    """
    global _warned_outside
    if _warned_outside or uv.size == 0:
        return
    w, h = config.CAMERA_RESOLUTION
    if (uv[:, 0] < 0).any() or (uv[:, 0] > w).any() \
            or (uv[:, 1] < 0).any() or (uv[:, 1] > h).any():
        _warned_outside = True
        print(f"[fisheye] ⚠ 프레임({w}x{h}) 밖 좌표가 들어왔다. 왜곡 모델은 화면 "
              f"안에서만 유효하다 — 결과를 믿지 말 것.")


def _undistort_cv2(uv: np.ndarray) -> np.ndarray:
    """실측 K/D 기반. 모델 가정이 없어 이쪽이 항상 낫다."""
    import cv2

    K = np.array([[config.CAMERA_FX, 0.0, config.CAMERA_CX],
                  [0.0, config.CAMERA_FY, config.CAMERA_CY],
                  [0.0, 0.0, 1.0]])
    D = np.asarray(config.CAMERA_DISTORTION, dtype=float).reshape(-1, 1)[:4]
    # P=K 로 주면 정규화 좌표가 아니라 **원래 픽셀 스케일**로 되돌아온다.
    # trajectory.py가 config.CAMERA_FX를 그대로 쓰므로 스케일이 맞아야 한다.
    out = cv2.fisheye.undistortPoints(uv.reshape(-1, 1, 2), K, D, P=K)
    return out.reshape(-1, 2)


def _undistort_analytic(uv: np.ndarray, model: str) -> np.ndarray:
    """사양에서 역산한 근사. 캘리브레이션 전 임시용."""
    cx, cy = _principal()
    f = _focal()
    out = np.empty_like(uv)
    for i, (u, v) in enumerate(uv):
        du, dv = u - cx, v - cy
        r = math.hypot(du, dv)
        if r < 1e-9:
            out[i] = (u, v)
            continue
        theta = theta_from_radius(r, f, model)
        # 핀홀에서 같은 입사각이 만드는 반경. 90°에 가까우면 발산하므로 막는다.
        if theta >= math.radians(88.0):
            out[i] = (u, v)                # 이 각도는 애초에 쓰면 안 되는 관측이다
            continue
        scale = (f * math.tan(theta)) / r
        out[i] = (cx + du * scale, cy + dv * scale)
    return out
