"""② 좌표 변환 — 픽셀 ↔ body ↔ world.

순수 함수. 상태 없음. 하드웨어 없이 테스트 가능.

⚠ 이 렌즈는 핀홀이 아니다.
   번들 M12 렌즈는 f=2.8mm에 대각 화각 140°다. 핀홀(r = f·tanθ)이라면 96.5°가 나와야
   하므로 양립하지 않는다. 실제로는 **등입체각 어안 투영**(r = 2f·sin(θ/2))에 가깝고,
   이 모델이 f≈2.73mm에서 140°와 사양의 -17% TV 왜곡을 동시에 설명한다.

   핀홀 식을 그대로 쓰면 가장자리에서 x,y가 30% 넘게 틀리고 z가 18% 편향된다.
   (../docs/physics.md 7장)

좌표계 정의는 ../docs/pi5-algorithm.md 2장:
  world(W)  피코 부팅 시점 로봇 중심 원점. 관성계. 포물선 피팅은 반드시 여기서.
  body(B)   로봇 회전 중심. X 전방 / Y 좌측 / Z 위.
  image(C)  u 오른쪽, v 아래쪽. 광축은 +Z_B (수직 상방).
"""

from __future__ import annotations

import math

import numpy as np

import config
from datatypes import Detection, Pose


def rotate2(x: float, y: float, theta: float) -> tuple[float, float]:
    """2D 회전. (x, y)를 theta만큼 반시계로 돌린다."""
    c, s = math.cos(theta), math.sin(theta)
    return c * x - s * y, s * x + c * y


# ── 렌즈 투영 모델 ──────────────────────────────────────────────────────
# theta = 광축으로부터의 입사각(rad), r = 이미지 중심으로부터의 거리(px)

def ray_from_pixel(u: float, v: float) -> tuple[float, float]:
    """픽셀 (u, v) -> 입사 방향 (theta, phi).

    phi는 body X축 기준 방위각. 이미지 v축이 아래로 증가하므로 부호를 뒤집는다.
    """
    u0, v0 = config.PRINCIPAL_POINT_PX
    du, dv = u - u0, -(v - v0)
    r = math.hypot(du, dv)
    phi = math.atan2(dv, du)

    if config.PROJECTION_MODEL == "pinhole":
        theta = math.atan(r / config.FOCAL_LENGTH_PX)
    else:  # equisolid:  r = 2f·sin(theta/2)
        theta = 2.0 * math.asin(min(1.0, r / (2.0 * config.FOCAL_LENGTH_PX)))
    return theta, phi


def pixel_from_ray(theta: float, phi: float) -> tuple[float, float]:
    """ray_from_pixel의 역변환."""
    if config.PROJECTION_MODEL == "pinhole":
        r = config.FOCAL_LENGTH_PX * math.tan(theta)
    else:
        r = 2.0 * config.FOCAL_LENGTH_PX * math.sin(theta / 2.0)

    u0, v0 = config.PRINCIPAL_POINT_PX
    return u0 + r * math.cos(phi), v0 - r * math.sin(phi)


def local_scale_px(theta: float) -> float:
    """입사각 theta에서의 국소 배율 dr/dtheta [px/rad].

    물체의 각크기를 픽셀 크기로 바꾸는 계수다. 어안 렌즈에서는 가장자리로 갈수록
    작아진다(등입체각 기준 70°에서 중심의 82%). **이걸 무시하고 상수 f_px를 쓰면
    z가 화면 위치에 따라 최대 18% 편향된다.**
    """
    if config.PROJECTION_MODEL == "pinhole":
        c = math.cos(theta)
        return config.FOCAL_LENGTH_PX / (c * c)
    return config.FOCAL_LENGTH_PX * math.cos(theta / 2.0)


# ── 관측 -> body 좌표 ───────────────────────────────────────────────────

def estimate_range(det: Detection) -> float:
    """bbox 폭으로 카메라~물체 **직선거리 R**(m)을 역산한다.

        각크기 alpha ≈ W_real / R,   w_px = local_scale_px(theta) · alpha
        =>  R = local_scale_px(theta) · W_real / w_px

    폭을 쓰는 이유: 폭이 물체의 짧은 치수(캔 66mm)에 대응해 회전에 덜 민감하다.
    글로벌 셔터라 기하 왜곡은 없다. (../docs/physics.md 7.1장)
    """
    if det.w <= 0:
        raise ValueError(f"bbox 폭이 0 이하: {det.w}")
    theta, _ = ray_from_pixel(det.cx, det.cy)
    return local_scale_px(theta) * config.object_width_m(det.class_name) / det.w


def pixel_to_body(det: Detection) -> np.ndarray:
    """Detection -> body frame (x, y, z) [m]. 원점은 **로봇 회전 중심**이다.

    카메라는 통 입구 림 가장자리에 있어 로봇 중심에서 약 15 cm 떨어져 있으므로,
    관측(카메라 기준)에 오프셋을 더해 로봇 중심 기준으로 옮긴다. 로봇이 회전하지
    않으므로(omega=0) 상수 벡터 덧셈으로 충분하다.
    """
    # 정해야함: 캘리브레이션 실측 왜곡 반영 (config.DISTORTION_COEFFS).
    theta, phi = ray_from_pixel(det.cx, det.cy)
    R = estimate_range(det)

    x = R * math.sin(theta) * math.cos(phi)
    y = R * math.sin(theta) * math.sin(phi)
    z = R * math.cos(theta)

    x, y = rotate2(x, y, config.CAMERA_YAW_RAD)   # 장착 회전 오차 보정
    ox, oy = config.CAMERA_OFFSET_M               # 카메라 -> 로봇 중심 보정
    return np.array([x + ox, y + oy, z], dtype=float)


def body_to_pixel(p_body: np.ndarray) -> tuple[float, float]:
    """body (x, y, z) -> 이미지 (u, v). pixel_to_body의 역변환."""
    ox, oy = config.CAMERA_OFFSET_M               # 로봇 중심 -> 카메라 기준
    x, y = rotate2(float(p_body[0]) - ox, float(p_body[1]) - oy, -config.CAMERA_YAW_RAD)
    z = float(p_body[2])

    R = math.sqrt(x * x + y * y + z * z)
    if R <= 0 or z <= 0:
        raise ValueError(f"카메라 앞쪽이 아니라 투영 불가: z={z}")
    theta = math.acos(max(-1.0, min(1.0, z / R)))
    return pixel_from_ray(theta, math.atan2(y, x))


# ── body ↔ world ───────────────────────────────────────────────────────

def body_to_world(p_body: np.ndarray, pose: Pose) -> np.ndarray:
    """body -> world. 로봇 자세(x, y, theta)로 회전 + 평행이동."""
    x, y = rotate2(float(p_body[0]), float(p_body[1]), pose.theta)
    return np.array([x + pose.x, y + pose.y, float(p_body[2])], dtype=float)


def world_to_body(p_world: np.ndarray, pose: Pose) -> np.ndarray:
    """world -> body. body_to_world의 역변환."""
    x, y = rotate2(float(p_world[0]) - pose.x, float(p_world[1]) - pose.y, -pose.theta)
    return np.array([x, y, float(p_world[2])], dtype=float)


def world_to_pixel(p_world: np.ndarray, pose: Pose) -> tuple[float, float]:
    """world -> 이미지 (u, v). ROI 중심을 잡을 때 쓴다."""
    return body_to_pixel(world_to_body(p_world, pose))


# ── 측정 노이즈 ─────────────────────────────────────────────────────────

def measurement_sigma(det: Detection, R: float, sigma_bbox_px: float) -> np.ndarray:
    """이 관측의 (σx, σy, σz). 프레임마다 다르다.

    시선 방향(range)과 그에 수직한 방향(각도)으로 나눠 구한 뒤 xyz로 투영한다.

        σ_R    = R · σ_w / w_px                     bbox 폭 오차 -> 거리 오차
        σ_perp = R · σ_u / local_scale_px(theta)    픽셀 지터 -> 횡방향 오차
    """
    theta, phi = ray_from_pixel(det.cx, det.cy)
    sigma_R = R * sigma_bbox_px / det.w
    sigma_perp = R * sigma_bbox_px / local_scale_px(theta)

    st, ct = math.sin(theta), math.cos(theta)
    cp, sp = math.cos(phi), math.sin(phi)

    # 시선 성분과 횡 성분을 각 축으로 투영 (대각 근사)
    sigma_x = math.hypot(sigma_R * st * cp, sigma_perp * ct * cp, config.SIGMA_ODOM_M)
    sigma_y = math.hypot(sigma_R * st * sp, sigma_perp * ct * sp, config.SIGMA_ODOM_M)
    sigma_z = math.hypot(sigma_R * ct, sigma_perp * st)
    return np.array([sigma_x, sigma_y, sigma_z], dtype=float)
