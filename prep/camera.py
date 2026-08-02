"""카메라 추상화 — **카메라를 바꿀 때 손대는 곳은 이 파일 하나다.**

지금은 기본 CSI 카메라, 나중에 글로벌 셔터(CAM-IMX296Color-GS)로 교체한다.
캘리브레이션·수집 로직은 완전히 동일하고, 아래 CameraSpec 값만 바뀐다.

┌─ 카메라 교체 시 확인할 것 ────────────────────────────────────────────────┐
│ ① SPECS에 프로파일 추가/선택          — 해상도, fps, 노출                  │
│ ② 캘리브레이션 모델                   — 아래 표 참고. **이게 가장 중요**    │
│ ③ 수집한 데이터셋은 재사용 불가        — FOV·왜곡·셔터가 달라 다시 찍어야 함 │
└──────────────────────────────────────────────────────────────────────────┘

    | 카메라              | 렌즈        | 셔터   | 캘리브레이션 모델 |
    |---------------------|-------------|--------|-------------------|
    | CSI v2 (IMX219)     | 고정 ~62°   | 롤링   | pinhole           |
    | CSI v3 (IMX708)     | 고정 ~66°   | 롤링   | pinhole           |
    | IMX296-GS + 2.8mm   | 어안 140°   | 글로벌 | **fisheye**       |

  화각 90°를 넘어가면 핀홀 모델이 성립하지 않는다. 140° 렌즈를 pinhole로 캘리브레이션하면
  가장자리에서 크게 어긋난다 (../docs/physics.md 7.2장).
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraSpec:
    name: str
    width: int
    height: int
    fps: int
    calib_model: str          # "pinhole" | "fisheye"
    exposure_us: int | None   # None이면 자동 노출
    note: str = ""


SPECS: dict[str, CameraSpec] = {
    # 지금 쓰는 것 ────────────────────────────────────────────────────────
    "csi": CameraSpec(
        name="기본 CSI 카메라", width=1640, height=1232, fps=30,
        calib_model="pinhole", exposure_us=None,
        note="롤링 셔터. 낙하 물체 bbox가 기울어질 수 있다 — 수집 데이터는 임시용.",
    ),
    # 도착하면 이걸로 ─────────────────────────────────────────────────────
    "gs": CameraSpec(
        name="CAM-IMX296Color-GS + M12 2.8mm", width=1456, height=1088, fps=60,
        calib_model="fisheye", exposure_us=800,
        note="글로벌 셔터. 대각 140° 어안이라 반드시 fisheye로 캘리브레이션할 것.",
    ),
    # PC 웹캠 (코드 점검용) ────────────────────────────────────────────────
    "webcam": CameraSpec(
        name="PC 웹캠", width=1280, height=720, fps=30,
        calib_model="pinhole", exposure_us=None,
        note="라파이 없이 코드 흐름만 확인할 때.",
    ),
}

DEFAULT = "csi"


class Camera:
    """Picamera2(라파이) 또는 OpenCV(웹캠/USB)를 같은 인터페이스로 감싼다."""

    def __init__(self, spec: CameraSpec, backend: str = "auto") -> None:
        self.spec = spec
        self._impl = None
        self._kind = None

        if backend in ("auto", "picamera2"):
            try:
                self._open_picamera2()
            except Exception as exc:  # noqa: BLE001
                if backend == "picamera2":
                    raise
                print(f"[camera] Picamera2 사용 불가 ({exc}) -> OpenCV로 대체")
        if self._impl is None:
            self._open_opencv()

    # ── 백엔드 ────────────────────────────────────────────────────────
    def _open_picamera2(self) -> None:
        from picamera2 import Picamera2  # 라파이에만 있다

        cam = Picamera2()
        cfg = cam.create_video_configuration(
            main={"size": (self.spec.width, self.spec.height), "format": "RGB888"},
            controls={"FrameRate": float(self.spec.fps)},
        )
        cam.configure(cfg)
        if self.spec.exposure_us is not None:
            # 노출 고정. 모션 블러를 억제하려면 짧게 잡아야 한다.
            cam.set_controls({"ExposureTime": self.spec.exposure_us, "AeEnable": False})
        cam.start()
        time.sleep(1.0)                      # AWB/AGC 안정화
        self._impl, self._kind = cam, "picamera2"

    def _open_opencv(self) -> None:
        import cv2

        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            raise RuntimeError("카메라를 열 수 없다. 연결과 권한을 확인하라.")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.spec.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.spec.height)
        cap.set(cv2.CAP_PROP_FPS, self.spec.fps)
        self._impl, self._kind = cap, "opencv"

    # ── 사용 ──────────────────────────────────────────────────────────
    @property
    def backend(self) -> str:
        return self._kind or "none"

    def read(self) -> tuple[np.ndarray, float]:
        """(BGR 프레임, 캡처 시각[s])."""
        t = time.monotonic()
        if self._kind == "picamera2":
            frame = self._impl.capture_array()          # RGB888
            return frame[:, :, ::-1].copy(), t          # -> BGR (OpenCV 관례)
        ok, frame = self._impl.read()
        if not ok:
            raise RuntimeError("프레임 읽기 실패")
        return frame, t

    def close(self) -> None:
        if self._impl is None:
            return
        if self._kind == "picamera2":
            self._impl.stop()
        else:
            self._impl.release()
        self._impl = None


def open_camera(profile: str = DEFAULT, backend: str = "auto") -> Camera:
    if profile not in SPECS:
        raise KeyError(f"알 수 없는 프로파일 '{profile}'. 가능: {list(SPECS)}")
    spec = SPECS[profile]
    cam = Camera(spec, backend)
    print(f"[camera] {spec.name} ({spec.width}x{spec.height} @{spec.fps}fps, "
          f"backend={cam.backend}, calib={spec.calib_model})")
    if spec.note:
        print(f"[camera] {spec.note}")
    return cam


def add_profile_arg(parser) -> None:
    """모든 스크립트가 공유하는 --camera 옵션."""
    parser.add_argument(
        "--camera", default=DEFAULT, choices=list(SPECS),
        help=f"카메라 프로파일 (기본 {DEFAULT}). 교체 시 이 값만 바꾸면 된다.",
    )
