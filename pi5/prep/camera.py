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

import dataclasses
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
    gain: float | None = None  # None이면 자동. exposure_us를 짧게 유지하며 밝기를
                                # 확보하려면 이걸 올린다 (TROUBLESHOOTING.md 2-1).
    note: str = ""


SPECS: dict[str, CameraSpec] = {
    # 지금 쓰는 것 ────────────────────────────────────────────────────────
    "csi": CameraSpec(
        name="기본 CSI 카메라", width=1640, height=1232, fps=30,
        calib_model="pinhole", exposure_us=10000, gain=2.0,
        note="롤링 셔터. 낙하 물체 bbox가 기울어질 수 있다 — 수집 데이터는 임시용. "
             "exposure_us/gain은 조명 밝기 보고 조정할 것 — 어두우면 gain부터 올리고, "
             "그래도 부족하면 exposure_us를 올린다 (블러 대신 밝기를 게인으로 확보). "
             "check_setup.py의 밝기(mean) 진단을 보며 --exposure-us/--gain 으로 맞출 것.",
    ),
    # 도착하면 이걸로 ─────────────────────────────────────────────────────
    "gs": CameraSpec(
        name="CAM-IMX296Color-GS + M12 2.8mm", width=1456, height=1088, fps=60,
        calib_model="fisheye", exposure_us=1000, gain=4.0,
        note="글로벌 셔터. 대각 140° 어안이라 반드시 fisheye로 캘리브레이션할 것. "
             "exposure_us/gain은 data_collection/brain/capture/source.py에서 검증된 값.",
    ),
    # PC 웹캠 (코드 점검용) ────────────────────────────────────────────────
    "webcam": CameraSpec(
        name="PC 웹캠", width=1280, height=720, fps=30,
        calib_model="pinhole", exposure_us=None, gain=None,
        note="라파이 없이 코드 흐름만 확인할 때.",
    ),
}

DEFAULT = "csi"


class Camera:
    """Picamera2(라파이) 또는 OpenCV(웹캠/USB)를 같은 인터페이스로 감싼다."""

    def __init__(self, spec: CameraSpec, backend: str = "auto", auto_lock: bool = False) -> None:
        self.spec = spec
        self.auto_lock = auto_lock
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
            if self.auto_lock:
                print("[camera] auto_lock은 Picamera2 전용이다 — OpenCV 백엔드에선 무시된다.")
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
        cam.start()
        time.sleep(1.0)                      # AWB/AGC 안정화 (기본은 auto로 뜬다)

        if self.auto_lock:
            # 자동 노출/화밸을 켜둔 채 지금 조명에 수렴할 때까지 기다린 다음,
            # 그 순간의 값을 읽어 고정한다. MOG2는 프레임마다 노출이 흔들리면
            # "배경이 움직였다"고 오인하므로 결국 고정은 필수인데, 값 자체는
            # 조명이 바뀔 때마다 사람이 손으로 맞추지 않고 카메라가 재기 하게 둔다.
            cam.set_controls({"AeEnable": True, "AwbEnable": True})
            for _ in range(30):
                cam.capture_array()
            meta = cam.capture_metadata()
            exposure_us = int(meta["ExposureTime"])
            gain = float(meta["AnalogueGain"])
            lock = {"AeEnable": False, "AwbEnable": False,
                    "ExposureTime": exposure_us, "AnalogueGain": gain}
            if "ColourGains" in meta:
                lock["ColourGains"] = meta["ColourGains"]   # AWB도 같이 얼린다
            cam.set_controls(lock)
            time.sleep(0.3)                  # 고정값 적용 안정화
            self.spec = dataclasses.replace(self.spec, exposure_us=exposure_us, gain=gain)
            print(f"[camera] 자동 노출을 읽어서 고정했다 — "
                  f"exposure_us={exposure_us}, gain={gain:.2f}")
        elif self.spec.exposure_us is not None:
            # 노출 고정. 모션 블러를 억제하려면 짧게 잡아야 한다.
            # AWB도 같이 꺼야 한다 — 밝기만 고정하고 색온도가 계속 자동이면
            # 채널별 값이 흔들려서 MOG2가 여전히 배경을 "움직인다"고 오인한다.
            controls = {"ExposureTime": self.spec.exposure_us, "AeEnable": False,
                        "AwbEnable": False}
            if self.spec.gain is not None:
                # 노출은 짧게 유지(모션 블러 억제)하고 밝기는 게인으로 확보한다.
                # AE를 끈 상태에서 게인을 안 정해주면 꺼지기 직전 값이 그대로 남아
                # 세션마다 밝기가 들쭉날쭉해진다.
                controls["AnalogueGain"] = self.spec.gain
            cam.set_controls(controls)
            time.sleep(0.3)                  # 고정값 적용 안정화
        # 둘 다 아니면(exposure_us=None, auto_lock=False) AE/AWB를 계속 켜둔 채로 둔다
        # — webcam 프로파일처럼 애초에 고정이 필요 없는 경우.

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


def open_camera(profile: str = DEFAULT, backend: str = "auto",
                 exposure_us: int | None = None, gain: float | None = None,
                 auto_lock: bool = False) -> Camera:
    if profile not in SPECS:
        raise KeyError(f"알 수 없는 프로파일 '{profile}'. 가능: {list(SPECS)}")
    spec = SPECS[profile]
    if auto_lock and (exposure_us is not None or gain is not None):
        print("[camera] --auto-lock-exposure와 --exposure-us/--gain을 같이 줬다 — "
              "auto_lock이 우선이고 수동 값은 무시된다.")
    if not auto_lock and (exposure_us is not None or gain is not None):
        # 조명이 세션마다 바뀌므로 SPECS 기본값을 매번 코드에서 고치는 대신
        # CLI에서 덮어쓴다.
        overrides = {}
        if exposure_us is not None:
            overrides["exposure_us"] = exposure_us
        if gain is not None:
            overrides["gain"] = gain
        spec = dataclasses.replace(spec, **overrides)
    cam = Camera(spec, backend, auto_lock=auto_lock)
    spec = cam.spec   # auto_lock이면 여기서 실측값으로 갱신돼 있다
    print(f"[camera] {spec.name} ({spec.width}x{spec.height} @{spec.fps}fps, "
          f"backend={cam.backend}, calib={spec.calib_model}, "
          f"exposure_us={spec.exposure_us}, gain={spec.gain})")
    if spec.note:
        print(f"[camera] {spec.note}")
    return cam


def add_profile_arg(parser) -> None:
    """모든 스크립트가 공유하는 --camera/--exposure-us/--gain/--auto-lock-exposure 옵션."""
    parser.add_argument(
        "--camera", default=DEFAULT, choices=list(SPECS),
        help=f"카메라 프로파일 (기본 {DEFAULT}). 교체 시 이 값만 바꾸면 된다.",
    )
    parser.add_argument(
        "--exposure-us", type=int, default=None,
        help="프로파일 기본 노출(µs)을 덮어쓴다. 조명 바꿀 때마다 camera.py를 "
             "고치지 않고 이걸로 튜닝할 것 (check_setup.py의 밝기 진단을 보면서).",
    )
    parser.add_argument(
        "--gain", type=float, default=None,
        help="프로파일 기본 게인을 덮어쓴다. 어두우면 노출보다 이걸 먼저 올려라 — "
             "노출을 늘리면 낙하 물체 모션 블러가 커진다.",
    )
    parser.add_argument(
        "--auto-lock-exposure", dest="auto_lock_exposure", action="store_true",
        help="시작할 때 AE/AWB를 잠깐 켜서 지금 조명에 맞는 노출/게인을 재고, 그 값으로 "
             "고정한 채 촬영한다. --exposure-us/--gain 수동 지정보다 우선한다.",
    )
