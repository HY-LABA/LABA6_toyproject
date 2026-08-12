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

┌─ ⚠ 화면이 어두운 문제 ────────────────────────────────────────────────────┐
│ 이 프로젝트는 **카메라가 천장을 향한다.** 천장에는 조명이 있다.             │
│ 즉 화면 안에 아주 밝은 광원이 들어오고, 자동노출(AE)이 그 광원에 맞춰       │
│ 노출을 줄여버린다. 결과적으로 **정작 보고 싶은 물체가 까맣게 나온다.**      │
│ (사진에서 역광 인물이 실루엣이 되는 것과 같은 현상)                        │
│                                                                          │
│ 원인이 둘 중 무엇인지 반드시 먼저 구분할 것 — 해법이 정반대다:             │
│   ⓐ AE가 조명에 속았다   → 노출 여유가 남아 있다. EV를 올리면 해결된다     │
│   ⓑ 빛이 진짜로 부족하다 → 노출·게인이 이미 한계다. 소프트웨어로는 못 고친다│
│                                                                          │
│ `diagnose()`가 이 둘을 구분해준다. 추측하지 말고 그 출력을 볼 것.          │
└──────────────────────────────────────────────────────────────────────────┘
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
    gain: float | None = None  # None이면 자동 (아날로그 게인)
    ev: float = 0.0           # 자동노출 보정. +1 = 두 배 밝게. 역광 보정용
    note: str = ""


SPECS: dict[str, CameraSpec] = {
    # 지금 쓰는 것 ────────────────────────────────────────────────────────
    "csi": CameraSpec(
        name="기본 CSI 카메라", width=1640, height=1232, fps=30,
        calib_model="pinhole", exposure_us=None, gain=None,
        # 천장 조명이 화면에 들어와 AE가 어둡게 잡는다. +1.5 EV(약 2.8배)로 시작한다.
        # 여전히 어두우면 실행 중 ] 키로 더 올리면 된다.
        ev=1.5,
        note="롤링 셔터. 낙하 물체 bbox가 기울어질 수 있다 — 수집 데이터는 임시용.",
    ),
    # 도착하면 이걸로 ─────────────────────────────────────────────────────
    "gs": CameraSpec(
        name="CAM-IMX296Color-GS + M12 2.8mm", width=1456, height=1088, fps=60,
        calib_model="fisheye", exposure_us=800, gain=8.0, ev=0.0,
        # 노출 800us는 모션 블러를 막기 위한 값이라 매우 어둡다. 게인으로 벌충하고,
        # 그래도 모자라면 조명(스트로브 LED)이 답이다 — ../docs/hardware.md 3.1장.
        note="글로벌 셔터. 대각 140° 어안이라 반드시 fisheye로 캘리브레이션할 것.",
    ),
    # PC 웹캠 (코드 점검용) ────────────────────────────────────────────────
    "webcam": CameraSpec(
        name="PC 웹캠", width=1280, height=720, fps=30,
        calib_model="pinhole", exposure_us=None, gain=None, ev=0.0,
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
        self.exposure_us = spec.exposure_us
        self.gain = spec.gain
        self.ev = spec.ev
        self._gain_warned = False

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
        cam.start()
        self._impl, self._kind = cam, "picamera2"
        self._apply_controls()
        self._settle()

    def _open_opencv(self) -> None:
        import cv2

        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            raise RuntimeError("카메라를 열 수 없다. 연결과 권한을 확인하라.")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.spec.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.spec.height)
        cap.set(cv2.CAP_PROP_FPS, self.spec.fps)
        self._impl, self._kind = cap, "opencv"
        self._apply_controls()

    # ── 노출 제어 ─────────────────────────────────────────────────────
    @property
    def max_exposure_us(self) -> int:
        """이 프레임레이트에서 물리적으로 가능한 최대 노출. 여유가 남았는지 판정 기준."""
        return int(1e6 / self.spec.fps)

    def _apply_controls(self) -> None:
        if self._kind == "picamera2":
            auto = self.exposure_us is None
            ctrl: dict = {"AeEnable": auto}
            if auto:
                # 화면 안의 조명이 AE를 끌어내린다. ExposureValue로 그만큼 되올린다.
                ctrl["ExposureValue"] = float(self.ev)
                try:                              # 구버전 libcamera에는 없을 수 있다
                    from libcamera import controls as _c
                    # 밝은 광원(천장등)에 덜 끌리도록 중앙 가중으로 측광한다
                    ctrl["AeMeteringMode"] = _c.AeMeteringModeEnum.CentreWeighted
                except Exception:  # noqa: BLE001
                    pass
                if self.gain is not None and not self._gain_warned:
                    # AeEnable=True면 게인은 AE가 정한다. 사용자가 준 값은 먹지 않는다.
                    print("[camera] ⚠ 자동노출에서는 --gain이 무시된다 (AE가 게인을 정한다). "
                          "--exposure 로 수동 전환하거나 --ev 를 올려라")
                    self._gain_warned = True
            else:
                ctrl["ExposureTime"] = int(self.exposure_us)
                ctrl["AnalogueGain"] = float(self.gain or 1.0)
            self._impl.set_controls(ctrl)
        elif self._kind == "opencv":
            import cv2
            cap = self._impl
            if self.exposure_us is None:
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)   # V4L2: 0.75=자동
                # 웹캠에는 EV 컨트롤이 없는 경우가 많아 밝기로 대신한다
                if self.ev:
                    cap.set(cv2.CAP_PROP_BRIGHTNESS, 128 + 30 * self.ev)
            else:
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)   # V4L2: 0.25=수동
                # V4L2 노출 단위는 100us. 드라이버마다 다르니 참고값이다.
                cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure_us / 100.0)
                if self.gain is not None:
                    cap.set(cv2.CAP_PROP_GAIN, self.gain)

    def _settle(self, timeout_s: float = 3.0) -> None:
        """AE/AWB가 수렴할 때까지 기다린다.

        고정 sleep이 아니라 **실제 노출값이 안정될 때까지** 기다린다.
        1초 고정으로는 어두운 실내에서 수렴이 안 끝나 첫 프레임이 까맣게 나온다.
        """
        if self._kind != "picamera2":
            time.sleep(0.5)
            return
        deadline = time.monotonic() + timeout_s
        prev = None
        while time.monotonic() < deadline:
            time.sleep(0.15)
            md = self._impl.capture_metadata()
            cur = md.get("ExposureTime")
            if cur is None:
                break
            if prev is not None and abs(cur - prev) <= max(50, prev * 0.05):
                return
            prev = cur

    def set_ev(self, ev: float) -> None:
        """자동노출 보정. +1 = 노출 두 배. 역광(천장 조명) 보정의 주 수단."""
        self.ev = max(-8.0, min(8.0, ev))
        self._apply_controls()

    def set_exposure(self, exposure_us: int | None, gain: float | None = None) -> None:
        """수동 노출로 전환한다. None을 주면 다시 자동으로 돌아간다."""
        self.exposure_us = exposure_us
        if gain is not None:
            self.gain = gain
        self._apply_controls()
        self._settle(timeout_s=1.0)

    # ── 사용 ──────────────────────────────────────────────────────────
    @property
    def backend(self) -> str:
        return self._kind or "none"

    def read(self) -> tuple[np.ndarray, float]:
        """(BGR 프레임, 캡처 시각[s])."""
        t = time.monotonic()
        if self._kind == "picamera2":
            # ⚠ Picamera2에서 "RGB888"은 실제로 메모리상 B,G,R 순서다 (libcamera가
            #   포맷 이름을 리틀엔디안 순으로 붙이기 때문). 즉 이미 OpenCV의 BGR이라
            #   여기서 채널을 뒤집으면 오히려 적/청이 뒤바뀐다. 그대로 넘긴다.
            return self._impl.capture_array(), t
        ok, frame = self._impl.read()
        if not ok:
            raise RuntimeError("프레임 읽기 실패")
        return frame, t

    def stats(self) -> dict:
        """현재 노출/게인/밝기. 어두움의 원인을 가르는 근거 데이터."""
        out: dict = {"exposure_us": self.exposure_us, "gain": self.gain,
                     "ev": self.ev, "auto": self.exposure_us is None,
                     # 실제 적용된 노출을 읽어올 수 있는가. 못 읽으면 진단이 추측이 된다.
                     "measured": False}
        if self._kind == "picamera2":
            try:
                md = self._impl.capture_metadata()
                out["exposure_us"] = md.get("ExposureTime", out["exposure_us"])
                out["gain"] = md.get("AnalogueGain", out["gain"])
                out["digital_gain"] = md.get("DigitalGain")
                out["lux"] = md.get("Lux")
                out["measured"] = md.get("ExposureTime") is not None
            except Exception:  # noqa: BLE001
                pass
        elif self._kind == "opencv":
            try:
                import cv2
                # V4L2/DirectShow는 노출을 못 돌려주는 드라이버가 흔하다. 0이면 모르는 것.
                e = self._impl.get(cv2.CAP_PROP_EXPOSURE)
                g = self._impl.get(cv2.CAP_PROP_GAIN)
                if e and e > 0:
                    out["exposure_us"], out["measured"] = e * 100.0, True
                if g and g > 0:
                    out["gain"] = g
            except Exception:  # noqa: BLE001
                pass
        return out

    def close(self) -> None:
        if self._impl is None:
            return
        if self._kind == "picamera2":
            self._impl.stop()
        else:
            self._impl.release()
        self._impl = None


# ── 어두움 진단 ────────────────────────────────────────────────────────

def diagnose(cam: Camera, frame: np.ndarray) -> tuple[bool, str]:
    """(문제없음, 설명)을 돌려준다.

    핵심은 **노출 여유가 남았는지**다.
      · 여유가 남았는데 어둡다  -> AE가 조명에 속은 것. EV를 올리면 해결된다
      · 여유가 없는데 어둡다    -> 빛이 부족한 것. 소프트웨어로는 못 고친다
    이 구분을 안 하면 EV만 계속 올리다가 노이즈만 키우게 된다.
    """
    mean = float(np.asarray(frame).mean())
    st = cam.stats()
    exp = st.get("exposure_us") or 0
    gain = st.get("gain") or 1.0
    cap = cam.max_exposure_us
    headroom = exp < cap * 0.6           # 노출을 아직 더 늘릴 수 있는가

    if mean >= 60:
        return True, f"밝기 평균 {mean:.0f} — 정상"

    if not st.get("measured"):
        # 노출을 못 읽으면 ⓐ와 ⓑ를 가를 근거가 없다. 아는 척하지 않는다.
        return False, (f"밝기 평균 {mean:.0f} (60 이상 권장) — 어둡다.\n"
                       f"    -> 이 백엔드({cam.backend})는 실제 노출값을 돌려주지 않아 "
                       f"원인을 자동으로 가를 수 없다.\n"
                       f"       순서대로 시도: --ev 1.5  ->  --ev 3  ->  "
                       f"--exposure {cap // 2} --gain 4  ->  그래도 어두우면 조명 문제다.\n"
                       f"       (렌즈 캡·프라이버시 셔터도 확인할 것)")

    detail = (f"밝기 평균 {mean:.0f} (60 이상 권장), 노출 {exp/1000:.1f}ms "
              f"/ 최대 {cap/1000:.1f}ms, 게인 {gain:.1f}x")
    starved = (f"{detail}\n"
               f"    -> **빛이 부족하다. 소프트웨어로는 못 고친다.**\n"
               f"       조명을 켜거나, fps를 낮춰 노출 상한을 늘려라 "
               f"(30fps -> 최대 33ms, 60fps -> 16ms).")

    if st["auto"]:
        # 자동노출에서는 노출도 게인도 AE가 정한다. 사람이 돌릴 손잡이는 EV 하나뿐이다.
        if headroom:
            return False, (f"{detail}\n"
                           f"    -> **AE가 천장 조명에 속았다.** 노출 여유가 남아 있는데도 어둡다.\n"
                           f"       EV를 올려라:  --ev {cam.ev + 1.5:.1f}   (실행 중에는 ] 키)")
        if gain < 8:
            return False, (f"{detail}\n"
                           f"    -> 노출은 한계지만 AE가 게인을 더 쓸 여지가 있다.\n"
                           f"       --ev {cam.ev + 1.0:.1f} 로 AE를 밀어라 (실행 중에는 ] 키)")
        return False, starved

    if headroom:
        return False, (f"{detail}\n"
                       f"    -> **수동 노출이 너무 짧다.** 여유가 남아 있다.\n"
                       f"       --exposure {min(cap, max(2000, exp * 3)):.0f}"
                       f"  또는 --gain {gain*2:.0f}")
    if gain < 8:
        return False, (f"{detail}\n"
                       f"    -> 노출은 한계인데 게인 여유가 있다.  --gain {min(16, gain*2):.0f}")
    return False, starved


def preview_boost(frame: np.ndarray, gamma: float = 0.45) -> np.ndarray:
    """**화면 확인용으로만** 밝게 편다. 저장되는 이미지에는 영향이 없다.

    감마 보정이라 어두운 쪽을 크게 끌어올리면서 밝은 쪽은 덜 날린다 —
    천장 조명처럼 명암 차가 큰 장면에 단순 곱셈보다 낫다.
    """
    lut = (np.linspace(0, 1, 256) ** gamma * 255).astype(np.uint8)
    return lut[np.asarray(frame)]


# ── CLI 배선 ───────────────────────────────────────────────────────────

def open_camera(profile: str = DEFAULT, backend: str = "auto", *,
                exposure_us: int | None = None, gain: float | None = None,
                ev: float | None = None, auto: bool | None = None) -> Camera:
    if profile not in SPECS:
        raise KeyError(f"알 수 없는 프로파일 '{profile}'. 가능: {list(SPECS)}")
    spec = SPECS[profile]
    if exposure_us is not None:
        spec = replace_spec(spec, exposure_us=exposure_us)
    if auto:
        spec = replace_spec(spec, exposure_us=None)
    if gain is not None:
        spec = replace_spec(spec, gain=gain)
    if ev is not None:
        spec = replace_spec(spec, ev=ev)

    cam = Camera(spec, backend)
    st = cam.stats()
    mode = "자동노출" if st["auto"] else f"수동 {st['exposure_us']}us"
    print(f"[camera] {spec.name} ({spec.width}x{spec.height} @{spec.fps}fps, "
          f"backend={cam.backend}, calib={spec.calib_model})")
    print(f"[camera] {mode}, EV {spec.ev:+.1f}, 게인 {st.get('gain') or 'auto'}"
          + (f", 조도 {st['lux']:.0f} lux" if st.get("lux") else ""))
    if spec.note:
        print(f"[camera] {spec.note}")
    return cam


def replace_spec(spec: CameraSpec, **kw) -> CameraSpec:
    import dataclasses
    return dataclasses.replace(spec, **kw)


def add_profile_arg(parser) -> None:
    """모든 스크립트가 공유하는 카메라 옵션 (프로파일 + 노출)."""
    parser.add_argument(
        "--camera", default=DEFAULT, choices=list(SPECS),
        help=f"카메라 프로파일 (기본 {DEFAULT}). 교체 시 이 값만 바꾸면 된다.",
    )
    g = parser.add_argument_group(
        "노출 (화면이 어두울 때)",
        "천장 조명이 화면에 들어오면 자동노출이 물체를 어둡게 만든다. 먼저 --ev를 올려볼 것.")
    g.add_argument("--ev", type=float, default=None,
                   help="자동노출 보정. +1 = 두 배 밝게. 역광 보정의 주 수단")
    g.add_argument("--exposure", type=int, default=None, dest="exposure_us",
                   help="수동 노출(us). 지정하면 자동노출을 끈다")
    g.add_argument("--gain", type=float, default=None, help="아날로그 게인. 밝아지지만 노이즈도 는다")
    g.add_argument("--auto-exposure", action="store_true", dest="auto",
                   help="프로파일이 수동이어도 자동노출로 강제 전환")


def open_from_args(args, backend: str = "auto") -> Camera:
    """add_profile_arg로 받은 인자를 그대로 카메라에 반영해 연다."""
    return open_camera(args.camera, backend,
                       exposure_us=getattr(args, "exposure_us", None),
                       gain=getattr(args, "gain", None),
                       ev=getattr(args, "ev", None),
                       auto=getattr(args, "auto", None))
