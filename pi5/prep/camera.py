"""카메라 추상화 — **카메라를 바꿀 때 손대는 곳은 이 파일 하나다.**

현재 기본값은 "imx219m12" (Pi 8MP IMX219 + M12 렌즈 LS40136 / B0103, 2026-08-10).
카메라를 바꿀 때마다 **수집한 데이터는 재사용할 수 없다** — 화각·왜곡·셔터가 달라
물체가 다른 크기/형태로 찍힌다. 캘리브레이션·수집 로직은 완전히 동일하고,
아래 CameraSpec 값만 바뀐다.

┌─ 카메라 교체 시 확인할 것 ────────────────────────────────────────────────┐
│ ① SPECS에 프로파일 추가/선택          — 해상도, fps, 노출                  │
│ ② 캘리브레이션 모델                   — 아래 표 참고. **이게 가장 중요**    │
│ ③ 수집한 데이터셋은 재사용 불가        — FOV·왜곡·셔터가 달라 다시 찍어야 함 │
└──────────────────────────────────────────────────────────────────────────┘

    | 카메라                 | 렌즈          | 셔터   | 캘리브레이션 모델 |
    |------------------------|---------------|--------|-------------------|
    | CSI v2 (IMX219)        | 고정 ~62°     | 롤링   | pinhole           |
    | CSI v3 (IMX708)        | 고정 ~66°     | 롤링   | pinhole           |
    | IMX219 + M12 LS40136   | 광각 (확인필요) | 롤링   | **화각 보고 결정** |
    | IMX296-GS + 2.8mm      | 어안 140°     | 글로벌 | **fisheye**       |

  화각 90°를 넘어가면 핀홀 모델이 성립하지 않는다. 140° 렌즈를 pinhole로 캘리브레이션하면
  가장자리에서 크게 어긋난다.

  ⚠ 궤적 추정이 중력 기반 최소제곱으로 바뀌었으므로(pi5/trajectory.py) 캘리브레이션
    정확도가 예전보다 **더** 중요해졌다. f_px와 주점이 틀리면 깊이가 통째로 틀어진다.
    반드시 체스보드로 재서 pi5/config.py의 CAMERA_FX/FY/CX/CY에 넣을 것.
"""

from __future__ import annotations

import argparse
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
    max_exposure_us: int | None = None  # auto_lock 전용 "셔터 우선" 상한. AE가 이보다
                                         # 긴 노출을 고르면 이 값으로 줄이고, 줄어든 빛은
                                         # 게인으로 보정한다 (낙하 물체 블러 억제).
    max_gain: float | None = None       # 위 보정의 게인 안전상한. 넘으면 방이 너무
                                         # 어둡다는 뜻 — 조명을 밝히거나 상한을 올릴 것.
    note: str = ""


SPECS: dict[str, CameraSpec] = {
    # 예전 v2 — 데이터 재사용 불가, 파이프라인 점검 용도로만 남겨둠 ──────────
    "csi": CameraSpec(
        name="기본 CSI 카메라", width=1640, height=1232, fps=30,
        calib_model="pinhole", exposure_us=10000, gain=2.0,
        max_exposure_us=3000, max_gain=16.0,   # ← 실측 전 placeholder. "살살 던짐"(쓰레기통
                                                # 투척, ~1.5-2m/s) 가정 — 블러 ~5-6mm로 억제.
                                                # 세게 던지는 걸로 바뀌면 더 낮출 것.
        note="롤링 셔터. 낙하 물체 bbox가 기울어질 수 있다 — 수집 데이터는 임시용. "
             "기본은 auto_lock + 셔터 우선 상한(3000µs)이라 알아서 노출을 짧게 눌러준다. "
             "그래도 흐리면 --max-exposure-us를 더 낮추고, 화면이 어두우면 --max-gain을 "
             "올려라. check_setup.py로 밝기(mean)와 콘솔에 찍히는 실측값을 보며 맞출 것.",
    ),
    # 지금 쓰는 것 — 도착해서 장착 완료 (2026-08-06) ─────────────────────────
    "gs": CameraSpec(
        name="InnoMaker CAM-IMX296Color-GS + 6mm CS 렌즈",
        width=1456, height=1088, fps=60,
        calib_model="pinhole", exposure_us=2000, gain=9.0,
        max_exposure_us=2000, max_gain=16.0,   # auto_lock이 그 이상으로 늘리지 못하게
                                                # 상한을 exposure_us와 같이 잡아뒀다.
                                                # ⚠ 실측으로 확정(2026-08-12, tune_camera.py):
                                                # exposure_us=2000/gain=9 조합이 밝기(mean≈80,
                                                # 건강 범위 40~200)와 노이즈 둘 다 괜찮았다.
                                                # 블러 계산상(2ms×1.5~2m/s≈3~4mm) 여유도 있음.
                                                # ⚠ max_gain은 32까지 실험했다가 노이즈가 너무
                                                # 심해서(2026-08-18) 16으로 다시 내림. 9(실제
                                                # 동작점) 대비 여유는 주면서 32 같은 과도한
                                                # 노이즈 구간엔 안 들어가게 하는 선.
        note="라즈베리파이 공식 GS 카메라와 동일 스펙(제조사가 호환품으로 표기). "
             "Sony IMX296 Color, 1456x1088, 픽셀 3.45µm, 센서 대각 6.3mm(1/2.9\"), "
             "글로벌 셔터, 최대 60fps, C/CS 마운트, 최소 노출 30µs. "
             "출력이 YUV라 공식(RAW10)과 다르지만 libcamera가 변환하므로 "
             "RGB888 요청 그대로 쓰면 된다. 외부 하드웨어 트리거도 지원하나 "
             "센서 타임스탬프로 충분해서 쓰지 않는다. "
             "6mm 렌즈에서 f_px=6.0/3.45µm=1739, "
             "화각 수평45°/수직35°/대각55°. 렌즈 이름의 '광각'은 HQ 카메라(대각 7.9mm) "
             "기준이고 이 센서(6.3mm)에서는 오히려 표준에 가깝다 — **대각 55°라 "
             "pinhole로 캘리브레이션한다. 어안 아니다.** 글로벌 셔터라 낙하 물체가 "
             "기울어지지 않고, 픽셀이 커서(IMX219의 3배) 노출을 아주 짧게 가져갈 수 "
             "있다(제조사 표기 최소 30µs). 모션 블러가 줄면 검출 노이즈가 줄고 그게 "
             "곧 깊이 추정 정확도다 — 이 카메라의 가장 큰 이점.",
    ),
    # 지금 쓰는 것 — IMX219 + M12 교환식 렌즈 (2026-08-10) ──────────────────
    "imx219m12": CameraSpec(
        name="Arducam B0103 (IMX219 + M12 LS40136)", width=1640, height=1232, fps=30,
        calib_model="pinhole", exposure_us=2000, gain=4.0,
        max_exposure_us=3000, max_gain=16.0,
        note="스펙: 센서 3.674x2.760mm, 픽셀 1.12µm, 수평화각 70°(대각 82°) → f_px≈1171 "
             "@1640x1232. 대각 90° 미만이라 pinhole이 맞다(어안 캘리브레이션 불필요). "
             "1640x1232 full FOV 비닝은 30fps가 상한이다 — 1280x720이면 60fps가 되지만 "
             "그건 '크롭'이라 수직화각이 55°->35°로 좁아진다. 궤적 피팅은 프레임 수보다 "
             "관측 시간 스팬이 지배적이고 화각이 좁으면 물체가 빨리 프레임을 벗어나 스팬이 "
             "줄어들므로, full FOV 30fps가 낫다. ⚠ 롤링 셔터다 — 다만 이제 쓰는 건 bbox "
             "'중심'뿐이라 폭이 밀리는 것보다 훨씬 덜 해롭다.",
    ),
    # PC 웹캠 (코드 점검용) ────────────────────────────────────────────────
    "webcam": CameraSpec(
        name="PC 웹캠", width=1280, height=720, fps=30,
        calib_model="pinhole", exposure_us=None, gain=None,
        note="라파이 없이 코드 흐름만 확인할 때.",
    ),
}

DEFAULT = "gs"


class Camera:
    """Picamera2(라파이) 또는 OpenCV(웹캠/USB)를 같은 인터페이스로 감싼다."""

    def __init__(self, spec: CameraSpec, backend: str = "auto", auto_lock: bool = False,
                 want_lores: bool = False) -> None:
        self.spec = spec
        self.auto_lock = auto_lock
        self.want_lores = want_lores
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
        # want_lores: 녹화 중 가벼운 움직임 감지(버저용)를 하려고 저해상도 보조
        # 스트림을 따로 연다. 메인 스트림(녹화용)과 별개 경로라 인코더에 영향 없다.
        lores = {"size": (320, 240), "format": "YUV420"} if self.want_lores else None
        cfg = cam.create_video_configuration(
            main={"size": (self.spec.width, self.spec.height), "format": "RGB888"},
            lores=lores,
            controls={"FrameRate": float(self.spec.fps)},
        )
        cam.configure(cfg)
        try:
            # libcamera 기본 노이즈 리덕션은 여러 프레임을 섞어(temporal blending)
            # 잡음을 줄인다. MOG2 입장에선 최악이다 — 물체가 지나간 자리에 몇 프레임
            # 동안 잔상이 남아 "하얀 덩어리가 천천히 검게 줄어드는" 것처럼 보인다.
            # 배경차분은 프레임끼리 절대 안 섞여야 하므로 끈다.
            from libcamera import controls as libcontrols
            cam.set_controls(
                {"NoiseReductionMode": libcontrols.draft.NoiseReductionModeEnum.Off})
        except Exception as exc:  # noqa: BLE001
            print(f"[camera] NoiseReductionMode 끄기 실패 ({exc}) — "
                  f"MOG2 마스크에 잔상이 남을 수 있다.")
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

            cap = self.spec.max_exposure_us
            if cap is not None and exposure_us > cap:
                # AE는 밝기만 맞추지 블러는 신경 안 쓴다. 노출이 상한을 넘으면
                # "셔터 우선"으로 강제로 줄이고, 잃은 빛의 양만큼 게인을 올려
                # 원래 AE가 맞춰둔 밝기를 최대한 보존한다.
                scale = exposure_us / cap
                wanted_gain = gain * scale
                max_gain = self.spec.max_gain
                capped_gain = min(wanted_gain, max_gain) if max_gain is not None else wanted_gain
                print(f"[camera] AE가 고른 노출({exposure_us}µs)이 상한({cap}µs)보다 길다 — "
                      f"셔터 우선으로 {cap}µs로 줄이고 게인을 {gain:.2f}->{capped_gain:.2f}로 보정.")
                if max_gain is not None and wanted_gain > max_gain:
                    print(f"[camera] ⚠ 사실 게인이 {wanted_gain:.1f}까지 필요한데 상한"
                          f"({max_gain})에 걸렸다 — 화면이 어두울 수 있다. 조명을 밝히거나 "
                          f"--max-exposure-us를 늘려라.")
                exposure_us, gain = cap, capped_gain

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
            # ⚠ Picamera2의 유명한 함정: format="RGB888"로 설정해도 capture_array()는
            # 이미 BGR 순서로 채워서 준다 (libcamera 픽셀 포맷 이름과 실제 메모리
            # 순서가 반대). 예전엔 여기서 [:, :, ::-1]로 한 번 더 뒤집었는데, 이미
            # BGR인 걸 또 뒤집으면 R/B가 스왑돼 빨간 물체가 파랗게 찍힌다.
            return self._impl.capture_array().copy(), t
        ok, frame = self._impl.read()
        if not ok:
            raise RuntimeError("프레임 읽기 실패")
        return frame, t

    def read_with_sensor_ts(self) -> tuple[np.ndarray, float]:
        """(BGR 프레임, 센서 타임스탬프[s]). Picamera2 전용 — 실시간 궤적 피팅(vision.py)이
        쓴다.

        read()와 다르게 time.monotonic()(파이썬이 프레임을 받은 시각, 스케줄링 지터가
        섞임) 대신 **SensorTimestamp**(센서가 실제로 노출을 끊은 시각)를 쓴다.
        capture_request()로 프레임과 메타데이터를 한 번에 받아야 서로 다른 프레임의
        값이 섞이지 않는다 (capture_array()+capture_metadata()를 따로 부르면 그 사이
        다음 프레임이 끼어들 수 있다).

        SensorTimestamp의 기준(0점)은 임의(보통 부팅 시각)라 절대값은 의미 없다 —
        trajectory.py도 차이만 쓰므로 문제없다.
        """
        if self._kind != "picamera2":
            raise RuntimeError("read_with_sensor_ts는 Picamera2 전용이다.")
        request = self._impl.capture_request()
        try:
            frame = request.make_array("main").copy()
            meta = request.get_metadata()
        finally:
            request.release()
        t = meta["SensorTimestamp"] / 1e9   # ns -> s
        return frame, t

    def capture_lores(self) -> np.ndarray:
        """저해상도 보조 스트림에서 밝기(흑백) 프레임만 뽑는다.

        want_lores=True로 열었을 때만 쓸 수 있다. capture_video.py가 녹화 중
        가벼운 움직임 감지(버저)용으로 쓴다 — 메인 스트림/인코더와는 무관하다.
        """
        if self._kind != "picamera2" or not self.want_lores:
            raise RuntimeError("capture_lores는 want_lores=True로 연 Picamera2에서만 된다.")
        arr = self._impl.capture_array("lores")
        # YUV420 평면 배열: 위쪽 2/3가 밝기(Y) 평면이다. 움직임 감지엔 밝기면 충분해서
        # 색상(U/V) 평면은 버린다.
        y_h = arr.shape[0] * 2 // 3
        return arr[:y_h]

    def set_manual(self, exposure_us: int | None = None, gain: float | None = None) -> None:
        """실행 중에 노출/게인을 바꾼다 (Picamera2 전용, tune_camera.py가 씀).

        재시작 없이 바로 반영된다 — AE/AWB는 계속 꺼둔 채로 값만 갱신한다.
        """
        if self._kind != "picamera2":
            print("[camera] set_manual은 Picamera2 전용이다 — OpenCV 백엔드에서는 무시된다.")
            return
        controls: dict = {"AeEnable": False, "AwbEnable": False}
        if exposure_us is not None:
            controls["ExposureTime"] = exposure_us
        if gain is not None:
            controls["AnalogueGain"] = gain
        self._impl.set_controls(controls)
        self.spec = dataclasses.replace(
            self.spec,
            exposure_us=exposure_us if exposure_us is not None else self.spec.exposure_us,
            gain=gain if gain is not None else self.spec.gain,
        )

    def start_recording(self, path: str, bitrate: int = 20_000_000,
                         codec: str = "h264") -> None:
        """영상 녹화 시작 (Picamera2 전용, capture_video.py가 씀).

        MOG2/imwrite 없이 인코더로 바로 쓰므로 캡처 중 Python 처리로 인한 프레임
        드랍이 없다 — 단, 인코딩 자체가 병목이 될 수 있다.

        ⚠ 라즈베리파이 5는 하드웨어 H.264 인코더가 없다(Pi4까지는 있었음). 그래서
        `codec="h264"`는 소프트웨어 인코딩이라 1456x1088@60fps를 못 따라가면
        녹화 중 자체적으로 프레임이 밀릴 수 있다. `codec="mjpeg"`는 프레임마다
        독립적으로 JPEG 압축하는 방식이라 계산이 훨씬 가볍고(모션 보상 없음),
        capture_dataset.py가 원래 쓰던 프레임 단위 JPEG 압축과 특성이 같아
        검출 노이즈 걱정도 덜하다. 대신 파일 용량은 더 크다. 어느 쪽이 실제로
        fps를 잘 유지하는지는 실기에서 확인해야 한다.

        ffmpeg가 시스템에 설치돼 있어야 한다 (`sudo apt install ffmpeg`).
        """
        if self._kind != "picamera2":
            raise RuntimeError("영상 녹화는 Picamera2 전용이다.")
        from picamera2.outputs import FfmpegOutput

        if codec == "h264":
            from picamera2.encoders import H264Encoder
            self._encoder = H264Encoder(bitrate=bitrate)
        elif codec == "mjpeg":
            from picamera2.encoders import MJPEGEncoder
            self._encoder = MJPEGEncoder(bitrate=bitrate)
        else:
            raise ValueError(f"알 수 없는 codec: {codec!r} ('h264' 또는 'mjpeg')")
        self._impl.start_recording(self._encoder, FfmpegOutput(path))

    def stop_recording(self) -> None:
        if self._kind == "picamera2" and getattr(self, "_encoder", None) is not None:
            self._impl.stop_recording()
            self._encoder = None

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
                 auto_lock: bool | None = None,
                 max_exposure_us: int | None = None, max_gain: float | None = None,
                 want_lores: bool = False) -> Camera:
    """auto_lock=None(기본)이면: 수동으로 exposure_us/gain을 안 줬을 때만 자동측정 후
    고정한다. 매번 킬 때 조명이 뭐가 됐든 알아서 재고 고정하는 게 기본 동작이고,
    명시적으로 숫자를 준 경우에만 그 숫자를 존중해 자동측정을 건너뛴다.
    max_exposure_us/max_gain은 auto_lock이 고른 노출이 너무 길 때(=블러) 강제로
    줄이고 게인으로 보정하는 "셔터 우선" 상한이다."""
    if profile not in SPECS:
        raise KeyError(f"알 수 없는 프로파일 '{profile}'. 가능: {list(SPECS)}")
    spec = SPECS[profile]
    manual = exposure_us is not None or gain is not None
    if auto_lock is None:
        auto_lock = not manual
    elif auto_lock and manual:
        print("[camera] --auto-lock-exposure와 --exposure-us/--gain을 같이 줬다 — "
              "auto_lock이 우선이고 수동 값은 무시된다.")
    overrides = {}
    if not auto_lock and manual:
        # 조명이 세션마다 바뀌므로 SPECS 기본값을 매번 코드에서 고치는 대신
        # CLI에서 덮어쓴다.
        if exposure_us is not None:
            overrides["exposure_us"] = exposure_us
        if gain is not None:
            overrides["gain"] = gain
    if max_exposure_us is not None:
        overrides["max_exposure_us"] = max_exposure_us
    if max_gain is not None:
        overrides["max_gain"] = max_gain
    if overrides:
        spec = dataclasses.replace(spec, **overrides)
    cam = Camera(spec, backend, auto_lock=auto_lock, want_lores=want_lores)
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
        "--auto-lock-exposure", dest="auto_lock_exposure", default=None,
        action=argparse.BooleanOptionalAction,
        help="시작할 때 AE/AWB를 잠깐 켜서 지금 조명에 맞는 노출/게인을 재고, 그 값으로 "
             "고정한 채 촬영한다. 기본 동작이라 안 줘도 켜진다 — --exposure-us/--gain을 "
             "직접 준 경우에만 자동으로 꺼진다. 굳이 끄려면 --no-auto-lock-exposure.",
    )
    parser.add_argument(
        "--max-exposure-us", type=int, default=None,
        help="auto-lock 전용 '셔터 우선' 상한(µs). AE가 이보다 긴 노출을 고르면 이 값으로 "
             "줄이고 부족한 빛은 게인으로 보정한다. 낙하 물체가 흐리게 찍히면 낮출 것 "
             "(프로파일 기본값을 덮어씀).",
    )
    parser.add_argument(
        "--max-gain", type=float, default=None,
        help="위 보정에서 게인이 이 값을 넘지 않게 막는 안전상한. 여기 걸리면 경고가 뜨는데, "
             "그건 방이 그 셔터스피드로는 너무 어둡다는 뜻이다 (프로파일 기본값을 덮어씀).",
    )
