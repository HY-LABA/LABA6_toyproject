"""
카메라 캡처 + YOLO 검출 → **bbox 중심 픽셀 좌표**.

"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import config
import utils


@dataclass
class Detection:

    u: float                # bbox 중심 x (px, 왜곡 보정 후)
    v: float                # bbox 중심 y (px, 왜곡 보정 후)
    t: float                # 캡처 시각 (s)
    confidence: float
    bbox: tuple[float, float, float, float]  # cx, cy, w, h (원본 픽셀, 보정 전)


def _prep_dir() -> "pathlib.Path":
    import pathlib
    return pathlib.Path(__file__).resolve().parent / "prep"


class Camera:
    """Picamera2 래퍼. prep/camera.py를 그대로 재사용한다 — 노출/게인/auto_lock/
    노이즈리덕션 설정이 전부 거기 있고, 카메라를 바꿔도 여긴 안 건드려도 된다.

    ⚠ **타임스탬프는 아직 개선 여지가 있다.** 지금은 파이썬이 버퍼를 받은 시각
    (`time.monotonic()`)을 쓰는데, 여기엔 스케줄링 지터가 섞여 있고 피팅은 그 지터를
    물체의 운동으로 읽는다. picamera2 메타데이터의 `SensorTimestamp`를 쓰면 줄일 수
    있다 — 다만 실기에서 두 시계의 기준(CLOCK_MONOTONIC vs CLOCK_BOOTTIME)이 맞는지
    확인한 뒤에 바꿔야 한다. 정확도가 기대만큼 안 나오면 여기를 의심할 것.
    """

    def __init__(self, resolution: tuple[int, int], fps: int) -> None:
        self.resolution = resolution
        self.fps = fps

        import sys
        prep_dir = str(_prep_dir())
        if prep_dir not in sys.path:
            sys.path.insert(0, prep_dir)
        import camera as camlib  # prep/camera.py

        # config.CAMERA_RESOLUTION/FPS는 프로파일 값과 같아야 한다 
        # 카메라를 바꾸면 prep/camera.py에 새 프로파일을 추가하고 DEFAULT도 옮길 것.
        profile = camlib.DEFAULT
        spec = camlib.SPECS[profile]
        if (spec.width, spec.height) != tuple(resolution) or spec.fps != fps:
            print(f"[vision.Camera] ⚠ config.CAMERA_RESOLUTION/FPS "
                  f"({resolution[0]}x{resolution[1]}@{fps})가 프로파일 '{profile}' "
                  f"({spec.width}x{spec.height}@{spec.fps})와 다르다 — "
                  f"prep/camera.py의 SPECS나 config.py를 확인할 것.")

        # backend="picamera2" 고정
        self._cam = camlib.open_camera(profile=profile, backend="picamera2")

    def capture(self) -> tuple[np.ndarray, float]:
        """(BGR 프레임, 캡처 시각[s]). 시각은 `time.monotonic()` 기준."""
        return self._cam.read()

    def close(self) -> None:
        self._cam.close()


class _HailoYolo:
    """HailoRT 4.24.0 (hailo_platform) 기준.

    `hailortcli parse-hef`로 실측 확인함 (2026-08-24):
        입력  best/input_layer1              UINT8, NHWC(640x640x3)
        출력  best/yolov8_nms_postprocess     FLOAT32, HAILO NMS BY CLASS
              (클래스 1개, 클래스당 최대 100박스) — YOLOV8-Post-Process 연산이
              HEF 안에 이미 구워져 있다. **NMS를 호스트에서 직접 안 해도 된다** —
              train_yolo.py 주석의 "NMS는 host에서" 전제가 필요 없어졌다.

    ⚠ HAILO_NMS 출력의 정확한 파이썬 반환 형태(클래스별 리스트인지, 각 박스가
    [ymin,xmin,ymax,xmax,score]인지 [xmin,ymin,xmax,ymax,score]인지)는
    HailoRT 문서 기준 관례를 따라 짰지만 **실기에서 첫 실행 때 반드시 확인할 것**
    — `_decode_output()`의 진단 print를 보고, bbox가 화면 밖 이상한 위치에
    찍히면 좌표 순서를 의심할 것.
    """

    def __init__(self, model_path: str) -> None:
        from hailo_platform import (HEF, VDevice, ConfigureParams,
                                     InputVStreamParams, OutputVStreamParams,
                                     HailoStreamInterface, FormatType)

        self.model_path = model_path
        self._hef = HEF(model_path)

        self._vdevice = VDevice(VDevice.create_params())

        configure_params = ConfigureParams.create_from_hef(
            hef=self._hef, interface=HailoStreamInterface.PCIe)
        network_groups = self._vdevice.configure(self._hef, configure_params)
        self._network_group = network_groups[0]
        self._network_group_params = self._network_group.create_params()

        in_infos = self._hef.get_input_vstream_infos()
        out_infos = self._hef.get_output_vstream_infos()
        if len(in_infos) != 1 or len(out_infos) != 1:
            print(f"[_HailoYolo] ⚠ 입력 {len(in_infos)}개, 출력 {len(out_infos)}개 — "
                  f"각 1개를 가정한 아래 코드가 안 맞을 수 있다.")
        self._in_info = in_infos[0]
        self._out_name = out_infos[0].name
        self._in_h, self._in_w = self._in_info.shape[0], self._in_info.shape[1]

        # parse-hef 확인 결과: 입력 UINT8(0~255 그대로), 출력은 NMS후처리라 FLOAT32.
        self._input_params = InputVStreamParams.make_from_network_group(
            self._network_group, quantized=True, format_type=FormatType.UINT8)
        self._output_params = OutputVStreamParams.make_from_network_group(
            self._network_group, quantized=False, format_type=FormatType.FLOAT32)

        self._first_call = True
        print(f"[_HailoYolo] 입력 {self._in_info.name} {self._in_info.shape} UINT8  "
              f"출력 {self._out_name} (HAILO NMS BY CLASS)")

    def _letterbox(self, frame: np.ndarray) -> tuple[np.ndarray, float, int, int]:
        """YOLO 학습/추론과 같은 방식(비율 유지 축소 + 여백)으로 입력 크기에 맞춘다.
        train_yolo.py가 imgsz=640으로 학습했고 HEF 입력도 640x640으로 확인됨."""
        import cv2

        h0, w0 = frame.shape[:2]
        scale = min(self._in_w / w0, self._in_h / h0)
        new_w, new_h = int(round(w0 * scale)), int(round(h0 * scale))
        resized = cv2.resize(frame, (new_w, new_h))
        pad_x, pad_y = (self._in_w - new_w) // 2, (self._in_h - new_h) // 2
        padded = np.full((self._in_h, self._in_w, 3), 114, dtype=np.uint8)
        padded[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
        return padded, scale, pad_x, pad_y

    def infer(self, frame: np.ndarray) -> list[tuple[tuple[float, float, float, float], float]]:
        """[(bbox(cx,cy,w,h), confidence), ...]. 클래스가 하나라 클래스명은 안 돌려준다."""
        from hailo_platform import InferVStreams

        padded, scale, pad_x, pad_y = self._letterbox(frame)
        # UINT8 그대로 넣는다 — /255 정규화하면 안 된다(parse-hef가 UINT8 입력이라고
        # 확인해줬다. HEF 내부에서 양자화 스케일을 이미 알고 있다).
        input_data = {self._in_info.name: np.expand_dims(padded, axis=0)}

        with self._network_group.activate(self._network_group_params):
            with InferVStreams(self._network_group, self._input_params,
                                self._output_params) as pipeline:
                raw = pipeline.infer(input_data)

        return self._decode_output(raw, scale, pad_x, pad_y)

    def _decode_output(self, raw: dict, scale: float, pad_x: int, pad_y: int
                        ) -> list[tuple[tuple[float, float, float, float], float]]:
        """HAILO NMS BY CLASS 출력을 (bbox, conf) 목록으로 바꾼다.

        클래스가 1개라 보통 [클래스0의 박스들] 형태(ndarray, shape (N,5)) 하나만
        오거나, 길이 1짜리 리스트로 감싸서 온다 — 둘 다 처리한다. 박스 행 하나는
        [ymin,xmin,ymax,xmax,score]로 가정한다(Hailo NMS 관례), **640x640 입력
        기준 0~1 정규화 좌표**다.
        """
        out = raw[self._out_name]
        if isinstance(out, (list, tuple)):
            out = out[0] if out else np.zeros((0, 5), dtype=np.float32)
        out = np.asarray(out)

        if self._first_call:
            print(f"[_HailoYolo] 첫 추론 출력 shape={out.shape} dtype={out.dtype} — "
                  f"이상하면(예: 마지막 축이 5가 아니면) 좌표 파싱을 다시 확인할 것.")
            self._first_call = False

        results = []
        for row in out.reshape(-1, out.shape[-1]):
            if row.shape[0] < 5:
                continue
            ymin, xmin, ymax, xmax, score = row[:5]
            if score <= 0.0:
                continue
            # 640x640 입력 기준 정규화 좌표 -> 픽셀 -> letterbox 역변환 -> 원본 프레임 좌표
            x1 = (xmin * self._in_w - pad_x) / scale
            y1 = (ymin * self._in_h - pad_y) / scale
            x2 = (xmax * self._in_w - pad_x) / scale
            y2 = (ymax * self._in_h - pad_y) / scale
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            w, h = x2 - x1, y2 - y1
            results.append(((float(cx), float(cy), float(w), float(h)), float(score)))
        return results


_yolo: _HailoYolo | None = None


def _model_path() -> str:
    import os

    return os.environ.get("TRASH_HEF") or config.YOLO_MODEL_PATH


def _get_yolo() -> _HailoYolo:
    global _yolo
    if _yolo is None:
        import pathlib as _pl

        path = _pl.Path(_model_path()).expanduser()
        if not path.is_file():
            raise FileNotFoundError(
                f"Hailo 모델을 못 찾았다: {path}\n"
                f"  .hef 는 .gitignore 대상이라 git으로 안 따라온다. 직접 복사해야 한다:\n"
                f"    1) 파이에 이미 있는지: find ~ -name '*.hef'\n"
                f"    2) 팀원 PC에서:      scp best.hef pi@<파이IP>:{path.parent}/\n"
                f"  다른 파일을 쓰려면: TRASH_HEF=/경로/모델.hef python main.py")
        _yolo = _HailoYolo(str(path))
    return _yolo


def _undistort(u: float, v: float) -> tuple[float, float]:
    """렌즈 왜곡을 보정해 이상적인 핀홀 좌표로 옮긴다.

    **이미지 전체를 펴지 않고 점 하나만 보정한다.** 40fps로 프레임을 통째로
    undistort하는 건 파이 입장에서 낭비

    LS40136은 M12 광각 렌즈다. 화각이 넓으면 왜곡이 가장자리에서 커지고, 궤적
    피팅은 화면 전체를 가로지르는 궤적을 쓰므로 보정 없이는 잔차가 계통적으로
    커진다. 캘리브레이션이 끝나기 전까지는 보정 없이 돌아가되(원본 좌표 그대로),
    그 상태의 residual_px는 렌즈 왜곡을 포함한 값임을 기억할 것.
    """
    if config.CAMERA_DISTORTION is None:
        return u, v

    import cv2

    K = np.array([[config.CAMERA_FX, 0.0, config.CAMERA_CX],
                  [0.0, config.CAMERA_FY, config.CAMERA_CY],
                  [0.0, 0.0, 1.0]])
    d = np.asarray(config.CAMERA_DISTORTION, dtype=float)
    pts = np.array([[[float(u), float(v)]]], dtype=np.float64)

    if config.CAMERA_MODEL == "fisheye":
        out = cv2.fisheye.undistortPoints(pts, K, d.reshape(4, 1), P=K)
    else:
        out = cv2.undistortPoints(pts, K, d, P=K)
    return float(out[0, 0, 0]), float(out[0, 0, 1])


def detect_all(frame: object, t: float) -> list[Detection]:
    """한 프레임의 **모든** 검출 (신뢰도 임계값 통과분). 신뢰도 내림차순.

      YOLO가 에어컨·공유기·조명을 오탐할 때, 그게 진짜 쓰레기보다 **높은 신뢰도**를
      받는 경우가 있다. 최고점 하나만 쓰면 그 프레임에서 진짜 물체는 아예 안 보이고,
      오탐 좌표가 트랙에 섞인다. 합성 검증에서 25프레임 중 2프레임만 오염돼도
      재투영 잔차가 무한대로 튀어 예측이 통째로 죽었다.

      그래서 후보를 전부 넘긴다. **어느 것이 진짜인지 여기서도 tracker에서도
      미리 고르지 않는다** — 후보마다 가설을 돌리고 포물선 물리를 만족한 것만
      채택한다 (`tracker.TrackerPool`).
    """
    out = []
    for bbox, conf in _get_yolo().infer(frame):
        if conf < config.YOLO_CONF_THRESHOLD:
            continue
        cx_px, cy_px, _w, _h = bbox
        u, v = _undistort(cx_px, cy_px)
        out.append(Detection(u=u, v=v, t=t, confidence=conf, bbox=bbox))
    out.sort(key=lambda d: -d.confidence)
    if out:
        best = out[0]
        utils.log_detection(best.confidence, best.bbox, (best.u, best.v))
    return out


def detect(frame: object, t: float) -> Detection | None:
    """가장 신뢰도 높은 검출 하나. **게이팅을 안 쓸 때만** 의미가 있다.
    실제 구동 루프는 `detect_all` + `TrackerPool`을 쓴다. 이 함수는 단독 디버깅용.
    """
    hits = detect_all(frame, t)
    return hits[0] if hits else None


def observe(cam: Camera) -> tuple[list[Detection], float, "np.ndarray"]:
    """한 프레임 캡처해서 **(후보 전체, 캡처 시각, 원본 프레임)** 을 돌려준다.

    시각을 함께 주는 이유: 호출부가 `time.monotonic()`을 따로 부르면 캡처 시각과
    수십 ms 어긋난다. 그 값이 궤적 투영(`project`)의 t로 들어가면 예측 위치가
    밀려서 연관이 빗나간다. **피팅에 쓰는 시각과 루프가 쓰는 시각은 같아야 한다.**

    프레임도 돌려주는 이유: `test_accuracy.py --video`가 검출 박스를 그려 영상으로
    남긴다. 이것 때문에 캡처+검출을 따로 부르게 하면 **구동 루프와 테스트 루프가
    갈라진다** — 그러면 테스트가 실제와 같다는 보장이 깨진다. 구동 쪽은 프레임을
    그냥 무시하면 되므로 비용이 없다(복사 안 한다).
    """
    frame, t = cam.capture()
    return detect_all(frame, t), t, frame
