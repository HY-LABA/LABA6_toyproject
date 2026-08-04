"""실행 전 환경 점검 — 다른 스크립트를 돌리기 전에 먼저 이걸 실행한다.

    python check_setup.py              # 전체 점검
    python check_setup.py --camera gs  # 특정 카메라 프로파일로

무엇이 없는지, 그게 어느 단계에서 필요한지까지 알려준다.
"""

from __future__ import annotations

import argparse
import importlib
import platform
import shutil
import sys

import camera as camlib

# (모듈명, 설치명, 필요한 단계, 필수 여부)
PACKAGES = [
    ("cv2", "opencv-python", "캘리브레이션 · 데이터 수집 · 검수", True),
    ("numpy", "numpy", "전 단계", True),
    ("yaml", "pyyaml", "데이터셋 준비", True),
    ("picamera2", "python3-picamera2 (apt)", "라파이 카메라 캡처", False),
    ("ultralytics", "ultralytics", "YOLO 학습 (PC)", False),
    ("torch", "torch", "YOLO 학습 (PC)", False),
]


def _check(label: str, ok: bool, detail: str = "") -> bool:
    print(f"  [{'OK ' if ok else 'X  '}] {label}" + (f"  — {detail}" if detail else ""))
    return ok


def check_python() -> bool:
    v = sys.version_info
    ok = v >= (3, 9)
    return _check(f"Python {v.major}.{v.minor}.{v.micro}", ok,
                  "" if ok else "3.9 이상 필요")


def check_packages() -> list[str]:
    missing = []
    for mod, pip_name, stage, required in PACKAGES:
        try:
            m = importlib.import_module(mod)
            ver = getattr(m, "__version__", "")
            _check(f"{mod} {ver}".strip(), True, stage)
        except ImportError:
            mark = "필수" if required else "선택"
            _check(f"{mod} 없음 ({mark})", not required, f"{stage} / pip install {pip_name}")
            if required:
                missing.append(pip_name)
    return missing


def check_camera(profile: str) -> bool:
    spec = camlib.SPECS[profile]
    print(f"  프로파일: {spec.name} — {spec.width}x{spec.height} @{spec.fps}fps, "
          f"캘리브레이션 모델 = {spec.calib_model}")
    try:
        cam = camlib.open_camera(profile)
    except Exception as exc:  # noqa: BLE001
        return _check("카메라 열기", False, str(exc))
    try:
        frame, _ = cam.read()
        h, w = frame.shape[:2]
        ok = True
        _check(f"프레임 캡처 {w}x{h}", True, f"backend={cam.backend}")
        if (w, h) != (spec.width, spec.height):
            print(f"       ⚠ 요청 해상도({spec.width}x{spec.height})와 다르다. "
                  f"드라이버가 가장 가까운 모드를 골랐을 수 있다.")
        mean = float(frame.mean())
        if mean < 10:
            print("       ⚠ 화면이 거의 검다. 렌즈 캡을 벗겼는지, 노출이 너무 짧지 않은지 확인.")
        elif mean > 245:
            print("       ⚠ 화면이 포화됐다. 노출을 줄여라.")
        return ok
    except Exception as exc:  # noqa: BLE001
        return _check("프레임 캡처", False, str(exc))
    finally:
        cam.close()


def check_disk() -> bool:
    free_gb = shutil.disk_usage(".").free / 1e9
    # 1500장 x 약 1MB + 여유
    ok = free_gb > 5
    return _check(f"여유 디스크 {free_gb:.1f} GB", ok, "" if ok else "5 GB 이상 권장")


def main() -> int:
    ap = argparse.ArgumentParser(description="준비 단계 환경 점검")
    camlib.add_profile_arg(ap)
    ap.add_argument("--skip-camera", action="store_true", help="카메라 점검 건너뛰기")
    args = ap.parse_args()

    print(f"\n=== 환경 ===  {platform.platform()}")
    py_ok = check_python()

    print("\n=== 패키지 ===")
    missing = check_packages()

    print("\n=== 디스크 ===")
    disk_ok = check_disk()

    cam_ok = True
    if not args.skip_camera:
        print("\n=== 카메라 ===")
        cam_ok = check_camera(args.camera)

    print("\n=== 요약 ===")
    if missing:
        print(f"  필수 패키지 누락: pip install {' '.join(missing)}")
    if not cam_ok:
        print("  카메라를 못 열었다. 라파이라면:")
        print("    - /boot/firmware/config.txt 의 dtoverlay 확인")
        print("    - `rpicam-hello --list-cameras` 로 인식 여부 확인")
        print("    - 리본 케이블 방향/체결 확인")
    ready = py_ok and not missing and disk_ok and cam_ok
    print(f"\n  {'준비 완료 — calibrate.py 부터 시작하면 된다.' if ready else '위 항목을 먼저 해결할 것.'}")
    return 0 if ready else 1


if __name__ == "__main__":
    raise SystemExit(main())
