"""config.CAMERA_YAW_RAD 실측 도구 — **파이 + 카메라 모듈만 있으면 된다.**

피코도, 모터도, YOLO/Hailo 도, 던지기도 필요 없다. 카메라만 붙이고 돌리면 된다.

무엇을 재는가
-------------
`trajectory.py` 가 푸는 X/Y 는 **화면에서 u/v 가 커지는 방향**이고, 오도메트리·
역기구학이 쓰는 건 **로봇 body 축**(+X=우측, +Y=전방=M1)이다. 카메라를 로봇에 몇 도
돌려 붙였느냐만큼 이 둘이 어긋나 있고, 그 각도가 `config.CAMERA_YAW_RAD` 다.
이 값이 틀리면 **거리는 맞는데 방향이 틀린 곳**으로 간다.

원리 (왜 던지지 않아도 되나)
----------------------------
카메라가 하늘(위)을 보므로, **카메라보다 높이 있는 물체는 그 물체가 있는 방위각
그대로** 화면에 찍힌다. 그러니 "로봇 기준 어느 쪽인지 아는 물체"를 하나 카메라 위에
두고 그게 화면 어디에 찍히는지만 보면 된다.

    robot_vec = R(yaw) · cam_vec        (tracker.cam_to_robot 의 정의)
    => yaw = (로봇축 방위각) − (카메라축 방위각)

카메라축 방위각은 관측 픽셀에서 바로 나온다:

    X/Z = (u − cx)/fx ,  Y/Z = (v − cy)/fy   =>  방위각 = atan2(Y/Z, X/Z)

★ 이 측정은 **렌즈 왜곡 모델에 의존하지 않는다.** 왜곡이 주점을 중심으로 방사형
  이라 반경만 바꾸고 방위각은 보존하기 때문이다. 그래도 fx≠fy 비대칭은 있으므로
  아래 코드는 `fisheye.to_pinhole_px()` 로 한 번 펴고 fx/fy 로 각각 나눈다.
  즉 **아직 안 맞을 수도 있는 왜곡계수 때문에 이 값이 틀어질 일은 없다.**

쓰는 법 — 실시간 모드 (권장)
----------------------------
    python measure_camera_yaw.py live

창이 뜨고 주점 십자선과 방위각 눈금이 겹쳐 보인다. 그 상태로:

  ① 로봇을 바닥에 세우고 **M1(전방)이 어느 쪽인지** 확인한다.
  ② 카메라보다 확실히 높은 곳에서, 로봇 기준 **전방(M1)** 자리에 물체를 둔다.
     제일 쉬운 방법: 사람이 로봇 앞쪽 1 m 쯤에 서서 카메라를 내려다본다.
     ⚠ 방향은 **로봇 중심이 아니라 카메라 광학중심 기준**이다. 카메라가 통 림에
       달려 중심에서 비켜 있으면, 카메라를 지나는 M1 방향 선 위에 서야 한다.
  ③ 화면에서 그 물체를 **마우스로 클릭**한다. 방위각이 바로 표시된다.
  ④ `f` 를 눌러 "이건 전방" 이라고 기록한다.
  ⑤ 물체를 로봇 **우측**으로 옮기고 다시 클릭 → `r`.  (최소 두 방향은 재야 한다)
  ⑥ `s` 를 누르면 CAMERA_YAW_RAD 를 풀어서 터미널에 출력한다.
  ⑦ 출력된 줄을 `config.py` 에 붙여넣는다.

  키: f=전방  r=우측  b=후방  l=좌측  s=풀기  c=지우기  q/ESC=종료

디스플레이가 없을 때 (SSH 등)
-----------------------------
    python measure_camera_yaw.py capture          # 눈금 그린 사진 저장
    (사진을 열어 물체의 픽셀 좌표를 읽고)
    python measure_camera_yaw.py solve --obs front 1180 620 --obs right 900 980

이미 찍어둔 사진이 있으면:  python measure_camera_yaw.py capture --image 찍은것.jpg
"""

from __future__ import annotations

import math

import numpy as np

import config

# 로봇 body 축에서의 방위각. +X(우측)=0°, 반시계가 +. (config.py 의 배치도와 같다)
DIRECTIONS = {
    "right": 0.0,      # +X
    "front": 90.0,     # +Y — M1 방향
    "left": 180.0,     # -X
    "back": 270.0,     # -Y
}

_KEY_TO_DIR = {"f": "front", "r": "right", "b": "back", "l": "left"}


# ── 각도 계산 ─────────────────────────────────────────────────────────────

def cam_azimuth_deg(u: float, v: float) -> float:
    """관측 픽셀 -> **카메라 축** 방위각(도).

    왜곡을 편 뒤 fx/fy 로 나눠서 X/Z, Y/Z 를 만든다. 방위각은 왜곡이 보존하므로
    이 단계에서 실제로 하는 일은 fx != fy 비대칭 보정이 전부다 (0.05° 수준).
    """
    import fisheye

    uv = fisheye.to_pinhole_px([(float(u), float(v))])
    x = (uv[0, 0] - config.CAMERA_CX) / config.CAMERA_FX
    y = (uv[0, 1] - config.CAMERA_CY) / config.CAMERA_FY
    return math.degrees(math.atan2(y, x))


def yaw_from_observation(u: float, v: float, robot_deg: float) -> float:
    """관측 하나 -> CAMERA_YAW_RAD 후보(도). yaw = 로봇축 − 카메라축."""
    return _wrap180(robot_deg - cam_azimuth_deg(u, v))


def _wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def _circular_mean_deg(vals: list[float]) -> float:
    s = sum(math.sin(math.radians(a)) for a in vals)
    c = sum(math.cos(math.radians(a)) for a in vals)
    return math.degrees(math.atan2(s, c))


# ── 풀이 · 출력 ───────────────────────────────────────────────────────────

def solve(observations: list[tuple[str, float, float]]) -> int:
    """observations: [(방향이름 또는 각도, u, v), ...]"""
    print()
    print(f"주점 (cx, cy) = ({config.CAMERA_CX}, {config.CAMERA_CY})   "
          f"해상도 {config.CAMERA_RESOLUTION[0]}x{config.CAMERA_RESOLUTION[1]}")
    print()
    print(f"{'방향':>8}{'로봇축':>10}{'픽셀':>18}{'카메라축':>11}{'yaw 후보':>12}")
    print("  " + "-" * 56)

    cands: list[float] = []
    w, h = config.CAMERA_RESOLUTION
    for name, u, v in observations:
        if name in DIRECTIONS:
            robot_deg = DIRECTIONS[name]
        else:
            try:
                robot_deg = float(name)
            except ValueError:
                raise SystemExit(
                    f"방향은 {sorted(DIRECTIONS)} 중 하나거나 각도(도)여야 한다: {name!r}")
        if not (0 <= u <= w and 0 <= v <= h):
            print(f"  ⚠ ({u:.0f},{v:.0f}) 가 프레임({w}x{h}) 밖이다 — 좌표를 다시 볼 것")
        r = math.hypot(u - config.CAMERA_CX, v - config.CAMERA_CY)
        if r < 40.0:
            print(f"  ⚠ ({u:.0f},{v:.0f}) 는 주점에서 {r:.0f}px 밖에 안 떨어졌다. "
                  f"방위각이 노이즈에 크게 흔들린다 — 물체를 더 옆으로 옮겨 다시 잴 것")
        cam_deg = cam_azimuth_deg(u, v)
        yaw = yaw_from_observation(u, v, robot_deg)
        cands.append(yaw)
        print(f"{name:>8}{robot_deg:>9.1f}°  ({u:>7.1f},{v:>7.1f}){cam_deg:>10.1f}°{yaw:>11.1f}°")

    mean = _circular_mean_deg(cands)
    print()
    if len(cands) >= 2:
        spread = max(abs(_wrap180(a - mean)) for a in cands)
        print(f"관측 {len(cands)}개 평균 = {mean:+.1f}°,  최대 편차 = {spread:.1f}°")
        if spread > 10.0:
            print("  ⚠ 편차가 10°를 넘는다. 둘 중 하나다:")
            print("     - 물체를 둔 방향이 실제 로봇 축과 달랐다 (다시 재는 게 빠르다)")
            print("     - 카메라가 정확히 위를 안 보고 기울어져 있다. 이 경우 회전각")
            print("       하나로는 못 맞춘다 — config.GRAVITY_CAM 가정도 같이 깨지므로")
            print("       카메라를 수평으로 다시 다는 게 맞다")
        else:
            print("  편차가 작다 — 카메라가 하늘을 잘 보고 있다는 뜻이기도 하다.")
    else:
        print("⚠ 관측이 하나뿐이다. **최소 두 방향으로 재서 서로 맞는지 확인할 것** —")
        print("  하나만으로는 카메라가 기울어진 건지 돌아간 건지 구분이 안 된다.")

    print()
    print("config.py 에 붙여넣을 줄:")
    print()
    nice = {0.0: "0.0", 90.0: "math.pi / 2", 180.0: "math.pi", -90.0: "-math.pi / 2"}
    snapped = min(nice, key=lambda a: abs(_wrap180(mean - a)))
    off = abs(_wrap180(mean - snapped))
    if off <= 5.0:
        print(f"    CAMERA_YAW_RAD = {nice[snapped]}   # 실측 {mean:+.1f}° "
              f"(-> {snapped:+.0f}° 로 반올림)")
        print()
        print(f"  실측값이 {snapped:+.0f}° 에서 {off:.1f}° 안에 들어온다. 카메라를 직각으로")
        print("  달았다는 뜻이므로 딱 떨어지는 값을 쓰는 게 낫다 — 조립 오차와 클릭 오차까지")
        print("  각도에 굳혀 넣을 이유가 없다.")
    else:
        print(f"    CAMERA_YAW_RAD = math.radians({mean:.1f})   # 실측")
        print()
        print(f"  {mean:+.1f}° 는 직각에서 {off:.1f}° 벗어나 있다. 의도한 장착이면 그대로 쓰고,")
        print("  아니면 카메라를 다시 맞춰 다는 게 낫다.")
    return 0


# ── 프레임 오버레이 ───────────────────────────────────────────────────────

def _annotate(img, marked=None, recorded=None):
    """주점 십자선 + 방위각 눈금(30도마다) + 기록된 관측을 그린다."""
    import cv2

    cx, cy = int(round(config.CAMERA_CX)), int(round(config.CAMERA_CY))
    h, w = img.shape[:2]

    cv2.line(img, (cx, 0), (cx, h), (0, 0, 255), 1)
    cv2.line(img, (0, cy), (w, cy), (0, 0, 255), 1)

    radius = int(min(w, h) * 0.45)
    for deg in range(0, 360, 30):
        th = math.radians(deg)
        x = int(cx + radius * math.cos(th))
        y = int(cy + radius * math.sin(th))
        cv2.line(img, (cx, cy), (x, y), (0, 200, 200), 1)
        cv2.putText(img, str(deg), (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 200, 200), 1)
    cv2.circle(img, (cx, cy), 6, (0, 0, 255), -1)

    for name, u, v in (recorded or []):
        p = (int(u), int(v))
        cv2.circle(img, p, 9, (0, 255, 0), 2)
        cv2.line(img, (cx, cy), p, (0, 255, 0), 2)
        cv2.putText(img, name, (p[0] + 12, p[1]), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 0), 2)

    if marked is not None:
        u, v = marked
        p = (int(u), int(v))
        cv2.drawMarker(img, p, (255, 255, 255), cv2.MARKER_CROSS, 22, 2)
        cv2.putText(img, f"{cam_azimuth_deg(u, v):+.1f} deg", (p[0] + 12, p[1] + 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return img


def _open_camera():
    """prep/camera.py 로 카메라만 연다.

    `vision.py` 를 거치지 않는 이유: 이 도구는 **카메라 모듈만으로** 돌아야 한다.
    Hailo 도, .hef 도, 피코도 없이 파이에 카메라만 붙은 상태에서 쓰는 물건이다.
    """
    import pathlib
    import sys

    prep_dir = str(pathlib.Path(__file__).resolve().parent / "prep")
    if prep_dir not in sys.path:
        sys.path.insert(0, prep_dir)
    import camera as camlib

    return camlib.open_camera(profile=camlib.DEFAULT, backend="picamera2")


# ── 모드 ──────────────────────────────────────────────────────────────────

def live() -> int:
    import cv2

    cam = _open_camera()
    state = {"marked": None}
    recorded: list[tuple[str, float, float]] = []
    win = "camera yaw"

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            state["marked"] = (float(x), float(y))
            print(f"  클릭 ({x}, {y}) -> 카메라축 {cam_azimuth_deg(x, y):+.1f}°   "
                  f"[f=전방 r=우측 b=후방 l=좌측 으로 기록]")

    print(__doc__.split("쓰는 법 — 실시간 모드 (권장)")[1].split("디스플레이가 없을 때")[0])
    try:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 1000, 750)
        cv2.setMouseCallback(win, on_mouse)
    except cv2.error as exc:
        cam.close()
        print(f"창을 못 열었다 ({exc}).")
        print("디스플레이가 없는 환경이면 `capture` + `solve` 를 쓸 것 — 파일 맨 위 설명 참고.")
        return 2

    try:
        while True:
            frame, _t = cam.read()
            img = _annotate(np.ascontiguousarray(frame.copy()),
                            state["marked"], recorded)
            cv2.putText(img, "click object | f/r/b/l record | s solve | c clear | q quit",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(img, f"recorded: {len(recorded)}", (10, 56),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow(win, img)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("c"):
                recorded.clear()
                state["marked"] = None
                print("  기록을 지웠다.")
            elif 0 < key < 128 and chr(key) in _KEY_TO_DIR:
                if state["marked"] is None:
                    print("  ⚠ 먼저 화면에서 물체를 클릭할 것.")
                else:
                    name = _KEY_TO_DIR[chr(key)]
                    recorded[:] = [r for r in recorded if r[0] != name]
                    recorded.append((name, *state["marked"]))
                    print(f"  기록: {name} <- ({state['marked'][0]:.0f}, "
                          f"{state['marked'][1]:.0f})   (총 {len(recorded)}개)")
            elif key == ord("s"):
                if not recorded:
                    print("  ⚠ 기록된 관측이 없다.")
                else:
                    solve(list(recorded))
    finally:
        cam.close()
        cv2.destroyAllWindows()
    return 0


def capture(out_path: str, image: str | None) -> int:
    import cv2

    if image:
        frame = cv2.imread(image)
        if frame is None:
            raise SystemExit(f"이미지를 못 읽었다: {image}")
    else:
        cam = _open_camera()
        try:
            for _ in range(5):        # 노출·화이트밸런스가 안정될 시간을 준다
                frame, _t = cam.read()
        finally:
            cam.close()

    img = _annotate(np.ascontiguousarray(frame.copy()))
    h, w = img.shape[:2]
    cv2.putText(img, "read the object pixel (u,v), then:", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(img, "measure_camera_yaw.py solve --obs front <u> <v>", (10, 52),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.imwrite(out_path, img)
    print(f"저장했다: {out_path}  ({w}x{h})")
    print("이 그림에서 물체의 픽셀 좌표를 읽어 solve 로 넘길 것.")
    return 0


def main() -> int:
    import argparse

    ap = argparse.ArgumentParser(
        description="config.CAMERA_YAW_RAD 실측 — 파이 + 카메라 모듈만 있으면 된다")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("live", help="실시간 미리보기에서 클릭으로 잰다 (권장)")

    c = sub.add_parser("capture", help="눈금 그린 프레임을 파일로 저장 (디스플레이 없을 때)")
    c.add_argument("-o", "--out", default="camera_yaw_ref.jpg")
    c.add_argument("--image", default=None, help="캡처 대신 이미 찍어둔 사진을 쓴다")

    s = sub.add_parser("solve", help="관측 픽셀에서 각도를 푼다")
    s.add_argument("--obs", nargs=3, action="append", required=True,
                   metavar=("방향", "U", "V"),
                   help="방향은 front/right/back/left 또는 각도(도). 여러 번 쓸 것")

    args = ap.parse_args()
    if args.cmd == "live":
        return live()
    if args.cmd == "capture":
        return capture(args.out, args.image)
    return solve([(name, float(u), float(v)) for name, u, v in args.obs])


if __name__ == "__main__":
    raise SystemExit(main())
