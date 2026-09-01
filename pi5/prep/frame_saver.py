"""JPEG 인코딩과 디스크 쓰기를 캡처 루프 밖으로 빼는 백그라운드 저장기.

capture_dataset.py(검출된 것만 저장)와 record_raw.py(전부 저장)가 함께 쓴다.

**왜 스레드로 빼는가:** 1456x1088 을 JPEG 로 인코딩해 SD카드에 쓰는 건 60fps 프레임
예산(16.7ms)을 넘기기 쉽다. 캡처 루프 안에서 하면 하필 **물체를 찍고 있는 동안** 매
프레임 그 비용이 발생해서, 프레임이 제일 필요한 순간에 프레임을 놓친다.
"""

from __future__ import annotations

import pathlib
import queue
import threading


class FrameSaver:
    """큐에 넣으면 백그라운드 스레드가 쓴다.

    파일 이름은 **제출 시점(메인 스레드)에 확정**한다 — 그래야 저장 순서가 스레드
    스케줄링에 따라 뒤바뀌지 않는다.
    """

    def __init__(self, cv2, out_img: pathlib.Path, out_lbl: pathlib.Path | None = None,
                 quality: int = 85, maxsize: int = 256) -> None:
        self.cv2 = cv2
        self.out_img = out_img
        self.out_lbl = out_lbl          # None 이면 라벨을 쓰지 않는다 (원본 녹화용)
        self.params = [cv2.IMWRITE_JPEG_QUALITY, int(quality)]
        self.q: queue.Queue = queue.Queue(maxsize=maxsize)
        self.dropped = 0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, name: str, frame, label_line: str | None = None) -> bool:
        """큐에 넣기만 하고 즉시 돌아온다. 큐가 차면 버리고 False."""
        try:
            self.q.put_nowait((name, frame, label_line))
            return True
        except queue.Full:
            # 여기서 기다리면 분리한 의미가 없다. 버리되 반드시 세어서 알린다 —
            # 큐가 찬다는 건 디스크가 근본적으로 못 따라온다는 뜻이다.
            self.dropped += 1
            return False

    def _run(self) -> None:
        while True:
            item = self.q.get()
            if item is None:
                self.q.task_done()
                return
            name, frame, label_line = item
            try:
                self.cv2.imwrite(str(self.out_img / f"{name}.jpg"), frame, self.params)
                if label_line is not None and self.out_lbl is not None:
                    (self.out_lbl / f"{name}.txt").write_text(label_line, encoding="utf-8")
            except Exception as exc:  # noqa: BLE001 - 한 장 실패로 수집을 멈추지 않는다
                print(f"[saver] {name} 저장 실패: {exc}")
            finally:
                self.q.task_done()

    def close(self) -> None:
        """남은 것을 전부 쓰고 스레드를 정리한다."""
        pending = self.q.qsize()
        if pending:
            print(f"  저장 대기 {pending}장 처리 중...")
        self.q.put(None)
        self._thread.join(timeout=60.0)
