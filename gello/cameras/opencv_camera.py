import os
from typing import Optional, Tuple

import numpy as np


def _resolve_backend(backend) -> int:
    """把 None / 字符串 / 整数统一成 cv2 的后端常量。"""
    import cv2

    if backend is None:
        return cv2.CAP_V4L2
    if isinstance(backend, str):
        return getattr(cv2, f"CAP_{backend.upper()}")
    return int(backend)


class OpenCVCamera:
    """USB 单目相机(V4L2 + MJPG),遵守 gello 的 CameraDriver 协议。

    read() 返回 (rgb, depth):depth 恒为同尺寸全 0 的 uint16 占位，
    因为 USB 相机没有深度流，但协议要求返回两个数组。
    """

    def __repr__(self) -> str:
        return f"OpenCVCamera(index_or_path={self._index_or_path!r})"

    def __init__(
        self,
        index_or_path="/dev/video2",
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        fourcc: Optional[str] = "MJPG",
        backend=None,
        flip: bool = False,
    ):
        import cv2

        self._index_or_path = index_or_path
        self._width = width
        self._height = height
        self._flip = flip

        self._cap = cv2.VideoCapture(index_or_path, _resolve_backend(backend))
        if not self._cap.isOpened():
            raise RuntimeError(f"无法打开相机: {index_or_path!r}")

        # 顺序很重要：先定编码格式，再定分辨率/帧率（与 LeRobot 的写法一致）
        if fourcc is not None:
            self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
        self._cap.set(cv2.CAP_PROP_FPS, float(fps))

        # 丢几帧，等自动曝光/白平衡稳定
        for _ in range(5):
            self._cap.read()

        print(f"[OpenCVCamera] {self} 实际协商: "
              f"{self._cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x"
              f"{self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f} @ "
              f"{self._cap.get(cv2.CAP_PROP_FPS):.0f}fps")

    def read(
        self,
        img_size: Optional[Tuple[int, int]] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """读一帧。

        Args:
            img_size: 期望的 (宽, 高)。None 表示用相机当前分辨率。

        Returns:
            np.ndarray: 彩色图,shape=(H, W, 3),RGB 通道顺序
            np.ndarray: 深度图占位,shape=(H, W, 1)，全 0
        """
        import cv2

        ok, frame = self._cap.read()
        if not ok or frame is None:
            raise RuntimeError(f"读帧失败: {self._index_or_path!r}")

        if self._flip:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        if img_size is not None:
            frame = cv2.resize(frame, img_size)

        image = frame[:, :, ::-1]  # BGR -> RGB
        depth = np.zeros((image.shape[0], image.shape[1], 1), dtype=np.uint16)
        return image, depth

    def close(self) -> None:
        if getattr(self, "_cap", None) is not None:
            self._cap.release()
            self._cap = None


def _debug_read(camera, save_datastream: bool = False):
    import cv2

    cv2.namedWindow("image")
    counter = 0
    if not os.path.exists("images"):
        os.makedirs("images")
    if save_datastream and not os.path.exists("stream"):
        os.makedirs("stream")
    try:
        while True:
            image, _ = camera.read()
            cv2.imshow("image", image[:, :, ::-1])  # RGB -> BGR 供 imshow
            key = cv2.waitKey(1)
            if key == ord("s"):
                cv2.imwrite(f"images/image_{counter}.png", image[:, :, ::-1])
                print(f"已保存 images/image_{counter}.png")
            if save_datastream:
                cv2.imwrite(f"stream/image_{counter}.png", image[:, :, ::-1])
            counter += 1
            if key == 27:  # Esc
                break
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="OpenCV 相机调试预览")
    parser.add_argument("index_or_path", nargs="?", default="/dev/video2",
                        help="设备路径或索引，默认 /dev/video2")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--fourcc", type=str, default="MJPG")
    parser.add_argument("--backend", type=str, default=None,
                        help="V4L2 / ANY / DSHOW ...（默认 V4L2）")
    parser.add_argument("--flip", action="store_true", help="旋转 180°")
    parser.add_argument("--save-stream", action="store_true", help="持续把帧写到 stream/")
    args = parser.parse_args()

    path = args.index_or_path
    if path.isdigit():
        path = int(path)

    cam = OpenCVCamera(
        index_or_path=path,
        width=args.width,
        height=args.height,
        fps=args.fps,
        fourcc=args.fourcc,
        backend=args.backend,
        flip=args.flip,
    )
    print("按 s 存图，Esc 退出")
    _debug_read(cam, save_datastream=args.save_stream)
    cam.close()
