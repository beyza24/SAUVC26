#!/usr/bin/env python3
"""Turuncu fişek için HS histogram modeli.

Sentetik görüntülerden eğitir; --camera ile Gazebo ön kamerasından
gerçek kareleri alıp modeli rafine eder.

  python3 train_orange_flare.py
  ros2 run abdu_control train_orange_flare --ros-args -p camera:=true
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError as exc:  # pragma: no cover
    raise SystemExit("opencv-python gerekli") from exc

OUT = Path(__file__).resolve().parent / "data" / "orange_flare_model.npz"

# Abdu/OrangeFlare diffuse 1.00 0.22 0.00 → BGR
FLARE_BGR = np.array([20.0, 90.0, 255.0], dtype=np.float32)


def _pool_bg(h: int, w: int, rng: np.random.Generator) -> np.ndarray:
    img = np.zeros((h, w, 3), np.uint8)
    water = np.array(
        [
            int(rng.integers(90, 150)),
            int(rng.integers(70, 130)),
            int(rng.integers(10, 50)),
        ],
        np.int32,
    )
    floor = np.array(
        [
            int(rng.integers(180, 230)),
            int(rng.integers(190, 240)),
            int(rng.integers(140, 200)),
        ],
        np.int32,
    )
    horizon = int(h * rng.uniform(0.38, 0.55))
    img[:horizon] = water
    img[horizon:] = floor
    for y in range(horizon, h):
        t = (y - horizon) / max(1, h - horizon)
        img[y] = np.clip((1.0 - 0.25 * t) * floor + 0.15 * t * water, 0, 255)
    return img


def _flare_bgr(rng: np.random.Generator) -> tuple[int, int, int]:
    light = float(rng.uniform(0.55, 1.15))
    water_wash = float(rng.uniform(0.0, 0.22))
    base = FLARE_BGR * light
    wash = np.array([150.0, 170.0, 80.0])
    col = (1.0 - water_wash) * base + water_wash * wash
    col[1] = np.clip(col[1] + rng.uniform(0, 55), 0, 255)
    col = np.clip(col + rng.uniform(-18, 18, size=3), 0, 255)
    return int(col[0]), int(col[1]), int(col[2])


def _draw_flare(img: np.ndarray, mask: np.ndarray, rng: np.random.Generator) -> None:
    h, w = img.shape[:2]
    bw = max(3, int(rng.uniform(4, 18)))
    bh = max(int(h * 0.25), int(rng.uniform(h * 0.35, h * 0.85)))
    cx = int(rng.integers(bw + 4, w - bw - 4))
    y1 = int(rng.integers(8, max(9, h - bh - 4)))
    x0, x1 = cx - bw // 2, cx + (bw + 1) // 2
    y0, y2 = y1, y1 + bh
    color = _flare_bgr(rng)
    cv2.rectangle(img, (x0, y0), (x1, y2), color, -1)
    hi = tuple(min(255, c + 35) for c in color)
    cv2.rectangle(img, (x0, y0), (x0 + max(1, bw // 3), y2), hi, -1)
    cv2.rectangle(mask, (x0, y0), (x1, y2), 255, -1)


def _draw_distractors(img: np.ndarray, rng: np.random.Generator) -> None:
    h, w = img.shape[:2]
    if rng.random() < 0.55:
        x = int(rng.integers(8, w - 8))
        cv2.rectangle(img, (x, int(h * 0.35)), (x + 3, int(h * 0.85)), (20, 20, 210), -1)
        cv2.rectangle(img, (x, int(h * 0.35)), (x + 3, int(h * 0.50)), (15, 15, 15), -1)
    if rng.random() < 0.55:
        x = int(rng.integers(8, w - 8))
        cv2.rectangle(img, (x, int(h * 0.35)), (x + 3, int(h * 0.85)), (40, 200, 40), -1)
        cv2.rectangle(img, (x, int(h * 0.35)), (x + 3, int(h * 0.50)), (15, 15, 15), -1)


def synthesize(n: int = 520) -> tuple[np.ndarray, np.ndarray]:
    rng = np.random.default_rng(7)
    h, w = 180, 320
    fg, bg = [], []
    for _ in range(n):
        img = _pool_bg(h, w, rng)
        mask = np.zeros((h, w), np.uint8)
        _draw_distractors(img, rng)
        _draw_flare(img, mask, rng)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        pix = hsv[mask > 0]
        if len(pix):
            fg.append(pix)
        other = hsv[mask == 0]
        if len(other) > 500:
            other = other[rng.choice(len(other), 500, replace=False)]
        bg.append(other)
    return np.concatenate(fg, axis=0), np.concatenate(bg, axis=0)


def fit(pixels: np.ndarray, background: np.ndarray) -> dict:
    yimg = pixels.reshape(1, -1, 3).astype(np.uint8)
    hist = cv2.calcHist([yimg], [0, 1], None, [36, 32], [0.0, 180.0, 0.0, 256.0])
    cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
    yp = pixels.reshape(-1, 1, 3).astype(np.uint8)
    bp = cv2.calcBackProject([yp], [0, 1], hist, [0.0, 180.0, 0.0, 256.0], 1)
    h0, h1 = np.percentile(pixels[:, 0], [8, 92])
    s0 = np.percentile(pixels[:, 1], 10)
    v0 = np.percentile(pixels[:, 2], 12)
    return {
        "hist": hist.astype(np.float32),
        "thresh": np.float32(max(12.0, float(np.percentile(bp, 20)))),
        "h_lo": np.float32(max(4.0, h0 - 3)),
        "h_hi": np.float32(max(18.0, min(28.0, h1 + 4))),
        "s_lo": np.float32(min(90.0, max(55.0, s0 - 40))),
        "v_lo": np.float32(min(70.0, max(35.0, v0 - 40))),
        "v_hi": np.float32(255.0),
        "n_orange": np.int32(len(pixels)),
        "n_bg": np.int32(len(background)),
    }


def save_model(model: dict, path: Path = OUT) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **model)
    print(
        "Yazildi %s  flema_px=%d  H=[%.0f,%.0f] S>=%.0f  thr=%.1f"
        % (
            path,
            int(model["n_orange"]),
            float(model["h_lo"]),
            float(model["h_hi"]),
            float(model["s_lo"]),
            float(model["thresh"]),
        )
    )


def collect_camera_pixels(seconds: float = 8.0) -> np.ndarray:
    import time

    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    pixels = []

    class Collector(Node):
        def __init__(self) -> None:
            super().__init__("abdu_flare_collect")
            self.bridge = CvBridge()
            self.create_subscription(Image, "/abdu/front_camera/image_raw", self._cb, 10)

        def _cb(self, msg: Image) -> None:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            img = cv2.resize(img, (320, 180))
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            gate = cv2.inRange(hsv, (5, 80, 40), (28, 255, 255))
            pix = hsv[gate > 0]
            if len(pix) > 30:
                pixels.append(pix)

    rclpy.init()
    node = Collector()
    end = time.time() + seconds
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if not pixels:
        print("Kameradan turuncu piksel gelmedi, sentetik model kullanilacak.")
        return np.zeros((0, 3), np.uint8)
    stacked = np.concatenate(pixels, axis=0)
    print(f"Kameradan {len(stacked)} turuncu piksel alindi.")
    return stacked


def main() -> None:
    import argparse
    import time

    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", action="store_true", help="Gazebo on kamerasindan rafine et")
    parser.add_argument("--seconds", type=float, default=8.0)
    args, _unknown = parser.parse_known_args()

    fg, bg = synthesize()
    if args.camera:
        time.sleep(0.5)
        cam = collect_camera_pixels(args.seconds)
        if len(cam):
            fg = np.concatenate([fg, cam], axis=0)
    model = fit(fg, bg)
    save_model(model)


if __name__ == "__main__":
    main()
