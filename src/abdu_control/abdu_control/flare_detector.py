"""Ön kamera: turuncu flema + kırmızı/sarı/mavi direk tespiti ve çizim."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

try:
    import cv2
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover
    cv2 = None
    CvBridge = None

from sensor_msgs.msg import Image


def default_model_path() -> Path:
    here = Path(__file__).resolve().parent / "data" / "orange_flare_model.npz"
    if here.is_file():
        return here
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("abdu_control")) / "data" / "orange_flare_model.npz"
        if share.is_file():
            return share
    except Exception:
        pass
    return here


@dataclass
class FlareObs:
    seen: bool = False
    close: bool = False
    pixels: int = 0
    cx: float = 0.0
    cy: float = 0.0
    width: float = 0.0
    height: float = 0.0
    side: str = "yok"


@dataclass
class ColorHit:
    name: str
    pixels: int
    x: int
    y: int
    w: int
    h: int
    nx: float
    ny: float
    side: str


@dataclass
class SceneObs:
    flare: FlareObs = field(default_factory=FlareObs)
    poles: dict = field(default_factory=dict)


class OrangeFlareModel:
    def __init__(self, path: Path | None = None) -> None:
        self.hist = None
        self.thresh = 18.0
        self.h_lo, self.h_hi = 8.0, 25.0
        self.s_lo, self.v_lo, self.v_hi = 90.0, 50.0, 255.0
        path = path or default_model_path()
        if path.is_file() and cv2 is not None:
            z = np.load(path)
            self.hist = z["hist"]
            self.thresh = float(z["thresh"])
            self.h_lo = float(z["h_lo"])
            self.h_hi = float(z["h_hi"])
            self.s_lo = float(z["s_lo"])
            self.v_lo = float(z["v_lo"])
            self.v_hi = float(z["v_hi"]) if "v_hi" in z.files else 255.0

    def mask(self, hsv: np.ndarray) -> np.ndarray:
        if cv2 is None:
            return np.zeros(hsv.shape[:2], np.uint8)
        gate = cv2.inRange(
            hsv,
            (int(self.h_lo), int(self.s_lo), int(self.v_lo)),
            (int(self.h_hi), 255, int(self.v_hi)),
        )
        if self.hist is None:
            return gate
        bp = cv2.calcBackProject([hsv], [0, 1], self.hist, [0.0, 180.0, 0.0, 256.0], 1)
        cut = max(8.0, float(self.thresh) * 0.35)
        learned = cv2.threshold(bp, cut, 255, cv2.THRESH_BINARY)[1]
        out = cv2.bitwise_and(gate, learned)
        if cv2.countNonZero(out) == 0:
            return gate
        return out


def _side(nx: float) -> str:
    if nx > 0.08:
        return "sag"
    if nx < -0.08:
        return "sol"
    return "orta"


def _clean(mask):
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    return mask


def _isolate_vertical(mask):
    """Yatay su/zemin şeritlerini çıkar, dikey direkleri bırak."""
    horiz = cv2.getStructuringElement(cv2.MORPH_RECT, (45, 3))
    horizontal = cv2.morphologyEx(mask, cv2.MORPH_OPEN, horiz)
    return cv2.subtract(mask, horizontal)


def _best_tall(mask, min_area: int, min_aspect: float, img_w: int, img_h: int, pole: bool):
    work = _isolate_vertical(mask) if pole else mask
    n_comp, _labels, stats, cents = cv2.connectedComponentsWithStats(work, 8)
    best_i, best_score = 0, -1.0
    close_n = 0.06 * img_w * img_h
    for i in range(1, n_comp):
        area = int(stats[i, cv2.CC_STAT_AREA])
        w = int(stats[i, cv2.CC_STAT_WIDTH])
        h = int(stats[i, cv2.CC_STAT_HEIGHT])
        if area < min_area or w < 2 or h < 2:
            continue
        close = area >= close_n and h >= 0.70 * w
        tall = h >= w * min_aspect and w <= 0.32 * img_w and h >= 0.10 * img_h
        if not (close or tall):
            continue
        if pole:
            cy = float(cents[i][1])
            if cy > 0.70 * img_h and not close:
                continue
        score = float(area) * (h / max(1.0, w))
        if score > best_score:
            best_score, best_i = score, i
    if best_i <= 0:
        return None
    return stats, cents, best_i, int(stats[best_i, cv2.CC_STAT_AREA])


class FlareDetector:
    SEE_PX = 80
    CLOSE_PX = 480
    POLE_PX = 25

    POLES = (
        (
            "kirmizi",
            ((0, 110, 50), (6, 255, 255)),
            ((170, 110, 50), (180, 255, 255)),
            (40, 40, 255),
        ),
        ("sari", ((24, 100, 70), (40, 255, 255)), None, (0, 220, 255)),
        ("mavi", ((94, 90, 50), (128, 255, 255)), None, (255, 90, 40)),
    )

    def __init__(self, model_path: Path | None = None) -> None:
        self._bridge = CvBridge() if CvBridge is not None else None
        self.model = OrangeFlareModel(model_path)
        self.obs = FlareObs()
        self.scene = SceneObs()
        self.last_bgr = None
        self.debug_bgr = None

    def on_image(self, msg: Image, hud: str = "") -> None:
        if cv2 is None or self._bridge is None:
            return
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        h0, w0 = img.shape[:2]
        if w0 > 640:
            img = cv2.resize(img, (640, max(1, int(h0 * 640 / w0))))
        self.last_bgr = img
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, w = img.shape[:2]
        dbg = img.copy()
        poles: dict[str, ColorHit] = {}

        flare_mask = _clean(self.model.mask(hsv))
        flare_hit = self._hit_from_mask("flema", flare_mask, w, h, self.SEE_PX, 0.8, pole=False)
        if flare_hit is not None:
            self.obs = FlareObs(
                seen=flare_hit.pixels >= self.SEE_PX,
                close=flare_hit.pixels >= self.CLOSE_PX,
                pixels=flare_hit.pixels,
                cx=flare_hit.nx,
                cy=flare_hit.ny,
                width=flare_hit.w / w,
                height=flare_hit.h / h,
                side=flare_hit.side,
            )
        else:
            self.obs = FlareObs()

        for name, lo, hi2, _color in self.POLES:
            mask = cv2.inRange(hsv, lo[0], lo[1])
            if hi2 is not None:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, hi2[0], hi2[1]))
            if name in ("sari", "kirmizi"):
                mask = cv2.bitwise_and(mask, cv2.bitwise_not(flare_mask))
            mask = _clean(mask)
            hit = self._hit_from_mask(name, mask, w, h, self.POLE_PX, 1.35, pole=True)
            if hit is not None:
                poles[name] = hit

        gate = self._gate_align(hsv, w, h, flare_mask)
        self.scene = SceneObs(flare=self.obs, poles=poles)
        self.debug_bgr = self._draw(dbg, flare_hit, poles, hud, gate)

    def _hit_from_mask(self, name, mask, w, h, min_area, min_aspect, pole: bool):
        found = _best_tall(mask, min_area, min_aspect, w, h, pole)
        if found is None:
            return None
        stats, cents, idx, area = found
        x = int(stats[idx, cv2.CC_STAT_LEFT])
        y = int(stats[idx, cv2.CC_STAT_TOP])
        bw = int(stats[idx, cv2.CC_STAT_WIDTH])
        bh = int(stats[idx, cv2.CC_STAT_HEIGHT])
        cx = float(cents[idx][0])
        cy = float(cents[idx][1])
        nx = (cx / max(1.0, w - 1.0)) * 2.0 - 1.0
        ny = (cy / max(1.0, h - 1.0)) * 2.0 - 1.0
        return ColorHit(name, area, x, y, bw, bh, nx, ny, _side(nx))

    def _gate_align(self, hsv, w, h, flare_mask):
        red = cv2.inRange(hsv, (0, 90, 40), (8, 255, 255))
        red = cv2.bitwise_or(red, cv2.inRange(hsv, (170, 90, 40), (180, 255, 255)))
        red = cv2.bitwise_and(_clean(red), cv2.bitwise_not(flare_mask))
        green = _clean(cv2.inRange(hsv, (45, 70, 40), (85, 255, 255)))
        hit_r = self._hit_from_mask("kapi_k", red, w, h, 28, 1.25, pole=True)
        hit_g = self._hit_from_mask("kapi_y", green, w, h, 28, 1.25, pole=True)
        if hit_r is None or hit_g is None:
            return None
        cx_r = hit_r.x + hit_r.w / 2.0
        cx_g = hit_g.x + hit_g.w / 2.0
        if abs(cx_r - cx_g) < 40:
            return None
        return hit_r, hit_g, (cx_r + cx_g) / 2.0

    def _draw(self, dbg, flare_hit, poles: dict, hud: str, gate=None):
        color_map = {"flema": (0, 165, 255), "kirmizi": (40, 40, 255), "sari": (0, 220, 255), "mavi": (255, 90, 40)}
        h, w = dbg.shape[:2]
        if gate is not None:
            hit_r, hit_g, mid = gate
            mid_i = int(mid)
            cv2.line(dbg, (hit_r.x + hit_r.w // 2, 0), (hit_r.x + hit_r.w // 2, h), (40, 40, 255), 1)
            cv2.line(dbg, (hit_g.x + hit_g.w // 2, 0), (hit_g.x + hit_g.w // 2, h), (40, 220, 40), 1)
            cv2.line(dbg, (mid_i, 20), (mid_i, h - 20), (255, 255, 255), 2)
            cv2.circle(dbg, (mid_i, h // 2), 10, (255, 255, 255), 2)
            cv2.putText(
                dbg,
                "kapi orta",
                (mid_i + 8, h // 2 - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        lines = []
        if hud:
            lines.append(hud)
        for name, color in color_map.items():
            hit = flare_hit if name == "flema" else poles.get(name)
            if hit is None:
                lines.append(f"{name}: yok")
                continue
            cv2.rectangle(dbg, (hit.x, hit.y), (hit.x + hit.w, hit.y + hit.h), color, 2)
            cv2.circle(dbg, (hit.x + hit.w // 2, hit.y + hit.h // 2), 4, color, -1)
            tag = f"{name} {hit.pixels}px {hit.side}"
            tx = hit.x
            ty = max(16, hit.y - 6)
            cv2.putText(dbg, tag, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
            lines.append(tag)
        y = 18
        bar_h = 8 + 16 * len(lines)
        overlay = dbg.copy()
        cv2.rectangle(overlay, (0, 0), (dbg.shape[1], bar_h), (0, 0, 0), -1)
        dbg = cv2.addWeighted(overlay, 0.45, dbg, 0.55, 0)
        for line in lines:
            cv2.putText(dbg, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (240, 240, 240), 1, cv2.LINE_AA)
            y += 16
        return dbg
