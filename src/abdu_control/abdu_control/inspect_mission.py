"""Ön kamera ile flemayı gör, kaç, kapıdan geç, dubalara çarp, kapıdan geri geç, evde dur."""

from __future__ import annotations

import math
import os
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image

from abdu_control.flare_detector import FlareDetector


def _record_dir() -> Path:
    for p in Path(__file__).resolve().parents:
        if (p / "calistir.sh").is_file():
            return p / "recordings"
    return Path.home() / "abdu_recordings"


class CameraRecorder:
    def __init__(self, log) -> None:
        self._log = log
        self._writer = None
        self.path: Path | None = None
        self._size = None
        out_dir = _record_dir()
        out_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = out_dir / f"abdu_on_kamera_{stamp}.mp4"

    def write(self, bgr) -> None:
        if bgr is None or bgr.size == 0:
            return
        h, w = bgr.shape[:2]
        size = (w, h)
        if self._writer is None:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._writer = cv2.VideoWriter(str(self.path), fourcc, 15.0, size)
            if not self._writer.isOpened():
                self.path = self.path.with_suffix(".avi")
                self._writer = cv2.VideoWriter(
                    str(self.path), cv2.VideoWriter_fourcc(*"XVID"), 15.0, size
                )
            self._size = size
            if self._writer.isOpened():
                self._log.info(f"Kamera kaydi: {self.path}")
            else:
                self._log.error("Kamera kaydi acilamadi")
                return
        if (w, h) != self._size:
            bgr = cv2.resize(bgr, self._size)
        self._writer.write(bgr)

    def close(self) -> None:
        if self._writer is not None:
            self._writer.release()
            self._writer = None
            if self.path is not None:
                self._log.info(f"Kamera kaydi kaydedildi: {self.path}")


def _wrap(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class GateFlareMission(Node):
    GATE_X = 3.0
    FLARE_X = 0.55
    HOME_X = -3.6
    DEPTH = -1.10
    DODGE_Y = 0.52
    TARGETS = (
        (4.90, 0.70, "kirmizi"),
        (6.55, -0.90, "sari"),
        (8.10, 0.50, "mavi"),
    )

    def __init__(self) -> None:
        super().__init__("abdu_inspect_mission")
        self.detector = FlareDetector()
        self.bridge = CvBridge()
        self.recorder = CameraRecorder(self.get_logger())
        self.declare_parameter("preview", True)
        self._show = bool(os.environ.get("DISPLAY")) and bool(
            self.get_parameter("preview").get_parameter_value().bool_value
        )
        self._win_name = "ABDU kamera tespit"
        self._win_ready = False
        self.pose = None
        self.state = "APPROACH"
        self.dodge_sign = 1.0
        self.hit_i = 0
        self.cmd_pub = self.create_publisher(Twist, "/abdu/cmd_vel", 10)
        self.debug_pub = self.create_publisher(Image, "/abdu/debug/view", 10)
        self.create_subscription(Odometry, "/abdu/odom", self._on_odom, 10)
        self.create_subscription(Image, "/abdu/front_camera/image_raw", self._on_image, 10)
        self.create_timer(0.05, self._on_timer)
        self.get_logger().info(
            "Gorev: on kamera ile flema kac, kapidan gec, "
            "kirmizi/sari/mavi dubalara carpi, kapidan geri gec, baslangicta dur"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self.pose = msg

    def _on_image(self, msg: Image) -> None:
        self.detector.on_image(msg, hud=f"durum {self.state}")
        obs = self.detector.obs
        if obs.seen:
            self.dodge_sign = 1.0 if obs.cx >= 0.0 else -1.0
        dbg = self.detector.debug_bgr
        if dbg is None:
            return
        self.recorder.write(dbg)
        self._preview(dbg)
        try:
            out = self.bridge.cv2_to_imgmsg(dbg, "bgr8")
            out.header = msg.header
            self.debug_pub.publish(out)
        except Exception:
            pass

    def _preview(self, dbg) -> None:
        if not self._show:
            return
        try:
            if not self._win_ready:
                cv2.namedWindow(self._win_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self._win_name, 640, 360)
                self._win_ready = True
            visible = cv2.getWindowProperty(self._win_name, cv2.WND_PROP_VISIBLE)
            if visible < 1:
                self._close_preview()
                return
            cv2.imshow(self._win_name, dbg)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                self._close_preview()
                return
            if cv2.getWindowProperty(self._win_name, cv2.WND_PROP_VISIBLE) < 1:
                self._close_preview()
        except cv2.error:
            self._close_preview()

    def _close_preview(self) -> None:
        self._show = False
        self._win_ready = False
        try:
            cv2.destroyWindow(self._win_name)
            cv2.waitKey(1)
        except Exception:
            pass
        self.get_logger().info("Kamera penceresi kapatildi (kayit devam ediyor)")

    def destroy_node(self) -> None:
        self.recorder.close()
        if self._win_ready:
            try:
                cv2.destroyAllWindows()
                cv2.waitKey(1)
            except Exception:
                pass
        super().destroy_node()

    def _hold_depth(self, z: float) -> float:
        return _clamp(0.7 * (self.DEPTH - z), -0.35, 0.35)

    def _lane_y(self, x: float, going_plus_x: bool) -> float:
        """Fleme hizasinda yan koridor, kapı/evde tekrar orta."""
        if going_plus_x:
            if x < self.FLARE_X + 0.85:
                return self.DODGE_Y * self.dodge_sign
            return 0.0
        if x > self.FLARE_X - 0.85:
            return self.DODGE_Y * self.dodge_sign
        return 0.0

    def _go_to(self, x: float, y: float, yaw_cmd: float | None, p, yaw: float) -> Twist:
        dx, dy = x - p.x, y - p.y
        dist_xy = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        yaw_err = _wrap(bearing - yaw) if dist_xy > 0.18 else _wrap((yaw_cmd if yaw_cmd is not None else yaw) - yaw)
        cmd = Twist()
        cmd.linear.x = _clamp(0.50 * dist_xy * math.cos(yaw_err), -0.40, 0.50)
        cmd.linear.y = _clamp(0.70 * dist_xy * math.sin(yaw_err) + 0.55 * (y - p.y), -0.45, 0.45)
        cmd.linear.z = self._hold_depth(p.z)
        cmd.angular.z = _clamp(1.25 * yaw_err, -0.75, 0.75)
        return cmd

    def _ram_cmd(self, tgt, p, yaw: float) -> Twist:
        tx, ty, _name = tgt
        cmd = self._go_to(tx + 0.32, ty, 0.0, p, yaw)
        cmd.linear.x = max(cmd.linear.x, 0.48)
        cmd.linear.y = _clamp(1.15 * (ty - p.y), -0.55, 0.55)
        return cmd

    def _on_timer(self) -> None:
        if self.pose is None:
            return
        p = self.pose.pose.pose.position
        yaw = _yaw_from_quat(self.pose.pose.pose.orientation)
        obs = self.detector.obs
        cmd = Twist()
        cmd.linear.z = self._hold_depth(p.z)

        if self.state == "APPROACH":
            vision_hit = obs.seen and (obs.close or (obs.pixels > 90 and abs(obs.cx) < 0.55))
            near = p.x > self.FLARE_X - 1.4
            if vision_hit or near:
                self.state = "AVOID"
                why = f"{obs.pixels}px {obs.side}" if vision_hit else "mesafe"
                self.get_logger().info(f"Flema → kacis ({why})")
                cmd = self._go_to(self.FLARE_X + 1.0, self._lane_y(p.x, True), 0.0, p, yaw)
            else:
                cmd = self._go_to(self.FLARE_X - 1.2, 0.0, 0.0, p, yaw)

        elif self.state == "AVOID":
            if p.x > self.FLARE_X + 0.70 and abs(p.y) > 0.28:
                self.state = "GATE"
                self.get_logger().info("Flema gecildi → kapı")
                cmd = self._go_to(self.GATE_X + 1.15, self._lane_y(p.x, True), 0.0, p, yaw)
            else:
                cmd = self._go_to(self.FLARE_X + 1.0, self._lane_y(p.x, True), 0.0, p, yaw)
                if obs.seen and obs.close:
                    cmd.linear.x = min(cmd.linear.x, 0.22)
                    cmd.linear.y = _clamp(cmd.linear.y + self.dodge_sign * 0.15, -0.5, 0.5)

        elif self.state == "GATE":
            if p.x > self.GATE_X + 0.75:
                self.state = "RAM"
                self.hit_i = 0
                self.get_logger().info("Kapı gecildi → kirmizi duba")
                cmd = self._ram_cmd(self.TARGETS[0], p, yaw)
            else:
                cmd = self._go_to(self.GATE_X + 1.15, self._lane_y(p.x, True), 0.0, p, yaw)

        elif self.state == "RAM":
            tgt = self.TARGETS[self.hit_i]
            tx, ty, name = tgt
            dist = math.hypot(p.x - tx, p.y - ty)
            aligned = abs(p.y - ty) < 0.18
            through = p.x > tx + 0.08
            if aligned and (dist < 0.30 or through):
                self.get_logger().info(f"{name} dubaya carpidı")
                self.hit_i += 1
                if self.hit_i >= len(self.TARGETS):
                    self.state = "TURN"
                    self.get_logger().info("Dubalar bitti → kapıya don")
                    err = _wrap(math.pi - yaw)
                    cmd.angular.z = _clamp(1.5 * err, -0.9, 0.9)
                    cmd.linear.x = 0.0
                    cmd.linear.z = self._hold_depth(p.z)
                else:
                    cmd = self._ram_cmd(self.TARGETS[self.hit_i], p, yaw)
            else:
                cmd = self._ram_cmd(tgt, p, yaw)

        elif self.state == "TURN":
            err = _wrap(math.pi - yaw)
            cmd.angular.z = _clamp(1.5 * err, -0.9, 0.9)
            cmd.linear.x = 0.0
            cmd.linear.y = _clamp(-0.70 * p.y, -0.30, 0.30)
            cmd.linear.z = self._hold_depth(p.z)
            if abs(err) < 0.16:
                self.state = "GATE_BACK"
                self.get_logger().info("Kapıdan geri gecis")

        elif self.state == "GATE_BACK":
            if p.x < self.GATE_X - 0.75:
                self.state = "RETURN"
                self.get_logger().info("Kapı geri gecildi → flema kac, eve don")
                cmd = self._go_to(self.HOME_X, self._lane_y(p.x, False), math.pi, p, yaw)
            else:
                cmd = self._go_to(self.GATE_X - 1.20, 0.0, math.pi, p, yaw)
                if p.x < self.GATE_X + 1.8:
                    cmd.linear.y = _clamp(-1.25 * p.y, -0.45, 0.45)
                    cmd.linear.x = min(cmd.linear.x, 0.34)

        elif self.state == "RETURN":
            if p.x < self.HOME_X + 0.22 and abs(p.y) < 0.16:
                self.state = "HOME"
                self.get_logger().info("Baslangica donuldu, duruyor")
                cmd = Twist()
                cmd.linear.z = self._hold_depth(p.z)
            else:
                cmd = self._go_to(self.HOME_X, self._lane_y(p.x, False), math.pi, p, yaw)
                if obs.seen and obs.close:
                    cmd.linear.x = min(abs(cmd.linear.x), 0.22)
                    cmd.linear.y = _clamp(cmd.linear.y + self.dodge_sign * 0.15, -0.5, 0.5)

        elif self.state == "HOME":
            dx = self.HOME_X - p.x
            dist = math.hypot(dx, p.y)
            if dist < 0.16:
                cmd = Twist()
                cmd.linear.z = self._hold_depth(p.z)
            else:
                cmd = self._go_to(self.HOME_X, 0.0, 0.0, p, yaw)
                cmd.linear.x = _clamp(cmd.linear.x, -0.18, 0.18)
                cmd.linear.y = _clamp(cmd.linear.y, -0.22, 0.22)

        self.cmd_pub.publish(cmd)


def main() -> None:
    rclpy.init()
    node = GateFlareMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
