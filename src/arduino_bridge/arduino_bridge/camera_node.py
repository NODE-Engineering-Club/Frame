"""
Camera node for color detection (blue, green, red) using OpenCV.

Publishes a short string on the `detected_colors` topic (std_msgs/String):
- "R", "G", "B" for single colours
- combinations like "RG", "GB", "RGB" when multiple colours present
- "0" when no target colour is detected

The `arduino_serial` node already subscribes to `detected_colors` and sends
that string over serial to the Arduino (so no extra glue is required).

Parameters (ROS2):
- `video_source` (int|string) : camera device (default '0' string), path/pipeline, or the special value `rpicam` (libcamera)
- `video_width` (int)         : capture width (default 640)
- `video_height` (int)        : capture height (default 480)
- `video_fps` (int)           : capture framerate (default 30)
- `publish_rate` (double)     : Hz (default 5.0)
- `min_area` (int)            : minimum mask pixel count to consider a colour
- `show_window` (bool)        : optionally display annotated frames

Notes:
- Use `video_source:=rpicam` to capture from Raspberry Pi Camera (libcamera) — the node will use a GStreamer/libcamera pipeline.
- Alternatively pass a GStreamer pipeline string prefixed with `gst:` (e.g. `gst:...`) to `video_source`.
"""
from __future__ import annotations

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CameraDetectNode(Node):
    """Detects red/green/blue areas in frames and publishes labels."""

    def __init__(self):
        super().__init__('camera_detect')

        # parameters
        self.declare_parameter('video_source', '0')
        self.declare_parameter('video_width', 640)
        self.declare_parameter('video_height', 480)
        self.declare_parameter('video_fps', 30)
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('min_area', 2000)
        self.declare_parameter('show_window', False)

        src = self.get_parameter('video_source').value
        self.video_width = int(self.get_parameter('video_width').value)
        self.video_height = int(self.get_parameter('video_height').value)
        self.video_fps = int(self.get_parameter('video_fps').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.min_area = int(self.get_parameter('min_area').value)
        self.show_window = bool(self.get_parameter('show_window').value)

        # accept '0' or 0 or '/dev/video0' etc.
        if isinstance(src, str) and src.isdigit():
            src = int(src)

        # Support Raspberry Pi libcamera (rpicam) and GStreamer pipelines:
        # - `rpicam` will build a libcamerasrc GStreamer pipeline using the
        #   configured width/height/fps and open it with the GStreamer backend.
        # - any string prefixed with `gst:` will be treated as a user-supplied
        #   GStreamer pipeline (omit the `gst:` prefix when passed to OpenCV).
        gst_pipeline = None
        if isinstance(src, str):
            lowered = src.lower()
            if lowered in ('rpicam', 'libcamera'):
                gst_pipeline = (
                    f"libcamerasrc ! video/x-raw,width={self.video_width},height={self.video_height},framerate={self.video_fps}/1 "
                    "! videoconvert ! video/x-raw,format=BGR ! appsink drop=true"
                )
                self.get_logger().info('Using libcamera GStreamer pipeline for rpicam')
            elif lowered.startswith('gst:'):
                gst_pipeline = src[4:]
                self.get_logger().info('Using user-specified GStreamer pipeline for VideoCapture')

        if gst_pipeline:
            self._cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        else:
            self._cap = cv2.VideoCapture(src)

        if not self._cap.isOpened():
            self.get_logger().error(f'Cannot open video source: {src}')
            raise RuntimeError(f'Cannot open video source: {src}')

        self.pub = self.create_publisher(String, 'detected_colors', 10)
        self._last_published = None

        timer_period = 1.0 / max(self.publish_rate, 1e-6)
        self.create_timer(timer_period, self._timer_cb)
        self.get_logger().info('Camera detect node started')

    # ---------- colour detection helpers ----------
    @staticmethod
    def _mask_for_hsv(hsv: np.ndarray, lower: tuple, upper: tuple) -> np.ndarray:
        return cv2.inRange(hsv, np.array(lower, dtype=np.uint8),
                           np.array(upper, dtype=np.uint8))

    def _detect_colours(self, frame: np.ndarray) -> tuple[str, dict]:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # red needs two ranges because hue wraps
        red_mask1 = self._mask_for_hsv(hsv, (0, 100, 50), (10, 255, 255))
        red_mask2 = self._mask_for_hsv(hsv, (170, 100, 50), (180, 255, 255))
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        green_mask = self._mask_for_hsv(hsv, (40, 70, 50), (90, 255, 255))
        blue_mask = self._mask_for_hsv(hsv, (100, 150, 50), (140, 255, 255))

        # clean up masks
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)

        counts = {
            'R': int(cv2.countNonZero(red_mask)),
            'G': int(cv2.countNonZero(green_mask)),
            'B': int(cv2.countNonZero(blue_mask)),
        }

        present = ''.join([c for c in 'RGB' if counts[c] >= self.min_area])
        if not present:
            present = '0'

        masks = {'R': red_mask, 'G': green_mask, 'B': blue_mask}
        return present, {'counts': counts, 'masks': masks}

    # ---------- timer callback ----------
    def _timer_cb(self):
        ok, frame = self._cap.read()
        if not ok or frame is None:
            self.get_logger().warning('Empty frame received from camera')
            return

        detected, debug = self._detect_colours(frame)
        if detected != self._last_published:
            self._last_published = detected
            msg = String()
            msg.data = detected
            self.pub.publish(msg)
            self.get_logger().info(f'Published detected colours: {detected}')

        if self.show_window:
            display = frame.copy()
            counts = debug['counts']
            # annotate counts
            cv2.putText(
                display,
                f"R:{counts['R']} G:{counts['G']} B:{counts['B']}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
            )
            # overlay masks (for visual debugging)
            combined = np.zeros_like(display)
            combined[:, :, 2] = debug['masks']['R']
            combined[:, :, 1] = debug['masks']['G']
            combined[:, :, 0] = debug['masks']['B']
            overlay = cv2.addWeighted(display, 0.6, combined, 0.4, 0)
            cv2.imshow('camera_detect', overlay)
            cv2.waitKey(1)

    def destroy_node(self):
        if self._cap is not None and self._cap.isOpened():
            self._cap.release()
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
