#!/usr/bin/env python3
import argparse
import cv2
import logging
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from picamera2 import Picamera2
import time

# ----------------- CLI -----------------
parser = argparse.ArgumentParser(description="Picamera2 ROS2 publisher (bandwidth-friendly)")
parser.add_argument("--resolution", type=str, default="640x480", help="WIDTHxHEIGHT (e.g., 640x480)")
parser.add_argument("--rate", type=float, default=10.0, help="Publish rate (Hz)")
parser.add_argument("--topic", type=str, default="/camera/image_raw", help="Topic base name")
parser.add_argument("--edge_detection", action="store_true", help="Enable edge detection")
parser.add_argument("--compressed", action="store_true", help="Publish CompressedImage (JPEG)")
parser.add_argument("--jpeg_quality", type=int, default=80, help="JPEG quality 1-100 (default 80)")
parser.add_argument("--downscale", type=float, default=1.0, help="Resize factor (e.g., 0.5 halves W/H)")
parser.add_argument("--best_effort", action="store_true", help="Use BEST_EFFORT QoS (good for Wi-Fi)")
parser.add_argument("--debug_colors", action="store_true", help="Draw red/blue boxes to verify color order")
args = parser.parse_args()

W, H = map(int, args.resolution.split("x"))

# ----------------- QoS -----------------
qos_profile = QoSProfile(
    reliability=(ReliabilityPolicy.BEST_EFFORT if args.best_effort else ReliabilityPolicy.RELIABLE),
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")

        # Always publish RAW (rgb8) base topic
        self.pub_raw = self.create_publisher(Image, args.topic, qos_profile)
        # Optionally publish compressed
        self.pub_comp = None
        if args.compressed:
            self.pub_comp = self.create_publisher(CompressedImage, args.topic + "/compressed", qos_profile)

        # Timer
        self.timer = self.create_timer(max(1.0/args.rate, 1e-3), self.publish_frame)

        # Camera
        self.picam2 = Picamera2()
        cfg = self.picam2.create_video_configuration(main={"size": (W, H), "format": "RGB888"})
        self.picam2.configure(cfg)
        self.picam2.start()
        time.sleep(1.0)  # AWB/AE warm-up for stable color

        # Bridge
        self.bridge = CvBridge()

        self.get_logger().info(
            f"Publishing RAW on: {args.topic} (encoding=rgb8), "
            f"{'and COMPRESSED on: ' + args.topic + '/compressed' if args.compressed else 'no compressed output'} | "
            f"res={W}x{H} | rate={args.rate}Hz | QoS={'BEST_EFFORT' if args.best_effort else 'RELIABLE'} | "
            f"downscale={args.downscale} | jpeg_q={args.jpeg_quality}"
        )

    def publish_frame(self):
        try:
            # 1) Capture: Picamera2 returns RGB
            frame_rgb = self.picam2.capture_array()  # RGB

            # 2) Optional edge detection (keep RGB)
            if args.edge_detection:
                gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
                edge = cv2.Canny(gray, 100, 200)
                frame_rgb = cv2.cvtColor(edge, cv2.COLOR_GRAY2RGB)

            # 3) Optional downscale
            if args.downscale and args.downscale != 1.0:
                frame_rgb = cv2.resize(frame_rgb, None, fx=args.downscale, fy=args.downscale, interpolation=cv2.INTER_AREA)

            # 4) Optional debug color overlay (to verify R/B correctness)
            if args.debug_colors:
                # RGB space overlays
                cv2.rectangle(frame_rgb, (10, 10), (110, 60), (255,   0,   0), -1)  # RED (RGB)
                cv2.rectangle(frame_rgb, (120,10), (220, 60), (  0,   0, 255), -1)  # BLUE (RGB)

            # 5) Publish RAW (exact RGB)
            self.pub_raw.publish(self.bridge.cv2_to_imgmsg(frame_rgb, encoding="rgb8"))

            # 6) Publish Compressed if enabled (convert to BGR just before JPEG)
            if self.pub_comp is not None:
                frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                if args.debug_colors:
                    # BGR space overlays (left red, right blue)
                    cv2.rectangle(frame_bgr, (230,10), (330, 60), (0,   0, 255), -1)  # RED (BGR)
                    cv2.rectangle(frame_bgr, (340,10), (440, 60), (255, 0,   0), -1)  # BLUE (BGR)

                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.format = "jpeg"
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), int(np.clip(args.jpeg_quality, 1, 100))]
                ok, buf = cv2.imencode(".jpg", frame_bgr, encode_param)
                if ok:
                    msg.data = buf.tobytes()
                    self.pub_comp.publish(msg)

        except Exception as e:
            self.get_logger().error(f"publish_frame error: {e}")

    def __del__(self):
        try:
            self.picam2.stop()
        except Exception:
            pass

def main(argv=None):
    rclpy.init(args=argv)
    try:
        node = CameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logging.error(f"Error in main: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
