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

        # Publisher (raw or compressed)
        if args.compressed:
            self.pub = self.create_publisher(CompressedImage, args.topic + "/compressed", qos_profile)
        else:
            self.pub = self.create_publisher(Image, args.topic, qos_profile)

        # Timer
        self.timer = self.create_timer(max(1.0/args.rate, 1e-3), self.publish_frame)

        # Camera
        self.picam2 = Picamera2()
        cfg = self.picam2.create_video_configuration(
            main={"size": (W, H), "format": "RGB888"}
        )
        self.picam2.configure(cfg)
        self.picam2.start()

        # Bridge
        self.bridge = CvBridge()

        self.get_logger().info(
            f"Publishing to: {self.pub.topic_name} | res={W}x{H} | rate={args.rate}Hz | "
            f"{'COMPRESSED(JPEG)' if args.compressed else 'RAW(bgr8)'} | "
            f"QoS={'BEST_EFFORT' if args.best_effort else 'RELIABLE'} | "
            f"downscale={args.downscale} | q={args.jpeg_quality}"
        )

    def publish_frame(self):
        try:
            frame = self.picam2.capture_array()  # RGB
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # Edge (optional)
            if args.edge_detection:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                frame = cv2.Canny(gray, 100, 200)
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

            # Downscale (optional)
            if args.downscale and args.downscale != 1.0:
                frame = cv2.resize(
                    frame, None, fx=args.downscale, fy=args.downscale,
                    interpolation=cv2.INTER_AREA
                )

            if args.compressed:
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.format = "jpeg"
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), int(np.clip(args.jpeg_quality, 1, 100))]
                ok, buf = cv2.imencode(".jpg", frame, encode_param)
                if not ok:
                    return
                msg.data = np.array(buf).tobytes()
                self.pub.publish(msg)
            else:
                # RAW (무거움: 네트워크가 버틸 때만)
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.pub.publish(ros_image)

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
