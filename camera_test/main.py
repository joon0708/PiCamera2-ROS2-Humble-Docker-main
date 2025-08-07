import argparse
import cv2
import logging
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Parse command line arguments
parser = argparse.ArgumentParser(description="Picamera2 ROS2 publishing demo with options")
parser.add_argument("--resolution", type=str, help="Video resolution in WIDTHxHEIGHT format (default: 640x480)", default="640x480")
parser.add_argument("--edge_detection", action="store_true", help="Enable edge detection")
parser.add_argument("--topic", type=str, help="ROS topic name for publishing images (default: /camera/image_raw)", default="/camera/image_raw")
parser.add_argument("--rate", type=float, help="Publishing rate in Hz (default: 10.0)", default=10.0)
args = parser.parse_args()

# Parse resolution
resolution = tuple(map(int, args.resolution.split('x')))


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Create publisher
        self.publisher = self.create_publisher(Image, args.topic, qos_profile)
        
        # Create timer for publishing at specified rate
        self.timer = self.create_timer(1.0/args.rate, self.publish_frame)
        
        # Initialize camera
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_video_configuration(main={"size": resolution}))
        self.picam2.start()
        
        # Initialize CV bridge for converting between OpenCV and ROS images
        self.bridge = CvBridge()
        
        self.get_logger().info(f'Camera publisher started. Publishing to topic: {args.topic}')
        self.get_logger().info(f'Resolution: {resolution[0]}x{resolution[1]}')
        self.get_logger().info(f'Publishing rate: {args.rate} Hz')
        if args.edge_detection:
            self.get_logger().info('Edge detection enabled')

    def publish_frame(self):
        try:
            # Capture frame from camera
            frame = self.picam2.capture_array()
            
            # Convert from RGBA to BGR (remove alpha channel)
            if frame.shape[2] == 4:  # RGBA
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            elif frame.shape[2] == 3:  # RGB
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Apply edge detection if enabled
            if args.edge_detection:
                # Convert to grayscale for edge detection
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                # Apply edge detection
                frame = cv2.Canny(gray, 100, 200)
                # Convert back to BGR for proper display
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            
            # Convert OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            
            # Publish the image
            self.publisher.publish(ros_image)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing frame: {str(e)}')

    def __del__(self):
        if hasattr(self, 'picam2'):
            self.picam2.stop()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        camera_publisher = CameraPublisher()
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logging.error(f'Error in main: {str(e)}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
