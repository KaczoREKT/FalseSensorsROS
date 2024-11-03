import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class FakeCamera(Node):
    def __init__(self):
        super().__init__('fake_camera')
        self.declare_parameter('video_file', '/path/to/video.mp4')
        self.declare_parameter('publish_rate', 30)

        self.video_file = self.get_parameter('video_file').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().integer_value

        self.publisher = self.create_publisher(Image, 'fake_camera_image', 10)
        self.bridge = CvBridge()

        if self.video_file and os.path.isfile(self.video_file):
            self.cap = cv2.VideoCapture(self.video_file)
        else:
            self.get_logger().warn("No video file provided, using default camera.")
            self.cap = cv2.VideoCapture(0)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()
        if frame is not None:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    fake_camera = FakeCamera()
    rclpy.spin(fake_camera)
    fake_camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
