import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from classify import Classify
import cv2


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('camera_reader')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(CompressedImage, 'image_raw/compressed', self.listener_callback, 1)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, image):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        cv_image = self.bridge.compressed_imgmsg_to_cv2(image, desired_encoding='passthrough')
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        # cv2.imshow("Image window", image)
        Classify(image)
        # cv2.waitKey(3)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()