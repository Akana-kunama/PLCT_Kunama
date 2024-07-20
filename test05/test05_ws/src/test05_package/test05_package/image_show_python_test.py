import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.declare_parameter('x', 100)  # Default values
        self.declare_parameter('y', 100)
        self.declare_parameter('width', 50)
        self.declare_parameter('height', 50)


        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Image, '/processed_image', 10)
        self.bridge = CvBridge()

    def listener_callback(self, data):
        self.get_logger().info('Processing image data...')
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            self.get_logger().info('Image processed successfully')
        except CvBridgeError as e:
            self.get_logger().info(f"Error converting image: {str(e)}")
            return

        # Retrieve parameters for rectangle dimensions and position
        x = self.get_parameter('x').get_parameter_value().integer_value
        y = self.get_parameter('y').get_parameter_value().integer_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value

        cv_image = self.add_rectangle(cv_image, x, y, width, height)

        try:
            new_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.publisher.publish(new_image_msg)
            self.get_logger().info('Image processed and published.')
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting image: {str(e)}")

    def add_rectangle(self, cv_image, x, y, width, height):
        """Draws a rectangle on an image."""
        top_left = (x, y)
        bottom_right = (x + width, y + height)
        cv2.rectangle(cv_image, top_left, bottom_right, (0, 255, 0), 2)
        return cv_image

def main(args=None):
    print('Hi from test05_package.')
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
