import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Initialize the ROS client library
rclpy.init()
node = Node('camera_publisher')
publisher = node.create_publisher(Image, '/image/video', 10)
bridge = CvBridge()
cap = cv2.VideoCapture(0)

def publish_image():
    ret, frame = cap.read()

    # Compress the image using OpenCV
    success, encoded_image = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80]) 
    compressed_frame = cv2.imdecode(np.frombuffer(encoded_image, np.uint8), cv2.IMREAD_COLOR)
    ros_image = bridge.cv2_to_imgmsg(compressed_frame, encoding='bgr8')
    publisher.publish(ros_image)
    #node.get_logger().info('Publishing compressed and decompressed image')

# Define a publishing rate of 10 Hz (0.1 seconds per callback)
node.create_timer(0.1, publish_image)

rclpy.spin(node)

cap.release()
node.destroy_node()
rclpy.shutdown()
