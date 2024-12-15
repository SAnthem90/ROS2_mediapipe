import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from cvzone.HandTrackingModule import HandDetector

# Initialize the ROS client library
rclpy.init()
node = Node('hand_tracking_publisher')
publisher = node.create_publisher(Image, '/image/video', 10)
bridge = CvBridge()
cap = cv2.VideoCapture(0)

# Initialize the hand detector from cvzone
hand_detector = HandDetector(detectionCon=0.8, maxHands=2)  # Adjust confidence and max hands

def publish_image():
    ret, frame = cap.read()
    if ret:
        # Detect hands and landmarks
        hands, img_with_hands = hand_detector.findHands(frame)  # Annotated frame returned

        # Process detected hands
        for hand in hands:
            lm_list = hand['lmList']  # List of 21 Landmark points
            bbox = hand['bbox']  # Bounding box info (x, y, w, h)
            center = hand['center']  # Center of the hand (cx, cy)

            # Example: Log details of the detected hand
            node.get_logger().info(f'Hand detected with center: {center} and bbox: {bbox}')

        # Convert the annotated frame to ROS Image message
        ros_image = bridge.cv2_to_imgmsg(img_with_hands, encoding='bgr8')
        publisher.publish(ros_image)
        node.get_logger().info('Publishing image with hand annotations')

# Define a publishing rate of 10 Hz (0.1 seconds per callback)
node.create_timer(0.1, publish_image)

rclpy.spin(node)

cap.release()
node.destroy_node()
rclpy.shutdown()
