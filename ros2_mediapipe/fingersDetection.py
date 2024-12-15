import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
from cvzone.HandTrackingModule import HandDetector
import numpy as np

rclpy.init()
node = Node('hand_tracking_publisher_subscriber')
bridge = CvBridge()
hand_detector = HandDetector(detectionCon=0.8, maxHands=2)  # Adjust confidence and max hands

# Publishers for the processed video and finger states
video_publisher = node.create_publisher(Image, '/processed_video', 10)
fingers_publisher = node.create_publisher(Float32MultiArray, '/fingers_state', 10)

# Subscription callback
def listener_callback(msg):
    # Convert ROS Image message to OpenCV image
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # Detect hands and landmarks
    hands, img_with_hands = hand_detector.findHands(frame)  # Annotated frame returned

    # Initialize array for finger states
    fingers_state_array = []

    # Process detected hands
    for hand in hands:
        lm_list = hand['lmList']  # List of 21 Landmark points
        bbox = hand['bbox']  # Bounding box info (x, y, w, h)
        center = hand['center']  # Center of the hand (cx, cy)

        # Check which fingers are open
        fingers = hand_detector.fingersUp(hand)
        fingers_state_array.append(fingers)

        # Log the states of fingers
        node.get_logger().info(f'Fingers state: {fingers}')

    # Flatten the array for publishing if multiple hands are detected
    if fingers_state_array:
        fingers_state_np = np.array(fingers_state_array).flatten().astype(float)
    else:
        fingers_state_np = np.array([], dtype=float)

    # Publish the finger states as a numpy array
    fingers_msg = Float32MultiArray()
    fingers_msg.data = fingers_state_np.tolist()
    fingers_publisher.publish(fingers_msg)

    # Publish the annotated video frame to the new topic
    ros_image = bridge.cv2_to_imgmsg(img_with_hands, encoding='bgr8')
    video_publisher.publish(ros_image)

# Subscribe to the topic
subscription = node.create_subscription(Image,'/image/video',listener_callback,10)
# Spin the node
rclpy.spin(node)
# Shutdown
node.destroy_node()
rclpy.shutdown()
