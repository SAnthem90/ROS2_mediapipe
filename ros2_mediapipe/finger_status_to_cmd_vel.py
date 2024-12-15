import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

def callback(msg):
    # Assuming the first element of the array controls the movement
    finger_value = msg.data[0] if len(msg.data) > 0 else 0

    # Create a new Twist message for cmd_vel
    cmd_vel = Twist()

    if finger_value == 0 :   # Stop
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        print("Stopped")
    elif finger_value == 1: # Forward
        cmd_vel.linear.x = 1.0
        cmd_vel.angular.z = 0.0
        print("Moving Forward")
    elif finger_value == 2: # Backward
        cmd_vel.linear.x = -1.0
        cmd_vel.angular.z = 0.0
        print("Moving Backward")
    elif finger_value == 3: # Right
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = -1.0
        print("Moving Right")
    elif finger_value == 4: # Left
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 1.0
        print("Moving Left")

    # Publish the command
    cmd_vel_publisher.publish(cmd_vel)
# Initialize rclpy
rclpy.init()
# Create a Node
node = Node('finger_status_listener')
# Publisher for cmd_vel
cmd_vel_publisher = node.create_publisher(Twist, '/cmd_vel', 10)
# Subscriber for finger_status
node.create_subscription(Float32MultiArray,'/finger_status',callback,10)
# Spin the node to keep it running and processing callbacks
rclpy.spin(node)
# Shutdown when done
rclpy.shutdown()

