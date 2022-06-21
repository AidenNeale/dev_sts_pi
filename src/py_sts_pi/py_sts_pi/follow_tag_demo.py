# Basic ROS 2 program to subscribe to real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sts_pi_interfaces.msg import ArUcoInfo
from geometry_msgs.msg import Twist

class FlashcardDemo(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('FlashcardDemo')

    self.subscription = self.create_subscription(
      ArUcoInfo,
      'aruco_tag',
      self.listener_callback,
      10)
    self.subscription # prevent unused variable warning

    self.publisher_ = self.create_publisher(
      Twist,
      'twist_motor',
      10
    )

    # We will publish a message every 0.05 seconds
    timer_period = 0.05  # seconds

    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)

    self.arucoID = -1
    self.arucoTag = None
    self.movePosition = 0


  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    # self.get_logger().info('Receiving video frame')

    # Convert ROS Image message to OpenCV image

    if self.arucoID != data.id:
      self.arucoID = data.id
      self.arucoTag = data
      self.movePosition = 0


  def timer_callback(self):
    msg = Twist()



  def stopMovement(self, msg):
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    self.publisher_.publish(msg)

  def linearMovement(self, msg, speed):
    msg.linear.x = speed
    msg.angular.z = 0.0
    self.publisher_.publish(msg)

  def angularMovement(self, msg, angular):
    msg.linear.x = 1.0
    msg.angular.z = angular
    self.publisher_.publish(msg)

  def combinedMovement(self, msg, speed, angular):
    msg.linear.x = speed
    msg.angular.z = angular
    self.publisher_.publish(msg)


def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  flashcard_node = FlashcardDemo()

  # Spin the node so the callback function is called.
  rclpy.spin(flashcard_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  flashcard_node.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
  main()
