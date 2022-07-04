# Import the necessary libraries
import time
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sts_pi_interfaces.msg import ArUcoInfo
from geometry_msgs.msg import Twist

class FollowTagDemo(Node):
  """
  Create an FollowTagDemo class, which is a subclass of the Node class. This node subscribes
  to aruco_tag, determines where it is on the screen and sends Twist messages to attempt to
  ensure the robot aligns as close to the centre of the screen as possible.
  """
  def __init__(self):
    """
    Class constructor to set up the node.
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('FollowTagDemo')

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

    # Variable initialisation
    self.arucoTag = None
    self.tag_last_seen = time.time()


  def listener_callback(self, data):
    """
    Callback function - this is called every time the topic is published to. This
    function illicites the following behaviour:
    - If no Tag
      - Wait 5 seconds and then spin slowly
    - If Tag:
      - Head towards tag ensuring tag remains centred
    """
    self.arucoTag = data
    if data.id == -1:
      if ((time.time() - self.tag_last_seen) > 5):
        self.spin_those_bots()
      else:
        self.stop_those_bots()
    else:
      self.tag_last_seen = time.time()
      self.move_those_bots()


  def move_those_bots(self):
    """
    This function controls the quantatitive movement through Twist messages dependent
    on the tags location relative to the frame. Sharper turns occur if the tag is further
    from the centre of the image.
    """
    msg = Twist()
    if (self.arucoTag.x <= 150):
      #turn right
      self.combinedMovement(msg, 0.4, 0.65)
    elif (self.arucoTag.x <= 300):
      #turn right
      self.combinedMovement(msg, 0.4, 0.3)
    elif (self.arucoTag.x >= 480):
      # turn left
      self.combinedMovement(msg, 0.4, -0.65)
    elif (self.arucoTag.x >= 340):
      # turn left
      self.combinedMovement(msg, 0.4, -0.3)
    else:
      #drive straight
      self.linearMovement(msg, 0.7)


  def spin_those_bots(self):
    """
    This function causes the robot to spin slowly anti-clockwise
    """
    msg = Twist()
    self.combinedMovement(msg, 0.0, 0.325)


  def stop_those_bots(self):
    """
    This function sends a stop Twist to the robot.
    """
    msg = Twist()
    self.stopMovement(msg)


  def stopMovement(self, msg):
    """
    This sends a Twist that equates to no linear or angular velocity

    Parameters:
    -----------
    msg: Twist
      This is an empty Twist message
    """
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    self.publisher_.publish(msg)


  def linearMovement(self, msg, speed):
    """
    This sends a Twist that equates to no angular velocity and variable linear velocity

    Parameters:
    -----------
    msg: Twist
      This is an empty Twist message
    speed: float
      A value ranging between -1.0 and 1.0
    """
    msg.linear.x = speed
    msg.angular.z = 0.0
    self.publisher_.publish(msg)


  def combinedMovement(self, msg, speed, angular):
    """
    This function allows for complete variable control of linear and angular
    velocity published.

    Parameters:
    -----------
    msg: Twist
      This is an empty Twist message
    speed: float
      A value ranging between -1.0 and 1.0
    angular: float
      A value ranging between -1.0 and 1.0
    """
    msg.linear.x = speed
    msg.angular.z = angular
    self.publisher_.publish(msg)


def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  follow_tag_demo = FollowTagDemo()

  # Spin the node so the callback function is called.
  rclpy.spin(follow_tag_demo)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  follow_tag_demo.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
  main()
