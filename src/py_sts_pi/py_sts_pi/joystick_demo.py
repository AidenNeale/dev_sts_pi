# Import the necessary libraries
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class Joystick(Node):
  """
  Create an Joystick class, which is a subclass of the Node class. This node
  subscribes to Joy and transforms those measurements into Twist messages

  Subscription:
  -------------
  /joy: Joy
    Receives a Joy message consisting of "Axes" and "Buttons" arrays

  Publisher:
  ----------
  /twist_motor: Twist
    Sends a twist dependent on which task it performs
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('Joystick')

    self.subscription = self.create_subscription(
      Joy,
      'joy',
      self.listener_callback,
      10)
    self.subscription # prevent unused variable warning

    self.publisher_ = self.create_publisher(
      Twist,
      'twist_motor',
      10
    )


  def listener_callback(self, data):
    """
    Callback function. This is called whenever anything is published to the topic.
    This function interprets controller messages and transforms that into the appropriate
    movement
    """
    msg = Twist()

    # Convert ROS Image message to OpenCV image
    self.joystick = data.axes # [0] = Left and Right, [1] = Up and Down
    self.combinedMovement(msg, self.joystick[1], self.joystick[0])


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
  joystick_node = Joystick()

  # Spin the node so the callback function is called.
  rclpy.spin(joystick_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  joystick_node.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
  main()
