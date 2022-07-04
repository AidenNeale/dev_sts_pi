import explorerhat
import math
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from geometry_msgs.msg import Twist
from sts_pi_interfaces.msg import MotorSpeeds

MAX_SPEED = 100

class Motors(Node):
  """
  Create an Motors class, which is a subclass of the Node class. The purpose of the
  node is to send motor control commands using the explorerhat library.

  Subscription:
  -------------
  /twist_motor: Twist
    Twist message containing linear and angular velocity ratios ranging from -1.0 and 1.0

  Publisher:
  ----------
  /motor_speeds: Custom Message ~ MotorSpeeds
    Publishes left and right motor speeds as float
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('motor_control')

    #Variable Initialisation
    self.left_motor_speed = 0.0
    self.right_motor_speed = 0.0
    self.instruction = Twist()

    self.subscription = self.create_subscription(
      Twist,
      'twist_motor',
      self.listener_callback,
      10)
    self.subscription # prevent unused variable warning

    # Create the publisher. This publisher will publish MotorSpeeds
    # to the motor_speeds topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(MotorSpeeds, 'motor_speeds', 10)

    # We will publish a message every 0.05 seconds
    timer_period = 0.05  # seconds

    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)


  def determine_speed(self, twist_msg):
    """
    This function utilises differential wheeled kinematics to calculate the
    left and right motor speeds using the knowledge that the maximum linear speed is
    0.14m/s on a concrete floor.
    Parameters:
    -----------
    twist_msg: Twist
      This is the twist message received in the callback
    """
    max_lin_speed = 0.146 #Maximum speed in m/s
    b = 0.12 #Length of wheel-wheel
    r = 0.025 #Radius of wheel
    scale_factor = 100 / (max_lin_speed / r)

    linear_velocity = twist_msg.linear.x * max_lin_speed
    angular_velocity = math.radians(twist_msg.angular.z * 60)

    self.left_motor_speed = ((linear_velocity + (angular_velocity * b / 2)) / r) * scale_factor
    self.right_motor_speed = ((linear_velocity - (angular_velocity * b / 2)) / r) * scale_factor

    if self.left_motor_speed > MAX_SPEED:
      self.left_motor_speed = MAX_SPEED
    elif self.left_motor_speed < -MAX_SPEED:
      self.left_motor_speed = -MAX_SPEED
    if self.right_motor_speed > MAX_SPEED:
      self.right_motor_speed = MAX_SPEED
    elif self.right_motor_speed < -MAX_SPEED:
      self.right_motor_speed = -MAX_SPEED


  def listener_callback(self, data):
    """
    This callback function is called whenever anything is published to the topic.
    This function is intended to take a Twist message and transform it to a value
    that the motors can understand ( a value between 0 - 100).
    """
    self.determine_speed(data)

    # As motor forwards cannot take negative values, this handles reversing
    if self.left_motor_speed < 0:
      explorerhat.motor.one.backwards(abs(self.left_motor_speed))
    else:
      explorerhat.motor.one.forwards(self.left_motor_speed)
    if self.right_motor_speed < 0:
      explorerhat.motor.two.backwards(abs(self.right_motor_speed))
    else:
      explorerhat.motor.two.forwards(self.right_motor_speed)


  def timer_callback(self):
    """
    This function is used to publish motor speeds every 0.05 seconds.
    """
    msg = MotorSpeeds()
    msg.left = float(self.left_motor_speed)
    msg.right = float(self.right_motor_speed)
    self.publisher_.publish(msg)


def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  motor_control = Motors()

  # Spin the node so the callback function is called.
  rclpy.spin(motor_control)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  motor_control.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
  main()
