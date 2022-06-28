import explorerhat
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from geometry_msgs.msg import Twist
from sts_pi_interfaces.msg import MotorSpeeds
from threading import Thread

MAX_SPEED = 100

class Motors(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('motor_control')

    self.left_motor_speed = 0.0
    self.right_motor_speed = 0.0
    self.instruction = Twist()

    self.subscription = self.create_subscription(
      Twist,
      'twist_motor',
      self.listener_callback,
      10)
    self.subscription # prevent unused variable warning

    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(MotorSpeeds, 'motor_speeds', 10)

    # We will publish a message every 0.05 seconds
    timer_period = 0.05  # seconds

    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)


  def determine_speed(self, twist_msg):
    linear_velocity = 1.0#twist_msg.linear.x
    angular_velocity = 0.3#twist_msg.angular.z
    b = 0.12
    r = 0.025


    self.left_motor_speed = ((linear_velocity - (angular_velocity * b / 2)) / r)
    self.right_motor_speed = ((linear_velocity + (angular_velocity * b / 2)) / r)

    if self.left_motor_speed > MAX_SPEED:
      self.left_motor_speed = MAX_SPEED
    elif self.left_motor_speed < -MAX_SPEED:
      self.left_motor_speed = -MAX_SPEED
    if self.right_motor_speed > MAX_SPEED:
      self.right_motor_speed = MAX_SPEED
    elif self.right_motor_speed < -MAX_SPEED:
      self.right_motor_speed = -MAX_SPEED
    # # Hard limits speeds to prevent motor problems from incorrect Twist messages
    # if abs(linear_velocity) > MAX_SPEED: 
    #     linear_velocity %= (MAX_SPEED + 1) 
    
    # if (angular_velocity != 0 and linear_velocity != 0): # STS-Pi is driving forwards with angular velocity
    #   if angular_velocity < 0: # Turn Left, left wheel slowed
    #     self.left_motor_speed = linear_velocity * (1-abs(angular_velocity * 2))
    #     self.right_motor_speed = linear_velocity
    #   elif angular_velocity > 0: # Turn Right, right wheel slowed
    #     self.left_motor_speed = linear_velocity
    #     self.right_motor_speed = linear_velocity * (1 - abs(angular_velocity * 2))

    # elif (angular_velocity != 0):

    # else: # STS-Pi is driving forwards with no angular velocity
    #   self.left_motor_speed = self.right_motor_speed = linear_velocity


  def listener_callback(self, data):
    # 'data' is in the form of a Twist
    # This is turned into relative movement 
    self.determine_speed(data)

    #Turn instruction to speed
    if self.left_motor_speed < 0:
      explorerhat.motor.one.backwards(abs(self.left_motor_speed))
    else:
      explorerhat.motor.one.forwards(self.left_motor_speed)
    if self.right_motor_speed < 0:
      explorerhat.motor.two.backwards(abs(self.right_motor_speed))
    else:
      explorerhat.motor.two.forwards(self.right_motor_speed)


  def timer_callback(self):
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
