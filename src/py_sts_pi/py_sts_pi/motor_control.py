import explorerhat
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from geometry_msgs.msg import Twist

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

    self.left = 0
    self.right = 0

    self.subscription = self.create_subscription(
      Twist,
      'twist_motor',
      self.listener_callback,
      10)
    self.subscription # prevent unused variable warning

    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Twist, 'motor_speeds', 10)

    # We will publish a message every 0.05 seconds
    timer_period = 0.05  # seconds

    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)

  def listener_callback(self, data):
    # 'data' is in the form of a Twist
    # This is turned into relative movement 
    print(data)


  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.01 seconds.
    """
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
      # self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)
    ret, frame = self.cap.read()

    if ret == True:
      # Publish the image.
      # The 'cv2_to_imgmsg' method converts an OpenCV
      # image to a ROS 2 image message
      self.publisher_.publish()

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
