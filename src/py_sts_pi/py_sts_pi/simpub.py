# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from std_msgs.msg import Float32MultiArray # Image is the message type
from sts_pi_interfaces.msg import ArUcoInfo

class SimplePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('simpub')

    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(ArUcoInfo, 'stuff', 10)

    # We will publish a message every 0.01 seconds
    timer_period = 0.05  # seconds

    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)


  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.01 seconds.
    """
    msg = ArUcoInfo()
    msg.id = 10
    msg.x = 1.0
    msg.y = 1.0
    msg.z = 0.0
    msg.theta = 10.0


    self.publisher_.publish(msg)

    # Display the message on the console
    # self.get_logger().info('Publishing video frame')

def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  simple_publisher = SimplePublisher()

  # Spin the node so the callback function is called.
  rclpy.spin(simple_publisher)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  simple_publisher.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
  main()