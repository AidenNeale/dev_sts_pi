# Import the necessary libraries
import cv2  # OpenCV library
import rclpy  # Python library for ROS 2
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage  # Image is the message type


class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class. The purpose
  of this node is to subscribe to a topic with type CompressedImage and display the frame
  in a window locally where the node is ran.

  Subscription:
  -------------
  /frame: CompressedImage
    Compressed ROS2 Image
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('display')

    # Create the subscriber. This subscriber will receive an Image
    # from the frame topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      CompressedImage,
      'frame',
      self.listener_callback,
      10)
    self.subscription # prevent unused variable warning

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

  def listener_callback(self, data):
    """
    Callback function. This is called
    whenever a message is published to the subscribed topic.
    """
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.compressed_imgmsg_to_cv2(data)
    cv2.namedWindow('camera',cv2.WINDOW_NORMAL) # Allows window to be resized
    # Display image
    cv2.imshow("camera", current_frame)

    cv2.waitKey(1)

def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  display = ImageSubscriber()

  # Spin the node so the callback function is called.
  rclpy.spin(display)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  display.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
  main()
