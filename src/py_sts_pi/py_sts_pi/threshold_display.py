# Import the necessary libraries
import cv2  # OpenCV library
import rclpy  # Python library for ROS 2
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage  # CompressedImage is the message type


class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class. The purpose of
  this node is to display the adaptive thresholding of the published video frame.

  Subscription:
  -------------
  /video_frames: CompressedImage
    ROS2 Standard Compressed Image
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('Threshold')

    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      CompressedImage,
      'video_frames',
      self.listener_callback,
      10)
    self.subscription # prevent unused variable warning

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()


  def listener_callback(self, data):
    """
    Callback function - This is called every time something is published to the topic.
    This function takes the passed frame and applies adaptive thresholding to the image.
    This is then displayed using the 'imshow' command.
    """
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.compressed_imgmsg_to_cv2(data)
    # Adaptive thresholding can only occur on grayscale images
    gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    #Applying gaussian blur helps to make images crisper after thresholding
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 21, 4)
    cv2.namedWindow('Threshold',cv2.WINDOW_NORMAL)
    # Display image
    cv2.imshow("Threshold", thresh)

    cv2.waitKey(1)


def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  image_subscriber = ImageSubscriber()

  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
  main()
