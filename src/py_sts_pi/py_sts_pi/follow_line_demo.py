# Basic ROS 2 program to subscribe to real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import cv2  # OpenCV library
import numpy as np
import rclpy  # Python library for ROS 2
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage  # Image is the message type
from geometry_msgs.msg import Twist


class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('display')

    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      CompressedImage,
      'frame',
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

    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    self.current_frame = None
    self.low_b = np.uint8([5,5,5])
    self.high_b = np.uint8([0,0,0])

  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    # self.get_logger().info('Receiving video frame')

    # Convert ROS Image message to OpenCV image
    self.current_frame = self.br.compressed_imgmsg_to_cv2(data)
    # Display image
    # cv2.imshow("camera", self.current_frame)

    # cv2.waitKey(1)

  def timer_calback(self):
    mask = cv2.inRange(self.current_frame, self.high_b, self.low_b)
    contours, hierarchy = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_NONE)
    if len(contours) > 0 :
      c = max(contours, key=cv2.contourArea)
      M = cv2.moments(c)
      if M["m00"] !=0 :
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        print("CX : "+str(cx)+"  CY : "+str(cy))

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
