# Basic ROS 2 program to subscribe to real-time streaming
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import math
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage # Image is the message type
from sts_pi_interfaces.msg import ArUcoInfo
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library


class ArUcoTagReader(Node):
  """
  Create an ArUcoTagReader class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('ArUco_Tag_Reader')
    '''
    Class Specific Variables
    '''
    # Used to store frame retrieved by subscription
    self.current_frame = None
    self.arUcoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    self.arUcoParams = cv2.aruco.DetectorParameters_create()
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      CompressedImage,
      'video_frames',
      self.listener_callback,
      10)
    self.subscription # prevent unused variable warning

    # Create two publishers. The frame publisher will forward the subscribed frame
    # with the addition of an ArUco outline. The aruco publisher will publish infomation
    # about any detected aruco tags
    self.publisher_frame_ = self.create_publisher(CompressedImage, 'frame', 10)

    self.publisher_aruco_ = self.create_publisher(ArUcoInfo, 'aruco_tag', 10)


  def get_robot_centre(self, topLeft, bottomRight):
    '''
    Calculates the centre of the robot by taking the averages of the Tag Corners
    Returns:
    --------
    Coordinates -> tuple
      The centre positionings of the ArUco Tags and assumes robot is on a flat plane
    '''
    centreX = (topLeft[0]+bottomRight[0])/2
    centreY = (topLeft[1]+bottomRight[1])/2
    centreZ = 0
    return (centreX, centreY, centreZ)


  def listener_callback(self, data):
    """
    Callback function for subscribing to video frame feed.

    Subscription:
    -------------
    video_frames: This is a default ROS2 msg format of an image from sensor_msgs.Image

    Published:
    -----------
    aruco_tag:
      id: int
      x: float
      y: float
      z: float
      theta: float

    frame: This is video_frames with outlines drawn around aruco tags present in the frame
    """
    # Display the message on the console
    # self.get_logger().info('Receiving video frame')

    # Convert ROS Image message to OpenCV image
    self.current_frame = self.br.compressed_imgmsg_to_cv2(data)

    if self.current_frame is not None: # Fail-safe to prevent crashes if frame isn't sent correctly
      msg = ArUcoInfo()
      (tags, ids, rejected) = cv2.aruco.detectMarkers(self.current_frame,
        self.arUcoDict, parameters=self.arUcoParams)
      if (len(tags) > 0):
        cv2.aruco.drawDetectedMarkers(self.current_frame, tags, borderColor = (0, 255, 0))

        if (len(tags) > 1):
          self.get_logger().info('Only one ArUco tag can be in frame at any one time')
        else:
          ids = ids.flatten()
          for (markerCorner, markerID) in zip(tags, ids):
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            frontOfTagX = (topLeft[0] + topRight[0]) / 2
            frontOfTagY = (topLeft[1] + topRight[1]) / 2

            # Finds the Centre Point of the robot and draws onto the frame
            centreX, centreY, centreZ = self.get_robot_centre(topLeft, bottomRight)

            bearing = math.atan2(frontOfTagY - centreY, frontOfTagX - centreX)

            msg.id = int(markerID)
            msg.x  = float(centreX)
            msg.y  = float(centreY)
            msg.z  = float(centreZ)
            msg.theta = float(bearing)
            # Publish ArUco Tag information if one is present
            self.publisher_aruco_.publish(msg)
      self.publisher_frame_.publish(self.br.cv2_to_compressed_imgmsg(self.current_frame))


def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)

  # Create the node
  arUco_reader = ArUcoTagReader()

  # Spin the node so the callback function is called.
  rclpy.spin(arUco_reader)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  arUco_reader.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()

if __name__ == '__main__':
  main()