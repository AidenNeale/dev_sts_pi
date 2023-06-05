# Import the necessary libraries
import math

import cv2  # OpenCV library
import rclpy  # Python Client Library for ROS 2
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import CompressedImage  # Image is the message type
from sts_pi_interfaces.msg import ArUcoInfo


class ArUcoTagReader(Node):
  """
  Create an ArUcoTagReader class, which is a subclass of the Node class.
  The purpose of this node is to analyse a subscribed frame for detection of
  an ArUco tag.

  Subscription:
  -------------
  /video_frames: CompressedImage
    Compressed ROS2 Image

  Publisher:
  ----------
  /frame: CompressedImage
    Compressed ROS2 Image

  /aruco_tag: Custom Message ArUcoInfo
    Information about detected ArUco Tag (ID, X, Y, Z, Theta),
    publishes -1 if no tag detected
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
    # ArUco Variables
    self.arUcoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    self.arUcoParams = cv2.aruco.DetectorParameters()
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

    Parameters:
    -----------
    topLeft -> List
      A list containing the X, Y coordinates of the top left corner of the tag

    bottomRight -> List
      A list containing the X, Y coordinates of the bottom right corner of the tag
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
    Callback function for subscribing to video frame feed. This is called
    whenever a message is published to the subscribed topic.

    Subscription:
    -------------
    /video_frames: This is a default ROS2 msg format of a compressed image from sensor_msgs.msgs.CompressedImage

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

    # Convert ROS2 Image message to OpenCV image
    self.current_frame = self.br.compressed_imgmsg_to_cv2(data)

    if self.current_frame is not None: # Fail-safe to prevent crashes if frame isn't sent correctly
      msg = ArUcoInfo()
      (tags, ids, rejected) = cv2.aruco.detectMarkers(self.current_frame,
        self.arUcoDict, parameters=self.arUcoParams)
      if (len(tags) > 0): # Ensures presence of at least one tag
        if (len(tags) > 1): # Ensures presence of exactly one tag
          self.get_logger().info('Only one ArUco tag can be in frame at any one time')
        else:
          corners = tags[0].tolist()[0]
          cv2.line(self.current_frame, (int(corners[0][0]), int(corners[0][1])), (int(corners[1][0]), int(corners[1][1])), (0, 255, 0), 10, lineType=cv2.LINE_AA)
          cv2.line(self.current_frame, (int(corners[1][0]), int(corners[1][1])), (int(corners[2][0]), int(corners[2][1])), (0, 255, 0), 10, lineType=cv2.LINE_AA)
          cv2.line(self.current_frame, (int(corners[2][0]), int(corners[2][1])), (int(corners[3][0]), int(corners[3][1])), (0, 255, 0), 10, lineType=cv2.LINE_AA)
          cv2.line(self.current_frame, (int(corners[3][0]), int(corners[3][1])), (int(corners[0][0]), int(corners[0][1])), (0, 255, 0), 10, lineType=cv2.LINE_AA)

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
      else:
        # Publishes -1 as the indication of no tag
        msg.id = -1
        msg.x  = -1.0
        msg.y  = -1.0
        msg.z  = -1.0
        msg.theta = -1000.0
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
