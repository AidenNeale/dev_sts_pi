# Basic ROS 2 program to subscribe to real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
  
# Import the necessary libraries
import math
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import Float32MultiArray
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
    self.arUcoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
    self.arUcoParams = cv2.aruco.DetectorParameters_create()
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    self.coordinates = ()




    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      'video_frames', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning


    # Create the publisher. This publisher will publish 
    self.publisher_ = self.create_publisher(Float32MultiArray, 'aruco_tag', 10)
    
    # We will publish a message every 0.01 seconds
    timer_period = 0.05  # seconds
    
    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)


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
    """
    # Display the message on the console
    # self.get_logger().info('Receiving video frame')
 
    # Convert ROS Image message to OpenCV image
    self.current_frame = self.br.imgmsg_to_cv2(data)

  
  def timer_callback(self):
    """
    Callback function for publishing ArUco Tag information.
    This function gets called every 0.01 seconds.
    """
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    if self.current_frame is not None:
      (tags, ids, rejected) = cv2.aruco.detectMarkers(self.current_frame,
        self.arUcoDict, parameters=self.arUcoParams)
      if (len(tags) > 0):
        ids = ids.flatten()
        for (markerCorner, markerID) in zip(tags, ids):
          corners = markerCorner.reshape((4, 2))
          (topLeft, topRight, bottomRight, bottomLeft) = corners

          frontOfTagX = (topLeft[0] + topRight[0]) / 2
          frontOfTagY = (topLeft[1] + topRight[1]) / 2

          # Finds the Centre Point of the robot and draws onto the frame
          centreX, centreY, centreZ = self.get_robot_centre(topLeft, bottomRight)

          bearing = math.atan2(frontOfTagY - centreY, frontOfTagX - centreX)

          self.coordinates = [centreX, centreY, centreZ, bearing]

        # Publish ArUco Tag information if one is present
        self.publisher_.publish(self.coordinates)
 
    # Display the message on the console
    # self.get_logger().info('Publishing video frame')
   



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