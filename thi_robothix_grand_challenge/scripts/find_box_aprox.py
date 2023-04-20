import numpy as np
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image
# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Initialize ROS Node
rospy.init_node('arrow_detection_node')

# Initialize the CvBridge class
bridge = CvBridge()

# Callback for the Image message
def image_callback(img_msg):
    # Try to convert ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: " + e)


    ### APLY FILTERS TO TARGET BOX
    # BGR to HSV conversion
    i_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    ### HSV FILTER: Target is box
    low_HSV     = (0, 0, 0)                                # Note, opencv HSVmin = 0
    high_HSV    = (180, 50, 50)                            # Hmax = 180, SVmax = 255
    i_hsv       = cv2.inRange(i_hsv, low_HSV, high_HSV)
    cv2.imshow("inRange whith target box", i_hsv)


    ### MORPHOLOGY OPERATIONS
    # OPENING
    shape_open = cv2.MORPH_RECT
    ksize_open = (3, 3)
    kernel_open = cv2.getStructuringElement(shape_open, ksize_open)
    i_open = cv2.morphologyEx(i_hsv, cv2.MORPH_OPEN, kernel_open)
    cv2.imshow("after opening", i_open)
    # CLOSING
    shape_close = cv2.MORPH_RECT
    ksize_close = (5, 5)
    kernel_close = cv2.getStructuringElement(shape_close, ksize_close)
    i_close = cv2.morphologyEx(i_open, cv2.MORPH_CLOSE, kernel_close)
    cv2.imshow("after closing", i_close)


    cv2.waitKey(250)
    #cv2.destroyAllWindows()

# Initialize a subscriber to the "/camera/color/image_raw" toic with the function "image_callback" as a callback
sub_image = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
     rospy.spin()