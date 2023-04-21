import numpy as np
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image
# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Initialize ROS Node named 'opencv_example', allow multiple nodes to be run with this name
rospy.init_node('arrow_detection_node')

# Initialize the CvBridge class
bridge = CvBridge()

def determinate_box_orientation(x_boton, y_boton, x_door, y_door):
    
    p_boton = np.array([x_boton, y_boton])
    p_door = np.array([x_door, y_door])
    v_b2d = p_door - p_boton


    angle_bd = np.arctan2(v_b2d[1], v_b2d[0]) * (-1) * 180 / np.pi       # (-1) bc KoSy gedreht
    angle_box = angle_bd - 11.0

    print("Box Angle in deg= %s" % (angle_box))


def find_blue_botton(i_hsv):
    ### HSV FILTER: Target is lue botton
    low_HSV     = (95, 150, 125)                             # Note, opencv HSVmin = 0
    high_HSV    = (115, 255, 255)                            # Hmax = 180, SVmax = 255
    i_hsv_blue       = cv2.inRange(i_hsv, low_HSV, high_HSV)
    #cv2.imshow("inRange for blue botton", i_hsv_blue)


    ### MORPHOLOGY OPERATIONS
    # OPENING
    shape_open = cv2.MORPH_ELLIPSE
    ksize_open = (5, 5)
    kernel_open = cv2.getStructuringElement(shape_open, ksize_open)
    i_open_blue = cv2.morphologyEx(i_hsv_blue, cv2.MORPH_OPEN, kernel_open)
    #cv2.imshow("after opening", i_open_blue)
    # CLOSING
    shape_close = cv2.MORPH_ELLIPSE
    ksize_close = (5, 5)
    kernel_close = cv2.getStructuringElement(shape_close, ksize_close)
    i_close_blue = cv2.morphologyEx(i_open_blue, cv2.MORPH_CLOSE, kernel_close)
    #cv2.imshow("after closing", i_close_blue)

    ### CONNECTIONS
    connectivity = 4 #choose between 4 or 8 for connectivity type
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(i_close_blue, connectivity, cv2.CV_16S)
    #print("Number of connections found= %s" % (num_labels - 1))     # background has label=0
    # Initialize a new image to store connections after filtering
    output = np.zeros(i_hsv_blue.shape, dtype="uint8")

    # FILTER CONNECTIONS
    counter = 0
    target_label = -1
    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        # x = round(centroids[i, 0])
        # y = round(centroids[i, 1])
        # width_box = stats[i, cv2.CC_STAT_WIDTH]
        # height_box = stats[i, cv2.CC_STAT_HEIGHT]
        #print("Connection=%s, [area=%s, x=%s, y=%s]" % (i, area, x, y))

        # FILTER
        if (area > 25) and (area < 400):
            componentMask = (labels == i).astype("uint8") * 255
            output = cv2.bitwise_or(output, componentMask)
            target_label = i
            counter += 1
    x = round(centroids[target_label, 0])
    y = round(centroids[target_label, 1])

    cv2.imshow("Blue botton", output)
    if (counter == 1):
        print("Blue botton found, Connection filtering OK")
    else:
        print("ERROR...(blue botton) Number of connections after filtering= %s\n\n\n" % (counter))

    return output, x, y


def find_door(i_hsv):
    ### HSV FILTER: Target is lue botton
    low_HSV     = (95, 5, 60)                             # Note, opencv HSVmin = 0
    high_HSV    = (135, 70, 140)                            # Hmax = 180, SVmax = 255
    i_hsv_door       = cv2.inRange(i_hsv, low_HSV, high_HSV)
    cv2.imshow("inRange for door", i_hsv_door)


    ### MORPHOLOGY OPERATIONS
    # OPENING
    shape_open = cv2.MORPH_ELLIPSE
    ksize_open = (5, 5)
    kernel_open = cv2.getStructuringElement(shape_open, ksize_open)
    i_open_door = cv2.morphologyEx(i_hsv_door, cv2.MORPH_OPEN, kernel_open)
    cv2.imshow("after opening", i_open_door)
    # CLOSING
    shape_close = cv2.MORPH_ELLIPSE
    ksize_close = (9, 9)
    kernel_close = cv2.getStructuringElement(shape_close, ksize_close)
    i_close_door = cv2.morphologyEx(i_open_door, cv2.MORPH_CLOSE, kernel_close)
    cv2.imshow("after closing", i_close_door)

    ### CONNECTIONS
    connectivity = 4 #choose between 4 or 8 for connectivity type
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(i_close_door, connectivity, cv2.CV_16S)
    print("Number of connections found= %s" % (num_labels - 1))     # background has label=0
    # Initialize a new image to store connections after filtering
    output = np.zeros(i_hsv_door.shape, dtype="uint8")

    # FILTER CONNECTIONS
    counter = 0
    target_label = -1
    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        x = round(centroids[i, 0])
        y = round(centroids[i, 1])
        width_box = stats[i, cv2.CC_STAT_WIDTH]
        height_box = stats[i, cv2.CC_STAT_HEIGHT]
        print("Connection=%s, [area=%s, x=%s, y=%s]" % (i, area, x, y))
        
        # FILTER
        if (area > 5000) & (area < 15000):

            width_box = stats[i, cv2.CC_STAT_WIDTH]
            height_box = stats[i, cv2.CC_STAT_HEIGHT]
            square_ratio = width_box/height_box

            if (square_ratio > 0.85) & (square_ratio < 1.15):
                print("After filter: Connection=%s, [area=%s, x=%s, y=%s, width= %s, heigth= %s]" % (i, area, x, y, width_box, height_box))
                print(square_ratio)
                componentMask = (labels == i).astype("uint8") * 255
                output = cv2.bitwise_or(output, componentMask)
                target_label = i
                counter += 1
    x = round(centroids[target_label, 0])
    y = round(centroids[target_label, 1])

    cv2.imshow("Door", output)

    if (counter == 1):
        print("Door found, Connection filtering OK")
    else:
        print("ERROR... (door) Number of connections after filtering= %s\n\n\n" % (counter))
    
    return output, x, y



# Callback for the Image message
def image_callback(img_msg):
    # Try to convert ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: " + e)
    
    cv2.imshow("original input", cv_image)

    # BGR to HSV conversion
    i_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)


    ### FIND BLUE BOTTON
    i_blue_botton, x_blue_botton, y_blue_botton = find_blue_botton(i_hsv)     ### WARNING: proof if error with display on.. if yes filter connections looking for circles


    ### FIND DOOR
    i_door, x_door, y_door = find_door(i_hsv)


    ### DETERMINATE BOX ORIENTATION
    determinate_box_orientation(x_blue_botton, y_blue_botton, x_door, y_door)


    i_or_targets = cv2.bitwise_or(i_door, i_blue_botton)
    cv2.imshow("targets: blue botton and door", i_or_targets)

    print("\n")


    cv2.waitKey(500)

# Initialize a subscriber to the "/camera/color/image_raw" toic with the function "image_callback" as a callback
sub_image = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
     rospy.spin()