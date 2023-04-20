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

# Callback for the Image message
def image_callback(img_msg):
    # Try to convert ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: " + e)
    


    ### HIDE NOT INTERESTING PARTS (by drawing black rectangles on top and right side)
    # res_img = cv2.rectangle(cv_image, (0, 0), (639, 160), (0, 0, 0), -1)
    # res_img = cv2.rectangle(res_img, (540, 0), (639, 479), (0, 0, 0), -1)
    # cv2.imshow("paint black out of target", res_img)



    ### APLY FILTERS TO TARGET RED CASE (also white spots bc of lights) AND DISPLAY
    # BGR to HSV conversion
    i_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    ### HSV FILTER: Target is display bright
    low_HSV     = (60, 180, 220)                             # Note, opencv HSVmin = 0
    high_HSV    = (100, 240, 255)                            # Hmax = 180, SVmax = 255
    i_hsv       = cv2.inRange(i_hsv, low_HSV, high_HSV)
    cv2.imshow("inRange for display bright", i_hsv)


    ### MORPHOLOGY OPERATIONS
    # OPENING
    shape_open = cv2.MORPH_RECT
    ksize_open = (3, 3)
    kernel_open = cv2.getStructuringElement(shape_open, ksize_open)
    i_open = cv2.morphologyEx(i_hsv, cv2.MORPH_OPEN, kernel_open)
    cv2.imshow("after opening", i_open)
    # CLOSING
    shape_close = cv2.MORPH_RECT
    ksize_close = (17, 17)
    kernel_close = cv2.getStructuringElement(shape_close, ksize_close)
    i_close = cv2.morphologyEx(i_open, cv2.MORPH_CLOSE, kernel_close)
    cv2.imshow("after closing", i_close)



    ### CONNECTIONS
    connectivity = 4 #choose between 4 or 8 for connectivity type
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(i_close, connectivity, cv2.CV_16S)
    print("Number of connections found= %s" % (num_labels - 1))     # background has label=0
    # Initialize a new image to store connections after filtering
    output = np.zeros(i_close.shape, dtype="uint8")

    # FILTER CONNECTIONS
    counter = 0
    target_label = -1
    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        x = round(centroids[i, 0])
        y = round(centroids[i, 1])
        # width_box = stats[i, cv2.CC_STAT_WIDTH]
        # height_box = stats[i, cv2.CC_STAT_HEIGHT]
        print("Connection=%s, [area=%s, x=%s, y=%s]" % (i, area, x, y))

        # FILTER
        if (area > 3000) and (area < 20000):
            componentMask = (labels == i).astype("uint8") * 255
            output = cv2.bitwise_or(output, componentMask)
            target_label = i
            counter += 1
    
    cv2.imshow("Connections after filter.. target is bottom bright part of display\n", output)
    if (counter == 1):
        print("Connection filtering OK, label= %s" % (target_label))
    else:
        print("ERROR... Number of connections after filtering= %s\n\n\n" % (counter))

    ### CALCULATE display width and x_coordinate from start (will be needed to calculate offset from slider to second arrow)
    x_display_left  = stats[target_label, cv2.CC_STAT_LEFT]
    y_display_top   = stats[target_label, cv2.CC_STAT_TOP]
    display_width   = stats[target_label, cv2.CC_STAT_WIDTH]
    display_height  = stats[target_label, cv2.CC_STAT_HEIGHT]
    print("x_display_left= %s, y_display_top= %s, display_width= %s, display_height= %s" % (x_display_left, y_display_top, display_width, display_height))


    ### FOUND x_coordinate from target arrow
    i_black = np.zeros(output.shape, dtype="uint8")
    x1 = x_display_left
    y1 = y_display_top
    x2 = x1 + display_width
    y2 = y1 + display_height
    i_rectangle = cv2.rectangle(i_black, (x1, y1), (x2, y2), 255, -1)
    cv2.imshow("white rectangle at display bright bottom", i_rectangle)

    ### WE KNOW THE FIRST ARROW IS LOCATED IN THE CENTRE... since the second arrow can be close the first one we need to eliminate de first one
    x_centre    = x_display_left + (display_width // 2)
    y_arrow_top = y_display_top - 10

    # draw a triangle
    i_black = np.zeros(output.shape, dtype="uint8")
    pt1 = (x_centre, y_arrow_top)
    pt2 = (x_centre-25, y_arrow_top+45)
    pt3 = (x_centre+25, y_arrow_top+45)
    triangle_cnt = np.array([pt1, pt2, pt3 ])
    i_triangle = cv2.drawContours(i_black, [triangle_cnt], 0, 255, -1)
    cv2.imshow("i_triangle", i_triangle)

    ### EROSION RECTANGLE TO HAVE SOME ROTATION TOLERANCE
    shape_erosion = cv2.MORPH_RECT
    ksize_erosion = (5, 5)
    kernel_erosion = cv2.getStructuringElement(shape_erosion, ksize_close)
    i_rect_erosion = cv2.morphologyEx(i_rectangle, cv2.MORPH_ERODE, kernel_erosion)
    cv2.imshow("rectangle_after_erosion", i_rect_erosion)

    i_mask_2nd = cv2.bitwise_and(i_rect_erosion, cv2.bitwise_not(i_triangle))
    cv2.imshow("i_mask_2nd", i_mask_2nd)

    first = cv2.bitwise_and(i_mask_2nd, cv2.bitwise_not(i_hsv))
    cv2.imshow("first", first)




    ####### LOCALIZE pixel coordinates of second arrow
    ### CONNECTIONS_2
    connectivity = 4 #choose between 4 or 8 for connectivity type
    num_labels_2, labels_2, stats_2, centroids_2 = cv2.connectedComponentsWithStats(first, connectivity, cv2.CV_16S)
    print("Number of connections_2 found for 2nd arrow= %s" % (num_labels_2 - 1))     # background has label=0
    # Initialize a new image to store connections after filtering
    output_2 = np.zeros(first.shape, dtype="uint8")

    # FILTER CONNECTIONS
    counter_2 = 0
    target_label_2 = -1
    for j in range(1, num_labels_2):
        area_2 = stats_2[j, cv2.CC_STAT_AREA]
        x_2 = round(centroids_2[j, 0])
        y_2 = round(centroids_2[j, 1])
        print("Connection_2=%s, [area_2=%s, x_2=%s, y_2=%s]" % (j, area_2, x_2, y_2))

        # FILTER
        if (area_2 > 50) and (area_2 < 8000):
            componentMask_2 = (labels_2 == j).astype("uint8") * 255
            output_2 = cv2.bitwise_or(output_2, componentMask_2)
            target_label_2 = j
            counter_2 += 1
    
    cv2.imshow("Connections_2 after filter.. target second arrow", output_2)
    if (counter_2 == 1):
        print("Connection_2 filtering OK, label_2= %s" % (target_label_2))
    else:
        print("ERROR... Number of connections_2 after filtering= %s\n\n\n" % (counter_2))


    ### SECOND ARROW COORDINATES
    x_second_arrow = centroids_2[target_label_2, 0]
    distance_to_x_display_left = x_second_arrow - x_display_left
    porcentage_from_slider = distance_to_x_display_left / display_width
    print("porcentage_from_slider= %s\n\n" % (porcentage_from_slider))



    cv2.waitKey(250)
    #cv2.destroyAllWindows()

# Initialize a subscriber to the "/camera/color/image_raw" toic with the function "image_callback" as a callback
sub_image = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
     rospy.spin()