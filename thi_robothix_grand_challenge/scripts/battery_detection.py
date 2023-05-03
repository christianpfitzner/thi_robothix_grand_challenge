import numpy as np
from math import cos, sin
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError


def rotate_vector(vector, theta):
    rot     = np.array([[cos(theta), -sin(theta)], 
                        [sin(theta), cos(theta)]])
    v = np.dot(rot, vector)

    return v


def normalize_vector(vector):
    vector_len  = np.linalg.norm(vector)
    vector_norm = vector / vector_len
    return vector_norm


def determinate_battery_orientation(p_blue, p_gray):
    v_b2g       = p_gray - p_blue
    angle_b2g    = np.arctan2(v_b2g[1], v_b2g[0]) * (-1) # (*-1) bc OpenCV uses left-handed coordinates
    v_b2g_norm  = normalize_vector(v_b2g)

    ### DEFINE BATTERY FIXED COORDINATES
    vector_y = v_b2g_norm
    vector_x = rotate_vector(vector_y, ((np.pi)/2))     # OpenCV left-handed coordinates

    ### FOR DEBUG
    print("Angle y-axis to horizontal in deg=%s" % (np.rad2deg(angle_b2g)))

    return angle_b2g, vector_x, vector_y


def find_blue_area(i_hsv):
    ### HSV FILTER: Target is blue
    low_HSV         = (95, 100, 100)                             # OpenCV:      H_min = 0       S_min = 0       V_min = 0
    high_HSV        = (115, 255, 255)                            #              H_max = 180     S_max = 255     V_max = 255
    i_hsv           = cv2.inRange(i_hsv, low_HSV, high_HSV)
    # cv2.imshow("(blue) inRange HSV", i_hsv)

    ### MORPHOLOGY OPERATIONS
    # OPENING
    shape_open      = cv2.MORPH_ELLIPSE
    ksize_open      = (3, 3)
    kernel_open     = cv2.getStructuringElement(shape_open, ksize_open)
    i_open          = cv2.morphologyEx(i_hsv, cv2.MORPH_OPEN, kernel_open)
    # cv2.imshow("(blue) after opening", i_open)
    # CLOSING
    shape_close     = cv2.MORPH_ELLIPSE
    ksize_close     = (5, 5)
    kernel_close    = cv2.getStructuringElement(shape_close, ksize_close)
    i_close         = cv2.morphologyEx(i_open, cv2.MORPH_CLOSE, kernel_close)
    # cv2.imshow("(blue) after closing", i_close)

    # ### CONNECTIONS
    # connectivity = 8        # 4 or 8 for connectivity type
    # num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(i_close, connectivity, cv2.CV_16S)
    # # print("(blue) Number of connections found= %s" % (num_labels - 1))     # background has label=0

    # # FILTER CONNECTIONS
    # output = np.zeros(i_hsv.shape, dtype="uint8")       # new image to store connections
    # counter = 0
    # target_label = -1
    # for i in range(1, num_labels):
    #     area = stats[i, cv2.CC_STAT_AREA]
        
    #     if (area > 4000) & (area < 16000):
    #         width_box       = stats[i, cv2.CC_STAT_WIDTH]
    #         height_box      = stats[i, cv2.CC_STAT_HEIGHT]
    #         square_ratio    = width_box/height_box
            
    #         ### FOR DEBUG
    #         # x = int(centroids[i, 0])
    #         # y = int(centroids[i, 1])
    #         # print("Connection=%s, [area=%s, x=%s, y=%s, square_ratio=%s]" % (i, area, x, y, square_ratio))

    #         if (square_ratio > 0.75) & (square_ratio < 1.25):
    #             componentMask = (labels == i).astype("uint8") * 255
    #             output = cv2.bitwise_or(output, componentMask)
    #             target_label = i
    #             counter += 1

    #             ### FOR DEBUG
    #             # x = int(centroids[i, 0])
    #             # y = int(centroids[i, 1])
    #             # print("Connection=%s, [area=%s, x=%s, y=%s, square_ratio=%s]" % (i, area, x, y, square_ratio))

    # # cv2.imshow("(after filters) Target is ONLY blue", output)
    # blue_found = False
    # if (counter == 1):
    #     print("Blue found, Connection filtering OK")
    #     blue_found = True
    # else:
    #     print("ERROR... (blue) Number of connections after filtering= %s" % (counter))
    
    # x_blue  = int(centroids[target_label, 0])
    # y_blue  = int(centroids[target_label, 1])
    # p_blue  = np.array([x_blue, y_blue])

    # return output, p_blue, blue_found
    return


def find_gray_area(i_hsv):
    ### HSV FILTER: Target is gray area
    low_HSV         = (90, 5, 30)                            # OpenCV:      H_min = 0       S_min = 0       V_min = 0
    high_HSV        = (145, 80, 115)                         #              H_max = 180     S_max = 255     V_max = 255
    i_hsv           = cv2.inRange(i_hsv, low_HSV, high_HSV)
    # cv2.imshow("(gray) inRange HSV", i_hsv)

    ### MORPHOLOGY OPERATIONS
    # OPENING
    shape_open      = cv2.MORPH_ELLIPSE
    ksize_open      = (3, 3)
    kernel_open     = cv2.getStructuringElement(shape_open, ksize_open)
    i_open          = cv2.morphologyEx(i_hsv, cv2.MORPH_OPEN, kernel_open)
    # cv2.imshow("(gray) after opening", i_open)
    # CLOSING
    shape_close     = cv2.MORPH_ELLIPSE
    ksize_close     = (5, 5)
    kernel_close    = cv2.getStructuringElement(shape_close, ksize_close)
    i_close         = cv2.morphologyEx(i_open, cv2.MORPH_CLOSE, kernel_close)
    # cv2.imshow("(gray) after closing", i_close)

    # ### CONNECTIONS
    # connectivity = 8        # 4 or 8 for connectivity type
    # num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(i_close, connectivity, cv2.CV_16S)
    # # print("(gray) Number of connections found= %s" % (num_labels - 1))     # background has label=0

    # # FILTER CONNECTIONS
    # output = np.zeros(i_hsv.shape, dtype="uint8")       # new image to store connections
    # counter = 0
    # target_label = -1
    # for i in range(1, num_labels):
    #     area = stats[i, cv2.CC_STAT_AREA]
        
    #     if (area > 4000) & (area < 16000):
    #         width_box       = stats[i, cv2.CC_STAT_WIDTH]
    #         height_box      = stats[i, cv2.CC_STAT_HEIGHT]
    #         square_ratio    = width_box/height_box
            
    #         ### FOR DEBUG
    #         # x = int(centroids[i, 0])
    #         # y = int(centroids[i, 1])
    #         # print("Connection=%s, [area=%s, x=%s, y=%s, square_ratio=%s]" % (i, area, x, y, square_ratio))

    #         if (square_ratio > 0.75) & (square_ratio < 1.25):
    #             componentMask = (labels == i).astype("uint8") * 255
    #             output = cv2.bitwise_or(output, componentMask)
    #             target_label = i
    #             counter += 1

    #             ### FOR DEBUG
    #             # x = int(centroids[i, 0])
    #             # y = int(centroids[i, 1])
    #             # print("Connection=%s, [area=%s, x=%s, y=%s, square_ratio=%s]" % (i, area, x, y, square_ratio))

    # # cv2.imshow("(after filters) Target is ONLY gray", output)
    # gray_found = False
    # if (counter == 1):
    #     print("Gray found, Connection filtering OK")
    #     gray_found = True
    # else:
    #     print("ERROR... (gray) Number of connections after filtering= %s" % (counter))
    
    # x_gray  = int(centroids[target_label, 0])
    # y_gray  = int(centroids[target_label, 1])
    # p_gray  = np.array([x_gray, y_gray])

    # return output, p_gray, gray_found
    return


def draw_circle(img, center, color):
    thickness = 2
    line_type = 8
    img = cv2.circle(img, (int(center[0]), int(center[1])), 10, color, thickness, line_type)

    return img


def draw_line(img, start, end, color):
    thickness = 2
    line_type = 8
    cv2.line(img, (int(start[0]), int(start[1])), (int(end[0]), int(end[1])), color, thickness, line_type)

    return img


def image_callback(img_msg):
    ### Convert ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: " + e)
    cv2.imshow("original input", cv_image)

    ### BGR to HSV conversion
    i_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    ### FIND BLUE AREA
    i_blue, p_blue, blue_found  = find_blue_area(i_hsv)

    ### FIND GRAY AREA
    i_gray, p_gray, gray_found  = find_gray_area(i_hsv)

    if(blue_found and gray_found):
        ### UNSUSCRIBE FROM IMAGE TOPIC, STOP RECEIVING FRAMES
        sub_image.unregister()

        ### DETERMINATE BATTERY ORIENTATION
        angle_bat, vector_x, vector_y   = determinate_battery_orientation(p_blue, p_gray)

        ### CALCULATE GRIP TARGET
        grip_target     = p_blue + vector_y * 10    # above the center


        ### CONVERT PIXEL TO ROBOT COORDINATES [m]



        # ### VISUALIZE RESULTS
        # i_targets_bw    = cv2.bitwise_or(i_door, i_button)
        # i_viz           = cv2.cvtColor(i_targets_bw, cv2.COLOR_GRAY2BGR)
        # # Battery axis
        # x_axis = p_button + vector_x * (45)     # to visualize results, not needed
        # y_axis = p_button + vector_y * (45)     # to visualize results, not needed
        # i_viz = draw_line(i_viz, origin, x_axis, (0, 0, 255))   # red
        # i_viz = draw_line(i_viz, origin, y_axis, (0, 255, 0))   # green
        # font = cv2.FONT_ITALIC
        # i_viz = cv2.putText(i_viz, 'x_bat', (int(x_axis[0])-15, int(x_axis[1])-15), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        # i_viz = cv2.putText(i_viz, 'y_bat', (int(y_axis[0])-15, int(y_axis[1])-15), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # # Robot Coordinates
        # i_viz = cv2.line(i_viz, (3, 3), (3, 240), (0, 0, 255), 5, 8)    # x-axis, red
        # i_viz = cv2.line(i_viz, (3, 3), (340, 3), (0, 255, 0), 5, 8)    # y-axis, green
        # i_viz = cv2.putText(i_viz, 'x', (10, 240), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
        # i_viz = cv2.putText(i_viz, 'y', (340, 25), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
        # i_viz = cv2.putText(i_viz, 'P1', (int(p_1[0])-15, int(p_1[1])-15), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # cv2.imshow("Visualization box_aprox_detection", i_viz)

        print("DONE, unsuscribed from image topic")
    
    else:
        print("TRYING AGAIN WITH ANOTHER FRAME [blue_found=%s, door_found=%s]" % (blue_found, gray_found))

    print("\n")
    cv2.waitKey(250)


# def battery_detection_callback(req):
#     # ... call image processing
#
#     # Return a response message to the main node with grip_target
#     return TriggerResponse(success=True, message="Battery detection done")


# Initialize ROS Node named 'battery_detection'
rospy.init_node('battery_detection')

# Initialize the CvBridge class
bridge = CvBridge()



# # Create a service to handle incoming requests from the main node
# battery_detection_service = rospy.Service('battery_detection_trigger', Trigger, battery_detection_callback)



# Initialize a subscriber to the "/camera/color/image_raw" topic
sub_image = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
     rospy.spin()