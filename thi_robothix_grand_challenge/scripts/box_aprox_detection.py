###################################################################################
### CONDITIONS FOR SCRIPT TO WORK:                                              ###
# ! lights off on top of robot to avoid saturation                              ###
# ! white background on table (or another color that is not ~blue or ~gray)     ###
# ! fixed X,Y,Z for the camera (area filters depend on height)                  ###
# ! camera-side of EE heading in direction of world +x-axis                     ###
###################################################################################

###################################################################################
### TBD:                                                                        ###
# + set a fixed camera Pose                                                     ###
# + trigger method for image processing ON                                      ###
# + improve offset (in f determinate_box_orientation(...))                      ###
# + check HSV values (in f find_blue_button(...) and find_door(...))            ###
# + check area filters (in f find_blue_button(...) and find_door(...))          ###
# + conversion method from pixel_robot_coordinates to robot coordinates [m]     ###
# + pass robot coordinates [m] to robot (plan & execute... Kraft-sensitiv)      ###
# + trigger method for DONE & CONTINUE                                          ###
###################################################################################

import numpy as np
from math import cos, sin
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image
# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError


# # import package for point cloud 
# from sensor_msgs.msg import PointCloud2
# import sensor_msgs.point_cloud2 as pc2



# g_point_cloud = None

# # add subscriber for point cloud
# def cloud_callback(data):
#     global g_point_cloud
    
#     g_point_cloud = data



def rotate_vector(vector, theta):
    rot     = np.array([[cos(theta), -sin(theta)], 
                        [sin(theta), cos(theta)]])
    v = np.dot(rot, vector)

    return v


def normalize_vector(vector):
    vector_len  = np.linalg.norm(vector)
    vector_norm = vector / vector_len
    return vector_norm


def get_P1_and_P1goal(p_button, vector_x, vector_y):
    p_1         = p_button + vector_y * (220)
    p_1         = p_1 + vector_x * (-210)
    p_1goal     = p_1 + vector_x * (100)

    return p_1, p_1goal, vector_x


def get_P2_and_P2goal(p_button, vector_x, vector_y):
    p_2         = p_button + vector_y * (20)
    p_2         = p_2 + vector_x * (-210)
    p_2goal     = p_2 + vector_x * (100)

    return p_2, p_2goal, vector_x


def get_P3_and_P3goal(p_button, vector_x, vector_y):
    p_3         = p_button + vector_x * (-20)
    p_3         = p_3 + vector_y * (-90)
    p_3goal     = p_3 + vector_y * (100)

    return p_3, p_3goal, vector_y


def determinate_box_orientation(p_button, p_door):
    v_b2d       = p_door - p_button
    angle_bd    = np.arctan2(v_b2d[1], v_b2d[0]) * (-1) # (*-1) bc OpenCV uses left-handed coordinates
    v_b2d_norm  = normalize_vector(v_b2d)

    #######################
    ### TBD: improve offset
    #######################
    offset_box  = np.deg2rad(11)                        # angle between v_b2d & y-axis from box
    angle_box   = angle_bd - offset_box

    ### DEFINE BOX FIXED COORDINATES
    vector_y = rotate_vector(v_b2d_norm, offset_box)
    vector_x = rotate_vector(vector_y, ((np.pi)/2))     # OpenCV left-handed coordinates

    ### FOR DEBUG
    # print("Angle BD to horizontal in deg=%s" % (np.rad2deg(angle_bd)))
    # print("Offset to Box in deg=%s" % (np.rad2deg(offset_box)))
    print("Angle y_box to horizontal in deg=%s" % (np.rad2deg(angle_box)))

    return angle_box, vector_x, vector_y


def find_blue_button(i_hsv):
    ### HSV FILTER: Target is lue button
    low_HSV         = (95, 100, 100)                             # OpenCV:      H_min = 0       S_min = 0       V_min = 0
    high_HSV        = (115, 255, 255)                            #              H_max = 180     S_max = 255     V_max = 255
    i_hsv           = cv2.inRange(i_hsv, low_HSV, high_HSV)
    # cv2.imshow("(blue button) inRange HSV", i_hsv)

    ### MORPHOLOGY OPERATIONS
    # OPENING
    shape_open      = cv2.MORPH_ELLIPSE
    ksize_open      = (3, 3)
    kernel_open     = cv2.getStructuringElement(shape_open, ksize_open)
    i_open          = cv2.morphologyEx(i_hsv, cv2.MORPH_OPEN, kernel_open)
    # cv2.imshow("(blue button) after opening", i_open)
    # CLOSING
    shape_close     = cv2.MORPH_ELLIPSE
    ksize_close     = (5, 5)
    kernel_close    = cv2.getStructuringElement(shape_close, ksize_close)
    i_close         = cv2.morphologyEx(i_open, cv2.MORPH_CLOSE, kernel_close)
    # cv2.imshow("(blue button) after closing", i_close)

    ### CONNECTIONS
    connectivity = 8        # 4 or 8 for connectivity type
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(i_close, connectivity, cv2.CV_16S)
    # print("(blue button) Number of connections found= %s" % (num_labels - 1))     # background has label=0
    
    # FILTER CONNECTIONS
    output = np.zeros(i_hsv.shape, dtype="uint8")       # new image to store connections
    counter = 0
    target_label = -1
    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]

        ### FOR DEBUG
        # x = int(centroids[i, 0])
        # y = int(centroids[i, 1])
        # print("Connection=%s, [area=%s, x=%s, y=%s]" % (i, area, x, y))

        # FILTER
        if (area > 40) and (area < 400):

            ### FOR DEBUG
            # x = int(centroids[i, 0])
            # y = int(centroids[i, 1])
            # print("Connection=%s, [area=%s, x=%s, y=%s]" % (i, area, x, y))

            componentMask = (labels == i).astype("uint8") * 255
            output = cv2.bitwise_or(output, componentMask)
            target_label = i
            counter += 1

    # cv2.imshow("(after filters) Target is ONLY blue button", output)
    button_found = False
    if (counter == 1):
        print("Blue button found, Connection filtering OK")
        button_found = True
    else:
        print("ERROR... (blue button) Number of connections after filtering= %s" % (counter))

    x_button = int(centroids[target_label, 0])
    y_button = int(centroids[target_label, 1])
    p_button    = np.array([x_button, y_button])

    return output, p_button, button_found


def find_door(i_hsv):
    ### HSV FILTER: Target is door
    low_HSV         = (90, 5, 30)                            # OpenCV:      H_min = 0       S_min = 0       V_min = 0
    high_HSV        = (145, 80, 115)                         #              H_max = 180     S_max = 255     V_max = 255
    i_hsv           = cv2.inRange(i_hsv, low_HSV, high_HSV)
    # cv2.imshow("(door) inRange HSV", i_hsv)

    ### MORPHOLOGY OPERATIONS
    # OPENING
    shape_open      = cv2.MORPH_ELLIPSE
    ksize_open      = (3, 3)
    kernel_open     = cv2.getStructuringElement(shape_open, ksize_open)
    i_open          = cv2.morphologyEx(i_hsv, cv2.MORPH_OPEN, kernel_open)
    # cv2.imshow("(door) after opening", i_open)
    # CLOSING
    shape_close     = cv2.MORPH_ELLIPSE
    ksize_close     = (5, 5)
    kernel_close    = cv2.getStructuringElement(shape_close, ksize_close)
    i_close         = cv2.morphologyEx(i_open, cv2.MORPH_CLOSE, kernel_close)
    # cv2.imshow("(door) after closing", i_close)

    ### CONNECTIONS
    connectivity = 8        # 4 or 8 for connectivity type
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(i_close, connectivity, cv2.CV_16S)
    # print("(door) Number of connections found= %s" % (num_labels - 1))     # background has label=0

    # FILTER CONNECTIONS
    output = np.zeros(i_hsv.shape, dtype="uint8")       # new image to store connections
    counter = 0
    target_label = -1
    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        
        if (area > 4000) & (area < 16000):
            width_box       = stats[i, cv2.CC_STAT_WIDTH]
            height_box      = stats[i, cv2.CC_STAT_HEIGHT]
            square_ratio    = width_box/height_box
            
            ### FOR DEBUG
            # x = int(centroids[i, 0])
            # y = int(centroids[i, 1])
            # print("Connection=%s, [area=%s, x=%s, y=%s, square_ratio=%s]" % (i, area, x, y, square_ratio))

            if (square_ratio > 0.75) & (square_ratio < 1.25):
                componentMask = (labels == i).astype("uint8") * 255
                output = cv2.bitwise_or(output, componentMask)
                target_label = i
                counter += 1

                ### FOR DEBUG
                # x = int(centroids[i, 0])
                # y = int(centroids[i, 1])
                # print("Connection=%s, [area=%s, x=%s, y=%s, square_ratio=%s]" % (i, area, x, y, square_ratio))

    # cv2.imshow("(after filters) Target is ONLY door", output)
    door_found = False
    if (counter == 1):
        print("Door found, Connection filtering OK")
        door_found = True
    else:
        print("ERROR... (door) Number of connections after filtering= %s" % (counter))
    
    x_door = int(centroids[target_label, 0])
    y_door = int(centroids[target_label, 1])
    p_door  = np.array([x_door, y_door])

    return output, p_door, door_found


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



# get_3d_coordinate function definition 
# def pixelTo3DPoint(cloud, u, v):
#     data_np = ros_numpy.numpify(cloud)



def cartesian_from_pixel(pixel, focal, dist, c_x, c_y):
    x = (pixel[0] - c_x) * dist  / focal
    y = (pixel[1] - c_y) * dist  / focal

    return x, y

def image_callback(img_msg):
    ### Convert ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: " + e)
    cv2.imshow("original input", cv_image)

    ### BGR to HSV conversion
    i_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    ### FIND BLUE BUTTON
    i_button, p_button, button_found    = find_blue_button(i_hsv)

    ### FIND DOOR
    i_door, p_door, door_found          = find_door(i_hsv)

    if(button_found and door_found):
        ### UNSUSCRIBE FROM IMAGE TOPIC, STOP RECEIVING FRAMES
        sub_image.unregister()

        ### DETERMINATE BOX ORIENTATION & BOX FIXED COORDINATES, blue button is origin
        angle_box, vector_x, vector_y   = determinate_box_orientation(p_button, p_door)
        origin = p_button

        ### GET P1 --> P1goal
        p_1, p_1goal, v_1 = get_P1_and_P1goal(p_button, vector_x, vector_y)

        ### GET P2 --> P2goal
        p_2, p_2goal, v_2 = get_P2_and_P2goal(p_button, vector_x, vector_y)

        ### GET P3 --> P3goal
        p_3, p_3goal, v_3 = get_P3_and_P3goal(p_button, vector_x, vector_y)




        ### from u[pixel] & v[pixel] to x_camera[m] & y_camera[m]
        c_x  = 327.1772766113281
        c_y  = 243.1417541503906
        f    = 578.52294921875
        dist = 0.4

        # get the 3d coordinate of p1 based on the g_point_cloud
        q_1     = cartesian_from_pixel(p_1,     f, dist, c_x, c_y)
        q_1goal = cartesian_from_pixel(p_1goal, f, dist, c_x, c_y)
        q_2     = cartesian_from_pixel(p_2,     f, dist, c_x, c_y)
        q_2goal = cartesian_from_pixel(p_2goal, f, dist, c_x, c_y)
        q_3     = cartesian_from_pixel(p_3,     f, dist, c_x, c_y)
        q_3goal = cartesian_from_pixel(p_3goal, f, dist, c_x, c_y)

        ### FOR DEBUG
        q_button = cartesian_from_pixel(p_button, f, dist, c_x, c_y)
        print("q_button: ", q_button)

        # print("q_1: ", q_1)
        # print("q_1goal: ", q_1goal)
        # print("q_2: ", q_2)
        # print("q_2goal: ", q_2goal)
        # print("q_3: ", q_3)
        # print("q_3goal: ", q_3goal)


        ### VISUALIZE RESULTS
        # i_targets_bw    = cv2.bitwise_or(i_door, i_button)
        # i_viz           = cv2.cvtColor(i_targets_bw, cv2.COLOR_GRAY2BGR)
        i_viz  = cv_image
        # axis
        x_axis = p_button + vector_x * (45)     # to visualize results, not needed
        y_axis = p_button + vector_y * (45)     # to visualize results, not needed
        i_viz = draw_line(i_viz, origin, x_axis, (0, 0, 255))   # red
        i_viz = draw_line(i_viz, origin, y_axis, (0, 255, 0))   # green
        font = cv2.FONT_ITALIC
        i_viz = cv2.putText(i_viz, 'x_box', (int(x_axis[0])-15, int(x_axis[1])-15), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        i_viz = cv2.putText(i_viz, 'y_box', (int(y_axis[0])-15, int(y_axis[1])-15), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        # blue button and door
        i_viz = draw_circle(i_viz, origin, (255, 0, 0))         # blue
        i_viz = draw_circle(i_viz, p_door, (128, 128, 128))     # gray
        # P1 --> P1goal
        i_viz = draw_circle(i_viz, p_1, (0, 255, 255))          # yellow
        i_viz = draw_circle(i_viz, p_1goal, (0, 165, 255))      # orange
        i_viz = draw_line(i_viz, p_1, p_1goal, (0, 75, 150))    # brown
        # P2 --> P2goal
        i_viz = draw_circle(i_viz, p_2, (0, 255, 255))          # yellow
        i_viz = draw_circle(i_viz, p_2goal, (0, 165, 255))      # orange
        i_viz = draw_line(i_viz, p_2, p_2goal, (0, 75, 150))    # brown
        # P3 --> P3goal
        i_viz = draw_circle(i_viz, p_3, (0, 255, 255))          # yellow
        i_viz = draw_circle(i_viz, p_3goal, (0, 165, 255))      # orange
        i_viz = draw_line(i_viz, p_3, p_3goal, (0, 75, 150))    # brown
        # Robot Coordinates
        i_viz = cv2.line(i_viz, (3, 3), (3, 240), (0, 0, 255), 5, 8)    # x-axis, red
        i_viz = cv2.line(i_viz, (3, 3), (340, 3), (0, 255, 0), 5, 8)    # y-axis, green
        i_viz = cv2.putText(i_viz, 'x_robot', (10, 240), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
        i_viz = cv2.putText(i_viz, 'y_robot', (340, 25), font, 1, (255, 255, 255), 1, cv2.LINE_AA)
        i_viz = cv2.putText(i_viz, 'P1', (int(p_1[0])-15, int(p_1[1])-15), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        i_viz = cv2.putText(i_viz, 'P2', (int(p_2[0])-15, int(p_2[1])-15), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        i_viz = cv2.putText(i_viz, 'P3', (int(p_3[0])-15, int(p_3[1])-15), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        i_viz = cv2.putText(i_viz, 'P1_goal', (int(p_1goal[0])-15, int(p_1goal[1])-15), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        i_viz = cv2.putText(i_viz, 'P2_goal', (int(p_2goal[0])-15, int(p_2goal[1])-15), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        i_viz = cv2.putText(i_viz, 'P3_goal', (int(p_3goal[0])-15, int(p_3goal[1])-15), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow("Visualization box_aprox_detection", i_viz)

        print("DONE, unsuscribed from image topic")
    
    else:
        print("TRYING AGAIN WITH ANOTHER FRAME [button_found=%s, door_found=%s]" % (button_found, door_found))

    print("\n")
    cv2.waitKey(250)


# Initialize ROS Node named 'box_aprox_detection’
rospy.init_node('box_aprox_detection')

# Initialize the CvBridge class
bridge = CvBridge()

# Initialize a subscriber to the "/camera/color/image_raw" topic
sub_image = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

# cloud_sub = rospy.Subscriber("/camera/depth_registered/points", PointCloud2, cloud_callback)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
     rospy.spin()