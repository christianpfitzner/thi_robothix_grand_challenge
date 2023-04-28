import numpy as np
from math import cos, sin
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image
# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError


def create_masks_for_digit(img, origin, y_down, offset_x_origins, digits_height):
    ### MASKS FOR DOT AND 7-SEGMENTS
    # DOT from origin i
    x_dot         = int(origin)
    y_dot_start   = int(y_down + 8)
    y_dot_end     = int(y_down - 4)
    mask_dot      = np.zeros(img.shape, dtype="uint8")     # new image to store mask
    mask_dot      = cv2.line(mask_dot, (x_dot, y_dot_start), (x_dot, y_dot_end), 50, 2, 8)

    # SEGMENT a from origin i
    x_a         = int(origin + 0.6 * offset_x_origins)
    y_a_start   = int(y_down - digits_height + 8)
    y_a_end     = int(y_down - digits_height - 4)
    mask_a      = np.zeros(img.shape, dtype="uint8")       # new image to store mask
    mask_a      = cv2.line(mask_a, (x_a, y_a_start), (x_a, y_a_end), 50, 2, 8)

    # SEGMENT b from origin i
    x_b_start   = int(origin + 0.7 * offset_x_origins)
    x_b_end     = int(origin + 1.0 * offset_x_origins)
    y_b         = int(y_down - 0.75 * digits_height)
    mask_b      = np.zeros(img.shape, dtype="uint8")       # new image to store mask
    mask_b      = cv2.line(mask_b, (x_b_start, y_b), (x_b_end, y_b), 50, 2, 8)

    # SEGMENT c from origin i
    x_c_start   = int(origin + 0.6 * offset_x_origins)
    x_c_end     = int(origin + 0.9 * offset_x_origins)
    y_c         = int(y_down - 0.25 * digits_height)
    mask_c      = np.zeros(img.shape, dtype="uint8")       # new image to store mask
    mask_c      = cv2.line(mask_c, (x_c_start, y_c), (x_c_end, y_c), 50, 2, 8)

    # SEGMENT d from origin i
    x_d         = int(origin + 0.5 * offset_x_origins)
    y_d_start   = int(y_down + 8)
    y_d_end     = int(y_down - 8)
    mask_d      = np.zeros(img.shape, dtype="uint8")       # new image to store mask
    mask_d      = cv2.line(mask_d, (x_d, y_d_start), (x_d, y_d_end), 50, 2, 8)

    # SEGMENT e from origin i
    x_e_start   = int(origin + 0.1 * offset_x_origins)
    x_e_end     = int(origin + 0.4 * offset_x_origins)
    y_e         = int(y_down - 0.25 * digits_height)
    mask_e      = np.zeros(img.shape, dtype="uint8")       # new image to store mask
    mask_e      = cv2.line(mask_e, (x_e_start, y_e), (x_e_end, y_e), 50, 2, 8)

    # SEGMENT f from origin i
    x_f_start   = int(origin + 0.2 * offset_x_origins)
    x_f_end     = int(origin + 0.5 * offset_x_origins)
    y_f         = int(y_down - 0.75 * digits_height)
    mask_f      = np.zeros(img.shape, dtype="uint8")       # new image to store mask
    mask_f      = cv2.line(mask_f, (x_f_start, y_f), (x_f_end, y_f), 50, 2, 8)

    # SEGMENT g from origin i
    x_g         = int(origin + 0.55 * offset_x_origins)
    y_g_start   = int(y_down - digits_height/2 + 8)
    y_g_end     = int(y_down - digits_height/2 - 8)
    mask_g      = np.zeros(img.shape, dtype="uint8")       # new image to store mask
    mask_g      = cv2.line(mask_g, (x_g, y_g_start), (x_g, y_g_end), 50, 2, 8)

    
    ### FOR DEBUG
    # masks_sum = cv2.bitwise_or(mask_dot, mask_a)
    # masks_sum = cv2.bitwise_or(masks_sum, mask_b)
    # masks_sum = cv2.bitwise_or(masks_sum, mask_c)
    # masks_sum = cv2.bitwise_or(masks_sum, mask_d)
    # masks_sum = cv2.bitwise_or(masks_sum, mask_e)
    # masks_sum = cv2.bitwise_or(masks_sum, mask_f)
    # masks_sum = cv2.bitwise_or(masks_sum, mask_g)
    # cv2.imshow("masks_sum", masks_sum)

    return mask_dot, mask_a, mask_b, mask_c, mask_d, mask_e, mask_f, mask_g

###
### TBD: use (width_button * factor) for offset_to_right insted off 5 pixels (to have more tolerance with camera distance)
###
### img only needed FOR DEBUG
def locate_digits_origins(img, x_right_side, y_top_side, width_button, height_button):
    offset_to_right     = 5                         # offset from end-last digit to right side of white button
    offset_to_top       = 1.25 * height_button      # offset from bottom digits to top side of white button
    offset_x_origins    = width_button * 1.07       # distance between each origin
    digits_height       = height_button * 2.25

    ### X reference coordinate for digits
    ende_4th_digit     = int (x_right_side - offset_to_right)
    
    ### Y reference coordinates for digits
    y_down  = int(y_top_side - offset_to_top)
    y_up    = int(y_down - digits_height)

    ### ORIGINS FOR EACH DIGIT (left-bottom corner)
    x_origin_1    = int(ende_4th_digit - (4 * offset_x_origins))
    x_origin_2    = int(ende_4th_digit - (3 * offset_x_origins))
    x_origin_3    = int(ende_4th_digit - (2 * offset_x_origins))
    x_origin_4    = int(ende_4th_digit - (1 * offset_x_origins))

    ### FOR DEBUG
    # boxes = np.zeros(img.shape, dtype="uint8")                                                # new image to store mask
    # boxes = cv2.line(boxes, (x_origin_1, y_down), (x_origin_1, y_up), 125, 1, 8)              # start 1st number
    # boxes = cv2.line(boxes, (x_origin_2, y_down), (x_origin_2, y_up), 125, 1, 8)              # start 2nd number
    # boxes = cv2.line(boxes, (x_origin_3, y_down), (x_origin_3, y_up), 125, 1, 8)              # start 3rd number
    # boxes = cv2.line(boxes, (x_origin_4, y_down), (x_origin_4, y_up), 125, 1, 8)              # start 4th number
    # boxes = cv2.line(boxes, (ende_4th_digit, y_down), (ende_4th_digit, y_up), 125, 1, 8)      # end 4th number
    # boxes = cv2.line(boxes, (x_origin_1, y_down), (ende_4th_digit, y_down), 125, 1, 8)        # bottom numbers
    # boxes = cv2.line(boxes, (x_origin_1, y_up), (ende_4th_digit, y_up), 125, 1, 8)            # top numbers
    # boxes_and_image = cv2.bitwise_or(img, boxes)
    # cv2.imshow("boxes (expecting each box to start at dot position and digit to be inside)", boxes_and_image)
    
    return x_origin_1, x_origin_2, x_origin_3, x_origin_4, y_down, offset_x_origins, digits_height


def find_white_button(i_t):
    ### CLEAR OUTPUT IMG to reduce noise
    # OPENING
    shape_open      = cv2.MORPH_ELLIPSE
    ksize_open      = (7, 7)
    kernel_open     = cv2.getStructuringElement(shape_open, ksize_open)
    i_open          = cv2.morphologyEx(i_t, cv2.MORPH_OPEN, kernel_open)
    # cv2.imshow("(img) after opening", i_open)  

    ### CONNECTIONS
    connectivity = 8        # 4 or 8 for connectivity type
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(i_open, connectivity, cv2.CV_16S)
    
    # FILTER CONNECTIONS
    output = np.zeros(i_t.shape, dtype="uint8")       # new image to store connections, ONLY FOR DEBUG
    counter = 0
    target_label = -1
    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        
        if (area > 450) & (area < 1000):                                # target has around 760
            width_box       = stats[i, cv2.CC_STAT_WIDTH]
            height_box      = stats[i, cv2.CC_STAT_HEIGHT]
            square_ratio    = width_box/height_box
            
            ## FOR DEBUG
            # x = int(centroids[i, 0])
            # y = int(centroids[i, 1])
            # print("Connection=%s, [area=%s, x=%s, y=%s, square_ratio=%s]" % (i, area, x, y, square_ratio))

            if (square_ratio > 1) & (square_ratio < 1.6):               # target has 1.28
                componentMask = (labels == i).astype("uint8") * 255
                output = cv2.bitwise_or(output, componentMask)
                target_label = i
                counter += 1

                ## FOR DEBUG
                # x = int(centroids[i, 0])
                # y = int(centroids[i, 1])
                # print("Connection=%s, [area=%s, x=%s, y=%s, square_ratio=%s]" % (i, area, x, y, square_ratio))

    # cv2.imshow("(after filter connections) Target is white button", output)
    
    button_found = False
    if (counter == 1):
        # print("White button found, Connection filtering OK")
        button_found = True
    else:
        print("ERROR... (white button) Number of connections after filtering= %s" % (counter))

    x_button    = int(centroids[target_label, 0])
    y_button    = int(centroids[target_label, 1])
    p_button = np.array([x_button, y_button])

    width       = stats[target_label, cv2.CC_STAT_WIDTH]
    height      = stats[target_label, cv2.CC_STAT_HEIGHT]

    x_left      = stats[target_label, cv2.CC_STAT_LEFT]
    x_right     = x_left + width
    y_top       = stats[target_label, cv2.CC_STAT_TOP]

    return output, p_button, x_right, y_top, width, height


def digit_recognition(i_t, x_origin, y_down, offset_x_origins, digits_height):
    mask_dot, mask_a, mask_b, mask_c, mask_d, mask_e, mask_f, mask_g    = create_masks_for_digit(i_t, x_origin, y_down, offset_x_origins, digits_height)

    zero    = np.array([1, 1, 1, 1, 1, 1, 0])
    one     = np.array([0, 1, 1, 0, 0, 0, 0])
    two     = np.array([1, 1, 0, 1, 1, 0, 1])
    three   = np.array([1, 1, 1, 1, 0, 0, 1])
    four    = np.array([0, 1, 1, 0, 0, 1, 1])
    five    = np.array([1, 0, 1, 1, 0, 1, 1])
    six     = np.array([1, 0, 1, 1, 1, 1, 1])
    seven   = np.array([1, 1, 1, 0, 0, 0, 0])
    eight   = np.array([1, 1, 1, 1, 1, 1, 1])
    nine    = np.array([1, 1, 1, 1, 0, 1, 1])

    dot = False
    number  = np.array([0, 0, 0, 0, 0, 0, 0])

    if (cv2.countNonZero(cv2.bitwise_and(i_t, mask_dot)) > 0):
        dot = True

    if (cv2.countNonZero(cv2.bitwise_and(i_t, mask_a)) > 0):
        number[0] = 1
    
    if (cv2.countNonZero(cv2.bitwise_and(i_t, mask_b)) > 0):
        number[1] = 1
    
    if (cv2.countNonZero(cv2.bitwise_and(i_t, mask_c)) > 0):
        number[2] = 1

    if (cv2.countNonZero(cv2.bitwise_and(i_t, mask_d)) > 0):
        number[3] = 1

    if (cv2.countNonZero(cv2.bitwise_and(i_t, mask_e)) > 0):
        number[4] = 1

    if (cv2.countNonZero(cv2.bitwise_and(i_t, mask_f)) > 0):
        number[5] = 1

    if (cv2.countNonZero(cv2.bitwise_and(i_t, mask_g)) > 0):
        number[6] = 1

    ### DEFINE DIGIT
    digit = ""
    ### DOT yes/no
    if (dot):
        digit = digit + "."
    
    ### NUMBER 0 ... 9  OR  x
    if (number==zero).all():
        digit = digit + "0"
    elif (number==one).all():
        digit = digit + "1"
    elif (number==two).all():
        digit = digit + "2"
    elif (number==three).all():
        digit = digit + "3"
    elif (number==four).all():
        digit = digit + "4"
    elif (number==five).all():
        digit = digit + "5"
    elif (number==six).all():
        digit = digit + "6"
    elif (number==seven).all():
        digit = digit + "7"
    elif (number==eight).all():
        digit = digit + "8"
    elif (number==nine).all():
        digit = digit + "9"
    else:
        digit = digit + "x"
    
    return digit


def image_callback(img_msg):
    ### Convert ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: " + e)
    cv2.imshow("original input", cv_image)
    
    ### BGR to GRAY
    i_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # Threshold
    _, i_t = cv2.threshold(i_gray, 150, 255, cv2.THRESH_BINARY)
    # cv2.imshow("gray threshold, NUMBERS AND DOT should be there", i_t)
    

    ### FIND WHITE BUTTON, it is our reference point
    i_button, p_button, x_right_side, y_top_side, width_button, height_button    =   find_white_button(i_t)
    # cv2.imshow("ONLY white button", i_button)


    ### LOCATE DIGITS ORIGINS
    x_origin_1, x_origin_2, x_origin_3, x_origin_4, y_down, offset_x_origins, digits_height  = locate_digits_origins(i_t, x_right_side, y_top_side, width_button, height_button) #i_t only needed FOR DEBUG


    answer = ""
    ### RECOGNIZE DIGIT 1
    digit_1 = digit_recognition(i_t, x_origin_1, y_down, offset_x_origins, digits_height)
    answer = answer + digit_1

    ### RECOGNIZE DIGIT 2
    digit_2 = digit_recognition(i_t, x_origin_2, y_down, offset_x_origins, digits_height)
    answer = answer + digit_2

    ### RECOGNIZE DIGIT 3
    digit_3 = digit_recognition(i_t, x_origin_3, y_down, offset_x_origins, digits_height)
    answer = answer + digit_3

    ### RECOGNIZE DIGIT 4
    digit_4 = digit_recognition(i_t, x_origin_4, y_down, offset_x_origins, digits_height)
    answer = answer + digit_4


    ### OUTPUT
    output_screen = "Battery Voltage= " + answer + " V"
    print(output_screen)

    cv2.waitKey(50)


# Initialize ROS Node named 'read_display’
rospy.init_node('read_display')

# Initialize the CvBridge class
bridge = CvBridge()

# Initialize a subscriber to the "/camera/color/image_raw" topic
sub_image = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
     rospy.spin()