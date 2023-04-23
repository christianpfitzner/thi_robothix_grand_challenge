import rospy
import threading
from std_srvs.srv import Trigger
import tf
import math

position_glob = "probe"

class Server:
    global position_glob

    def __init__(self, topic, position):
        self.position = position
        self.topic = topic
        self.srv = rospy.Service(topic, Trigger, self.callback)

    def callback(self, data):
        position_glob = self.position

def tcp_broadcaster():
    while not rospy.is_shutdown():
        br = tf.TransformBroadcaster()
        if position_glob == "gripper_bottom":    
            br.sendTransform((0, 0, 0.11),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "panda_hand_tcp",
                        "panda_hand")
        elif position_glob == "gripper_center":
            br.sendTransform((0, 0, 0.103),
                        tf.transformations.quaternion_from_euler(0, 0, 0),
                        rospy.Time.now(),
                        "panda_hand_tcp",
                        "panda_hand")
        elif position_glob == "probe":
            br.sendTransform((0.1061, 0, 0.157),
                        tf.transformations.quaternion_from_euler(0, math.pi/2-0.471, 0),
                        rospy.Time.now(),
                        "panda_hand_tcp",
                        "panda_hand")
    
if __name__ == '__main__':

    rospy.init_node('tcp_position_publisher')
    threading.Thread(target=tcp_broadcaster).start()

    srvr_gripper_bottom = Server('tcp_position_publisher/gripper_bottom', "gripper_bottom")
    srvr_gripper_bottom = Server('tcp_position_publisher/gripper_center', "gripper_center")
    srvr_gripper_bottom = Server('tcp_position_publisher/probe', "probe")

    rospy.spin()
