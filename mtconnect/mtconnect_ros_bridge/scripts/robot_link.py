import read_config_file
import roslib
roslib.load_manifest('mtconnect_msgs')
import rospy
import mtconnect_msgs.msg

def talker():
    pub = rospy.Publisher('industrial_msgs/RobotStates', mtconnect_msgs.msg.RobotStates)
    rospy.init_node('robot_link')
    msg = mtconnect_msgs.msg.RobotStates()
    
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        msg.avail.val = 1 # Available
        msg.mode.val = 2 # Automatic
        msg.rexec.val = 1 # Active

        pub.publish(msg)
    
if __name__ == '__main__':
    try:
        rospy.loginfo('Launching robot_link node, CTRL-C to terminate')
        talker()
    except rospy.ROSInterruptException:
        pass
