import read_config_file
import roslib
roslib.load_manifest('mtconnect_msgs')
import rospy
import mtconnect_msgs.msg

def talker():
    pub1 = rospy.Publisher('RobotStateTopic', mtconnect_msgs.msg.RobotStates)
    pub2 = rospy.Publisher('RobotSpindleTopic', mtconnect_msgs.msg.RobotSpindle)
    rospy.init_node('robot_link')
    msg1 = mtconnect_msgs.msg.RobotStates()
    msg2 = mtconnect_msgs.msg.RobotSpindle()
    
    while not rospy.is_shutdown():
        msg1.header.stamp = rospy.Time.now()
        msg1.avail.val = 1 # Available
        msg1.mode.val = 2 # Automatic
        msg1.rexec.val = 1 # Active
        
        msg2.c_unclamp.val = 1
        msg2.s_inter.val = 1

        pub1.publish(msg1)
        pub2.publish(msg2)
      
    
if __name__ == '__main__':
    try:
        rospy.loginfo('Launching robot_link node, CTRL-C to terminate')
        talker()
    except rospy.ROSInterruptException:
        pass
