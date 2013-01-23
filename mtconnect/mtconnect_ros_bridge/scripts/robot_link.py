import sys
import os

# Import custom Python modules for MTConnect Adapter interface
path, file = os.path.split(__file__)
sys.path.append(os.path.realpath(path) + '/src')
from data_item import Event, SimpleCondition, Sample, ThreeDSample
from mtconnect_adapter import Adapter

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

def robot_init():
    adapter = Adapter(('0.0.0.0', 7900))

    # Start the adapter
    rospy.loginfo('Start the Robot Link adapter')
    adapter.start()
    
#    material_load_di = Event('material_load')
#    adapter.add_data_item(material_load_di)

#    material_unload_di = Event('material_unload')
#    adapter.add_data_item(material_unload_di)

    open_door_di = Event('open_door')
    adapter.add_data_item(open_door_di)

    close_door_di = Event('close_door')
    adapter.add_data_item(close_door_di)

    open_chuck_di = Event('open_chuck')
    adapter.add_data_item(open_chuck_di)

    close_chuck_di = Event('close_chuck')
    adapter.add_data_item(close_chuck_di)

    adapter.begin_gather()

#    material_load_di.set_value('READY')
#    material_unload_di.set_value('READY')
    open_door_di.set_value('READY')
    close_door_di.set_value('READY')
    open_chuck_di.set_value('READY')
    close_chuck_di.set_value('READY')

    adapter.complete_gather()
    return

    
if __name__ == '__main__':
    try:
        rospy.loginfo('Launching robot_link node, CTRL-C to terminate')
        robot_init()
        talker()
    except rospy.ROSInterruptException:
        pass
