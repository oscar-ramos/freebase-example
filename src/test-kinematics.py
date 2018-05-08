#!/usr/bin/python
#
# Test of the robot kinematics using the floating base
#
#    roslaunch freebase-example display.launch
#    rosrun freebase-example test-kinematics.py
#

import rospy
from sensor_msgs.msg import JointState
from markers import *
from robot import Robot

# Joint names
jnames = [
    "px", "py", "pz", "ew", "ex", "ey", "ez",
    "front_right_j1", "front_right_j2", "front_right_j3",
    "front_left_j1", "front_left_j2", "front_left_j3",
    "rear_right_j1", "rear_right_j2", "rear_right_j3",
    "rear_left_j1", "rear_left_j2", "rear_left_j3"]

# Initial configuration (position, quat, qactuated)
q0 = np.array([0.3, 0.4, 0.5, np.cos(-1.0/2.0), 0., np.sin(-1.0/2.0), 0.,
               0., 0., 0., 0., 0., 0.,
               0., 0., 0., 0., 0., 0.])

# Internal robot representation
robot = Robot()
robot.update_config(q0)

# Initialize the node
rospy.init_node('test')

# Ball markers
bmarker1 = BallMarker(color['RED'])
bmarker2 = BallMarker(color['RED'])
bmarker3 = BallMarker(color['RED'])
bmarker4 = BallMarker(color['RED'])

# Publisher for joint states 
pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
# Creation of a message
jstate = JointState()
jstate.name = jnames
jstate.position = q0

rate = rospy.Rate(10)
q = np.copy(q0)
while not rospy.is_shutdown():

    jstate.header.stamp = rospy.Time.now()
    pub.publish(jstate)

    # Forward kinematics
    robot.update_config(q)
    # To test for different legs use:
    #    fkine_fleft, fkine_fright, fkine_rleft, fkine_rright
    # Front right foot
    xfr = robot.fkine_fright()
    # Front left foot
    xfl = robot.fkine_fleft()
    # Rear right foot
    xrr = robot.fkine_rright()
    # Rear left foot
    xrl = robot.fkine_rleft()
    # Show the markers
    bmarker1.position(xfr)
    bmarker2.position(xfl)
    bmarker3.position(xrr)
    bmarker4.position(xrl)
    
    rate.sleep()



