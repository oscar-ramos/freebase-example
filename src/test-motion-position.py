#!/usr/bin/python
#
# Test of the robot motion using the floating base
#
#    roslaunch freebase-example display.launch
#    rosrun freebase-example test-motion-position.py
#

import rospy
from sensor_msgs.msg import JointState
from markers import *
from robot import Robot
from kcontroller import WQPController
from utils import Tmat, quaternionMult

# Joint names
jnames = [
    "px", "py", "pz", "ew", "ex", "ey", "ez",
    "front_right_j1", "front_right_j2", "front_right_j3",
    "front_left_j1", "front_left_j2", "front_left_j3",
    "rear_right_j1", "rear_right_j2", "rear_right_j3",
    "rear_left_j1", "rear_left_j2", "rear_left_j3"]

# Initial configuration (position, quat, qactuated)
angle = 0.0
q0 = np.array([0., 0., 0.48, np.cos(angle/2.0), 0., np.sin(angle/2.0), 0.,
               0., 0., 0., 0., 0., 0.,
               0., 0., 0., 0., 0., 0.])

# Internal robot representation
robot = Robot()
robot.update_config(q0)

# Initialize the node
rospy.init_node('test')
# Loop frequency
dt = 0.010
freq = 1.0/dt

# Initialize kinematic controller
weights = [1.0, 1.0, 1.0, 1.0]
#lambdas = [0.01, 0.01, 0.01, 0.01]
#lambdas = [0.1, 0.1, 0.1, 0.1]
lambdas = [0.5, 0.5, 0.5, 0.5]
#lambdas = [1.0, 1.0, 1.0, 1.0]
#lambdas = [10.0, 10.0, 10.0, 10.0]
solver = WQPController(weights, lambdas, dt)

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

robot.update_config(q0)
pfr_des = robot.fkine_fright()[0:3,3]
pfl_des = robot.fkine_fleft()[0:3,3]
prr_des = robot.fkine_rright()[0:3,3]
prl_des = robot.fkine_rleft()[0:3,3]

# Change initial configuration
pfr_des[2] = 0.15
pfr_des[1] += 0.05
# pfl_des[2] = 0.10
# pfl_des[1] += -0.05
# pfr_des[2] = 0.20
# pfr_des[1] += 0.05
# pfl_des[2] = 0.05
# pfl_des[1] += -0.05

rate = rospy.Rate(freq)
q = np.copy(q0)
while not rospy.is_shutdown():
    # Errors
    efr = robot.error_position_fright(pfr_des)
    efl = robot.error_position_fleft(pfl_des)
    err = robot.error_position_rright(prr_des)
    erl = robot.error_position_rleft(prl_des)
    # Task Jacobians
    Jfr = robot.taskj_position_fright()
    Jfl = robot.taskj_position_fleft()
    Jrr = robot.taskj_position_rright()
    Jrl = robot.taskj_position_rleft()
    
    # Get the joint velocity
    dq = solver.get_dq(q, efr, Jfr, efl, Jfl, err, Jrr, erl, Jrl)

    # Integrate rotation
    w = np.dot(Tmat(q), dq[3:7])
    dth = np.linalg.norm(w)
    if abs(dth)>1e-9:
        u = w/dth
        dQ = np.array([np.cos(dth*dt/2.0), u[0]*np.sin(dth*dt/2.0),
                       u[1]*np.sin(dth*dt/2.0), u[2]*np.sin(dth*dt/2.0)])
        Q = quaternionMult(dQ, q[3:7])
        q[3:7] = Q
    # Integrate position and joint configuration
    q[0:3] = q[0:3] + dt*dq[0:3]
    q[7:]  = q[7:] + dt*dq[7:]
    print 'dq:', dq
    print 'q:', np.round(q,2), '\n'
    # Update the robot configuration
    robot.update_config(q)

    # Set message
    jstate.header.stamp = rospy.Time.now()
    jstate.position = q
    pub.publish(jstate)
    
    # Show the markers
    bmarker1.xyz(pfr_des)
    bmarker2.xyz(pfl_des)
    bmarker3.xyz(prr_des)
    bmarker4.xyz(prl_des)
    
    rate.sleep()



