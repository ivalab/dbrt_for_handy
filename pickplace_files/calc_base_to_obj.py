#!/usr/bin/env python
import numpy as np
import math
import rospy
from sensor_msgs.msg import JointState

cameraMatrix = np.matrix([[524.82863, 0.0,  348.39958, 0],
                         [0.0, 525.53556, 228.20485, 0],
                         [0.0, 0.0, 1.0, 0]])


#camera_to_world = np.matrix([[-0.04326758, -0.9977821,   0.05058452, -0.06799752],
# [-0.85053546,  0.01022502, -0.5258183,   0.22248348],
# [ 0.52413486, -0.06577481, -0.84909147,  0.9599998 ],
# [ 0.,          0.,          0.,          1.        ]])

camera_to_world = np.matrix([[-0.05322374, -0.982154,    0.18039054, -0.17357615],
 [-0.85878157, -0.04716331, -0.51016647,  0.15909051],
 [ 0.50956986, -0.18206904, -0.84094555,  0.51758756],
 [ 0.,          0.,          0.,          1.        ]])




object_in_frame = np.matrix([482.90164185, 263.58447266, 1]).transpose()

camera_to_object = np.linalg.pinv(cameraMatrix) * object_in_frame
print(camera_to_object)


base_to_world = np.matrix([[ 1., 0., 0., 0.16],
      			[ 0., 1., 0., 0.3],
       			[ 0., 0., 1., -0.12],
       			[ 0., 0., 0., 1.]])

base_to_camera = base_to_world * np.linalg.inv(camera_to_world)
alpha = math.atan2(base_to_camera[1, 0], base_to_camera[0, 0])
beta = math.atan2(-1 * base_to_camera[2, 0], math.sqrt(math.pow(base_to_camera[2, 1], 2) + math.pow(base_to_camera[2, 2], 2)))
gamma = math.atan2(base_to_camera[2, 1], base_to_camera[2, 2])

print('')
print("BASE TO CAMERA")
print(base_to_camera)
print('')
print("rpy")
print([gamma, beta, alpha])


alpha = -2.958
beta = 0.018
gamma = -1.936
est_base_to_camera = np.matrix(
	[[math.cos(alpha)*math.cos(beta), math.cos(alpha)*math.sin(beta)*math.sin(gamma)-math.sin(alpha)*math.cos(gamma), math.cos(alpha)*math.sin(beta)*math.cos(gamma)+math.sin(alpha)*math.sin(gamma),  0.230],
 	[math.sin(alpha)*math.cos(beta),  math.sin(alpha)*math.sin(beta)*math.sin(gamma)+math.cos(alpha)*math.cos(gamma), math.sin(alpha)*math.sin(beta)*math.cos(gamma)-math.cos(alpha)*math.sin(gamma),  0.821],
 	[ -1*math.sin(beta),  math.cos(beta)*math.sin(gamma),  math.cos(beta)*math.cos(gamma),  0.295],
 	[ 0.,          0.,          0.,          1.        ]])


object_ray = base_to_camera * camera_to_object
#must solve equation for object_ray + base_to_camera_T = z_value of -0.12
base_to_camera_T = base_to_camera[:, 3]
factor = (-0.12 - base_to_camera_T[2])/object_ray[2]
object_position_init = float(factor) * object_ray + base_to_camera_T
print('')
print("initial base to object")
print(object_position_init)

#print('')
#print("est base to object")
#est_base_to_object = est_base_to_camera * camera_to_object
#print(est_base_to_object)
def callback(msg):
    x_adj = msg.position[0]
    y_adj = msg.position[1]
    z_adj = msg.position[2]
    beta = msg.position[3]
    alpha = msg.position[4]
    gamma = msg.position[5]
    base_to_camera_adj = np.matrix(
	[[math.cos(alpha)*math.cos(beta), math.cos(alpha)*math.sin(beta)*math.sin(gamma)-math.sin(alpha)*math.cos(gamma), math.cos(alpha)*math.sin(beta)*math.cos(gamma)+math.sin(alpha)*math.sin(gamma),  x_adj],
 	[math.sin(alpha)*math.cos(beta),  math.sin(alpha)*math.sin(beta)*math.sin(gamma)+math.cos(alpha)*math.cos(gamma), math.sin(alpha)*math.sin(beta)*math.cos(gamma)-math.cos(alpha)*math.sin(gamma),  y_adj],
 	[ -1*math.sin(beta),  math.cos(beta)*math.sin(gamma),  math.cos(beta)*math.cos(gamma),  z_adj],
 	[ 0.,          0.,          0.,          1.        ]])
    print(base_to_camera_adj)
    object_ray =  base_to_camera * np.linalg.pinv(base_to_camera_adj) * camera_to_object
    #must solve equation for object_ray + base_to_camera_T = z_value of -0.12
    base_to_camera_T = base_to_camera[:, 3]
    factor = (-0.12 - base_to_camera_T[2])/object_ray[2]
    object_position = float(factor) * object_ray + base_to_camera_T
    print('')
    print("initial base to object")
    print(object_position_init)
    print('')
    print("corrected base to object")
    print(object_position)
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/estimated/joint_states", JointState, callback)

    rospy.spin()

#listener()
