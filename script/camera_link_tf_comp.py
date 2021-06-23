import numpy as np
import math
import tf

def rotationMatrixToEulerAngles(R) :
 
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

def eulerAnglesToRotationMatrix(theta) :
     
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
         
         
                     
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                 
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                     
                     
    R = np.matmul(R_z, np.matmul( R_y, R_x ))
 
    return R

M_CL = np.array([[-0.02432746, -0.9994455,   0.02273481, -0.14896817],
                 [-0.76487787,  0.00396481, -0.64416311,  0.15989226],
                 [ 0.64371578, -0.0330602,  -0.7645502,  0.6544822 ],
                 [ 0.,          0.,          0.,          1.        ]])
M_BL = np.array([[1., 0., 0.,  0.3],
                 [0., 1., 0.,  0.3],
                 [0., 0., 1.,  -0.09],
                 [0., 0., 0.,  1.]])

M_CL_RGB = np.array([[1., 0., 0.,  0.0],
                 [0., 1., 0.,  -0.045],
                 [0., 0., 1.,  0.0],
                 [0., 0., 0.,  1.]])

M_CL_D = np.array([[1., 0., 0.,  0.0],
                 [0., 1., 0.,  -0.02],
                 [0., 0., 1.,  0.0],
                 [0., 0., 0.,  1.]])

R = tf.transformations.quaternion_matrix([-0.5, 0.5, -0.5, 0.5])
#R = eulerAnglesToRotationMatrix([-np.pi/2, 0, -np.pi/2])
#M_RGB_RGBO = np.array([[1., 0., 0.,  0.0],
#                     [0., 1., 0.,  0.0],
#                     [0., 0., 1.,  0.0],
#                     [0., 0., 0.,  1.]])
M_RGB_RGBO = R                     

M_BC = np.matmul(M_BL, np.linalg.inv(M_CL))
print(M_BC)
M_CL_RGBO = np.matmul(M_CL_RGB, M_RGB_RGBO)
M_B_CL = np.matmul(M_BC, np.linalg.inv(M_CL_RGBO))
euler_angle_1 = rotationMatrixToEulerAngles(M_B_CL[:-1, :-1])

M_CL_DO = np.matmul(M_CL_D, M_RGB_RGBO)
M_B_DO = np.matmul(M_B_CL, M_CL_DO)
euler_angle = rotationMatrixToEulerAngles(M_B_DO[:-1, :-1])
print(M_B_DO)
print(euler_angle)
print(M_B_CL)
print(euler_angle_1)

