import numpy as np
import sympy as sp
import rospy
import matplotlib.pyplot as plt
from mono_location.msg import corners
from mono_location.msg import GimbalState
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise
from sympy import pi,sin,cos,atan,sqrt
from enum import Enum
from geometry_msgs.msg import PoseStamped,Quaternion
from mavros_msgs.msg import HomePosition
from mpl_toolkits.mplot3d import Axes3D
from scipy.ndimage import gaussian_filter1d

import time
import rosbag
import pickle

from numpy.random import randn

class Constants(Enum):
    KU = 357.554867
    KV = 476.965401
    U0 = 314.713872
    V0 = 231.855112

    # The height between camera and flight control,unit m 
    DELTA_HEIGHT = 0.300
    # Distance between camera and flight control on the x axis,unit m 
    FLY_GIMBAL_X = 0.10

def quaternion_to_euler(quaternion:Quaternion):
    """
    # Convert quaternions to Euler angles
    parameters: 
        --quaternion: quaternions
    output:   
        --roll, pitch, yaw: Euler Angle (roll, pitch, yaw), measured in radians
    """
    # Calculate the Roll (rotation around the X-axis)
    sinr_cosp = 2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
    cosr_cosp = 1 - 2 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Calculate the Pitch (rotation around the Y-axis)
    sinp = 2 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp) 
    else:
        pitch = np.arcsin(sinp)

    # Calculate Yaw (rotation around the Z-axis)
    siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def rotation_matrix(alpha, beta, theta):

    R_x = np.array([[1, 0, 0],
                    [0, np.cos(alpha), -np.sin(alpha)],
                    [0, np.sin(alpha), np.cos(alpha)]])

    R_y = np.array([[np.cos(beta), 0, np.sin(beta)],
                    [0, 1, 0],
                    [-np.sin(beta), 0, np.cos(beta)]])

    R_z = np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])

    R = np.dot(R_z, np.dot(R_y, R_x))

    return R

#Measurement function: Hx（x)
def Hx(x):

    X_ = float(x[0])
    Y_ = float(x[2])
    Z_ = float(x[4])
    alpha = float(x[6])
    beta = float(x[8])
    theta = float(x[10])
    l_ = float(x[12])
    
    T_ = np.tile(np.array([[X_],[Y_],[Z_]]),(1,4))
    

    # The coordinates of the four corners in the object coordinate system ,
    # l stands for the length of the side of the square
    P_o = np.array([[ l_ ,l_, -l_,-l_],
                    [ l_ ,-l_,-l_,l_],
                    [ 0 , 0 , 0 , 0 ]])

    R_ = rotation_matrix(alpha,beta,theta)
    
    P_c = T_ + np.dot(R_,P_o)

    d_ = np.sqrt(X_**2 + Y_**2 + Z_**2)
    
    hx = np.array([Constants.KU.value*P_c[0][0]/P_c[2][0],
                   Constants.KV.value*P_c[1][0]/P_c[2][0],
                   Constants.KU.value*P_c[0][1]/P_c[2][1],
                   Constants.KV.value*P_c[1][1]/P_c[2][1],
                   Constants.KU.value*P_c[0][2]/P_c[2][2],
                   Constants.KV.value*P_c[1][2]/P_c[2][2],
                   Constants.KU.value*P_c[0][3]/P_c[2][3],
                   Constants.KV.value*P_c[1][3]/P_c[2][3],
                   d_])
    
    return hx

def Fx(x,dt):
    '''
    # function that returns the state x transformed by the state transistion function. 
    parameters:  
        --x: State at the last moment
        --dt: the time step , in seconds.
    output:  
        --x_new: State after the update
    '''
    #state-transition matrix(linear) : F
    dt_M = np.eye(12)
    dt_M[::2] *= dt
    dt_M[1::2] *= 0
    F = np.eye(13)
    F[0:0+dt_M.shape[0],1:1+dt_M.shape[1]] += dt_M

    x_new = F @ x

    return x_new

#Simulates the camera signal
def cameraSim(t):

    # X_ = 0.0
    # Y_ = 0.0
    # Z_ = 5.0
    # alpha = 180*np.pi/180
    # beta = 0*np.pi/180
    # theta = 0*np.pi/180
    # l_ = 0.8
    X_ = 2*np.cos(np.pi/6*t)
    Y_ = 2*np.sin(np.pi/6*t)
    Z_ = 5.0
    alpha = 180*np.pi/180
    beta = 0*np.pi/180
    theta = 0*np.pi/180
    l_ = 0.8

    T_ = np.tile(np.array([[X_],[Y_],[Z_]]),(1,4))

    # The coordinates of the four corners in the object coordinate system ,
    # l stands for the length of the side of the square
    P_o = np.array([[ l_ ,l_, -l_,-l_],
                    [ l_ ,-l_,-l_,l_],
                    [ 0 , 0 , 0 , 0 ]])

    R_ = rotation_matrix(alpha,beta,theta)
    
    P_c = T_ + np.dot(R_,P_o)

    d_ = np.sqrt(X_**2 + Y_**2 + Z_**2)
    
    hx = np.array([Constants.KU.value*P_c[0][0]/P_c[2][0]+0.5*randn(),
                   Constants.KV.value*P_c[1][0]/P_c[2][0]+0.5*randn(),
                   Constants.KU.value*P_c[0][1]/P_c[2][1]+0.5*randn(),
                   Constants.KV.value*P_c[1][1]/P_c[2][1]+0.5*randn(),
                   Constants.KU.value*P_c[0][2]/P_c[2][2]+0.5*randn(),
                   Constants.KV.value*P_c[1][2]/P_c[2][2]+0.5*randn(),
                   Constants.KU.value*P_c[0][3]/P_c[2][3]+0.5*randn(),
                   Constants.KV.value*P_c[1][3]/P_c[2][3]+0.5*randn(),
                    ])
    
    # phi_ = cal_phi(float(hx[2])+Constants.U0.value,float(hx[4])+Constants.U0.value,float(hx[3])+Constants.V0.value,float(hx[5])+Constants.V0.value)
    d_array = np.array([d_])
    
    hx = np.append(hx,d_array,axis=0)
    # print(hx)

    return hx

# The ninth observation（phi） is calculated using the corner coordinates
def cal_phi(x1,x2,y1,y2):
    # The internal parameter of the camera
    focus_distance = 1
    
    du = 357.554867
    dv = 476.965401

    delta_x1 = abs(x1 - Constants.U0.value)*du
    delta_x2 = abs(x2 - Constants.U0.value)*du
    delta_y1 = abs(y1 - Constants.V0.value)*dv
    delta_y2 = abs(y2 - Constants.V0.value)*dv

    l_left = np.sqrt(focus_distance**2 + delta_x1**2 + delta_y1**2)
    l_right = np.sqrt(focus_distance**2 + delta_x2**2 + delta_y2**2)
    delta_d2 = (delta_x1 + delta_x2)**2 + (delta_y1 + delta_y2)**2

    phi = np.arccos(abs(l_left**2 + l_right**2 - delta_d2)/(2*l_left*l_right))

    return phi

def imgCoord2camCoord(x,y):
    '''
    # Calculate the vector of the target point in the camera frame from the pixel coordinates
    parameters:  
        --x,y: the pixel coordinates of the points  (float)
    output:  
        --camera_vector: Vector in the camera frame  (np.array)
    '''
    pixel_homogeneous = np.array([x,y,1])
    K = np.array([[357.554867, 0, 314.713872],  
                  [0, 476.965401, 231.855112],  
                  [0, 0, 1]]) 
    
    K_inv = np.linalg.inv(K)
    camera_vector = K_inv @ pixel_homogeneous
    # camera_vector = camera_vector/np.linalg.norm(camera_vector)
    
    return camera_vector

def getHomoTransMatix321(alpha, beta, theta,tx, ty, tz):
    '''
    # Calculate the Homogeneous transformation matrix.The rotation order is z , y , x axis
    parameters:  
        --alpha,beta,theta: Euler Angle of rotation about three axes(radian),xyz
        --tx, ty, tz: Translation on three axes 
    output:  
        --T: the Homogeneous transformation matrix  (np.array)
    '''
    R = rotation_matrix(alpha, beta, theta)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    
    return T

def camCoord2inertialCoord(camera_vector, gimbalPose: GimbalState, uavLocalPos: PoseStamped ):
    '''
    # Calculate the vector of the target point in the inertial frame from the camera frame
    parameters: 
        --camera_vector: Vector in the camera frame  (np.array)
        --gimbalPose: The roll,pitch,yaw of the gimbal 
        --uavLocalPos: The uav pose in loacl camera (PoseStamped)
    output:   
        --inertial_vector: Vector in the inertial frame (np.array)
    '''
    # camera frame to gimbal frame :  P^g = g^T_c * P^c
    gimbal2camera_alpha = -np.pi/2
    gimbal2camera_beta  = 0
    gimbal2camera_theta = np.pi/2
    gimbal2camera_tx , gimbal2camera_ty , gimbal2camera_tz = 0, 0, 0

    gimbal2camera_TransMatix = getHomoTransMatix321(gimbal2camera_alpha,gimbal2camera_beta,gimbal2camera_theta,\
                                         gimbal2camera_tx , gimbal2camera_ty , gimbal2camera_tz)
    gimbal_vector =  gimbal2camera_TransMatix @ camera_vector 
    # print(gimbal_vector)

    # gimbal frame to inertial frame :  P^i = i^T_g * P^g
    gimbalRoll = np.radians(gimbalPose.current_roll)
    gimbalPitch = np.radians(gimbalPose.current_pitch)
    gimbalYaw = np.radians(gimbalPose.current_yaw)
    uavRoll,uavPitch,uavYaw = quaternion_to_euler(uavLocalPos.pose.orientation)

    inertial2gimbal_alpha = gimbalRoll
    inertial2gimbal_beta = gimbalPitch
    inertial2gimbal_theta = gimbalYaw + uavYaw - np.pi
    inertial2gimbal_tx , inertial2gimbal_ty , inertial2gimbal_tz = 0, 0, 0

    inertial2camera_TransMatix = getHomoTransMatix321(inertial2gimbal_alpha,inertial2gimbal_beta,inertial2gimbal_theta,\
                                         inertial2gimbal_tx , inertial2gimbal_ty , inertial2gimbal_tz)
    inertial_vector =  inertial2camera_TransMatix @ gimbal_vector
    # print(inertial_vector)
    return inertial_vector

def inertialCoord2localCoord(inertial_positon,  uavLocalPos: PoseStamped ):
    '''
    # Calculate the position of the target in the local frame
    parameters: 
        --inertial_positon: position in the inertial frame  (np.array)
        --uavLocalPos: The uav pose in loacl camera (PoseStamped)
    output:   
        --local_position: Vector in the inertial frame (np.array)
    '''
    # inertial frame to uav frame :  P^u = (i^T_u)^-1 * P^i
    uavRoll,uavPitch,uavYaw = quaternion_to_euler(uavLocalPos.pose.orientation)
    inertial2uav_alpha = uavRoll
    inertial2uav_beta = uavPitch
    inertial2uav_theta = uavYaw
    inertial2uav_tx , inertial2uav_ty , inertial2uav_tz = -Constants.FLY_GIMBAL_X.value , 0, Constants.DELTA_HEIGHT.value
    inertial2uav_TransMatix = getHomoTransMatix321(inertial2uav_alpha,inertial2uav_beta,inertial2uav_theta,\
                                         inertial2uav_tx , inertial2uav_ty , inertial2uav_tz)
    uav2inertial_TransMatix = np.linalg.inv(inertial2uav_TransMatix)
    uav_position = uav2inertial_TransMatix @ inertial_positon
    # print(uav_position)

    # uav frame to local frame :  P^l = l^T_u * P^u
    local2uav_alpha = uavRoll
    local2uav_beta = uavPitch
    local2uav_theta = uavYaw
    local2uav_TransMatix = getHomoTransMatix321(local2uav_alpha,local2uav_beta,local2uav_theta,\
                                         uavLocalPos.pose.position.x ,uavLocalPos.pose.position.y, uavLocalPos.pose.position.z)
    local_position = local2uav_TransMatix @ uav_position
    # print(local_position)
    return local_position

def inertialCoord2camCoord(inertial_position, gimbalPose: GimbalState, uavLocalPos: PoseStamped ):
    '''
    # Calculate the vector of the target point in the inertial frame from the camera frame
    parameters: 
        --inertial_positon: position in the inertial frame  (np.array)
        --gimbalPose: The roll,pitch,yaw of the gimbal 
        --uavLocalPos: The uav pose in loacl camera (PoseStamped)
    output:   
        --cam_position: Vector in the inertial frame (np.array)
    '''
    # inertial frame to gimbal frame :  P^g = (i^T_g)^(-1) * P^i
    gimbalRoll = np.radians(gimbalPose.current_roll)
    gimbalPitch = np.radians(gimbalPose.current_pitch)
    gimbalYaw = np.radians(gimbalPose.current_yaw)
    uavRoll,uavPitch,uavYaw = quaternion_to_euler(uavLocalPos.pose.orientation)

    inertial2gimbal_alpha = gimbalRoll
    inertial2gimbal_beta = gimbalPitch
    inertial2gimbal_theta = gimbalYaw + uavYaw - np.pi
    inertial2gimbal_tx , inertial2gimbal_ty , inertial2gimbal_tz = 0, 0, 0

    inertial2gimbal_TransMatix = getHomoTransMatix321(inertial2gimbal_alpha,inertial2gimbal_beta,inertial2gimbal_theta,\
                                         inertial2gimbal_tx , inertial2gimbal_ty , inertial2gimbal_tz)
    gimbal2inertial_TransMatix = np.linalg.inv(inertial2gimbal_TransMatix)
    gimbal_position =  gimbal2inertial_TransMatix @ inertial_position

    # gimbal frame to camera frame :  P^c = (g^T_c)^(-1) * P^g
    gimbal2camera_alpha = -np.pi/2
    gimbal2camera_beta  = 0
    gimbal2camera_theta = np.pi/2
    gimbal2camera_tx , gimbal2camera_ty , gimbal2camera_tz = 0, 0, 0

    gimbal2camera_TransMatix = getHomoTransMatix321(gimbal2camera_alpha,gimbal2camera_beta,gimbal2camera_theta,\
                                         gimbal2camera_tx , gimbal2camera_ty , gimbal2camera_tz)
    camera2gimbal_TransMatix = np.linalg.inv(gimbal2camera_TransMatix)
    cam_position =  camera2gimbal_TransMatix @ gimbal_position

    return cam_position

def cal_dis(x1,x2,y1,y2, gimbalPose: GimbalState , uavLocalPos: PoseStamped , uavHomePosZ):
    '''
    # Calculate the distance between the camera and the object as an observation value D
    parameters:  
        --x1,x2,y1,y2: Diagonal image coordinates of two corner points
        --gimbalPos: The roll,pitch,yaw of the gimbal
        --cameraHeight: The height of the camera
    output:   
        --dis: the distance between the camera and the object
    '''

    a1 = imgCoord2camCoord(x1,y1)
    a2 = imgCoord2camCoord(x2,y2)
    center_vector = (a1 + a2)/(np.linalg.norm(a1 + a2))
    center_vector = np.append(center_vector,1)
    # print("center_vector:",center_vector)
    vector_in_inertial = camCoord2inertialCoord(center_vector,gimbalPose,uavLocalPos)

    angle_with_inertialZ = np.arccos(vector_in_inertial[2])
    dis = (uavLocalPos.pose.position.z - uavHomePosZ - Constants.DELTA_HEIGHT.value) / np.abs(np.cos(angle_with_inertialZ))

    return dis

def cal_pos(x1,x2,y1,y2, gimbalPose: GimbalState , uavLocalPos: PoseStamped , uavHomePosZ):
    '''
    # Calculate the distance between the camera and the object as an observation value D
    parameters:  
        --x1,x2,y1,y2: Diagonal image coordinates of two corner points
        --gimbalPos: The roll,pitch,yaw of the gimbal
        --cameraHeight: The height of the camera
    output:   
        --dis: the distance between the camera and the object
    '''

    a1 = imgCoord2camCoord(x1,y1)
    a2 = imgCoord2camCoord(x2,y2)
    center_vector = (a1 + a2)/(np.linalg.norm(a1 + a2))
    center_vector = np.append(center_vector,1)
    # print("center_vector:",center_vector)
    vector_in_inertial = camCoord2inertialCoord(center_vector,gimbalPose,uavLocalPos)

    angle_with_inertialZ = np.arccos(vector_in_inertial[2])
    dis = (uavLocalPos.pose.position.z - uavHomePosZ - Constants.DELTA_HEIGHT.value) / np.abs(np.cos(angle_with_inertialZ))
    vector_in_inertial[:3] *= dis
    # print(vector_in_inertial)
    return vector_in_inertial

def find_nearest_time(target_time, timestamps):
    """
    # Find the timestamp index closest to target_time in timestamps
    parameters:
        --target_time : reference time
        --timestamps: A list of timestamps waiting to be aligned
    output: 
        -- The index value of the timestamp with the smallest interval from the reference time
    """
    return min(range(len(timestamps)), key=lambda i: abs(timestamps[i] - target_time))


if __name__ == '__main__':

    rospy.init_node("EKF_location_node",anonymous=True)

    # rospy.Subscriber("/mono_location/cornersInfo",corners,callback,queue_size=5)

    rate = rospy.Rate(20)

    # -----Configure the bag file path and target topic------

    bag_file = '~.bag'
    corner_topic = '/mono_location/cornersInfo'
    gimbal_topic = '/gimbal_control_node/gimbal_state'
    home_topic = '/mavros/home_position/home'
    local_topic = '/mavros/local_position/pose'
    car_local_topic = '/mavros_car/local_position/pose'

    corner_data = []
    gimbal_data = []
    home_data = []
    local_data = []
    car_local_data = []
    aligned_data = []

    # ---Reads data from the bag for the specified topic---
    try:
        with rosbag.Bag(bag_file, 'r') as bag:
            rospy.loginfo(f"Reading messages from {corner_topic},{gimbal_topic},{home_topic},{local_topic},{car_local_topic} in {bag_file}...")
            for topic, msg, t in bag.read_messages(topics=[corner_topic,gimbal_topic,home_topic,local_topic,car_local_topic]):
                if topic == corner_topic:
                    corner_data.append((t.to_sec(),msg))
                elif topic == gimbal_topic:
                    gimbal_data.append((t.to_sec(),msg))
                elif topic == home_topic:
                    home_data.append((t.to_sec(),msg))
                elif topic == local_topic:
                    local_data.append((t.to_sec(),msg))
                elif topic == car_local_topic:
                    car_local_data.append((t.to_sec(),msg))
        rospy.loginfo(f"Extracted {len(corner_data)} corner messages;\
                        {len(gimbal_data)} gimbal messages;\
                        {len(home_data)} home messages;\
                        {len(local_data)} local messages;\
                        {len(car_local_data)} car_local messages;")
    except Exception as e:
        rospy.logerr(f"Failed to read bag file: {e}")


    # ---Timestamp alignment---
    gimbal_timestamps = [x[0] for x in gimbal_data]
    home_timestamps = [x[0] for x in home_data]
    local_timestamps = [x[0] for x in local_data]
    car_local_timestamps = [x[0] for x in car_local_data]
    
    for corner_time, corner_msg in corner_data:
        nearest_gimbal_idx = find_nearest_time(corner_time, gimbal_timestamps)
        nearest_home_idx = find_nearest_time(corner_time, home_timestamps)
        nearest_local_idx = find_nearest_time(corner_time, local_timestamps)
        nearest_car_local_idx = find_nearest_time(corner_time, car_local_timestamps)
        aligned_data.append((corner_msg,gimbal_data[nearest_gimbal_idx][1],home_data[nearest_home_idx][1],\
                             local_data[nearest_local_idx][1],car_local_data[nearest_car_local_idx][1]))
    rospy.loginfo(f"Aligned {len(aligned_data)} pairs of data.")

   
    # ---UKF initialization---
    dt = 0.05
    # create sigma points to use in the filter. This is standard for Gaussian processes
    sigmaPoints = MerweScaledSigmaPoints(13, alpha=.01, beta=2., kappa=-10)
    locationUKF = UKF(dim_x=13,dim_z=9,dt=dt,fx=Fx,hx=Hx,points=sigmaPoints)

    #Initialize the state vector : X
    locationUKF.x = np.array([0.5,0.001,0.5,0.001,4,0.001,170*np.pi/180,1*np.pi/180,10*np.pi/180\
                              ,1*np.pi/180,10*np.pi/180,1*np.pi/180,0.5])
   
    #Initialize the covariance matrix :  P
    locationUKF.P *= 0.01
    
    #Initialize the Process noise matrix : Q
    locationUKF.Q[0:12,0:12] = Q_discrete_white_noise(dim=2,dt=dt,var=0.1,block_size=6)
    locationUKF.Q[12][12] = 0.0001
    
    #Initialize the Measurement noise matrix : R
    locationUKF.R *=0.001
    locationUKF.R[8][8] = 0.0001
    # locationUKF.R[9][9] = 0.0001
    # locationUKF.R[10][10] = 0.0001
    

    # -------Data drawing-----
    t_data = []
    x_data = []
    y_data = []
    z_data = []
    alpha_data = []
    beta_data = []
    theta_data = []
    l_data = []
    z_l_data = []
    local_x_data = []
    local_y_data = []
    local_z_data = []
    uav_local_x_data = []
    uav_local_y_data = []
    uav_local_z_data = []
    car_local_x_data = []
    car_local_y_data = []
    car_local_z_data = []
    ''' ---Two-dimensional drawing---
    # plt.ion()  
    # fig, ax = plt.subplots()
    # # Set image style
    # # line1, = ax.plot([], [], 'r-', label="X") 
    # # line2, = ax.plot([], [], 'b-', label="Y")  
    # # line3, = ax.plot([], [], 'g-', label="Z")  
    # line4, = ax.plot([], [], 'r-', label="real size")  
    # line5, = ax.plot([], [], 'g-', label="estimated size l")
    # # line4, = ax.plot([], [], 'y-', label="alpha")  
    # # line5, = ax.plot([], [], 'orange', label="beta")
    # # line6, = ax.plot([], [], 'purple', label="theta")  


    # ax.set_xlim(0, 10) 
    # ax.set_ylim(0, 10) 
    # # ax.set_xlabel("time(s)", fontsize=14)
    # # ax.set_ylabel("error(rad)", fontsize=14)
    # # ax.legend()
    # # ax.grid()
    # # fig.suptitle("UKF", fontsize=16)
    # ax.set_xlabel("time(s)", fontsize=30)
    # ax.set_ylabel("error(m)", fontsize=30)
    # ax.legend(fontsize=24)
    # ax.grid()
    # ax.tick_params(axis='both', labelsize=24)
    # # fig.suptitle("EKF", fontsize=30)
    '''
    #---start UKF---
    start_time = time.time()
    for corner_msg,gimbal_msg,home_msg,local_msg,car_local_msg in aligned_data:

        # Two-dimensional drawing ：Two-dimensional plotting data
        
        # x_data.append(float(locationUKF.x[0])-2*np.cos(np.pi/6*current_time))
        # y_data.append(float(locationUKF.x[2])-2*np.sin(np.pi/6*current_time))
        # z_data.append(float(locationUKF.x[4])-5)
        # alpha_data.append(float(locationUKF.x[6])-180*np.pi/180)
        # beta_data.append(float(locationUKF.x[8]))
        # theta_data.append(float(locationUKF.x[10]))
        # l_data.append(float(locationUKF.x[12]))
        # z_data.append(0.4)
        # z_l_data.append(float(locationUKF.x[4])/float(locationUKF.x[12])) 

        
        #  Two-dimensional drawing ：Two-dimensional plotting data
        # inertial_positon = cal_pos(msg.corner1.x,msg.corner3.x,msg.corner1.y,msg.corner3.y,gimbal_msg,local_msg,home_msg.position.z)
        # cam_position = inertialCoord2camCoord(inertial_positon,gimbal_msg,local_msg)
        # x_data.append(float(cam_position[0]))
        # y_data.append(float(cam_position[1]))
        # z_data.append(float(cam_position[2]))
        # l_data.append(float(locationEKF.x[12]))
        # z_l_data.append(float(locationEKF.x[4])/float(locationEKF.x[12])) 

        Z_ = np.array([corner_msg.corner1.x-Constants.U0.value,corner_msg.corner1.y-Constants.V0.value,\
                       corner_msg.corner2.x-Constants.U0.value,corner_msg.corner2.y-Constants.V0.value,\
                       corner_msg.corner3.x-Constants.U0.value,corner_msg.corner3.y-Constants.V0.value,\
                       corner_msg.corner4.x-Constants.U0.value,corner_msg.corner4.y-Constants.V0.value,\
                       cal_dis(msg.corner1.x,msg.corner3.x,msg.corner1.y,msg.corner3.y,gimbal_msg,local_msg,home_msg.position.z)])

        locationUKF.predict()
        locationUKF.update(Z_,hx=Hx)

        camera_positon = np.array([float(locationUKF.x[0]),float(locationUKF.x[2]),float(locationUKF.x[4]),1])
        inertial_positon = camCoord2inertialCoord(camera_positon,gimbal_msg,local_msg)
        # inertial_positon = cal_pos(msg.corner1.x,msg.corner3.x,msg.corner1.y,msg.corner3.y,gimbal_msg,local_msg,home_msg.position.z)
        local_positon = inertialCoord2localCoord(inertial_positon,local_msg)

        local_x_data.append(local_positon[0])
        local_y_data.append(local_positon[1])
        local_z_data.append(local_positon[2])
        uav_local_x_data.append(local_msg.pose.position.x )
        uav_local_y_data.append(local_msg.pose.position.y )
        uav_local_z_data.append(local_msg.pose.position.z )
        car_local_x_data.append(car_local_msg.pose.position.x )
        car_local_y_data.append(car_local_msg.pose.position.y )
        car_local_z_data.append(car_local_msg.pose.position.z )
        
        l_data.append(float(locationUKF.x[12]))
        local_x_data_s = gaussian_filter1d(local_x_data, sigma=5)
        local_y_data_s = gaussian_filter1d(local_y_data, sigma=5)
        local_z_data_s = gaussian_filter1d(local_z_data, sigma=5)
        l_data_s = gaussian_filter1d(l_data, sigma=5)
        # car_local_x_data.append(local_msg.pose.position.x )
        # car_local_y_data.append(local_msg.pose.position.y)
        # car_local_z_data.append(local_msg.pose.position.z )
        x_data.append(local_x_data_s[-1] - car_local_x_data[-1])
        y_data.append(local_y_data_s[-1] - car_local_y_data[-1])
        z_data.append(local_z_data_s[-1] - car_local_z_data[-1])
        z_l_data.append(l_data_s[-1] - 0.4)
        
    '''    #--- Two-dimensional drawing:---
    #     current_time = time.time() - start_time
    #     t_data.append(current_time)
    #     # # update data
    #     # line1.set_data(t_data,x_data)
    #     # line2.set_data(t_data,y_data)
    #     # line3.set_data(t_data,z_data)
    #     # line4.set_data(t_data,local_x_data)
    #     # line5.set_data(t_data,local_y_data)
    #     # line6.set_data(t_data,local_z_data)
    #     # line4.set_data(t_data,alpha_data)
    #     # line5.set_data(t_data,beta_data)
    #     # line6.set_data(t_data,theta_data)
    #     line4.set_data(t_data,z_data)
    #     line5.set_data(t_data,l_data_s)
        
    #     # # Dynamically adjust the coordinate range
    #     ax.set_xlim(0, max(10, current_time))  # x Axis dynamic expansion
    #     ax.set_ylim(min(min(l_data_s),min(z_data)) - 0.1,\
    #                     max(max(l_data_s),max(z_data)) + 0.1)
    #     # ax.set_ylim(min(min(alpha_data),min(beta_data),min(theta_data)) - 1,\
    #     #                 max(max(alpha_data),max(beta_data),max(theta_data)) + 1)  
    #     # ax.set_ylim(min(min(x_data),min(y_data),min(z_data)) - 1,\
    #     #              max(max(x_data),max(y_data),max(z_data)) + 1)  
        
    #     
    #     # plt.pause(0.005)  # Pause for a moment to facilitate the display of updates
    #     rate.sleep()
    # # Two-dimensional drawing :
    # plt.ioff()
    # plt.show()'''

    # # ---Three-dimensional drawing---
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(local_x_data[0], local_y_data[0], local_z_data[0],
        c='blue',  marker='o'
    )
    ax.plot(local_x_data_s, local_y_data_s, local_z_data_s,
        c='blue', label='target localization estimation', linestyle='--', linewidth=3)
    
    ax.scatter(car_local_x_data[0], car_local_y_data[0], car_local_z_data[0],
        c='red',  marker='o'
    )
    ax.plot(car_local_x_data, car_local_y_data, car_local_z_data,
            label='true trajectory of target',color='red', linewidth=3)
    
    ax.plot(uav_local_x_data, uav_local_y_data, uav_local_z_data,
            label='true trajectory of UAV',color='green', linewidth=3)
    
    # Add legends and labels
    ax.set_title("3D Trajectory", fontsize=22)
    ax.set_xlabel("X (m)", fontsize=20)
    ax.set_ylabel("Y (m)", fontsize=20)
    ax.set_zlabel("Z (m)", fontsize=20)
    ax.legend(fontsize=18)
    ax.tick_params(axis='both', labelsize=16)


    plt.show()
    

    # Save the original data
    # data = [x_data, y_data, z_data, z_l_data]
    # with open('UKF_data.pkl', 'wb') as f:
    #     pickle.dump(data, f)
