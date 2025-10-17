#ifndef GIMBAL_ORIENTATION_TRACK_H
#define GIMBAL_ORIENTATION_TRACK_H

#include <ros/ros.h>

// include ROS msg,srv
#include <geometry_msgs/Point32.h>
#include "gimbal_viservo/ModeSwitch.h"
#include "gimbal_viservo/TargetPoint.h"
#include "gimbal_viservo/GimbalState.h"

// #include "gimbal_viservo/gimabl_kinematics.h"
#include <cmath>
#include <eigen3/Eigen/Dense>

//Gimbal's parameters uint:mm
#define HEIGHT 67
#define LENGTH 40
#define WIDTH1 20
#define WIDTH2 30

enum GIMBAL_MODE
{
    SPEED_MODE = 1,
    ANGLE_MODE,
    RETURN_MODE
};




class gimbalOrienTrack
{
private:
    
    void gimbalStateCallback(const gimbal_viservo::GimbalState &msg); //update currentGimbalState_
    
    

    ros::NodeHandle nodeHandle_;

    ros::Subscriber gimbalStateSub_;
    
   
    ros::ServiceServer targetPointTrackService_;
    ros::ServiceClient gimbalControlSClient_;


    gimbal_viservo::GimbalState currentGimbalState_;

    // exp_yaw: -1350~0~1350(-135.0°~0°~135.0°): 
    // exp_pitch: -900~0~250(-90.0°~0°~25.0°): 
    gimbal_viservo::ModeSwitch expAngle_;

    geometry_msgs::Point32 targetPoint_;

    bool calExpAngle(const geometry_msgs::Point32 &tarPoint); //calculate the expected Pitch and Yaw

    // float yaw_;
    // float pitch_;
    // float roll_;
    // 目标点回调函数改变成员变量赋值，主循环一直在call mode_switch_server 服务端

    
public:

    gimbalOrienTrack(const ros::NodeHandle &nh);
    ~gimbalOrienTrack();


     //update targetPoint_ , Calculate angle and  call mode_switch_server 
    bool targetPointCallback(gimbal_viservo::TargetPoint::Request &req,
                             gimbal_viservo::TargetPoint::Response &res);
    

    //calculate the gimbal DH transformational matrix using yaw_、pitch_、roll_
    // Eigen::Matrix4d getGimbalTransMatrix() ;
    // Eigen::Matrix4f calGimbalCenter();
    // int calExpYaw_();
    // int calExpPitch_();
};



#endif 