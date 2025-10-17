#ifndef GIMBAL_VISERVO_CONTROLLER_H
#define GIMBAL_VISERVO_CONTROLLER_H

#include <ros/ros.h>
#include <cmath>

#include <iostream>
#include <algorithm>
#include <chrono>

#include "yolov5_new/DetectBox.h"
#include "gimbal_viservo/SpeedControl.h"
#include "gimbal_viservo/ModeSwitch.h"


const int IMAGE_W = 640;
const int IMAGE_H = 480;


// using namespace std;

class gimPIDController
{
private:

    int calYawSpeed(const int16_t &measured_X);
    int calPitchSpeed(const int16_t &measured_Y);

    ros::NodeHandle nh_;

    ros::Subscriber sub_;

    ros::Publisher pub_;

    //pid paramter
    const double yawP_,yawI_,yawD_;
    const double pitchP_,pitchI_,pitchD_;
    const double integralLimitX_,integralLimitY_;

    //Previous error
    float prevErrorX_; 
    float prevErrorY_;

    std::chrono::high_resolution_clock::time_point lastTimeX_;
    std::chrono::high_resolution_clock::time_point lastTimeY_;

    //cumulative intergral
    float integralX_;
    float integralY_;

public:
    gimPIDController(const ros::NodeHandle &nh,double yawP, double yawI, double yawD, 
                                            double pitchP, double pitchI, double pitchD,
                                            double integralLimitX_, double integralLimitY_);
                                            
    ~gimPIDController();

    
    void callback(const yolov5_new::DetectBox::ConstPtr &msg);


    // gimbal_viservo::SpeedControl getControllRate(const float &current_x,const float &current_y)

};



#endif