#include "gimbal_viservo/gimbal_viservo_controller.h"

gimPIDController::gimPIDController(const ros::NodeHandle &nh,double yawP, double yawI, double yawD, 
                                            double pitchP, double pitchI, double pitchD,
                                            double integralLimitX, double integralLimitY):
                                                                        nh_(nh),
                                                                        yawP_(yawP),
                                                                        yawI_(yawI),
                                                                        yawD_(yawD),
                                                                        pitchP_(pitchP),
                                                                        pitchI_(pitchI),
                                                                        pitchD_(pitchD),
                                                                        integralLimitX_(integralLimitX),
                                                                        integralLimitY_(integralLimitY),
                                                                        prevErrorX_(0.0),
                                                                        prevErrorY_(0.0),
                                                                        integralX_(0.0),
                                                                        integralY_(0.0)                                                           
{
    sub_ = nh_.subscribe<yolov5_new::DetectBox>(
        "/target_detect_node/detectBox",10,&gimPIDController::callback,this
        );

    pub_ = nh_.advertise<gimbal_viservo::SpeedControl>(
        "/gimbal_viservo_controller_node/reqSpeed",10
        );

    lastTimeX_ = std::chrono::high_resolution_clock::now();
    lastTimeY_ = std::chrono::high_resolution_clock::now();
    // printf("yawP_ %.2lf;yawI_ %2lf;yawD_ %2lf;pitchP_ %2lf;pitchI_ %2lf;pitchD_ %2lf",
    //         yawP_,yawI_,yawD_,pitchP_,pitchI_,pitchD_);
}

gimPIDController::~gimPIDController()
{
}


void gimPIDController::callback(const yolov5_new::DetectBox::ConstPtr &msg)
{
    int16_t targetX ;
    int16_t targetY ;
    if (msg->logo_update && msg->qr_update)
    {
        targetX = (msg->logo_center_x + msg->qr_center_x)/2;
        targetY = (msg->logo_center_y + msg->qr_center_y)/2;
    }
    else if (msg->logo_update && !msg->qr_update)
    {
        targetX = msg->logo_center_x;
        targetY = msg->logo_center_y;
    }
    else if (!msg->logo_update && msg->qr_update)
    {
        targetX = msg->qr_center_x;
        targetY = msg->qr_center_y;
    }
    else
    {
        targetX = (msg->logo_center_x + msg->qr_center_x)/2;
        targetY = (msg->logo_center_y + msg->qr_center_y)/2;
    }
    
    gimbal_viservo::SpeedControl gimbalSpeedMsg;

    //The gimbal stop turning when the target is lost
    if (msg->logo_update || msg->qr_update)
    {
        gimbalSpeedMsg.yaw_speed = calYawSpeed(targetX);
        gimbalSpeedMsg.pitch_speed = calPitchSpeed(targetY);
    }
    else
    {
        gimbalSpeedMsg.yaw_speed = 0;
        gimbalSpeedMsg.pitch_speed = 0;
        ROS_INFO_STREAM_THROTTLE(2,"Target has been lost!");
    }
    

    // printf("yaw_speed %d;pitch_speed %d",gimbalSpeedMsg.yaw_speed,gimbalSpeedMsg.pitch_speed);
    // ROS_INFO_STREAM_THROTTLE(1,"Gimbal is moving Yaw speed ("<<gimbalSpeedMsg.yaw_speed<<"),Pitch speed ("<<gimbalSpeedMsg.pitch_speed<<")");

    pub_.publish(gimbalSpeedMsg);


}


int gimPIDController::calYawSpeed(const int16_t &measured_X)
{
    auto now =std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elaspsed = now - lastTimeX_;
    double deltaTime = elaspsed.count();

    //PID proportion,intergral,derivative terms
    int error = -(IMAGE_W/2 - measured_X);

    /*Prevent saturation of the integral term*/
    integralX_ += error*deltaTime;
    if (integralX_ > integralLimitX_)
    {
        integralX_ = integralLimitX_;
    }
    else if (integralX_ < integralLimitX_)
    {
        integralX_ = -integralLimitX_;
    }
    
    double derivativeTrem = (error - prevErrorX_)/deltaTime;

    //Saving last error and time
    prevErrorX_ = error;
    lastTimeX_ = now;

    //Calculating optput(-100~100)
    if (std::abs(error)>5)
    {
        float output = (yawP_*error)+(yawI_*integralX_)+(yawD_*derivativeTrem);
        // return static_cast<int>(std::clamp(static_cast<int>(output),-100,100));
        return static_cast<int>(output);
    }
    else
    {   
        ROS_INFO_STREAM_THROTTLE(2,"The yaw axis is pointed at the target!");
        return 0;
    }
}


int gimPIDController::calPitchSpeed(const int16_t &measured_Y)
{
    auto now =std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elaspsed = now - lastTimeY_;
    double deltaTime = elaspsed.count();

    //PID proportion,intergral,derivative terms
    int error = IMAGE_H/2 - measured_Y;
    /*Prevent saturation of the integral term*/
    integralY_ += error*deltaTime;
    if (integralY_ > integralLimitY_)
    {
        integralY_ = integralLimitY_;
    }
    else if (integralY_ < integralLimitY_)
    {
        integralY_ = -integralLimitY_;
    }

    double derivativeTrem = (error - prevErrorY_)/deltaTime;

    //Saving last error and time
    prevErrorY_ = error;
    lastTimeY_ = now;

    if (std::abs(error)>5)
        {
            float output = (pitchP_*error)+(pitchI_*integralY_)+(pitchD_*derivativeTrem);
            // return static_cast<int>(std::clamp(static_cast<int>(output),-100,100));
            return static_cast<int>(output);
    
        }
        else
        {   
            ROS_INFO_STREAM_THROTTLE(2,"The pitch axis is pointed at the target!");
            return 0;
        }
}


