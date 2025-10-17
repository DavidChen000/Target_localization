#include "gimbal_viservo/gimbal_orientation_track.h"

using namespace std;

gimbalOrienTrack::gimbalOrienTrack(const ros::NodeHandle &nh)
        : nodeHandle_(nh),
          expAngle_(),
          targetPoint_()
{  
    gimbalStateSub_ = nodeHandle_.subscribe("gimbal_state",10,&gimbalOrienTrack::gimbalStateCallback,this);
    targetPointTrackService_ = nodeHandle_.advertiseService("PointTrackSer",&gimbalOrienTrack::targetPointCallback,this);
    gimbalControlSClient_ = nodeHandle_.serviceClient<gimbal_viservo::ModeSwitch>("mode_switch_server");
}


void gimbalOrienTrack::gimbalStateCallback(const gimbal_viservo::GimbalState &msg)
{
    currentGimbalState_.current_pitch = msg.current_pitch; 
    currentGimbalState_.current_yaw = msg.current_yaw;
    currentGimbalState_.current_roll = msg.current_roll;
    currentGimbalState_.current_pitch_speed = msg.current_pitch_speed; 
    currentGimbalState_.current_yaw_speed = msg.current_yaw_speed;
    currentGimbalState_.current_roll_speed = msg.current_roll_speed;
}

bool gimbalOrienTrack::targetPointCallback(gimbal_viservo::TargetPoint::Request &req,
                                            gimbal_viservo::TargetPoint::Response &res) 
{
    targetPoint_.x = req.x;
    targetPoint_.y = req.y;
    targetPoint_.z = req.z;

    expAngle_.request.mode = ANGLE_MODE;
    calExpAngle(targetPoint_);
    expAngle_.request.exp_yaw = 10*expAngle_.request.exp_yaw;
    
    // printf("exp_yaw %i .",expAngle_.request.exp_yaw);

    expAngle_.request.exp_pitch = 10*expAngle_.request.exp_pitch;

    // printf("exp_pitch %i .",expAngle_.request.exp_pitch);
   
    if(gimbalControlSClient_.call(expAngle_))
    {
        ROS_INFO_STREAM_THROTTLE(1,"Going to target point");
        res.success = true;
        return true;
    }
    else
    {
        ROS_ERROR("Failed to go to target point!");
        res.success = false;
        return false;
    }
    
}

bool gimbalOrienTrack::calExpAngle(const geometry_msgs::Point32 &tarPoint)
{  
    // printf("target %f,%f,%f .",tarPoint.x,tarPoint.y,tarPoint.z);

    double YawRadian = atan2(tarPoint.y,tarPoint.x);
    double Yaw = YawRadian*180.0/M_PI;

    expAngle_.request.exp_yaw = static_cast<int>(round(Yaw));

    double Pitch;
    double Nume = tarPoint.z + HEIGHT;
    double Deno = pow((tarPoint.x-(WIDTH2-WIDTH1)*cos(YawRadian)),2)+pow((tarPoint.y-(WIDTH2-WIDTH1)*sin(YawRadian)),2);
    printf("Nume: %f;Deno: %f",Nume,Deno);



    
    if((Deno == 0.0) && (Nume > 0.0))
    {
        printf("Deno == 0 && Nume >0");
        Pitch = 25.00;
    }
    else if((Deno == 0.0) && (Nume < 0.0))
    {
        printf("Deno == 0 && Nume <0");
        Pitch = -90.00;
    }
    else
    {
        Pitch = atan2(Nume,sqrt(Deno))*180.0/M_PI;
    }

    printf("Pitch : %f",Pitch);

    expAngle_.request.exp_pitch = static_cast<int>(round(Pitch));

    return true;
}

gimbalOrienTrack::~gimbalOrienTrack()
{
}


