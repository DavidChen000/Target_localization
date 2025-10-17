#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <deque>

#include "geometry_msgs/Point.h"
#include "yolov5_new/YoloRes.h"



yolov5_new::YoloRes LocalOrigin_;
yolov5_new::YoloRes LocalFiltered_;
std::deque<geometry_msgs::Point> windowPoints_;

bool updateLocalOrigin_;


void callback(const yolov5_new::YoloRes::ConstPtr &msg){
	
	LocalOrigin_ = *msg;
	updateLocalOrigin_ = true;

}

bool ZscoreFilter(float16_t alpha){
	
	geometry_msgs::Point mean;
	geometry_msgs::Point sum;
	geometry_msgs::Point SD;
    geometry_msgs::Point Zscore;
	
    //Calculate the mean in the current window
	for (const auto& point : windowPoints_) 
    {
        mean.x += point.x;
        mean.y += point.y;
        mean.z += point.z;
    } 
        
    size_t count = windowPoints_.size();
    if (count > 0) 
    {
        mean.x /= count;
        mean.y /= count;
        mean.z /= count;
    }
    
    std::cout<<"count"<<count<<std::endl;
    std::cout<<"mean"<<mean<<std::endl;
        
   //Calculate the Standard Deviation in the current window
	for(const auto& point : windowPoints_)
    {
        sum.x += std::pow(point.x-mean.x,2);
        sum.y += std::pow(point.y-mean.y,2);
        sum.z += std::pow(point.z-mean.z,2);
    }
    SD.x = std::sqrt(sum.x/windowPoints_.size());
    SD.y = std::sqrt(sum.y/windowPoints_.size());
    SD.z = std::sqrt(sum.z/windowPoints_.size());

    std::cout<<"SD"<<SD<<std::endl;

     //Calculate the alpha by Z-score
    Zscore.x = std::abs((LocalOrigin_.logo_pos.x-mean.x)/SD.x);
    Zscore.y = std::abs((LocalOrigin_.logo_pos.y-mean.y)/SD.y);
    Zscore.z = std::abs((LocalOrigin_.logo_pos.z-mean.z)/SD.z);

    LocalFiltered_.QR_pos.x = Zscore.x ;
    LocalFiltered_.QR_pos.y = Zscore.y ;
    LocalFiltered_.QR_pos.z = Zscore.z ;
    std::cout<<"Zscore4"<<Zscore<<std::endl;
    
    if (Zscore.x>3 || Zscore.y>3 )//outlier
    {
        LocalFiltered_.logo_pos.x = LocalOrigin_.logo_pos.x ;
        LocalFiltered_.logo_pos.y = LocalOrigin_.logo_pos.y ;
        LocalFiltered_.logo_pos.z = LocalOrigin_.logo_pos.z ;
        
        return false;
    }
    else
    {
        LocalFiltered_.logo_pos.x = alpha *LocalOrigin_.logo_pos.x + (1-alpha)*LocalFiltered_.logo_pos.x;
        LocalFiltered_.logo_pos.y = alpha *LocalOrigin_.logo_pos.y + (1-alpha)*LocalFiltered_.logo_pos.y;
        LocalFiltered_.logo_pos.z = alpha *LocalOrigin_.logo_pos.z + (1-alpha)*LocalFiltered_.logo_pos.z;
        return true;
    }

    //Calculate the alpha by Z-score
    // alphaZ.x = 1 / (std::pow( M_E , std::abs((LocalOrigin_.logo_pos.x-mean.x)/SD.x)));
    // alphaZ.y = 1 / (std::pow( M_E , std::abs((LocalOrigin_.logo_pos.y-mean.y)/SD.y)));
    // alphaZ.z = 1 / (std::pow( M_E , std::abs((LocalOrigin_.logo_pos.z-mean.z)/SD.z)));

    // std::cout<< "mean"<< std::endl;
    // std::cout<< mean<< std::endl;
    // std::cout<< "SD"<< std::endl;
    // std::cout<< SD<< std::endl;
    // std::cout<< "alphaZ"<< std::endl;
    // std::cout<< alphaZ<< std::endl;

    //EMA filter
    // LocalFiltered_.logo_pos.x = alphaZ.x *LocalOrigin_.logo_pos.x + (1-alphaZ.x)*windowPoints_.back().x;
    // LocalFiltered_.logo_pos.y = alphaZ.y *LocalOrigin_.logo_pos.y + (1-alphaZ.y)*windowPoints_.back().y;
    // LocalFiltered_.logo_pos.z = alphaZ.z *LocalOrigin_.logo_pos.z + (1-alphaZ.z)*windowPoints_.back().z;
}


// void applyZEMAFilter(){
	
//     if (windowPoints_.size() < windowSize_)
//     {
//         if (LocalOrigin_.logo_pos.x != 0 )
//         {
//             windowPoints_.push_back(LocalOrigin_.logo_pos);
//             // std::cout<< "windowPoints_"<< std::endl;
//             // std::cout<< windowPoints_.back()<< std::endl;
//         }
//     }

//     else
//     {
// 		windowPoints_.pop_front();
//         ZEMAFilter();
//         windowPoints_.push_back(LocalFiltered_.logo_pos);
//         // std::cout<< "windowPoints_2"<< std::endl;
//         // std::cout<< windowPoints_.back()<< std::endl;
// 	}
// }

void applyEMAFilter(float16_t alpha,size_t windowSize_){
    	
    if (windowPoints_.size() < windowSize_)
    {
        if (LocalOrigin_.logo_pos.x != 0 )
        {
            windowPoints_.push_back(LocalOrigin_.logo_pos);
        }
    }
    else
    {
		windowPoints_.pop_front();
        if (ZscoreFilter(alpha))
        {
           windowPoints_.push_back(LocalFiltered_.logo_pos);
        }
        else
        {
            windowPoints_.push_back(windowPoints_.back());
        }
        
        // std::cout<< "windowPoints_2"<< std::endl;
        // std::cout<< windowPoints_.back()<< std::endl;
	}

	// LocalFiltered_.logo_pos.x = alpha *LocalOrigin_.logo_pos.x + (1-alpha)*LocalFiltered_.logo_pos.x;
    // LocalFiltered_.logo_pos.y = alpha *LocalOrigin_.logo_pos.y + (1-alpha)*LocalFiltered_.logo_pos.y;
    // LocalFiltered_.logo_pos.z = alpha *LocalOrigin_.logo_pos.z + (1-alpha)*LocalFiltered_.logo_pos.z;
    
}


int main(int argc,char** argv){

	ros::init(argc,argv,"EMA_test_node");
	ros::NodeHandle nh;
	ros::Subscriber tarLocaLocalSub_ = nh.subscribe("target_location_node/tarLocation_LocalCoord",10,callback);
	ros::Publisher LocalFilteredPub_ = nh.advertise<yolov5_new::YoloRes>("target_location_node/LocalFiltered",10);
	
	ros::Rate loop_rate(20);
	size_t windowSize_ = 200;
    float16_t alpha = 0.7;
	LocalFiltered_.logo_pos.x = 0;
    LocalFiltered_.logo_pos.y = 0;
    LocalFiltered_.logo_pos.z = 0;

	while(ros::ok()){
	
		applyEMAFilter(alpha,windowSize_);
		//记得打上时间戳?

		LocalFilteredPub_.publish(LocalFiltered_);
	
		ros::spinOnce();
	 
		loop_rate.sleep();
    
	}
return 0;
}