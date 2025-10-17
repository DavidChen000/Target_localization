#include <opencv2/core/types_c.h>
#include <opencv2/opencv.hpp>
#include <iostream>  
#include <time.h>
#include <math.h>
#include <iostream>  
#include <sstream>
#include <set>

#include "mono_location/corners.h"
#include "mono_location/GimbalState.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>


using namespace cv;
using namespace std;

RNG rng(12345);

float getDistance(Point pointO, Point pointA);
float getAngle(Point pointM, Point pointL, Point pointR);
float getDist_P2L(Point pointP, Point pointA, Point pointB);
int list_connor(int i1, int i2, int i3);

Mat imgPreprocess(const Mat& rawImg);
vector<vector<Point>> contourExtract(const Mat& binaryImg, Mat& drawing);
vector<vector<Point>> connorExtract(const vector<vector<Point>>& rectContours, Mat& drawing);
void imageCallback(const sensor_msgs::ImageConstPtr &msg);


Mat srcImage0; // 用于存储每帧原始图像

int main(int argc, char *argv[])
{
	//读取图像
	// Mat srcImage0 = imread("/home/davidwsl/test_ws/src/mono_location/src/4.png", cv::IMREAD_COLOR);
    // if (srcImage0.empty()) {
    //     std::cerr << "Error: Could not load image!" << std::endl;
    //     return -1;  
    // }  
	ros::init(argc,argv,"get_points_node");
	ros::NodeHandle nh_;
	// 创建图像传输对象和订阅者
    image_transport::ImageTransport it(nh_);
    ros::Subscriber imgSub_ = nh_.subscribe("/video_capture_node/image_raw", 30, imageCallback);
	ros::Publisher cornersPub_ = nh_.advertise<mono_location::corners>("/mono_location/cornersInfo",10);
	ros::Rate loopRate(20);
	ros::AsyncSpinner spinner(3);
    spinner.start();

	//读取视频
	// string videoPath = "/home/davidwsl/test_ws/src/script/output_video3.avi";
	// VideoCapture capture(videoPath);
	// if (!capture.isOpened())
	// {
	// 	cerr<<"Error:Cannot open video file! "<<endl;
	// 	return -1;
	// }
	// double fps = capture.get(cv::CAP_PROP_FPS);          // 获取帧率
    // int frameWidth = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));  // 宽度
    // int frameHeight = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT)); // 高度
    // cout << "Video Info: " << fps << " FPS, " << frameWidth << "x" << frameHeight << endl;

	
	vector<vector<Point>> cornersNew_;

	while (ros::ok())
	{
		// capture >> srcImage0;
		// 检查是否到达视频末尾
        // if (srcImage0.empty()) {
        //     cout << "Video ended or cannot read frame!" <<endl;
		// 	char key1 = static_cast<char>(cv::waitKey(0));
        //     if (key1 == 'q' || key1 == 27) { // 'q' 或 'ESC' 键退出
        //     	break;
		// 	}
		// 	// break;
		// }
		
		if (!srcImage0.empty())
		{
			//图像预处理
			Mat edges = imgPreprocess(srcImage0);

			//矩形轮廓检测与提取
			vector<vector<Point>> RectContours = contourExtract(edges,srcImage0);
			
			//四边形角点提取
			vector<vector<Point>> RectConnors = connorExtract(RectContours,srcImage0);
			imshow("Corners and Outlines", srcImage0);
			cv::waitKey(1);

			mono_location::corners cornersInfo_;
			if (RectConnors.size()!=0)
			{
				cornersNew_ = RectConnors;
			}
			else
			{
				ROS_INFO_STREAM_THROTTLE(2, "The corner of the rectangular box could not be found!");
			}

			if (cornersNew_.size())
			{

				cornersInfo_.corner1.x = static_cast<float>(cornersNew_[0][0].x);
				cornersInfo_.corner1.y = static_cast<float>(cornersNew_[0][0].y);
				cornersInfo_.corner2.x = static_cast<float>(cornersNew_[0][1].x);
				cornersInfo_.corner2.y = static_cast<float>(cornersNew_[0][1].y);
				cornersInfo_.corner3.x = static_cast<float>(cornersNew_[0][2].x);
				cornersInfo_.corner3.y = static_cast<float>(cornersNew_[0][2].y);
				cornersInfo_.corner4.x = static_cast<float>(cornersNew_[0][3].x);
				cornersInfo_.corner4.y = static_cast<float>(cornersNew_[0][3].y);

				cornersPub_.publish(cornersInfo_);	
			}

			
			// 按 'q' 或 'ESC' 键退出
			// char key = static_cast<char>(cv::waitKey(1)); // 根据帧率设置延迟时间
			// if (key == 'q' || key == 27) { // 'q' 或 'ESC' 键退出
			// 	break;
			// }

		}
		else{ROS_INFO_STREAM_THROTTLE(2, "Image is empty!");}

		ros::spinOnce();
		loopRate.sleep();
	}
		
	// 释放视频资源并关闭窗口
	// capture.release();
	cv::destroyAllWindows();

	return 0;
	//float distance = 0, distanceMax = 0;
	//Point connorPoint1, connorPoint2, connorPoint3, connorPoint4;
	//int j = 0, k = 0;
	//vector<float>Theta(30);
	//vector<Point>ConnorPoint(4);
	//for (k = 0; k < RectContours.size(); k++)
	//{//历遍每个轮廓找角点
	//	j = 0;
	//	for (i = 0; i < RectContours[k].size(); i++)
	//	{//历遍当个轮廓各点间夹角
	//		if (i == 0)
	//		{
	//			Theta[i] = getAngle(RectContours[k][i], RectContours[k][RectContours[k].size() - 1], RectContours[k][i + 1]);
	//		}
	//		else if (i == RectContours[k].size() - 1)
	//		{
	//			Theta[i] = getAngle(RectContours[k][i], RectContours[k][i - 1], RectContours[k][0]);
	//		}
	//		else
	//		{
	//			Theta[i] = getAngle(RectContours[k][i], RectContours[k][i - 1], RectContours[k][i + 1]);
	//		}
	//		if (Theta[i] / 3.1415 * 180 < 170)
	//		{//两点间夹角小于170度
	//			if (getDistance(RectContours[k][i], ConnorPoint[0])>10 && getDistance(RectContours[k][i], ConnorPoint[1])>10
	//				&& getDistance(RectContours[k][i], ConnorPoint[2])>10 && getDistance(RectContours[k][i], ConnorPoint[3])>10)
	//			{//新找到的角点与已经保存的角点间距离要大于10
	//				ConnorPoint[j] = RectContours[k][i]; //四个角点
	//			//	circle(drawing, RectContours[k][i], 3, Scalar(255, 255, 255), FILLED, LINE_AA);
	//				circle(drawing, ConnorPoint[j], 3, Scalar(255, 255, 255), FILLED, LINE_AA);
	//				//每个四边形的角点显示逻辑这里还是有些问题
	//				cout << "\n轮廓 " << j << "  的四个角点坐标分别为：\n" << ConnorPoint[0] << ConnorPoint[1] << ConnorPoint[2] << ConnorPoint[3] << endl;
	//				j++;
	//			}
	//		}
	//	}
	//
	/********透视变换过程*************************************************************************/
	//检测是否是四边形，很多图片检测不到
	//	if (approx.size() != 4)
	//	{
	//		std::cout << "The object is not quadrilateral（四边形）!" << std::endl;
	//		return -1;
	//	}
	//	//get mass center  寻找四边形中点
	//	for (unsigned int i = 0; i < corners.size(); i++)
	//	{
	//		center += corners[i];
	//	}
	//	center *= (1. / corners.size());
	//
	//	//确定四个点的中心线
	//	sortCorners(corners, center);
	//
	//	cv::Mat dst = src.clone();
	//
	//	//Draw Lines  画直线
	//	for (unsigned int i = 0; i<lines.size(); i++)
	//	{
	//		cv::Vec4i v = lines[i];
	//		cv::line(dst, cv::Point(v[0], v[1]), cv::Point(v[2], v[3]), CV_RGB(0, 255, 0));    //目标版块画绿线   
	//	}
	//
	//	//draw corner points  画角点
	//	cv::circle(dst, corners[0], 3, CV_RGB(255, 0, 0), 2);
	//	cv::circle(dst, corners[1], 3, CV_RGB(0, 255, 0), 2);
	//	cv::circle(dst, corners[2], 3, CV_RGB(0, 0, 255), 2);
	//	cv::circle(dst, corners[3], 3, CV_RGB(255, 255, 255), 2);
	//
	//	//draw mass center  画出四边形中点
	//	cv::circle(dst, center, 3, CV_RGB(255, 255, 0), 2);
	//
	//	cv::Mat quad = cv::Mat::zeros(300, 220, CV_8UC3);//设定校正过的图片从320*240变为300*220  
	//
	//	//corners of the destination image  
	//	std::vector<cv::Point2f> quad_pts;
	//	quad_pts.push_back(cv::Point2f(0, 0));
	//	quad_pts.push_back(cv::Point2f(quad.cols, 0));//(220,0)  
	//	quad_pts.push_back(cv::Point2f(quad.cols, quad.rows));//(220,300)  
	//	quad_pts.push_back(cv::Point2f(0, quad.rows));
	//
	//	// Get transformation matrix  
	//	cv::Mat transmtx = cv::getPerspectiveTransform(corners, quad_pts);   //求源坐标系（已畸变的）与目标坐标系的转换矩阵  
	//
	//	// Apply perspective transformation透视转换  
	//	cv::warpPerspective(src, quad, transmtx, quad.size());
}


float getDistance(Point pointO, Point pointA)
{//求两点之间距离
	float distance;
	distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
	distance = sqrtf(distance);

	return distance;
}

float getAngle(Point pointM, Point pointL, Point pointR)
{//求三点之间的夹角
	Point L, R;
	float dist_L, dist_R, Theta;
	L.x = pointL.x - pointM.x;
	L.y = pointL.y - pointM.y;
	R.x = pointR.x - pointM.x;
	R.y = pointR.y - pointM.y;
	dist_L = getDistance(pointL, pointM);
	dist_R = getDistance(pointR, pointM);
	Theta = acos((L.x*R.x + L.y*R.y) / (dist_L*dist_R));
	return Theta;
}

float getDist_P2L(Point pointP, Point pointA, Point pointB)
{//点到直线的距离:P到AB的距离
	//求直线方程
	int A = 0, B = 0, C = 0;
	A = pointA.y - pointB.y;
	B = pointB.x - pointA.x;
	C = pointA.x*pointB.y - pointA.y*pointB.x;
	//代入点到直线距离公式
	float distance = 0;
	distance = ((float)abs(A*pointP.x + B*pointP.y + C)) / ((float)sqrtf(A*A + B*B));
	return distance;
}

//对角点进行排序，因为之前检测出的轮廓是带序号的
int list_connor(int i1, int i2, int i3)
{//排序
	int flag = 0;
	Point point_add;
	if (i1 >= i2&&i2 >= i3)
		flag = 123;
	else if (i1 >= i3&& i3 >= i2)
		flag = 132;
	else if (i2 >= i1&&i1 >= i3)
		flag = 213;
	else if (i2 >= i3&&i3 >= i1)
		flag = 231;
	else if (i3 >= i2&&i2 >= i1)
		flag = 321;
	else if (i3 >= i1&&i1 >= i2)
		flag = 312;
	return flag;
}

Mat imgPreprocess(const Mat& rawImg)
{
    Mat gray , binary, edges ;
    //彩色图转灰度图
	// imshow("rawImg ", rawImg);
    cvtColor(rawImg, gray, cv::COLOR_BGR2GRAY);

    //二值化 gray 中像素值大于200的点变为255（白），其余的为0（黑）
	gray = gray > 225;
    //Canny 算法 提取边缘，降噪
    // Canny(gray,edges, 45, 130);

	imshow("Binary image", gray);

    return gray;
}

vector<vector<Point>> contourExtract(const Mat& binaryImg, Mat& drawing)
{
    vector<vector<Point>> contours, RectContours;//轮廓，为点向量，
    
	findContours(binaryImg, contours, RETR_LIST, CHAIN_APPROX_NONE);//找轮廓
    
	vector<vector<Point>> hull(contours.size());//用于存放凸包

	vector<float> length(contours.size());//用于保存每个轮廓的长度
	vector<float> Area_contours(contours.size()), Area_hull(contours.size()), Rectangularity(contours.size()), circularity(contours.size());
	
	int bestRectContourIndex_ = -1;
	float maxRectangularity = 0.0 ;

    //把所有的轮廓画出来
    for (int i = 0; i < contours.size(); i++)
	{

		Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		length[i] = arcLength(contours[i], true);//轮廓的长度
        // cout<<"The length of"<< i+1 <<" contours"<<length[i]<<endl;
		if (length[i] >200 && length[i] <2000) //通过长度匹配滤除小轮廓
		{
			convexHull(Mat(contours[i]), hull[i], false);//寻找凸包

			Area_contours[i] = contourArea(contours[i]);   //轮廓面积
			Area_hull[i] = contourArea(hull[i]);           //凸包面积
			Rectangularity[i] = Area_contours[i] / Area_hull[i]; //矩形度
			
			circularity[i] = (4 * M_PI*Area_contours[i]) / (length[i] * length[i]);//圆形度

			// drawContours(drawing, hull, i, color, 5);//得到方框

            //通过矩形度和圆形度滤除场地图形干扰,并找到矩形度最大的值作为目标框
			if (Rectangularity[i]>0.85&&circularity[i]<0.9)
			{
				//找到矩形度最大的值作为目标框
				if (Rectangularity[i] >= maxRectangularity)
				{
					bestRectContourIndex_ = i;
					maxRectangularity = Rectangularity[i];
				}
				// RectContours.push_back(hull[i]);//把提取出来的方框导入到新的轮廓组
				// drawContours(drawing,  hull, i, color, 5);//得到方框
			}
		}
	}
	if (bestRectContourIndex_ != -1) {
		Scalar highlightColor = Scalar(255, 0,0); // 设置高亮颜色为绿色
		RectContours.push_back(hull[bestRectContourIndex_]);
		drawContours(drawing, hull, bestRectContourIndex_, highlightColor, 5); // 绘制最大矩形度的轮廓
	}
    return RectContours;
}

vector<vector<Point>> connorExtract(const vector<vector<Point>>& rectContours, Mat& drawing)
{
	float distance = 0, distanceMax = 0;
	Point connorPoint1, connorPoint2, connorPoint3, connorPoint4, point_add;
	vector<Point> connor4_add(3);  //先找到的三个角点
	int conP_i1, conP_i2, conP_i3, conP_i_add;
	int  flag = 0;

	vector<vector<Point>> finalContours;
	for (int j = 0; j < rectContours.size(); j++)  //四边形轮廓个数
	{
		distance = 0;
		distanceMax = 0;
		for (int i = 0; i < rectContours[j].size(); i++) //每个轮廓点的个数11到19点不等
		{//找第一个角点
			distance = getDistance(rectContours[j][i], rectContours[j][0]);
			if (distance>distanceMax)
			{
				distanceMax = distance;
				connorPoint1 = rectContours[j][i]; //第一个角点
				conP_i1 = i;
			}
		}
        // cout<<"凸包中第一个点"<<RectContours[j][0]<<endl;
        // cout<<"凸包中的点"<<RectContours[j]<<endl;
		distance = 0;
		distanceMax = 0;
		for (int i = 0; i < rectContours[j].size(); i++)
		{//找第二个角点
			distance = getDistance(rectContours[j][i], connorPoint1);
			if (distance>distanceMax)
			{
				distanceMax = distance;
				connorPoint2 = rectContours[j][i]; //第二个角点
				conP_i2 = i;
			}
		}
		distance = 0;
		distanceMax = 0;
		for (int i = 0; i < rectContours[j].size(); i++)
		{//找第三个角点
			distance = getDistance(rectContours[j][i], connorPoint1) + getDistance(rectContours[j][i], connorPoint2);
			if (distance>distanceMax)
			{
				distanceMax = distance;
				connorPoint3 = rectContours[j][i]; //第三个角点
				conP_i3 = i;
			}
		}
		flag = list_connor(conP_i1, conP_i2, conP_i3);//对三个角点由大到小排序
		switch (flag)
		{//对三个角点排序
            case 0:break;
            case 123:break;
            case 132:point_add = connorPoint2; connorPoint2 = connorPoint3; connorPoint3 = point_add; break;//2,3交换
            case 213:point_add = connorPoint1; connorPoint1 = connorPoint2; connorPoint2 = point_add; break;//1,2交换
            case 231:point_add = connorPoint1; connorPoint1 = connorPoint2; connorPoint2 = point_add;
                point_add = connorPoint2; connorPoint2 = connorPoint3; connorPoint3 = point_add; break;//1,2交换+2,3交换
            case 321:point_add = connorPoint3; connorPoint3 = connorPoint1; connorPoint1 = point_add; break;//1,3交换
            case 312:point_add = connorPoint3; connorPoint3 = connorPoint1; connorPoint1 = point_add;
                point_add = connorPoint2; connorPoint2 = connorPoint3; connorPoint3 = point_add; break;//1,3交换+2,3交换
		}
		switch (flag)
		{//对三个角点排序
            case 0:break;
            case 123:break;
            case 132:conP_i_add = conP_i2; conP_i2 = conP_i3; conP_i3 = conP_i_add; break;//2,3交换
            case 213:conP_i_add = conP_i1; conP_i1 = conP_i2; conP_i2 = conP_i_add; break;//1,2交换
            case 231:conP_i_add = conP_i1; conP_i1 = conP_i2; conP_i2 = conP_i_add;
                conP_i_add = conP_i2; conP_i2 = conP_i3; conP_i3 = conP_i_add; break;//1,2交换+2,3交换
            case 321:conP_i_add = conP_i3; conP_i3 = conP_i1; conP_i1 = conP_i_add; break;//1,3交换
            case 312:conP_i_add = conP_i3; conP_i3 = conP_i1; conP_i1 = conP_i_add;
                conP_i_add = conP_i2; conP_i2 = conP_i3; conP_i3 = conP_i_add; break;//1,3交换+2,3交换
		}
		distance = 0;
		distanceMax = 0;
		for (int i = conP_i3; i < conP_i2; i++)
		{//相隔两角点之间找到怀疑是4角点的点
			distance = getDistance(rectContours[j][i], connorPoint3) + getDistance(rectContours[j][i], connorPoint2);
			if (distance>distanceMax)
			{
				distanceMax = distance;
				connor4_add[0] = rectContours[j][i];
			}
		}
		distance = 0;
		distanceMax = 0;
		for (int i = conP_i2; i < conP_i1; i++)
		{//相隔两角点之间找到怀疑是4角点的点
			distance = getDistance(rectContours[j][i], connorPoint1) + getDistance(rectContours[j][i], connorPoint2);
			if (distance>distanceMax)
			{
				distanceMax = distance;
				connor4_add[1] = rectContours[j][i];
			}
		}
		distance = 0;
		distanceMax = 0;
		for (int i = conP_i1; i < rectContours[j].size() + conP_i3; i++)
		{//相隔两角点之间找到怀疑是4角点的点
			if (i< rectContours[j].size())
			{
				distance = getDistance(rectContours[j][i], connorPoint1) + getDistance(rectContours[j][i], connorPoint3);
				if (distance>distanceMax)
				{
					distanceMax = distance;
					connor4_add[2] = rectContours[j][i];
				}
			}
			else
			{
				distance = getDistance(rectContours[j][i - rectContours[j].size()], connorPoint1) + 
							getDistance(rectContours[j][i - rectContours[j].size()], connorPoint3);
				if (distance>distanceMax)
				{
					distanceMax = distance;
					connor4_add[2] = rectContours[j][i - rectContours[j].size()];
				}
			}
		}
		if (getAngle(connor4_add[0], connorPoint3, connorPoint2) / M_PI * 180 < 120)
		{
			connorPoint4 = connor4_add[0];
		}
		else if (getAngle(connor4_add[1], connorPoint2, connorPoint1)/ M_PI * 180 < 120)
		{
			connorPoint4 = connor4_add[1];
		}
		else if (getAngle(connor4_add[2], connorPoint1, connorPoint3)/ M_PI * 180 < 120)
		{
			connorPoint4 = connor4_add[2];
		}

		//角点排序
		// Step 1: 按照 y 坐标（升序）排序
		vector<Point> corners ={connorPoint1,connorPoint2,connorPoint3,connorPoint4};
		sort(corners.begin(), corners.end(), [](const Point& a, const Point& b) {
        return a.y < b.y;
		});
		// Step 2: 前两个点是上组，后两个点是下组
		vector<Point> top = {corners[0], corners[1]};
		vector<Point> bottom = {corners[2], corners[3]};
		// Step 3: 按照 x 坐标（升序）排序
		sort(top.begin(), top.end(), [](const Point& a, const Point& b) {
			return a.x < b.x;
		});
		sort(bottom.begin(), bottom.end(), [](const Point& a, const Point& b) {
			return a.x < b.x;
		});

		circle(drawing, top[0], 3, Scalar(253, 255, 0), FILLED, LINE_AA);
		circle(drawing, top[1], 3, Scalar(0, 0, 253), FILLED, LINE_AA);
		circle(drawing, bottom[0], 3, Scalar(0, 0, 253), FILLED, LINE_AA);
		circle(drawing, bottom[1], 3, Scalar(0, 253, 253), FILLED, LINE_AA);

		Point contours[] = {top[1],bottom[1],bottom[0],top[0]};
		vector<Point> connorPoints(contours,contours+4);
		// finally_contours[j][0] = connorPoint1;
		// finally_contours[j][1] = connorPoint2;
		// finally_contours[j][2] = connorPoint3;
		// finally_contours[j][3] = connorPoint4;
		finalContours.push_back(connorPoints);

		// cout << "\n轮廓 " << j+1 << "  的四个角点坐标分别为：\n" << finalContours[j][0] << finalContours[j][1] << finalContours[j][2] << finalContours[j][3] << endl;
	}
	return 	finalContours;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
		ROS_INFO_STREAM_ONCE( "Subscribing to images!");
		// cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        // 将ROS图像消息转换为OpenCV图像
        Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
		
		srcImage0 = image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Failed to convert ROS image to OpenCV image: %s", e.what());
    }
}