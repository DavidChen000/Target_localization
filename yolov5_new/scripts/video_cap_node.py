#!/usr/bin/env python3

import cv2
# import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time


def rtsp_video():


    rospy.loginfo("video starting")
    rtsp_url="rtsp://192.168.144.25:8554/main.264"
    
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        print("Error: Could not open video.")
        exit(1)

    fps = cap.get(cv2.CAP_PROP_FPS)
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    rospy.loginfo(f"video started fps:{fps},width:{width},height:{height}",)


    # ROS publisher
    pub = rospy.Publisher('/video_capture_node/image_raw', Image, queue_size=10)
    bridge = CvBridge()
    # lastPubTime_ = time.time()


    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        # cv2.imshow("a8_mini stream", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
               break

  
        # Convert the image to ROS image message and publish
        img = cv2.resize(frame, (640, 480))

        nowTime_ = time.time()
        # if (nowTime_-lastPubTime_)>0.1:
        #     ros_image = bridge.cv2_to_imgmsg(img, "bgr8")
        #     pub.publish(ros_image)
        #     lastPubTime_= nowTime_
        
        ros_image = bridge.cv2_to_imgmsg(img, "bgr8")
        ros_image.header.stamp = rospy.Time.now()
        pub.publish(ros_image)
        


    cap.release()
    cv2.destroyAllWindows()



if __name__ == "__main__":
    rospy.init_node('video_capture_node', anonymous=True)
    rtsp_video()