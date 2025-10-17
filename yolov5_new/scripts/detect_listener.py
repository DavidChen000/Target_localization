import rospy
from yolov5.msg import YoloRes




def callback(data):
    print("-----school badge--------")
    print("x:",data.logo_pos.x)
    print("y:",data.logo_pos.y)
    print("z:",data.logo_pos.z)

    print("-----QR  code--------")
    print("x:",data.label1_pos.x)
    print("y:",data.label1_pos.y)
    print("z:",data.label1_pos.z)




def listener():
    rospy.init_node("detect_listener_node",anonymous=True)

    rospy.Subscriber("/yolo/vision_info",YoloRes,callback)

    rospy.spin()




if __name__ == "__main__":
    listener()



