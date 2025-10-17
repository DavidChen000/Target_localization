#!/usr/bin/env python3
# coding:utf-8
import sys
import os
import copy
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir) 
# from functions import AppState, pointxyz
from function import outlier_detection
from utils.torch_utils import select_device
from utils.general import (check_img_size, non_max_suppression, scale_coords,
                           plot_one_box, plot_one_text, strip_optimizer, set_logging, xyxy2xywh )
from models.experimental import attempt_load
from numpy import random
import torch.backends.cudnn as cudnn
import numpy as np
# import pyrealsense2 as rs
import argparse
import torch
import time
import cv2
from yolov5.msg import YoloRes
# from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yolov5.msg import DetectBox

import rospy
import gi
gi.require_version('Gtk', '2.0')


def detect():

    rospy.init_node("target_detect_node",anonymous=True)
    
    # #Publishers
    vision_info_pub = rospy.Publisher(
        '/target_detect_node/vision_info', YoloRes, queue_size=10)
    image_ori_pub = rospy.Publisher(
        '/target_detect_node/image_raw', Image, queue_size=10)
    image_det_pub = rospy.Publisher(
        '/target_detect_node/image_det', Image, queue_size=10)
    detectBox_pub = rospy.Publisher(
    '/target_detect_node/detectBox', DetectBox, queue_size=10)
    bridge = CvBridge()
    
    #Set the publish frequency
    rate = rospy.Rate(30)

    #Get the configuration parameters
    imgsz = opt.img_size
    weights = opt.weights

    # # Initialize
    set_logging()
    touch_flag = torch.cuda.is_available()
    device = select_device(opt.device)
    half = device.type != 'cpu'

    # # Load model
    model = attempt_load(weights, map_location=device) 
    imgsz = check_img_size(imgsz, s=model.stride.max()) 
    if half:
        model.half() 
    cudnn.benchmark = True 

    # Get names and colors
    names = model.module.names if hasattr(model, 'module') else model.names
    colors = [[random.randint(0, 255) for _ in range(3)]
              for _ in range(len(names))]
    colors[0] = [0,0,255]
    colors[1] = [0,255,0]

    # Run inference
    img = torch.zeros((1, 3, imgsz, imgsz), device=device)
    _ = model(img.half() if half else img) if device.type != 'cpu' else None

    # Set path
    # t = time.localtime()
    # results_dir = os.path.join(current_dir, 'results/')
    # ori_images_dir = os.path.join(current_dir, 'ori_images/')
    # img_save_path = results_dir +\
    #                 str(t.tm_year)+'-'+str(t.tm_mon)+'-'+str(t.tm_mday)+'-'+str(t.tm_hour)+'-'+str(t.tm_min)+'-'+str(t.tm_sec)+'/'
    # ori_img_save_path = ori_images_dir +\
    #                 str(t.tm_year)+'-'+str(t.tm_mon)+'-'+str(t.tm_mday)+'-'+str(t.tm_hour)+'-'+str(t.tm_min)+'-'+str(t.tm_sec)+'/'
    # if not os.path.exists(img_save_path):
    #     os.makedirs(img_save_path)
    # if not os.path.exists(ori_img_save_path):
    #     os.makedirs(ori_img_save_path)
    # count = 0

    #Initialzes the variable to be published

    detect_box = DetectBox()
    # detect_box.qr_update = False
    # detect_box.logo_update = False

    #Adjustable parameters
    conf_level = 0.8
   

    #Containers for temporarily storing data
    detect_box_old = DetectBox() 
    prev_logo = [0,0]
    prev_QR = [0,0,0,0]
    # window_list = [YoloRes() for _ in range(window_size)]

    rtsp_url="rtsp://192.168.144.25:8554/main.264"
    cap = cv2.VideoCapture(rtsp_url)
    # 1280*720
    # print(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    # print(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    if not cap.isOpened():
        print("Error: Could not open video.")
        exit(1)


    # When ROS is OK
    while not rospy.is_shutdown():

        # Get frames
        ret, frame = cap.read()
        if not ret:
            break
        

        # Convert images to numpy arrays
        color_image = np.asanyarray(frame)
        img = cv2.resize(color_image, (640, 480))

        im0 = img.copy()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.transpose((2, 0, 1))
        img = np.ascontiguousarray(img)
        

        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float() 
        img /= 255.0 
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        
        #Convert image to rosmsg and pub
        image_ori_msg = bridge.cv2_to_imgmsg(frame,'bgr8')
        image_ori_msg.header.stamp = rospy.Time.now()
        image_ori_pub.publish(image_ori_msg)

        # Inference
        pred = model(img, augment=opt.augment)[0]
        pred = non_max_suppression(
            pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)

        # Detections per image
        for i, det in enumerate(pred):
            # Exist detect
            if det is not None and len(det):
                det[:, :4] = scale_coords(
                    img.shape[2:], det[:, :4], im0.shape).round()       
                # Write results
                for *xyxy, conf, cls in reversed(det):  
                    # When the confidence is not enough,maintaining the pervious value 
                    if conf.item() < conf_level :
                        if cls == 0: #QR
                            # detect_box.qr_update = False
                            detect_box.qr_center_x = detect_box_old.qr_center_x
                            detect_box.qr_center_y = detect_box_old.qr_center_y
                            detect_box.qr_height = detect_box_old.qr_height
                            detect_box.qr_width = detect_box_old.qr_width

                        elif cls == 1: #logo
                            # detect_box.logo_update = False
                            detect_box.logo_center_x = detect_box_old.logo_center_x
                            detect_box.logo_center_y = detect_box_old.logo_center_y
                            detect_box.logo_height = detect_box_old.logo_height
                            detect_box.logo_width = detect_box_old.logo_width
                    else:
                        if (int(cls) == 0) or (int(cls) == 1):
                            cls.cpu()
                            cls = cls.tolist()

                            #Obtain the center point and size of the detection box
                            xywh = xyxy2xywh(torch.tensor(xyxy).view(1,4)).view(-1).tolist()
                            
                            # Exist depth point 
                            # if num != 0:
                            if cls == 0:    # QR
                                # detect_box.qr_update = True
                                class_name = 'QR code'
                               
                                detect_box.qr_center_x = xywh[0]
                                detect_box.qr_center_y = xywh[1]
                                detect_box.qr_width = xywh[2]
                                detect_box.qr_height = xywh[3]

                                prev_QR[0] = int(xyxy[0])
                                prev_QR[1] = int(xyxy[1])
                                prev_QR[2] = int(xyxy[2])
                                prev_QR[3] = int(xyxy[3])
                                
                            elif cls == 1:  # logo
                                # detect_box.logo_update = True
                                class_name = 'School bedge'
                                
                                # print('QR_y2:'+str(prev_QR[3])+'logo_y2:'+str(int(xyxy[3])))
                                if int(xyxy[0]) >= prev_QR[0] and int(xyxy[2]) <= prev_QR[2] and \
                                        int(xyxy[1]) >= prev_QR[1] and int(xyxy[3]) <= prev_QR[3]: #The logo is within the QR code
                                    
                                    detect_box.logo_center_x = xywh[0]
                                    detect_box.logo_center_y = xywh[1]
                                    detect_box.logo_width = xywh[2]
                                    detect_box.logo_height = xywh[3]

                                else:
                                    # detect_box = detect_box_old
                                    detect_box.logo_center_x = detect_box_old.logo_center_x
                                    detect_box.logo_center_y = detect_box_old.logo_center_y
                                    detect_box.logo_height = detect_box_old.logo_height
                                    detect_box.logo_width = detect_box_old.logo_width
                            # else:
                            #     label = 'Nan'

                    label = class_name +'/Conf:'+str(conf.item())

                    plot_one_box(xyxy, im0, label = label,
                                color=colors[int(cls)], line_thickness=4)
                    plot_one_text(xyxy, im0, label = label,
                                color=colors[int(cls)], line_thickness=4)
            #Pub detectBox information
            Stamp = rospy.Time.now()
            detect_box.header.stamp = Stamp
            detectBox_pub.publish(detect_box)

            # save old data
            detect_box_old = detect_box

         #Pub processed images
        rgb_msg = bridge.cv2_to_imgmsg(im0,"bgr8")
        rgb_msg.header.stamp = rospy.Time.now()
        image_det_pub.publish(rgb_msg)

        cv2.imshow("video_test",im0)

        # if count % 3 == 0:
        #     cv2.imwrite(img_save_path + str(count) + '_.jpg', im0)
        #     cv2.imwrite(ori_img_save_path + str(count) + '_rgb.jpg', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        # count += 1
        # print(count)
        # rate.sleep()

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # Yolov5 configuration
    parser = argparse.ArgumentParser()
    pt_dir = os.path.join(current_dir, 'weights/yolov5s_SHHS.pt')
    parser.add_argument('--weights', nargs='+', type=str, default= pt_dir, help='model.pt path(s)')   
    parser.add_argument('--source', type=str, default='0', help='source')
    parser.add_argument('--output', type=str, default='inference/output', help='output folder')
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.5, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.5, help='IOU threshold for NMS')
    parser.add_argument('--device', default='0', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='display results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--update', action='store_true', help='update all models')
    opt = parser.parse_args()
    # print(opt)
    with torch.no_grad():
        if opt.update:
            for opt.weights in ['yolov5s.pt', 'yolov5m.pt', 'yolov5l.pt', 'yolov5x.pt']:
                detect()
                strip_optimizer(opt.weights)
        else:
            detect()
    # detect()
