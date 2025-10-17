#!/usr/bin/env python3
# coding:utf-8
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir) 
from utils.torch_utils import select_device
from utils.general import (check_img_size, non_max_suppression, scale_coords,
                           plot_one_box, plot_one_text, strip_optimizer, set_logging, xyxy2xywh )
from models.experimental import attempt_load
from numpy import random
import torch.backends.cudnn as cudnn
import numpy as np
import argparse
import torch

import cv2
from yolov5.msg import YoloRes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from yolov5.msg import DetectBox

import rospy
import gi
gi.require_version('Gtk', '2.0')
import gc
import time

class targetDetector:
    def __init__(self):

        # ROS Publishers and Sub
        self.imgSub_ = None
        self.imgDetPub_ = None
        self.detectBoxPub_ = None
        self.bridge = CvBridge()
        self.latest_image = None
        # self.class_name = 'None'
        self.detectFlag_ = False

        #Containers for temporarily storing data
        self.detectBox_ = DetectBox()
        self.detectBoxOld_ = DetectBox() 
        self.prev_QR = [0,0,0,0]

        #Adjustable parameters
        self.conf_level = 0.7

        self.colors = None

        #Get the configuration parameters
        self.imgsz = opt.img_size
        self.weights = opt.weights

        #  Initialize
        set_logging()
        touch_flag = torch.cuda.is_available()
        self.device = select_device(opt.device)
        self.half = self.device.type != 'cpu'
        # print(self.device)

        #  Load model
        self.model = attempt_load(self.weights, map_location=self.device) 
        self.imgsz = check_img_size(self.imgsz, s=self.model.stride.max()) 
        
        if self.half:
            self.model.half() 
        cudnn.benchmark = True 
        
    def run(self):
         # ROS init Publishers and Sub
        rospy.init_node("target_detect_node",anonymous=True)
        self.imgSub_ = rospy.Subscriber("/video_capture_node/image_raw",Image,self.imgCallback,queue_size=1)
        self.imgDetPub_ = rospy.Publisher('/target_detect_node/image_det', Image, queue_size=10)
        self.detectBoxPub_ = rospy.Publisher('/target_detect_node/detectBox', DetectBox, queue_size=10)

        # Get names and colors
        names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)]
                for _ in range(len(names))]
        self.colors[0] = [0,0,255]
        self.colors[1] = [0,255,0]

         # model warm-up
        img = torch.zeros((1, 3, self.imgsz, self.imgsz), device=self.device)
        _ = self.model(img.half() if self.half else img) if self.device.type != 'cpu' else None

        # torch.cuda.empty_cache()
        # rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.yoloDetect(self.latest_image)
            # if self.detectFlag_ == True:
            #     # t1 = time.time()
            #     self.yoloDetect(self.latest_image)
            #     self.detectFlag_ = False
            #     # t2 = time.time()
            #     # print(t2-t1)
            
            # # rate.sleep()
        

    def imgCallback(self,data):
        
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
            self.detectFlag_ = True
        except CvBridgeError as e:
            self.detectFlag_ = False
            self.latest_image = None
            rospy.logerr(f"CvBridge Error: {e}")
            return
        
        # if cv_img is not None:
        #     rospy.loginfo_once("Image recived and converted!")
        #     self.yoloDetect(cv_img)
        # else:
        #     rospy.logwarn("Convered image is None")


    def yoloDetect(self,frame):
        
         # Convert images to numpy arrays
        color_image = np.asanyarray(frame)
        img = cv2.resize(color_image, (640, 480))
        im0 = img.copy()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.transpose((2, 0, 1))
        img = np.ascontiguousarray(img)
        
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float() 
        img /= 255.0 
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        
         # Inference
        pred = self.model(img, augment=opt.augment)[0]
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
                    if conf.item() < self.conf_level :
                        if cls == 0: #QR
                            self.detectBox_.qr_update = False
                            
                            self.detectBox_.qr_center_x = self.detectBoxOld_.qr_center_x
                            self.detectBox_.qr_center_y = self.detectBoxOld_.qr_center_y
                            self.detectBox_.qr_height = self.detectBoxOld_.qr_height
                            self.detectBox_.qr_width = self.detectBoxOld_.qr_width

                        elif cls == 1: #logo
                            self.detectBox_.logo_update = False
                            self.detectBox_.logo_center_x = self.detectBoxOld_.logo_center_x
                            self.detectBox_.logo_center_y = self.detectBoxOld_.logo_center_y
                            self.detectBox_.logo_height = self.detectBoxOld_.logo_height
                            self.detectBox_.logo_width = self.detectBoxOld_.logo_width
                    else:
                        if (int(cls) == 0) or (int(cls) == 1):
                            cls.cpu()
                            cls = cls.tolist()

                            #Obtain the center point and size of the detection box
                            xywh = xyxy2xywh(torch.tensor(xyxy).view(1,4)).view(-1).tolist()
                            
                            # Exist depth point 
                            # if num != 0:
                            if cls == 0:    # QR
                                self.detectBox_.qr_update = True
                                # self.class_name = 'QR code'
                               
                                self.detectBox_.qr_center_x = int(xywh[0])
                                self.detectBox_.qr_center_y = int(xywh[1])
                                self.detectBox_.qr_width = int(xywh[2])
                                self.detectBox_.qr_height = int(xywh[3])
                               

                                self.prev_QR[0] = int(xyxy[0])
                                self.prev_QR[1] = int(xyxy[1])
                                self.prev_QR[2] = int(xyxy[2])
                                self.prev_QR[3] = int(xyxy[3])
                                
                            elif cls == 1:  # logo
                                self.detectBox_.logo_update = True
                                # self.class_name = 'logo'
                                
                                self.detectBox_.logo_center_x = int(xywh[0])
                                self.detectBox_.logo_center_y = int(xywh[1])
                                self.detectBox_.logo_width = int(xywh[2])
                                self.detectBox_.logo_height = int(xywh[3])

                                # print('QR_y2:'+str(prev_QR[3])+'logo_y2:'+str(int(xyxy[3])))
                                # if int(xyxy[0]) >= self.prev_QR[0] and int(xyxy[2]) <= self.prev_QR[2] and \
                                #         int(xyxy[1]) >= self.prev_QR[1] and int(xyxy[3]) <= self.prev_QR[3]: #The logo is within the QR code
                                    
                                #     self.detectBox_.logo_center_x = int(xywh[0])
                                #     self.detectBox_.logo_center_y = int(xywh[1])
                                #     self.detectBox_.logo_width = int(xywh[2])
                                #     self.detectBox_.logo_height = int(xywh[3])

                                # else:
                                #     # detect_box = detect_box_old
                                #     self.detectBox_.logo_center_x = self.detectBoxOld_.logo_center_x
                                #     self.detectBox_.logo_center_y = self.detectBoxOld_.logo_center_y
                                #     self.detectBox_.logo_height = self.detectBoxOld_.logo_height
                                #     self.detectBox_.logo_width = self.detectBoxOld_.logo_width
                            # else:
                            #     label = 'Nan'

                    
                    label = '/Conf:'+str(format(conf.item(),'.4f'))

                    plot_one_box(xyxy, im0, label = label,
                                color=self.colors[int(cls)], line_thickness=4)
                    plot_one_text(xyxy, im0, label = label,
                                color=self.colors[int(cls)], line_thickness=4)
            #Pub detectBox information
            Stamp = rospy.Time.now()
            self.detectBox_.header.stamp = Stamp
            self.detectBoxPub_.publish(self.detectBox_)

            # save old data
            self.detectBoxOld_ = self.detectBox_
            self.detectBox_.logo_update = False
            self.detectBox_.qr_update = False

        rgb_msg = self.bridge.cv2_to_imgmsg(im0,"bgr8")
        rgb_msg.header.stamp = rospy.Time.now()
        self.imgDetPub_.publish(rgb_msg)
        

        cv2.imshow("yolo",im0)
        cv2.waitKey(1)
        
        # del img,im0,pred,det
        # gc.collect()
        # torch.cuda.empty_cache()



if __name__ == '__main__':
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

    detector = targetDetector()

    with torch.no_grad():
        if opt.update:
            for opt.weights in ['yolov5s.pt', 'yolov5m.pt', 'yolov5l.pt', 'yolov5x.pt']:
                detector.run()
                strip_optimizer(opt.weights)
        else:
            detector.run()