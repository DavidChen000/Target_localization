#!/usr/bin/env python3
# coding:utf-8
import sys
import os
import copy
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir) 
from functions import AppState, pointxyz
from utils.torch_utils import select_device
from utils.general import (check_img_size, non_max_suppression, scale_coords,
                           plot_one_box, plot_one_text, strip_optimizer, set_logging)
from models.experimental import attempt_load
from numpy import random
import torch.backends.cudnn as cudnn
import numpy as np
import pyrealsense2 as rs
import argparse
import torch
import time
import cv2
from yolov5.msg import YoloRes
from std_msgs.msg import String
import rospy
import gi
gi.require_version('Gtk', '2.0')

# detect function
def detect(save_img=False):
    rospy.init_node('detect', anonymous=True)
    vision_info_pub = rospy.Publisher(
        '/yolo/vision_info', YoloRes, queue_size=10)
    rate = rospy.Rate(20)
    imgsz = opt.img_size
    weights = opt.weights
    
    # Initialize
    set_logging()
    touch_flag = torch.cuda.is_available()
    device = select_device(opt.device)
    half = device.type != 'cpu'

    # Load model
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
    
    # Rpun inference
    img = torch.zeros((1, 3, imgsz, imgsz), device=device)
    _ = model(img.half() if half else img) if device.type != 'cpu' else None

    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
    state = AppState()
    decimate = rs.decimation_filter()
    decimate.set_option(rs.option.filter_magnitude, 1 ** state.decimate)

    # Start streaming
    pipeline.start(config)
    pc = rs.pointcloud()
    
    # Set path
    t = time.localtime()
    results_dir = os.path.join(current_dir, 'results/')
    ori_images_dir = os.path.join(current_dir, 'ori_images/')
    img_save_path = results_dir +\
                    str(t.tm_year)+'-'+str(t.tm_mon)+'-'+str(t.tm_mday)+'-'+str(t.tm_hour)+'-'+str(t.tm_min)+'-'+str(t.tm_sec)+'/'
    ori_img_save_path = ori_images_dir +\
                    str(t.tm_year)+'-'+str(t.tm_mon)+'-'+str(t.tm_mday)+'-'+str(t.tm_hour)+'-'+str(t.tm_min)+'-'+str(t.tm_sec)+'/'
    if not os.path.exists(img_save_path):
        os.makedirs(img_save_path)
    if not os.path.exists(ori_img_save_path):
        os.makedirs(ori_img_save_path)
    count = 0
    
    vision_info = YoloRes() 
    vision_info_old = YoloRes() 
    vision_info.QR_pos.z = 0.1
    vision_info.logo_pos.z = 0.1

    
    res_list = []
    yolo_res_old = False
    # When ROS is OK
    while not rospy.is_shutdown():
        vision_info.update_yolo_res = False
        # Get frames
        t1 = time.time()
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue
        points = pc.calculate(depth_frame)
        pc.map_to(color_frame)
        v = points.get_vertices()
        depth_frame = decimate.process(depth_frame)
        verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        img = cv2.resize(color_image, (640, 480))
        img_ori = img.copy() 
        im0 = img.copy()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.transpose((2, 0, 1))
        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float() 
        img /= 255.0 
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

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
                for *xyxy, conf, cls in reversed(det):  #conf !!
                    if (int(cls) == 0) or (int(cls) == 1):
                        cls.cpu()
                        cls = cls.tolist()
                        x1 = int((xyxy[0] + xyxy[2]) / 2)
                        y1 = int((xyxy[1] + xyxy[3]) / 2)
                        x_av1, y_av1, dis1, num = pointxyz(x1, y1, verts, cls, xyxy)

                        p1 = [x_av1, y_av1, dis1, int(cls)]
                        x = round(p1[0], 2)
                        y = round(p1[1], 2)
                        z = round(p1[2], 2)
                        
                        # Exist depth point 
                        if num != 0:
                            if cls == 0:    # QR
                                class_name = 'QR code'
                                vision_info.QR_pos.x = x
                                vision_info.QR_pos.y = y
                                vision_info.QR_pos.z = z
                                # print("QR_x:", x)
                                # print("QR_x:", x)
                                # print("QR_x:", x)
                            elif cls == 1:  # logo
                                vision_info.update_yolo_res = True
                                class_name = 'School bedge'
                                vision_info.logo_pos.x = x
                                vision_info.logo_pos.y = y
                                vision_info.logo_pos.z = z
                                
                            label = ' x:' + str(x) + ' y:' + \
                                str(y) + ' z:' + str(z) + 'm'
                        else:
                            label = 'Nan'
                        
                            
                        plot_one_box(xyxy, im0, label = label,
                                     color=colors[int(cls)], line_thickness=4)
                        plot_one_text(xyxy, im0, label = label,
                                      color=colors[int(cls)], line_thickness=4)
                        res_list.append(p1)   

            # diff_x = abs(vision_info.logo_pos.x - vision_info_old.logo_pos.x)
            # diff_y = abs(vision_info.logo_pos.y - vision_info_old.logo_pos.y)
            # diff_z = abs(vision_info.logo_pos.z - vision_info_old.logo_pos.z)
            


            # # vision_info.QR_pos.x= vision_info.logo_pos.x
            # # vision_info.QR_pos.y= vision_info.logo_pos.y
            # # vision_info.QR_pos.z= vision_info.logo_pos.z


            # # Publisher position  
            # if (yolo_res_old == True) and (vision_info.update_yolo_res == True):
            #     if ((diff_x >= 0.3) or (diff_y >= 0.3) or (diff_z >= 0.3)):
            #         vision_info.logo_pos.x = vision_info_old.logo_pos.x
            #         vision_info.logo_pos.y = vision_info_old.logo_pos.y
            #         vision_info.logo_pos.z = vision_info_old.logo_pos.z
            # if (vision_info.logo_pos.z == 0):
            #     vision_info.logo_pos.x = vision_info_old.logo_pos.x
            #     vision_info.logo_pos.y = vision_info_old.logo_pos.y
            #     vision_info.logo_pos.z = vision_info_old.logo_pos.z
            vision_info_pub.publish(vision_info)


            # save old data
            # yolo_res_old = vision_info.update_yolo_res
            # vision_info_old.logo_pos.x = vision_info.logo_pos.x
            # vision_info_old.logo_pos.y = vision_info.logo_pos.y
            # vision_info_old.logo_pos.z = vision_info.logo_pos.z

            t2 = time.time()
            
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
            depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((im0, depth_colormap))
        cv2.imshow('img', images)
            
        if count % 3 == 0:
            cv2.imwrite(img_save_path + str(count) + '_depth.jpg', images)
            cv2.imwrite(ori_img_save_path + str(count) + '_rgb.jpg', img_ori)
        
        cv2.waitKey(1)
        count += 1
        rate.sleep()

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
    # print(opt)
    with torch.no_grad():
        if opt.update:
            for opt.weights in ['yolov5s.pt', 'yolov5m.pt', 'yolov5l.pt', 'yolov5x.pt']:
                detect()
                strip_optimizer(opt.weights)
        else:
            detect()
