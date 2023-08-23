#!/usr/bin/env python3
import numpy as np
import cv2
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort
import torch
import math 
import psutil

import rospy
from geometry_msgs.msg import Twist 
import pyrealsense2 as rs

pipeline = rs.pipeline()    # 定义流程pipeline，创建一个管道
config = rs.config()    # 定义配置config
config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipe_profile = pipeline.start(config)       # streaming流开始

# Load the YOLOv8 model#
# model = YOLO('yolov8n.pt')
#pipe_profile = pipeline.start(config)       # streaming流开始

# 创建对齐对象与color流对齐
align_to = rs.stream.color      # align_to 是计划对齐深度帧的流类型
align = rs.align(align_to)      # rs.align 执行深度帧与其他帧的对齐

#set up the tracker
object_tracker = DeepSort()
#set up  yolo 
#cap = cv2.VideoCapture("/Users/chenchinchi/Desktop/deep_sort/shopping.mp4")
cap = cv2.VideoCapture(0)
model = YOLO("yolov8n-pose.pt")
classes = model.names

puber = "/twist_cmd" 
pub = rospy.Publisher(puber, Twist, queue_size=5)
rospy.init_node('talker', anonymous=True)

ret_target = 1
ret = 0
'''
函式庫 ##########################################################
'''
def velocity(dis):
    kp = 0.3
    safe_dis = 0.8
    vel = kp * (dis-safe_dis)
    return vel 
    '''
    if vel < 0 : return 0
    elif vel >5: return 5
    else : return vel  
    '''
'''

def angle_rotation(angle): #80度 切成8等分
    ang = angle//10
    if ang > 3 :
        return -30*3.14159/180
    elif ang < -3:
        return 30*3.14159/180 
    else: return -ang*10*3.14159/180
'''
'''
def motion(vel,ang):        
    return {"linear":     
            {"x": 0.0, #vel  
            "y": 0.0,      
            "z": 0.0,},    
            "angular":     
            {"x": 0.0,     
            "y": 0.0,      
            "z": ang},     
            }    
'''
def motion(x_vel,z_ang):
    if x_vel >=0:
        cm = Twist()
        cm.linear.x = x_vel
        cm.linear.y = 0
        cm.linear.z = 0
        cm.angular.x = 0
        cm.angular.y = 0
        cm.angular.z = -z_ang
        return cm
    '''
    else :
        cm = Twist()
        cm.linear.x = x_vel
        cm.linear.y = 0
        cm.linear.z = 0
        cm.angular.x = 0
        cm.angular.y = 0
        cm.angular.z = z_ang
        return cm
    '''
''' 
获取对齐图像帧与相机参数
'''
def get_aligned_images():
    
    frames = pipeline.wait_for_frames()     # 等待获取图像帧，获取颜色和深度的框架集     
    aligned_frames = align.process(frames)      # 获取对齐帧，将深度框与颜色框对齐  

    aligned_depth_frame = aligned_frames.get_depth_frame()      # 获取对齐帧中的的depth帧 
    aligned_color_frame = aligned_frames.get_color_frame()      # 获取对齐帧中的的color帧

    #### 获取相机参数 ####
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics     # 获取深度参数（像素坐标系转相机坐标系会用到）
    color_intrin = aligned_color_frame.profile.as_video_stream_profile().intrinsics     # 获取相机内参


    #### 将images转为numpy arrays ####  
    img_color = np.asanyarray(aligned_color_frame.get_data())       # RGB图  
    img_depth = np.asanyarray(aligned_depth_frame.get_data())       # 深度图（默认16位）

    return color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame
''' 
获取点三维坐标
'''
def get_3d_camera_coordinate(depth_pixel, aligned_depth_frame, depth_intrin):
    x = depth_pixel[0]
    y = depth_pixel[1]
    if x < 0 : x = 0 
    elif x>1024: x = 1024
    if y<0: y = 1
    elif y>768:y = 766
    
    dis = aligned_depth_frame.get_distance(x, y)        # 获取该像素点对应的深度
    # print ('depth: ',dis)       # 深度单位是m
    camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dis)
    # print ('camera_coordinate: ',camera_coordinate)
    return dis, camera_coordinate


while True:
    
    color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame = get_aligned_images() #ＲＧＢ深度圖對齊
    ret = ret +1
    if ret==ret_target:
        image = img_color 
        results = model(image)
        annoated_frame = results[0].plot()
        #annoated_frame = cv2.copyMakeBorder(annoated_frame, 300, 300, 300, 300,0,annoated_frame, [255,255,255])
        for result in results:
            detections = []
            boxes = result.boxes
            for r in result.boxes.data.tolist():
                x1, y1, x2, y2 = r[:4]
                w, h = x2 - x1, y2 - y1
                coordinates = list((int(x1), int(y1), int(w), int(h)))
                conf = r[4]
                clsId = int(r[5])
                cls = classes[clsId]
                if cls == "person":
                    detections.append((coordinates, conf, cls))
            

            #print("detections: ", detections)
            tracks = object_tracker.update_tracks(detections, frame=image)
            
            for track in tracks:
                if not track.is_confirmed():
                    continue
                track_id = track.track_id
                bbox = track.to_ltrb()
                try:
                    if str(track_id) == "1":
                        print("positions: ",bbox)
                        position_2d = [int(0.5*(bbox[0]+bbox[2])), int(0.5*(bbox[1]+bbox[3]))]
                    
                        #get depth
                        
                        dis, camera_coordinate = get_3d_camera_coordinate(position_2d, aligned_depth_frame, depth_intrin)
                        vel = velocity(dis)
                        if camera_coordinate[2] == 0:
                            move = motion(vel,0.1)
                        else:
                            #follow the carrot
                            #move = motion(vel,float(math.atan(camera_coordinate[0]/camera_coordinate[2])))
                            #pure pursuit 
                            SAFE_DIS = 1
                            AXIS_DIS = 0.9
                            delta = 2*AXIS_DIS*camera_coordinate[0]/math.sqrt(camera_coordinate[2])
                            move = motion(velocity(dis),math.atan(delta))
                        #print(str(move))
                        if move!=None:
                            pub.publish(move)
                            rospy.loginfo(move)
                        
                            cv2.rectangle(
                            annoated_frame,
                            (int(bbox[0]), int(bbox[1])),
                            (int(bbox[2]), int(bbox[3])),
                            color=(255, 255, 255),
                            thickness=6,)
                            
                            cv2.putText(
                            annoated_frame,
                            "ID: " + str(track_id),
                            (int(bbox[0]), int(bbox[1]) ),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            2,
                            (0, 255, 0),
                            6,)
                            # draw positions
                            cv2.circle(annoated_frame,position_2d,5,(0, 255, 255), 10)
                except:
                    vel = vel- 0.03
                    move = motion(vel,0.1)
                    pub.publish(move)
            ret = 0

            cv2.putText(
                        annoated_frame,
                        "cpu " + str(psutil.cpu_percent()),
                        (100,100),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        2,
                        (0, 255, 0),
                        3,)
            a = psutil.virtual_memory()
            cv2.putText(
                        annoated_frame,
                        "ram " + str(a.percent),
                        (100,500),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        2,
                        (0, 255, 0),
                        3,)
                        
                        # draw positions
            cv2.imshow("Image", annoated_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

        



cap.release()
cv2.destroyAllWindows()