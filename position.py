#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import torch
import numpy as np
import math 
import rospy
from geometry_msgs.msg import Twist 

''' 
设置
'''
pipeline = rs.pipeline()    # 定义流程pipeline，创建一个管道
config = rs.config()    # 定义配置config
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)      # 配置depth流
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)     # 配置color流
# Load the YOLOv8 model
model = YOLO('yolov8n-pose.pt')



pipe_profile = pipeline.start(config)       # streaming流开始

# 创建对齐对象与color流对齐
align_to = rs.stream.color      # align_to 是计划对齐深度帧的流类型
align = rs.align(align_to)      # rs.align 执行深度帧与其他帧的对齐

puber = "/twist_cmd" 
pub = rospy.Publisher(puber, Twist, queue_size=10)
rospy.init_node('talker', anonymous=True)



def motion(x_vel,z_ang):
	cm = Twist()
	cm.linear.x = 0
	cm.linear.y = 0
	cm.linear.z = 0
	cm.angular.x = 0
	cm.angular.y = 0
	cm.angular.z = z_ang
	return cm

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
    dis = aligned_depth_frame.get_distance(x, y)        # 获取该像素点对应的深度
    # print ('depth: ',dis)       # 深度单位是m
    camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dis)
    # print ('camera_coordinate: ',camera_coordinate)
    return dis, camera_coordinate

if __name__=="__main__":

    while True:
        ''' 
        获取对齐图像帧与相机参数
        '''
        color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame = get_aligned_images()        # 获取对齐图像与相机参数
        results = model.track(img_color,conf=0.7,)
        img_color = results[0].plot()
        ''' 
        获取随机点三维坐标
        '''
        boxes = results[0].keypoints
        
        try:
            
            left_shoulder = results[0].keypoints[0][5]
            
            right_shoulder = results[0].keypoints[0][6]
           
            position = [int((left_shoulder[0]+right_shoulder[0])/2),int((left_shoulder[1]+right_shoulder[1])/2)]
            
            #print(position)
            '''
            無追蹤
            #coordinate = boxes.xyxy.numpy()
            #try:
                #coordinate =coordinate[0]
                #depth_pixel = [int((coordinate[0]+coordinate[2])/2), int((coordinate[1]+coordinate[3])/2)]        # 设置随机点，以相机中心点为例
            #except:
                #depth_pixel = [320, 240]
            '''
            dis, camera_coordinate = get_3d_camera_coordinate(position, aligned_depth_frame, depth_intrin)
            #print ('depth: ',dis)       # 深度单位是mm
            #print ('camera_coordinate: ',camera_coordinate)


            ''' 
            显示图像与标注
            '''
            #### 在图中标记随机点及其坐标 ####
            cv2.circle(img_color, position, 8, [255,0,255], thickness=-1)
            cv2.putText(img_color,"Dis:"+str(dis)+" m", (40,40), cv2.FONT_HERSHEY_COMPLEX, 1.,[0,0,255])
            cv2.putText(img_color,"X:"+str(camera_coordinate[0])+" m", (80,80), cv2.FONT_HERSHEY_COMPLEX, 1.,[255,0,0])
            cv2.putText(img_color,"Y:"+str(camera_coordinate[1])+" m", (80,120), cv2.FONT_HERSHEY_COMPLEX, 1.,[255,0,0])
            cv2.putText(img_color,"Z:"+str(camera_coordinate[2])+" m",  position, cv2.FONT_HERSHEY_COMPLEX, 1.,[255,0,0])

            rospy.loginfo(motion(camera_coordinate[2],math.atan(camera_coordinate[0]/camera_coordinate[2])))
            print(camera_coordinate)


        except:
            pass
        #### 显示画面 ####
        cv2.imshow('RealSence',img_color)
        key = cv2.waitKey(1)
        if key == ord('q') or key == 27: # Esc
            print('break')
            img_color.release()
            break
            
cv2.destroyAllWindows()
