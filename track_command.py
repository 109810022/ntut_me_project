#!/usr/bin/env python3
import numpy as np
import cv2
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort
import torch
import numpy as np
import math 

'''
import rospy
from geometry_msgs.msg import Twist 
import pyrealsense2 as rs
from command import velocity ,get_aligned_images,get_3d_camera_coordinate , motion
'''

''' 
#設置深度相機
config = rs.config()    # 定义配置config
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)      # 配置depth流
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)     # 配置color流
pipe_profile = pipeline.start(config)       # streaming流开始
# 创建对齐对象与color流对齐
align_to = rs.stream.color      # align_to 是计划对齐深度帧的流类型
align = rs.align(align_to)      # rs.align 执行深度帧与其他帧的对齐

#ROS node settings
puber = "/twist_cmd" 
pub = rospy.Publisher(puber, Twist, queue_size=10)
rospy.init_node('talker', anonymous=True)
'''

#set up the tracker
object_tracker = DeepSort()
#set up  yolo 
#cap = cv2.VideoCapture("/Users/chenchinchi/Desktop/deep_sort/shopping.mp4")
cap = cv2.VideoCapture(0)
model = YOLO("yolov8n.pt")
classes = model.names



while cap.isOpened():
    ret, image = cap.read()
    #color_intrin, depth_intrin, img_color, img_depth, aligned_depth_frame = get_aligned_images() #ＲＧＢ深度圖對齊
    if ret:
        results = model(image)
        annoated_frame = image #results[0].plot()
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
                try :
                    if str(track_id) == "1":
                        print("positions: ",bbox)
                        position_2d = [int(0.5*(bbox[0]+bbox[2])), int(0.5*(bbox[1]+bbox[3]))]
                    
                        #get depth
                        '''
                        dis, camera_coordinate = get_3d_camera_coordinate(position_2d, aligned_depth_frame, depth_intrin)
                    
                        #follow the carrot
                        move = motion(velocity(dis),math.atan(camera_coordinate[0]/camera_coordinate[2]))
                    
                        #pure pursuit 
                        SAFE_DIS = 1
                        AXIS_DIS = 0.9
                        delta = 2*AXIS_DIS*camera_coordinate[0]/math.sqrt(camera_coordinate[2])
                        move = motion(velocity(dis),math.atan(delta))

                        pub.publish(move)
                        '''
                    
                        # draw positions
                        cv2.circle(annoated_frame,position_2d,5,(0, 255, 255), 10)
                except:
                    pass
                '''
                    move = motion(0,0)
                    pub.publish(move)
                '''
                cv2.rectangle(
                        annoated_frame,
                        (int(bbox[0]), int(bbox[1])),
                        (int(bbox[2]), int(bbox[3])),
                        color=(0, 0, 255),
                        thickness=4,)
                        
                cv2.putText(
                        annoated_frame,
                        "ID: " + str(track_id),
                        (int(bbox[0]), int(bbox[1]) ),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        2,
                        (0, 255, 0),
                        6,)
                    


                
        
        cv2.imshow("Image", annoated_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


cap.release()
cv2.destroyAllWindows()
