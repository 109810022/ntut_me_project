import cv2
from ultralytics import YOLO
import time
import csv
from shapely import Polygon
# Load the YOLOv8 model
model = YOLO('yolov8n.pt')
#model = YOLO('best.pt')

# Open the video file
video_path = 0 #"path/to/your/video/file.mp4"
cap = cv2.VideoCapture(video_path)
#iou_range = [(100,100),(540,380)]
IOU = Polygon([(100,100),(100,380),(540,380),(540,100)])

# Loop through the video frames
while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()

    if success:
        # Run YOLOv8 inference on the frame
        results = model(frame,conf=0.7)

        # Visualize the results on the frame
        annotated_frame = results[0].plot()
        boxes = results[0].boxes
        print(boxes.xyxy)
        print(boxes.cls )
        print(annotated_frame.shape)
        cv2.rectangle(annotated_frame, (100, 100) ,(540,380), (0, 255, 0), 2)

        # Display the annotated frame
        cv2.imshow("YOLOv8 Inference", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if the end of the video is reached
        break

# Release the video capture object and close the display window
cap.release()
cv2.destroyAllWindows()