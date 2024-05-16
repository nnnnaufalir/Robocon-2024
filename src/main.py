import numpy as np
import cv2
from ultralytics import YOLO
import time

# Load class list from file
with open("Others/_YOLOV8/coco.txt", "r") as my_file:
    class_list = my_file.read().split("\n")

# Define detection colors BGR
# blue_ball
# blue_rice
# my_r1
# purple_ball
# red_ball
# red_rice
# silo
detection_colors = [(255, 0, 0), (255, 255, 0),
                    (255, 0, 255), (0, 0, 255), (0, 255, 255), (55, 255, 255),]

# Load YOLOv8 model
model = YOLO("models/best.pt", "v8")

# Video frame dimensions
frame_width = 640
frame_height = 640

cap = cv2.VideoCapture("Video/Testing/Test6.mp4")
if not cap.isOpened():
    print("Cannot open video file")
    exit()


frame_count = 0
start_time = time.time()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("End of video stream")
        break

    # Resize frame
    frame = cv2.resize(frame, (frame_width, frame_height))

    # Predict on frame
    detect_params = model.predict(
        source=[frame], conf=0.25, save=False, imgsz=640, iou=0.5)

    # Convert tensor array to numpy
    DP = detect_params[0].numpy()
    # print(DP)

    if len(DP) != 0:
        for i in range(len(detect_params[0])):
            # print(i)
            boxes = detect_params[0].boxes
            box = boxes[i]  # returns one box
            clsID = box.cls.numpy()[0]
            conf = box.conf.numpy()[0]
            bb = box.xyxy.numpy()[0]

            cv2.rectangle(frame, (int(bb[0]), int(bb[1])), (int(
                bb[2]), int(bb[3])), detection_colors[int(clsID)], 3,)

            # Display class name and confidence
            font = cv2.FONT_HERSHEY_COMPLEX
            cv2.putText(frame, class_list[int(clsID)] + " " + str(round(
                conf, 3)) + "%", (int(bb[0]), int(bb[1]) - 10), font, 0.5, detection_colors[int(clsID)], 2,)

    # Increment frame count
    frame_count += 1

    # Calculate FPS
    end_time = time.time()
    fps = frame_count / (end_time - start_time)

    # Display FPS
    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    # Display frame
    cv2.imshow('ObjectDetection', frame)

    # Terminate when "Q" is pressed
    if cv2.waitKey(1) == ord('q'):
        break

# Release video capture and close windows
cap.release()
cv2.destroyAllWindows()
