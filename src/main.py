from ultralytics import YOLO
import cv2
import time
import torch
from lib_krai.krai_cv2 import VideoCapture as vd, DetectionUtils as det
from lib_krai.krai_serial import SerialCommunicator as ser

print("Import Success")


def load_model(path):
    """Load the YOLO model."""
    model = YOLO(path)
    print("Model Loaded")
    return model


def draw_screen(frame, frame_width, frame_height, fps, error, device_value):
    """Draw informational elements on the screen."""
    center_line = frame_width // 2
    color_line = (0, 255, 0)

    cv2.line(frame, (center_line, 0),
             (center_line, frame_height), color_line, 1)
    cv2.circle(frame, (center_line, frame_height), 10, (255, 255, 255), -1)

    device_text = f'Device: cuda' if device_value == 0 else f'Device: {device_value}'
    cv2.putText(frame, device_text, (150, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_line, 2)
    cv2.putText(frame, f'FPS: {fps:.2f}', (10, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_line, 2)
    cv2.putText(frame, f'Position: {error}', (10, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_line, 2)


def main():
    model_path = "../models/yolov8_4.pt"

    class_names = ['B', 'BB', 'BBB', 'blue_ball', 'purple_ball', 'silo']

    class_colors = {
        0: (100, 100, 100),      # B - silo 1
        1: (50, 50, 50),    # BB - silo 2
        2: (200, 200, 200),  # BBB - silo 3
        3: (255, 0, 0),      # Blue Ball -
        4: (255, 0, 255),    # Purple Ball - Magenta
        5: (255, 255, 255),  # silo - White

    }

    global device_value

    frame_width = 640
    frame_height = 640

    size_obj_cm = 30  # Size of object in cm
    focal_length = 473.6842105263158  # Example value, depends on the camera

    device_value = 'cpu'
    conf_value = 0.2
    iou_value = 0.7
    max_det_value = 5

    model = load_model(model_path)
    # cap1 = vd("../videos/Testing/Berantakan.mp4")  # Kamera pertama
    # cap2 = vd("../videos/Testing/Test3.mp4")       # Kamera kedua
    cap1 = vd(0)  # Kamera pertama
    cap2 = vd(2)

    prev_time = time.time()
    error = "Not Defined"
    valid_class = False

    while True:
        frame1 = cap1.read_frame()
        frame2 = cap2.read_frame()
        if frame1 is None or frame2 is None:
            break

        frame1 = cv2.resize(frame1, (frame_width, frame_height))
        frame2 = cv2.resize(frame2, (frame_width//2, frame_height//2))

        # Kamera pertama untuk mencari target
        results1 = model.track(
            source=[frame1],
            conf=conf_value,
            iou=iou_value,
            imgsz=640,
            half=False,
            device=device_value,
            max_det=max_det_value,
            vid_stride=1,
            stream_buffer=False,
            visualize=False,
            augment=False,
            agnostic_nms=False,
            classes=None,
            retina_masks=False
        )

        closest_distance = float('inf')
        closest_ball_details = None

        for result in results1:
            boxes = result.boxes.xyxy
            confidences = result.boxes.conf
            class_ids = result.boxes.cls
            track_ids = result.boxes.id if hasattr(
                result.boxes, 'id') else None

            if boxes is not None and confidences is not None and class_ids is not None and track_ids is not None:
                for box, conf, cls_id, track_id in zip(boxes, confidences, class_ids, track_ids):
                    if cls_id == 3:  # Blue Ball
                        x1, y1, x2, y2 = map(int, box)
                        width_obj_pixel = det.calculate_obj_pixel(x1, x2)
                        distance = det.calculate_distance(
                            size_obj_cm, focal_length, width_obj_pixel)

                        if distance < closest_distance:
                            closest_distance = distance
                            closest_ball_details = (
                                box, conf, cls_id, track_id, distance)

        for result in results1:
            boxes = result.boxes.xyxy
            confidences = result.boxes.conf
            class_ids = result.boxes.cls
            track_ids = result.boxes.id if hasattr(
                result.boxes, 'id') else None

            if boxes is not None and confidences is not None and class_ids is not None and track_ids is not None:
                for box, conf, cls_id, track_id in zip(boxes, confidences, class_ids, track_ids):
                    x1, y1, x2, y2 = map(int, box)
                    color = class_colors.get(int(cls_id), (0, 0, 0))

                    # Jika deteksi adalah target atau tidak, tampilkan bounding box
                    class_name = class_names[int(cls_id)]
                    label = f'{class_name}: {conf:.2f} ID: {int(track_id)}'
                    cv2.rectangle(frame1, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame1, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                    if closest_ball_details and torch.equal(box, closest_ball_details[0]):
                        # Red color for the closest blue ball
                        color = (0, 0, 255)
                        class_name = class_names[int(cls_id)]
                        label = f'{class_name}: {conf:.2f} ID: {int(track_id)}'
                        cv2.rectangle(frame1, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(frame1, label, (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                        center_x, center_y = det.draw_point(
                            frame1, frame_width, frame_height, x1, y1, x2, y2, color)
                        error = det.error_pixel(center_x, center_y)
                        distance_display = f'{closest_ball_details[4]:.2f} cm'
                        cv2.putText(frame1, distance_display, (center_x + 10,
                                    center_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                        bottom_center_x = frame_width // 2
                        bottom_center_y = frame_height

                        cv2.line(frame1, (center_x, center_y),
                                 (bottom_center_x, bottom_center_y), (255, 255, 255), 2)
                    else:
                        error = "Not Defined"

        # Kamera kedua untuk memvalidasi warna bola yang diambil oleh robot
        results2 = model.track(
            source=[frame2],
            conf=conf_value,
            iou=iou_value,
            imgsz=320,
            half=False,
            device=device_value,
            max_det=1,
            vid_stride=10,
            stream_buffer=False,
            visualize=False,
            augment=False,
            agnostic_nms=False,
            classes=[3, 4],  # Only detect blue_ball (0) and purple_ball (1)
            retina_masks=False
        )

        valid_class = False  # Reset valid_class for each frame
        for result in results2:
            boxes = result.boxes.xyxy
            confidences = result.boxes.conf
            class_ids = result.boxes.cls
            track_ids = result.boxes.id if hasattr(
                result.boxes, 'id') else None

            if boxes is not None and confidences is not None and class_ids is not None and track_ids is not None:
                for box, conf, cls_id, track_id in zip(boxes, confidences, class_ids, track_ids):
                    x1, y1, x2, y2 = map(int, box)
                    color = class_colors.get(int(cls_id), (0, 0, 0))

                    # Validasi bola yang diambil oleh robot
                    if cls_id == 0:  # Blue Ball
                        valid_class = True
                    else:
                        valid_class = False

                    class_name = class_names[int(cls_id)]
                    label = f'{class_name}: {conf:.2f} ID: {int(track_id)}'
                    cv2.rectangle(frame2, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame2, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        fps, prev_time = det.calculate_fps(prev_time)

        draw_screen(frame1, frame_width, frame_height,
                    fps, error, device_value)
        draw_screen(frame2, frame_width, frame_height,
                    fps, valid_class, device_value)

        cv2.imshow('Tracking', frame1)
        cv2.imshow('Validator', frame2)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap1.release()
    cap2.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
