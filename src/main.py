from ultralytics import YOLO
import cv2
import time
import torch
from lib_krai.krai_cv2 import VideoCapture as vd, DetectionUtils as det
from lib_krai.krai_serial import SerialCommunicator as ser

print("Import Success")

# Initialize Serial Communication
serial_0 = ser("COM12", 57600)
serial_0.begin()
time.sleep(1)

serial_0.flush_input()
serial_0.flush_output()


def load_model(path):
    """Load the YOLO model."""
    model = YOLO(path)
    print("Model Loaded")
    return model


def draw_screen(frame, frame_width, frame_height, fps, angle, device_value, data_sent):
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
    cv2.putText(frame, f'Position: {angle}', (10, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_line, 2)
    cv2.putText(frame, f'Sent: {data_sent}', (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


def main():
    model_path = "../models/yolov8_4.pt"
    class_names = ['B', 'BB', 'BBB', 'blue_ball', 'purple_ball', 'silo']
    class_colors = {
        0: (100, 100, 100),    # B - silo 1
        1: (50, 50, 50),       # BB - silo 2
        2: (200, 200, 200),    # BBB - silo 3
        3: (255, 0, 0),        # Blue Ball
        4: (255, 0, 255),      # Purple Ball - Magenta
        5: (255, 255, 255)     # silo - White
    }

    frame_width = 640
    frame_height = 640

    size_obj_cm = 30  # Size of object in cm
    focal_length = 473.6842105263158  # Example value, depends on the camera

    device_value = 'cpu'
    conf_value = 0.2
    iou_value = 0.5
    max_det_value = 3

    model = load_model(model_path)
    cap1 = vd(0)  # First camera

    prev_time = time.time()
    angle = "Not Defined"
    data_sent = "None"

    try:
        while True:
            frame1 = cap1.read_frame()
            if frame1 is None:
                break

            frame1 = cv2.resize(frame1, (frame_width, frame_height))

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

                        class_name = class_names[int(cls_id)]
                        label = f'{class_name}: {conf:.2f} ID: {int(track_id)}'
                        cv2.rectangle(frame1, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(frame1, label, (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                        if closest_ball_details and torch.equal(box, closest_ball_details[0]):
                            color = (0, 0, 255)
                            cv2.rectangle(frame1, (x1, y1), (x2, y2), color, 2)
                            cv2.putText(frame1, label, (x1, y1 - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                            center_x, center_y = det.draw_point(
                                frame1, frame_width, frame_height, x1, y1, x2, y2, color)
                            angle = det.calculate_angle(
                                center_x, center_y, frame_width, frame_height)

                            distance_display = f'{closest_ball_details[4]:.2f} cm'
                            cv2.putText(frame1, distance_display, (center_x + 10,
                                        center_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                            bottom_center_x = frame_width // 2
                            bottom_center_y = frame_height

                            cv2.line(frame1, (center_x, center_y),
                                     (bottom_center_x, bottom_center_y), (255, 255, 255), 2)

            fps, prev_time = det.calculate_fps(prev_time)

            if closest_ball_details:
                # Ensure angle is a float and format it
                angle_display = f'{float(angle):.2f}'

                # Determine the command based on the angle
                # If the angle is within 2 degrees of 0 or 180
                if abs(float(angle)) <= 7.5 or abs(float(angle)) >= 172.5:
                    command = "maju"

                else:
                    command = "rotasi"

                # Prepare the data to be sent
                distance = closest_ball_details[4]
                data_sent = f"{command},{angle_display},{distance:.2f}"
                serial_0.write(data_sent, 0)
                print(f"Sent: {data_sent}")

            else:
                angle_display = "Not Defined"

            # Update the frame with the latest data_sent value
            draw_screen(frame1, frame_width, frame_height,
                        fps, angle_display, device_value, data_sent)

            cv2.imshow('Tracking', frame1)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        # Close serial communication
        serial_0.end()
        cap1.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
