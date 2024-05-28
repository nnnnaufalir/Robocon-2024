from ultralytics import YOLO
import cv2
import time
from lib_krai.krai_cv2 import VideoCapture as vd, DetectionUtils as det
from lib_krai.krai_serial import SerialCommunicator as ser

print("Import Success")


def load_model(path):
    """Load the YOLO model."""
    model = YOLO(path)
    print("Model Loaded")
    return model


def draw_screen(frame, frame_width, frame_height, fps, target, angle):
    center_line = frame_width // 2
    positions = [center_line, center_line //
                 2, center_line + (center_line // 2)]
    color_line = (0, 255, 0)

    for position in positions:
        cv2.line(frame, (position, 0), (position, frame_height), color_line, 1)

    cv2.circle(frame, (center_line, frame_height), 10, (255, 255, 255), -1)

    device_text = f'Device: cuda' if device_value == 0 else f'Device: {device_value}'
    cv2.putText(frame, device_text, (150, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_line, 2)
    cv2.putText(frame, f'FPS: {fps:.2f}', (10, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_line, 2)
    cv2.putText(frame, f'Rx: {data_receive_0}', (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_line, 2)
    cv2.putText(frame, f'Tx: {data_send_0}', (10, 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_line, 2)
    cv2.putText(frame, f'Target: {target}', (450, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_line, 2)
    cv2.putText(frame, f'Angle: {angle}', (450, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_line, 2)


def main():
    model_path = "../models/yolov8_1.pt"
    cam_path = "../videos/Testing/Test2.mp4"

    class_names = ['blue_ball', 'blue_rice',
                   'purple_ball', 'red_ball', 'red_rice', 'silo']
    class_colors = {
        0: (255, 0, 0),     # Blue Ball - Red
        1: (255, 255, 0),   # Blue Rice - Cyan
        2: (255, 0, 255),   # Purple Ball - Magenta
        3: (0, 0, 255),     # Red Ball - Blue
        4: (0, 255, 255),   # Red Rice - Yellow
        5: (255, 255, 255),  # Silo - White
    }

    global device_value, data_receive_0, data_send_0

    frame_width = 640
    frame_height = 640

    size_obj_cm = 20  # Misalnya, jika objek berukuran 20 cm
    focal_length = 438.6371154482302  # Contoh nilai, bergantung pada kamera

    device_value = 'cpu'
    conf_value = 0.4
    iou_value = 0.7
    max_det_value = 1

    model = load_model(model_path)
    cap = vd(cam_path)

    prev_time = time.time()
    angle_display = "Not Defined"
    class_name = "Not Defined"
    run_state = True

    while True:
        frame = cap.read_frame()
        if frame is None:
            break

        frame = cv2.resize(frame, (frame_width, frame_height))

        if run_state:
            print("Running")
            run_state = False

        results = model.predict(
            source=[frame],
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

        target_detected = False

        for result in results:
            boxes = result.boxes.xyxy
            confidences = result.boxes.conf
            class_ids = result.boxes.cls

            for box, conf, cls_id in zip(boxes, confidences, class_ids):
                x1, y1, x2, y2 = map(int, box)
                color = class_colors.get(int(cls_id), (0, 0, 0))
                class_name = class_names[int(cls_id)]
                label = f'{class_name}: {conf:.2f}'

                width_obj_pixel = det.calculate_obj_pixel(x1, x2)

                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                center_x, center_y = det.draw_point(
                    frame, frame_width, frame_height, x1, y1, x2, y2, color)

                angle = det.calculate_angle(
                    center_x, center_y, frame_width, frame_height)

                cv2.putText(frame, f'Angle: {angle:.2f}', (center_x + 10,
                            center_y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                distance = det.calculate_distance(
                    size_obj_cm, focal_length, width_obj_pixel)

                distance_display = f'{distance:.2f}'

                cv2.putText(frame, f'{distance:.2f} cm', (center_x + 10,
                            center_y + 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                if det.logic_class(class_name, 'blue_ball'):
                    target_detected = True
                    angle_display = f'{angle:.2f}'

        if not target_detected:
            angle_display = "Not Defined"

        data_receive_0 = ["path", "retry zone", "None"]
        data_send_0 = [class_name, angle_display, distance_display]

        fps, prev_time = det.calculate_fps(prev_time)
        draw_screen(frame, frame_width, frame_height,
                    fps, target_detected, angle_display)
        cv2.imshow('Object Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
