import cv2
import math
import time


class VideoCapture:
    def __init__(self, path):
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            print("Error: Cannot access the video.")
            exit()
        print("Video access successful.")

    def read_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Error: Cannot read frame.")
            return None
        return frame

    def release(self):
        self.cap.release()


class DetectionUtils:
    @staticmethod
    def draw_point(frame, frame_width, frame_height, x1, y1, x2, y2, color):
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        bottom_center_x = frame_width // 2
        bottom_center_y = frame_height

        cv2.line(frame, (center_x, center_y),
                 (bottom_center_x, bottom_center_y), (255, 255, 255), 2)
        cv2.circle(frame, (center_x, center_y), 5, color, -1)
        cv2.putText(frame, f'({center_x}, {center_y})', (center_x,
                    center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        return center_x, center_y

    @staticmethod
    def calculate_angle(center_x, center_y, frame_width, frame_height):
        bottom_center_x = frame_width // 2
        bottom_center_y = frame_height
        delta_x = bottom_center_x - center_x
        delta_y = bottom_center_y - center_y
        angle_rad = math.atan2(delta_y, delta_x)
        raw_angle_deg = math.degrees(angle_rad)

        threshold_angle = 90
        min_angle = -90
        max_angle = 90
        angle_deg = raw_angle_deg - threshold_angle
        if angle_deg <= min_angle:
            return min_angle
        elif angle_deg >= max_angle:
            return max_angle
        return angle_deg

    @staticmethod
    def calculate_obj_pixel(x1, x2):
        return abs(x2 - x1)

    @staticmethod
    def calculate_distance(size_obj_cm, focal_len, size_obj_pixel):
        return (((size_obj_cm / 100) * focal_len) / size_obj_pixel) * 100

    @staticmethod
    def calculate_fps(prev_time):
        curr_time = time.time()
        fps = 1 / (curr_time - prev_time)
        return fps, curr_time

    @staticmethod
    def logic_class(class_name, target):
        return class_name == target
