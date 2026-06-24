import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped  # 차체 코드와 타입을 맞추기 위해 변경
from std_msgs.msg import String
import os

# --- Configuration Parameters ---
os.environ["YOLO_CONFIG_DIR"] = "/tmp/ultralytics"
MODEL_PATH = '/home/rhkrgusdud/drokck_free/src/yolo_xyz_publisher/best.pt'
COLOR_WIDTH, COLOR_HEIGHT, COLOR_FORMAT, COLOR_FPS = 640, 480, rs.format.bgr8, 30
DEPTH_WIDTH, DEPTH_HEIGHT, DEPTH_FORMAT, DEPTH_FPS = 640, 480, rs.format.z16, 30
WINDOW_ORIGINAL = "1. Original RealSense Stream"
WINDOW_DETECTION = "2. YOLO Detection View (For Monitoring)"

def initialize_realsense_pipeline(config):
    pipeline = rs.pipeline()
    try:
        profile = pipeline.start(config)
        print("[INFO] RealSense pipeline started successfully.")
        return pipeline, profile
    except Exception as e:
        print(f"[ERROR] Failed to start RealSense pipeline: {e}", file=sys.stderr)
        return None, None

def get_depth_scale_and_intrinsics(profile):
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
    color_intrinsics = color_profile.get_intrinsics()
    return depth_scale, color_intrinsics

def process_frames(pipeline, align):
    try:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return None, None
        return color_frame, depth_frame
    except RuntimeError as e:
        print(f"[ERROR] Error processing frames: {e}", file=sys.stderr)
        return None, None

def main():
    rclpy.init()
    node = rclpy.create_node('yolo_xyz_node')
    
    # ✨ 차체 C++ 노드(track_1_4, track_5)와 정확히 일치하도록 퍼블리셔 통합 및 수정
    string_publisher = node.create_publisher(String, '/yolo_detected_object', 10)
    xyz_publisher = node.create_publisher(Vector3Stamped, '/yolo_object_xyz', 10)

    try:
        model = YOLO(MODEL_PATH)
        print(f"[INFO] YOLOv8 model loaded from {MODEL_PATH}")
    except Exception as e:
        print(f"[ERROR] Failed to load YOLOv8 model: {e}", file=sys.stderr)
        rclpy.shutdown()
        return

    config = rs.config()
    config.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, DEPTH_FORMAT, DEPTH_FPS)
    config.enable_stream(rs.stream.color, COLOR_WIDTH, COLOR_HEIGHT, COLOR_FORMAT, COLOR_FPS)

    pipeline, profile = initialize_realsense_pipeline(config)
    if pipeline is None:
        rclpy.shutdown()
        return

    depth_scale, color_intrinsics = get_depth_scale_and_intrinsics(profile)
    align = rs.align(rs.stream.color)

    cv2.namedWindow(WINDOW_ORIGINAL, cv2.WINDOW_NORMAL)
    cv2.namedWindow(WINDOW_DETECTION, cv2.WINDOW_NORMAL)

    try:
        while True:
            color_frame, depth_frame = process_frames(pipeline, align)
            if color_frame is None or depth_frame is None:
                continue

            original_image = np.frombuffer(color_frame.get_data(), dtype=np.uint8).reshape((COLOR_HEIGHT, COLOR_WIDTH, 3))
            detection_image = original_image.copy()

            try:
                results = model(detection_image, verbose=False)
            except Exception as e:
                print(f"[ERROR] YOLO inference failed: {e}", file=sys.stderr)
                continue

            for result in results:
                boxes = result.boxes.xyxy.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy()

                for box, cls_id in zip(boxes, classes):
                    x1, y1, x2, y2 = box.astype(int)
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)

                    class_id = int(cls_id)
                    class_name = model.names[class_id].lower()

                    # 🎨 1. 클래스별 모니터 화면 박스 색상 지정 (BGR)
                    if 'enemy' in class_name:
                        box_color = (0, 0, 255)       # 🔴 적군 = 빨간색
                    elif 'ally' in class_name:
                        box_color = (0, 255, 0)       # 🟢 아군 = 초록색
                    elif 'aruco' in class_name:
                        box_color = (255, 0, 255)     # 🟣 아루코 8종 (개별 분류 가능) = 보라색
                    elif 'robot_dog' in class_name or 'dog' in class_name:
                        box_color = (255, 255, 0)     # 🔵 로봇 개 = 하늘색
                    elif 'red' in class_name and 'light' in class_name:
                        box_color = (0, 0, 255)       # 🔴 빨간 신호등
                    elif 'green' in class_name and 'light' in class_name:
                        box_color = (0, 255, 0)       # 🟢 초록 신호등
                    else:
                        box_color = (0, 165, 255)     # 🟠 기타 (supply_box 등)

                    # 🖥️ 모니터 화면에 개별 아루코 이름 및 박스 시각화 (요구사항 반영)
                    cv2.rectangle(detection_image, (x1, y1), (x2, y2), box_color, 2)
                    cv2.putText(detection_image, f"[{class_name}]", (x1, y1 - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)

                    # 🔄 2. 차체 C++ 코드용 문자열 매핑 변환기
                    sending_string = ""
                    if 'red' in class_name and 'light' in class_name:
                        sending_string = "red_light"
                    elif 'supply_box' in class_name:
                        sending_string = "cargo_zone"
                    elif 'aruco' in class_name:
                        sending_string = "vision_marker"  # 8종 마커 모두 차체가 이해하는 단어로 통합 변환
                    elif 'robot_dog' in class_name or 'dog' in class_name:
                        sending_string = "robot_dog"

                    # 3. 문자열 토픽 발행
                    if sending_string != "":
                        str_msg = String()
                        str_msg.data = sending_string
                        string_publisher.publish(str_msg)
                        print(f"[YOLO -> C++] 클래스 송신 완료: {sending_string} (원래 물체: {class_name})")

                    # 🎯 4. 3D 위치 연산 및 Vector3Stamped 토픽 발행 (아루코 마커 추가 확장)
                    if 0 <= cx < DEPTH_WIDTH and 0 <= cy < DEPTH_HEIGHT:
                        depth_m = depth_frame.get_distance(cx, cy)
                        if depth_m > 0:
                            point_3d = rs.rs2_deproject_pixel_to_point(color_intrinsics, [cx, cy], depth_m)
                            real_x, real_y, real_z = point_3d

                            # 차체 요구사양에 맞춘 Vector3Stamped 메시지 빌드
                            xyz_msg = Vector3Stamped()
                            xyz_msg.header.stamp = node.get_clock().now().to_msg()
                            xyz_msg.header.frame_id = "camera_link"
                            xyz_msg.vector.x = float(real_z)   # 카메라 정면(Z) -> 로봇 앞방향(X)
                            xyz_msg.vector.y = float(-real_x)  # 카메라 오른쪽(X) -> 로봇 좌측(Y)
                            xyz_msg.vector.z = float(-real_y)  # 카메라 아래(Y) -> 로봇 위(Z)

                            # 좌표 전송 대상 조건 분기 (보급상자, 로봇개 + 아루코 마커 8종 추가)
                            if 'supply_box' in class_name or 'robot_dog' in class_name or 'aruco' in class_name:
                                cv2.circle(detection_image, (cx, cy), 5, (0, 255, 255), -1)
                                xyz_publisher.publish(xyz_msg)
                                print(f"[XYZ -> C++] {class_name} 위치 전송: X={xyz_msg.vector.x:.3f}, Y={xyz_msg.vector.y:.3f}, Z={xyz_msg.vector.z:.3f}")

            # 🖥️ 듀얼 윈도우 송출
            cv2.imshow(WINDOW_ORIGINAL, original_image)
            cv2.imshow(WINDOW_DETECTION, detection_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("[INFO] Interrupted by user.")
    except Exception as e:
        print(f"[CRITICAL ERROR] {e}", file=sys.stderr)
    finally:
        if pipeline:
            pipeline.stop()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
