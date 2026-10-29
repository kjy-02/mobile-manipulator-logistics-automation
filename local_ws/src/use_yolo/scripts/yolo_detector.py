#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLOv8 Multi-Class (3-Class) Box ONNX Detector for RealSense L515 & xArm5 ROS 1
Supports:
  - Class 0: original_box
  - Class 1: yellow_box
  - Class 2: black_box
Priority Grasp: Class 0 > Class 1 > Class 2
Publishes:
  - TF: 'camera_link' -> 'ap_target' (Selected priority target 3D pose)
  - /yolo/target_class (std_msgs/Int32): Current selected target class ID
  - /yolo/target_info (std_msgs/String): Detailed JSON string with target info
  - /yolo/detection_image (sensor_msgs/Image): Debug annotated stream
Subscribes:
  - /vision_enable (std_msgs/Bool): True when at initial pose, False when robot moving
  - /yolo_stop (std_msgs/Bool): Final shutdown signal after all pick & place steps completed
"""

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Int32, String
import tf

import os
import json
import threading
from pathlib import Path

INPUT_SIZE = 640

CLASS_NAMES = {
    0: "red_box",
    1: "yellow_box",
    2: "blue_box"
}

CLASS_COLORS = {
    0: (45, 45, 240),    # 빨간색: red_box (Class 0)
    1: (20, 205, 245),   # 노란색: yellow_box (Class 1)
    2: (240, 130, 45)    # 파란색: blue_box (Class 2)
}


def letterbox(frame):
    height, width = frame.shape[:2]
    scale = min(INPUT_SIZE / width, INPUT_SIZE / height)
    new_width, new_height = round(width * scale), round(height * scale)
    resized = cv2.resize(frame, (new_width, new_height))
    left = (INPUT_SIZE - new_width) // 2
    top = (INPUT_SIZE - new_height) // 2
    image = cv2.copyMakeBorder(
        resized, top, INPUT_SIZE - new_height - top,
        left, INPUT_SIZE - new_width - left,
        cv2.BORDER_CONSTANT, value=(114, 114, 114),
    )
    blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (INPUT_SIZE, INPUT_SIZE), swapRB=True)
    return blob, scale, left, top


def detect(net, frame, confidence=0.50, iou=0.45):
    """
    3-클래스 및 단일 클래스 YOLOv8 ONNX 출력을 모두 지원하는 검출 함수.
    출력 텐서 형태: (1, 4 + num_classes, 8400)
    반환값: [(box, score, class_id), ...]
    """
    blob, scale, left, top = letterbox(frame)
    net.setInput(blob)
    output = net.forward()
    if output.ndim != 3 or output.shape[0] != 1 or output.shape[1] < 5:
        return []

    boxes, scores, class_ids = [], [], []
    height, width = frame.shape[:2]

    # output[0]의 차원: (channels, 8400), channels = 4 + num_classes
    for row in output[0].T:
        cx, cy, box_width, box_height = row[:4]
        class_scores = row[4:]

        if len(class_scores) == 1:
            cls_id = 0
            score = float(class_scores[0])
        else:
            cls_id = int(np.argmax(class_scores))
            score = float(class_scores[cls_id])

        if score < confidence:
            continue

        x1 = max(0, min(width - 1, round((cx - box_width / 2 - left) / scale)))
        y1 = max(0, min(height - 1, round((cy - box_height / 2 - top) / scale)))
        x2 = max(0, min(width, round((cx + box_width / 2 - left) / scale)))
        y2 = max(0, min(height, round((cy + box_height / 2 - top) / scale)))
        if x2 > x1 and y2 > y1:
            boxes.append([x1, y1, x2 - x1, y2 - y1])
            scores.append(score)
            class_ids.append(cls_id)

    if len(boxes) == 0:
        return []

    indices = cv2.dnn.NMSBoxes(boxes, scores, confidence, iou)
    if len(indices) == 0:
        return []

    return [(boxes[int(idx)], scores[int(idx)], class_ids[int(idx)]) for idx in np.array(indices).flatten()]


class YoloDetectorNode:
    def __init__(self):
        rospy.init_node("yolo_detector_node", anonymous=False)

        self.bridge = CvBridge()
        self.tf_broadcaster = tf.TransformBroadcaster()

        # 1. 모델 경로 설정
        pkg_dir = Path(__file__).resolve().parent.parent
        default_model = pkg_dir / "box_yolov8n.onnx"

        self.model_path = rospy.get_param("~model_path", str(default_model))
        self.conf_threshold = rospy.get_param("~confidence", 0.50)
        self.iou_threshold = rospy.get_param("~iou", 0.45)
        self.camera_frame = rospy.get_param("~camera_frame", "camera_link")
        self.target_frame = rospy.get_param("~target_frame", "ap_target")
        self.min_target_depth = rospy.get_param("~min_depth", 0.25)  # 작업대 최소 유효 거리 (m)
        self.max_target_depth = rospy.get_param("~max_depth", 0.58)  # 작업대 최대 유효 거리 (바닥/배경 배제)
        self.show_window = rospy.get_param("~show_window", True)
        self.window_name = "L515 - Multi-Class Box Detection"

        # 2. 토픽 파라미터
        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_rect_raw")

        # 3. OpenCV DNN 모델 로드
        rospy.loginfo(f"[YOLO Node] Loading ONNX model from: {self.model_path}")
        if not os.path.isfile(self.model_path):
            rospy.logerr(f"[YOLO Node] Model file not found: {self.model_path}")
            return

        self.net = cv2.dnn.readNetFromONNX(str(self.model_path))
        rospy.loginfo("[YOLO Node] 3-Class YOLOv8 model loaded successfully via OpenCV DNN!")

        # 4. 카메라 내부 파라미터
        self.has_camera_info = False
        self.fx = 899.1
        self.fy = 899.6
        self.cx = 662.2
        self.cy = 380.5
        self.info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_cb)

        # 5. 상태 및 버퍼
        self.is_active = True
        self.vision_enabled = False  # 로봇이 초기 자세에 완전히 도착한 후 /vision_enable True 수신 시에만 활성화
        self.lock = threading.Lock()
        self.latest_rgb = None
        self.latest_depth = None
        self.rgb_count = 0
        self.depth_count = 0
        self.last_valid_depth = 0.518  # 작업대 기준 기본 안전 뎁스 초기화 (m)
        self.last_depth_time = rospy.Time(0)

        # 6. 제어 토픽 구독 및 퍼블리셔
        self.vision_enable_sub = rospy.Subscriber("/vision_enable", Bool, self.vision_enable_cb)
        self.stop_sub = rospy.Subscriber("/yolo_stop", Bool, self.stop_cb)

        self.debug_pub = rospy.Publisher("/yolo/detection_image", Image, queue_size=1)
        self.target_class_pub = rospy.Publisher("/yolo/target_class", Int32, queue_size=1)
        self.target_info_pub = rospy.Publisher("/yolo/target_info", String, queue_size=1)

        # 7. 영상 토픽 구독
        rospy.loginfo(f"[YOLO Node] Subscribing RGB: {self.rgb_topic}")
        rospy.loginfo(f"[YOLO Node] Subscribing Depth: {self.depth_topic}")

        self.rgb_sub = rospy.Subscriber(self.rgb_topic, Image, self.rgb_cb, queue_size=1)
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_cb, queue_size=1)

        rospy.loginfo("[YOLO Node] Ready! Starting multi-box vision processing loop...")

    def camera_info_cb(self, msg):
        if not self.has_camera_info:
            self.fx = msg.K[0] if msg.K[0] > 0 else 899.1
            self.cx = msg.K[2] if msg.K[2] > 0 else 662.2
            self.fy = msg.K[4] if msg.K[4] > 0 else 899.6
            self.cy = msg.K[5] if msg.K[5] > 0 else 380.5
            self.has_camera_info = True
            rospy.loginfo(f"[YOLO Node] Camera Intrinsics updated: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}")

    def vision_enable_cb(self, msg):
        self.vision_enabled = msg.data
        status_str = "ENABLED (Ready at Initial Pose)" if self.vision_enabled else "PAUSED (Robot Moving)"
        rospy.loginfo(f"[YOLO Node] Vision state updated: {status_str}")

    def stop_cb(self, msg):
        if msg.data and self.is_active:
            rospy.loginfo("[YOLO Node] Received final STOP signal from robot. Shutting down...")
            self.is_active = False
            rospy.signal_shutdown("Robot completed all pick & place cycles.")

    def rgb_cb(self, msg):
        self.rgb_count += 1
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.lock:
                self.latest_rgb = cv_img
        except CvBridgeError as e:
            rospy.logerr_throttle(2.0, f"[YOLO Node] CvBridge RGB error: {e}")

    def depth_cb(self, msg):
        self.depth_count += 1
        try:
            if msg.encoding in ("16UC1", "mono16"):
                depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            elif msg.encoding == "32FC1":
                depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            else:
                depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            rospy.loginfo_once(f"[YOLO Node] First Depth frame received! Resolution: {depth_img.shape[1]}x{depth_img.shape[0]}, Encoding: {msg.encoding}")

            with self.lock:
                self.latest_depth = depth_img
        except CvBridgeError as e:
            rospy.logerr_throttle(2.0, f"[YOLO Node] CvBridge Depth error: {e}")

    def get_box_depth(self, depth_img, rgb_box, rgb_shape):
        if depth_img is None:
            return 0.0

        rgb_h, rgb_w = rgb_shape[:2]
        depth_h, depth_w = depth_img.shape[:2]

        scale_x = depth_w / float(rgb_w)
        scale_y = depth_h / float(rgb_h)

        x, y, w, h = rgb_box
        dx = int(x * scale_x)
        dy = int(y * scale_y)
        dw = int(w * scale_x)
        dh = int(h * scale_y)

        cx = dx + dw // 2
        cy = dy + dh // 2
        r_w = max(4, int(dw * 0.18))
        r_h = max(4, int(dh * 0.18))

        x1 = max(0, cx - r_w)
        x2 = min(depth_w, cx + r_w + 1)
        y1 = max(0, cy - r_h)
        y2 = min(depth_h, cy + r_h + 1)

        patch = depth_img[y1:y2, x1:x2]
        raw_vals = patch[np.isfinite(patch) & (patch > 0)]

        def to_meters(v_array):
            if len(v_array) == 0:
                return v_array
            med_val = float(np.median(v_array))
            if med_val > 20.0:
                return v_array * 0.001
            return v_array

        m_vals = to_meters(raw_vals)
        table_vals = m_vals[(m_vals >= 0.25) & (m_vals <= 0.65)]

        if len(table_vals) < 5:
            x1 = max(0, dx + int(dw * 0.15))
            x2 = min(depth_w, dx + int(dw * 0.85))
            y1 = max(0, dy + int(dh * 0.15))
            y2 = min(depth_h, dy + int(dh * 0.85))
            if x2 > x1 and y2 > y1:
                patch = depth_img[y1:y2, x1:x2]
                raw_vals = patch[np.isfinite(patch) & (patch > 0)]
                m_vals = to_meters(raw_vals)
                table_vals = m_vals[(m_vals >= 0.25) & (m_vals <= 0.65)]

        if len(table_vals) >= 3:
            return float(np.median(table_vals))

        if self.last_valid_depth is not None and 0.25 <= self.last_valid_depth <= 0.60:
            return self.last_valid_depth

        return 0.518

    def run(self):
        rate = rospy.Rate(30)
        count = 0

        while not rospy.is_shutdown() and self.is_active:
            count += 1
            frame = None
            depth_map = None

            with self.lock:
                if self.latest_rgb is not None:
                    frame = self.latest_rgb.copy()
                if self.latest_depth is not None:
                    depth_map = self.latest_depth.copy()

            if frame is not None:
                rospy.loginfo_once(f"[YOLO Node] Video streaming active! Frame resolution: {frame.shape[1]}x{frame.shape[0]}")
                display_frame = frame.copy()

                detections = detect(self.net, frame, self.conf_threshold, self.iou_threshold)
                candidates = []

                for (x, y, width, height), score, cls_id in detections:
                    u_center = int(x + width / 2)
                    v_center = int(y + height / 2)
                    depth_z = self.get_box_depth(depth_map, (x, y, width, height), frame.shape)

                    # 뎁스 캐시 유지 (순간 결측 방지 및 작업대 범위 내 뎁스만 캐싱)
                    now = rospy.Time.now()
                    if self.min_target_depth <= depth_z <= self.max_target_depth:
                        self.last_valid_depth = depth_z
                        self.last_depth_time = now
                    elif self.last_valid_depth is not None and (now - self.last_depth_time).to_sec() < 1.0:
                        if depth_z <= 0.1 or depth_z > self.max_target_depth:
                            depth_z = self.last_valid_depth

                    cls_name = CLASS_NAMES.get(cls_id, f"box_{cls_id}")
                    color = CLASS_COLORS.get(cls_id, (0, 200, 80))

                    # 2D Bounding Box 시각화
                    cv2.rectangle(display_frame, (x, y), (x + width, y + height), color, 2)
                    cv2.circle(display_frame, (u_center, v_center), 4, (0, 0, 255), -1)

                    # [작업대(테이블) 유효 작업 깊이 필터링]:
                    # 바닥(Z > 1.0m)이나 적재함에 이미 떨어진 박스, 배경 물체를 배제하고
                    # 실제 작업대(테이블) 위에 놓인 박스만을 우선순위 타겟 후보로 등록
                    if self.min_target_depth <= depth_z <= self.max_target_depth:
                        label = f"[{cls_id}] {cls_name} {score:.2f} ({depth_z:.2f}m)"
                        target_x = (u_center - self.cx) * depth_z / self.fx
                        target_y = (v_center - self.cy) * depth_z / self.fy
                        target_z = depth_z

                        candidates.append({
                            "cls_id": cls_id,
                            "cls_name": cls_name,
                            "score": score,
                            "cam_coords": (target_x, target_y, target_z),
                            "pixel_center": (u_center, v_center),
                            "bbox": (x, y, width, height),
                            "depth": depth_z
                        })
                    elif depth_z > 0.1:
                        # 작업대 범위를 벗어난 바닥/배경 박스는 시각화만 하고 타겟 후보에서는 배제
                        label = f"[{cls_id}] {cls_name} {score:.2f} ({depth_z:.2f}m - OUT)"
                    else:
                        label = f"[{cls_id}] {cls_name} {score:.2f} (No Depth)"

                    cv2.putText(display_frame, label, (x, max(20, y - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 2)

                # ==============================================================
                # [우선순위 정렬]: 0번 오리지널 -> 1번 노란색 -> 2번 검은색 순서
                # 동일 클래스일 경우 신뢰도(score) 내림차순 정렬
                # ==============================================================
                best_target = None
                if len(candidates) > 0:
                    candidates.sort(key=lambda c: (c["cls_id"], -c["score"]))
                    best_target = candidates[0]

                if not self.vision_enabled:
                    # 로봇 팔이 물체를 집고 이동 중일 때는 TF 발행을 멈추고 안내 표시
                    cv2.rectangle(display_frame, (10, 10), (530, 42), (0, 140, 255), -1)
                    cv2.putText(display_frame, "ROBOT MOVING - VISION PAUSED", (15, 33),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                elif best_target is not None:
                    # 최고 우선순위 대상 하이라이트 (노란 이중 테두리)
                    bx, by, bw, bh = best_target["bbox"]
                    cv2.rectangle(display_frame, (bx - 3, by - 3), (bx + bw + 3, by + bh + 3), (0, 255, 255), 2)
                    tx, ty, tz = best_target["cam_coords"]

                    # 3D TF 발행
                    self.tf_broadcaster.sendTransform(
                        (tx, ty, tz),
                        (0.0, 0.0, 0.0, 1.0),
                        rospy.Time.now(),
                        self.target_frame,
                        self.camera_frame
                    )

                    # 클래스 ID 및 세부 정보 토픽 발행
                    self.target_class_pub.publish(best_target["cls_id"])

                    try:
                        info_dict = {
                            "class_id": best_target["cls_id"],
                            "class_name": best_target["cls_name"],
                            "score": round(best_target["score"], 3),
                            "cam_x": round(tx, 3),
                            "cam_y": round(ty, 3),
                            "cam_z": round(tz, 3),
                            "total_detected": len(candidates)
                        }
                        self.target_info_pub.publish(json.dumps(info_dict))
                    except Exception:
                        pass

                    rospy.loginfo_throttle(1.5, f"[YOLO Node] Priority Target [{best_target['cls_name']} (P{best_target['cls_id']})] "
                                                f"TF -> X: {tx:.3f}, Y: {ty:.3f}, Z: {tz:.3f}m [Score: {best_target['score']:.2f}, Total: {len(candidates)} boxes]")

                    # OSD 상단 정보 배너 (초록색)
                    info_text = f"TARGET: [Class {best_target['cls_id']}: {best_target['cls_name']}] X={tx:.2f} Y={ty:.2f} Z={tz:.2f}m (Total: {len(candidates)})"
                    cv2.rectangle(display_frame, (10, 10), (620, 42), (0, 150, 0), -1)
                    cv2.putText(display_frame, info_text, (15, 33), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 2)
                elif len(detections) > 0:
                    status_text = "BOX DETECTED - ACQUIRING DEPTH..."
                    if depth_map is None:
                        status_text = "BOX DETECTED - WAITING FOR DEPTH STREAM"
                    cv2.rectangle(display_frame, (10, 10), (450, 42), (0, 140, 255), -1)
                    cv2.putText(display_frame, status_text, (15, 33),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
                else:
                    cv2.rectangle(display_frame, (10, 10), (280, 42), (0, 0, 150), -1)
                    cv2.putText(display_frame, "SEARCHING FOR BOX...", (15, 33),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

                # 화면 표시
                if self.show_window:
                    cv2.imshow(self.window_name, display_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q")):
                        break

                # 디버그 영상 토픽 퍼블리시
                if self.debug_pub.get_num_connections() > 0:
                    try:
                        out_msg = self.bridge.cv2_to_imgmsg(display_frame, encoding="bgr8")
                        self.debug_pub.publish(out_msg)
                    except CvBridgeError:
                        pass
            else:
                waiting_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                waiting_frame[:] = (35, 35, 35)
                dots = "." * (1 + (count // 10) % 4)
                cv2.putText(waiting_frame, "WAITING FOR CAMERA STREAM" + dots, (40, 220),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
                cv2.putText(waiting_frame, f"RGB Topic: {self.rgb_topic}", (40, 260),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
                if self.show_window:
                    cv2.imshow(self.window_name, waiting_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q")):
                        break

            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

        cv2.destroyAllWindows()


def main():
    node = YoloDetectorNode()
    node.run()


if __name__ == "__main__":
    main()
