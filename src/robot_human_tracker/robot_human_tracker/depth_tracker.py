##!/usr/bin/env python3

import os
import time
import threading

import cv2
import numpy as np
import torch.nn.functional as F
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
)

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Vector3, TransformStamped
from std_msgs.msg import Int32, String

from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory

from ultralytics import YOLO

# RE-ID
from robot_human_tracker.person_reid import PersonReID


class DepthTracker(Node):
    def __init__(self):
        super().__init__("depth_tracker_node")

        # ============================================================
        # CONFIG
        # ============================================================
        #
        # self.rgb_topic = "/camera/camera/color/image_raw"
        # self.depth_topic = "/camera/camera/aligned_depth_to_color/image_raw"
        # self.info_topic = "/camera/camera/color/camera_info"
        self.rgb_topic = "/camera/camera/infra1/image_rect_raw"
        self.depth_topic = "/camera/camera/depth/image_rect_raw"
        self.info_topic = "/camera/camera/infra1/camera_info"
        self.output_topic = "/human_tracker/output"
        self.target_topic = "/human_tracker/target"
        self.lock_id_topic = "/human_tracker/lock_id"
        self.mode_topic = "/human_tracker/mode"
        self.conf = 0.40

        # SEARCHING: / ki
        #   Chưa LOCK target
        #
        # TRACKING:
        #   Đang theo target đã LOCK
        #
        # LOST:
        #   Target bị mất, đang tìm candidate để reacquire

        self.mode = "IDLE"
        self.locked_id = None
        # Target currently valid
        self.target_found = False
        # ============================================================
        # RE-ID
        # ============================================================

        self.reid = PersonReID(device="cuda")

        # Appearance embedding của người đã LOCK
        self.target_embedding = None

        # Ngưỡng RE-ID
        self.reid_threshold = 0.70

        # Nếu ByteTrack đổi ID, cho phép tìm lại target
        self.reid_enabled = True

        self.imgsz = 320

        self.output_width = 640
        self.output_height = 360
        self.output_fps = 10.0
        # Camera intrinsics - sẽ được cập nhật từ CameraInfo
        self.fx = 600.0
        self.fy = 600.0
        self.cx = 640.0
        self.cy = 360.0
        # Log
        self.last_log = time.perf_counter()
        self.frame_count = 0
        self.last_fps_time = time.perf_counter()
        # ROS
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.pub_image = self.create_publisher(
            Image,
            self.output_topic,
            image_qos,
        )

        self.pub_target = self.create_publisher(
            Vector3,
            self.target_topic,
            10,
        )

        self.sub_lock_id = self.create_subscription(
            Int32, self.lock_id_topic, self.lock_id_callback, 10
        )
        self.sub_mode_state = self.create_subscription(
            String, self.mode_topic, self.mode_callback, 10
        )
        # ============================================================
        # YOLO
        # ============================================================

        pkg_share = get_package_share_directory("robot_human_tracker")
        model_dir = os.path.join(pkg_share, "models")

        model_path = os.path.join(model_dir, "yolov8n.pt")

        if model_path is None:
            raise FileNotFoundError("model path not found")

        self.get_logger().info(f"Loading: {model_path}")

        self.model = YOLO(model_path)

        # ============================================================
        # CAMERA
        # ============================================================

        self.rgb_sub = Subscriber(
            self,
            Image,
            self.rgb_topic,
            qos_profile=qos_profile_sensor_data,
        )

        self.depth_sub = Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=qos_profile_sensor_data,
        )

        self.info_sub = Subscriber(
            self,
            CameraInfo,
            self.info_topic,
            qos_profile=qos_profile_sensor_data,
        )

        self.sync = ApproximateTimeSynchronizer(
            [
                self.rgb_sub,
                self.depth_sub,
                self.info_sub,
            ],
            queue_size=10,
            slop=0.10,
        )

        self.sync.registerCallback(self.callback)

        # ============================================================
        # ASYNC IMAGE PUBLISH
        # ============================================================

        self.image_lock = threading.Lock()
        self.latest_image = None
        self.latest_header = None

        self.output_thread = threading.Thread(
            target=self.image_worker,
            daemon=True,
        )
        self.output_thread.start()

        self.get_logger().info("================================")
        self.get_logger().info("Human Tracker started")
        self.get_logger().info(f"RGB   : {self.rgb_topic}")
        self.get_logger().info(f"Depth : {self.depth_topic}")
        self.get_logger().info(f"Output: {self.output_topic}")
        self.get_logger().info("Mode  : YOLO Person Detection")
        self.get_logger().info("================================")

    # ================================================================
    # CALLBACK
    # ================================================================

    def callback(self, rgb_msg, depth_msg, info_msg):

        start = time.perf_counter()

        # CameraInfo
        self.fx = float(info_msg.k[0])
        self.fy = float(info_msg.k[4])
        self.cx = float(info_msg.k[2])
        self.cy = float(info_msg.k[5])

        # ------------------------------------------------------------
        # Convert RGB
        # ------------------------------------------------------------

        try:
            image = self.bridge.imgmsg_to_cv2(
                rgb_msg,
                "bgr8",
            )

            depth = self.bridge.imgmsg_to_cv2(
                depth_msg,
                "passthrough",
            )

        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        # YOLO

        # result = self.model(
        #     image,
        #     imgsz=self.imgsz,
        #     conf=self.conf,
        #     classes=[0],
        #     device=0,
        #     verbose=False,
        # )[0]
        result = self.model.track(
            image,
            imgsz=self.imgsz,
            conf=self.conf,
            classes=[0],
            device=0,
            tracker="bytetrack.yaml",
            persist=True,
            verbose=False,
        )[0]
        # Find person

        person = self.find_person(result, depth, image)
        display = image.copy()

        # ------------------------------------------------------------
        # PERSON FOUND
        # ------------------------------------------------------------

        if person is not None:
            x1, y1, x2, y2 = person["box"]

            distance = person["depth"]
            center_x = person["cx"]

            # Lateral position
            lateral = (center_x - self.cx) * distance / self.fx

            # Camera X right -> Robot Y left
            lateral = -lateral

            # --------------------------------------------------------
            # TARGET
            # --------------------------------------------------------

            msg = Vector3()
            msg.x = float(distance)
            msg.y = float(lateral)
            msg.z = 1.0

            self.pub_target.publish(msg)
            # TF
            self.publish_tf(
                rgb_msg.header,
                distance,
                lateral,
            )
            # DRAW
            cv2.rectangle(
                display,
                (x1, y1),
                (x2, y2),
                (0, 255, 0),
                2,
            )

            cv2.circle(
                display,
                (person["cx"], person["cy"]),
                6,
                (0, 0, 255),
                -1,
            )

            cv2.putText(
                display,
                # f"Person {person['conf']:.2f}",
                f"ID {person['track_id']} | Conf {person['conf']:.2f}",
                (x1, max(20, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                display,
                f"MODE: {self.mode}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 0),
                2,
            )

            cv2.putText(
                display,
                f"D: {distance:.2f} m",
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )

            cv2.putText(
                display,
                f"Y: {lateral:.2f} m",
                (20, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )

            detected = True

        # NO PERSON

        else:
            msg = Vector3()
            msg.x = 0.0
            msg.y = 0.0
            msg.z = 0.0

            self.pub_target.publish(msg)

            cv2.putText(
                display,
                f"MODE: {self.mode}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 0),
                2,
            )

            distance = 0.0
            lateral = 0.0
            detected = False
        # Resize
        display = cv2.resize(
            display,
            (
                self.output_width,
                self.output_height,
            ),
            interpolation=cv2.INTER_AREA,
        )

        # Latest image only

        with self.image_lock:
            self.latest_image = display
            self.latest_header = rgb_msg.header

        # LOG

        self.frame_count += 1

        now = time.perf_counter()

        if now - self.last_fps_time >= 1.0:
            fps = self.frame_count / (now - self.last_fps_time)

            self.frame_count = 0
            self.last_fps_time = now

            if detected:
                self.get_logger().info(
                    # f"FPS: {fps:.1f} | "
                    f"PERSON: YES | "
                    f"Conf: {person['conf']:.2f} | "
                    f"Distance: {distance:.2f} m | "
                    f"Lateral: {lateral:.2f} m | "
                    f"Process: "
                    f"{(now - start) * 1000:.1f} ms"
                )

            else:
                self.get_logger().info(
                    # f"FPS: {fps:.1f} | "
                    f"PERSON: NO | Process: {(now - start) * 1000:.1f} ms"
                )

    # FIND PERSON

    def find_person(self, result, depth, image):
        if self.mode == "IDLE":
            return None

        if result.boxes is None:
            return None

        h, w = depth.shape[:2]

        candidates = []

        # ============================================================
        # EXTRACT ALL PERSON CANDIDATES
        # ============================================================

        for box in result.boxes:
            if box.id is None:
                continue

            track_id = int(box.id[0].detach().cpu().item())

            conf = float(box.conf[0].detach().cpu().item())

            if conf < self.conf:
                continue

            xyxy = box.xyxy[0].detach().cpu().numpy().astype(int)

            x1, y1, x2, y2 = xyxy

            x1 = max(0, min(x1, w - 1))
            x2 = max(0, min(x2, w - 1))

            y1 = max(0, min(y1, h - 1))
            y2 = max(0, min(y2, h - 1))

            if x2 <= x1 or y2 <= y1:
                continue

            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            # ========================================================
            # DEPTH
            # ========================================================

            rx1 = int(x1 + (x2 - x1) * 0.35)
            rx2 = int(x2 - (x2 - x1) * 0.35)

            ry1 = int(y1 + (y2 - y1) * 0.25)
            ry2 = int(y2 - (y2 - y1) * 0.25)

            roi = depth[ry1:ry2, rx1:rx2]

            valid = roi[(roi > 100) & (roi < 5000)]

            if valid.size == 0:
                continue

            distance = float(np.median(valid)) / 1000.0

            if distance <= 0:
                continue

            bbox = (x1, y1, x2, y2)

            candidates.append(
                {
                    "box": bbox,
                    "cx": cx,
                    "cy": cy,
                    "depth": distance,
                    "conf": conf,
                    "track_id": track_id,
                }
            )

        if not candidates:
            return None

        # CHƯA LOCK ID
        #
        # SEARCHING:
        # chọn người confidence cao nhất

        if self.locked_id is None:
            return max(candidates, key=lambda p: p["conf"])

        # ĐÃ LOCK ID
        # CASE 1:
        # ByteTrack vẫn giữ nguyên ID

        locked_candidate = None

        for person in candidates:
            if person["track_id"] == self.locked_id:
                locked_candidate = person
                break

        if locked_candidate is not None:
            # --------------------------------------------------------
            # Nếu chưa có RE-ID reference thì tạo reference
            # --------------------------------------------------------

            if self.target_embedding is None:
                embedding = self.reid.extract(
                    image,
                    locked_candidate["box"],
                )

                if embedding is not None:
                    self.target_embedding = embedding.clone()

                    self.get_logger().info("RE-ID reference created")

            # --------------------------------------------------------
            # Cập nhật appearance reference
            # --------------------------------------------------------

            else:
                new_embedding = self.reid.extract(
                    image,
                    locked_candidate["box"],
                )

                if new_embedding is not None:
                    self.target_embedding = F.normalize(
                        0.8 * self.target_embedding + 0.2 * new_embedding,
                        p=2,
                        dim=1,
                    )

            locked_candidate["reid_score"] = 1.0

            return locked_candidate

        # ============================================================
        # CASE 2:
        # BYTE TRACK ĐỔI ID
        #
        # Không còn locked_id
        # → dùng RE-ID tìm lại target
        # ============================================================

        if self.reid_enabled and self.target_embedding is not None:
            best_candidate = None
            best_score = -1.0

            for person in candidates:
                score = self.reid.similarity(
                    self.target_embedding,
                    image,
                    person["box"],
                )

                person["reid_score"] = score

                if score > best_score:
                    best_score = score
                    best_candidate = person

            # --------------------------------------------------------
            # RE-ID MATCH
            # --------------------------------------------------------

            if best_candidate is not None and best_score >= self.reid_threshold:
                old_id = self.locked_id
                new_id = best_candidate["track_id"]

                best_candidate["track_id"] = old_id

                self.get_logger().info(
                    f"RE-ID REACQUIRED: {new_id} -> {old_id} (score={best_score:.3f})"
                )

                # Update reference
                new_embedding = self.reid.extract(
                    image,
                    best_candidate["box"],
                )

                if new_embedding is not None:
                    self.target_embedding = F.normalize(
                        0.8 * self.target_embedding + 0.2 * new_embedding,
                        p=2,
                        dim=1,
                    )

                return best_candidate

        # ============================================================
        # KHÔNG TÌM ĐƯỢC TARGET
        # ============================================================

        return None

    def publish_tf(self, header, depth, lateral):

        tf = TransformStamped()

        tf.header.stamp = header.stamp
        tf.header.frame_id = "camera_link"
        tf.child_frame_id = "human_target"

        tf.transform.translation.x = float(depth)
        tf.transform.translation.y = float(lateral)
        tf.transform.translation.z = 0.0

        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf)

    def image_worker(self):

        period = 1.0 / self.output_fps

        while rclpy.ok():
            with self.image_lock:
                image = self.latest_image
                header = self.latest_header

                self.latest_image = None
                self.latest_header = None

            if image is None:
                time.sleep(0.01)
                continue

            try:
                self.publish_image(
                    image,
                    header,
                )

            except Exception as e:
                self.get_logger().error(f"Image publish error: {e}")

            time.sleep(period)

    def publish_image(self, image, header):

        msg = Image()

        msg.header = header
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = image.shape[1] * 3
        msg.data = image.tobytes()

        self.pub_image.publish(msg)

    def mode_callback(self, msg):
        mode = msg.data.strip().upper()

        if mode == "SEARCHING":
            self.mode = "SEARCHING"
            self.target_found = False

            self.get_logger().info("Mode: SEARCHING")

        elif mode == "IDLE":
            self.mode = "IDLE"
            self.target_found = False
            self.publish_stop_target()

            self.get_logger().info("Mode: IDLE")

        else:
            self.get_logger().warn(f"Unknown mode: {mode}")

    def lock_id_callback(self, msg):
        new_id = int(msg.data)
        # ------------------------------------------------------------
        # DELETE ID
        # ------------------------------------------------------------
        if new_id <= 0:
            self.locked_id = None
            self.mode = "SEARCHING"
            self.target_found = False

            self.publish_stop_target()

            self.get_logger().info("LOCK ID detele -> SEARCHING")

            return
        else:
            self.locked_id = new_id
            self.mode = "TRACKING"
            self.target_found = False

            self.target_embedding = None

            self.get_logger().info(f"LOCK TARGET ID = {self.locked_id}")

    # ================================================================
    # PUBLISH STOP TARGET
    # ================================================================

    def publish_stop_target(self):

        msg = Vector3()

        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0

        self.pub_target.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = DepthTracker()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(f"DepthTracker fatal error: {e}")

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
