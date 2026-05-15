#!/usr/bin/env python3
# encoding: utf-8
import base64
import collections
import csv
import json
import os
import queue
import re
import signal
import subprocess
import threading
import time
from datetime import datetime
from urllib.parse import urlparse

import cv2
import numpy as np
import requests
import rospy
from geometry_msgs.msg import Twist
from hiwonder_interfaces.msg import ObjectsInfo
from hiwonder_servo_controllers.bus_servo_control import set_servos
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Float64, String
from std_srvs.srv import Trigger


APPLICATION_TABLE = {
    "collision_avoidance": {"latency_sensitivity": "high", "accuracy_priority": "low"},
    "collision_detection": {"latency_sensitivity": "high", "accuracy_priority": "high"},
    "pedestrian_avoidance": {"latency_sensitivity": "high", "accuracy_priority": "high"},
    "pedestrian_detection": {"latency_sensitivity": "high", "accuracy_priority": "high"},
    "localization": {"latency_sensitivity": "high", "accuracy_priority": "high"},
    "traffic_light_detection": {"latency_sensitivity": "low", "accuracy_priority": "high"},
    "traffic_sign_detection": {"latency_sensitivity": "low", "accuracy_priority": "high"},
    "lane_detection": {"latency_sensitivity": "high", "accuracy_priority": "high"},
    "depth_estimation": {"latency_sensitivity": "high", "accuracy_priority": "high"},
    "drowsiness_detection": {"latency_sensitivity": "medium", "accuracy_priority": "high"},
    "drivable_area": {"latency_sensitivity": "high", "accuracy_priority": "high"},
    "infotainment": {"latency_sensitivity": "low", "accuracy_priority": "low"},
}


def get_application_table(application_name):
    return APPLICATION_TABLE.get(application_name)


def clamp(value, minimum=0.0, maximum=1.0):
    return max(minimum, min(maximum, value))


class LoadBalancerNode:
    def __init__(self, name):
        rospy.loginfo("Load balancer node initializing...")
        rospy.init_node(name, anonymous=True)
        self.name = name
        self.image = None
        self.is_running = True
        self.machine_type = os.environ.get("MACHINE_TYPE")
        self.lock = threading.Lock()
        self.session = requests.Session()

        self.frame_timing = {}
        self.frame_id = 0
        self.last_processing_time = 0.0

        self.upload_speed = 0.0
        self.download_speed = 0.0
        self.rtt_ms = None
        self.jitter_ms = 0.0
        self.network_ok = False
        rospy.Subscriber("/network/bandwidth", Float32MultiArray, self.bandwidth_callback)

        signal.signal(signal.SIGINT, self.shutdown)

        self._resource_cache = {
            "cpu_percent": 0.0,
            "gpu_percent": 0.0,
            "ram_percent": 0.0,
            "power_mw": 0.0,
            "power_avg_mw": 0.0,
        }
        self._resource_lock = threading.Lock()
        self._power_window = collections.deque(maxlen=15)
        threading.Thread(target=self._poll_tegrastats, daemon=True).start()

        self.mecanum_pub = rospy.Publisher("/hiwonder_controller/cmd_vel", Twist, queue_size=1)
        self.joints_pub = rospy.Publisher(
            "servo_controllers/port_id_1/multi_id_pos_dur", MultiRawIdPosDur, queue_size=1
        )

        self.lane_img_pub = rospy.Publisher("/load_balancer/lane_image", Image, queue_size=1)
        self.yolo_img_pub = rospy.Publisher("/load_balancer/yolo_image", Image, queue_size=1)
        self.yolo_traffic = rospy.Publisher("/load_balancer/yolo/traffic_image", Image, queue_size=3)
        self.yolo_obj = rospy.Publisher("/load_balancer/yolo/object_image", Image, queue_size=3)
        self.binary_img_pub = rospy.Publisher("/load_balancer/lane_binary_image", Image, queue_size=1)
        self.result_img_pub = rospy.Publisher("/load_balancer/lane_result_image", Image, queue_size=1)

        self.lane_pub = rospy.Publisher("/load_balancer/lane_detection", String, queue_size=1)
        self.obj_pub = rospy.Publisher("/load_balancer/object_detections", String, queue_size=1)
        self.traffic_pub = rospy.Publisher("/load_balancer/traffic_sign_detections", String, queue_size=1)
        self.start_pub = rospy.Publisher("/load_balaner/node_start_time", Float64, queue_size=1)

        self.server_url = rospy.get_param("~edge_server_url")
        parsed_url = urlparse(self.server_url)
        self.edge_host = parsed_url.hostname or "127.0.0.1"
        self.edge_port = parsed_url.port or (443 if parsed_url.scheme == "https" else 80)

        self.edge_target_width = rospy.get_param("~edge_target_width", 640)
        self.cloud_target_width = rospy.get_param("~cloud_target_width", 960)
        self.cloud_target_height = rospy.get_param("~cloud_target_height", 1080)
        self.low_res_scale = rospy.get_param("~low_res_scale", 0.65)
        self.edge_timeout = rospy.get_param("~edge_timeout", 2.0)
        self.cloud_delay_threshold = rospy.get_param("~cloud_delay_threshold", 0.75)
        self.dual_path_threshold = rospy.get_param("~dual_path_threshold", 0.80)
        self.dual_path_margin = rospy.get_param("~dual_path_margin", 0.10)
        self.power_budget_mw = rospy.get_param("~power_budget_mw", 3200.0)
        self.resource_threshold = rospy.get_param("~resource_threshold", 80.0)
        self.bandwidth_high_threshold = rospy.get_param("~bandwidth_high_threshold", 7.0)
        self.bandwidth_low_threshold = rospy.get_param("~bandwidth_low_threshold", 2.0)
        self.min_processing_interval = rospy.get_param("~min_processing_interval", 0.08)
        self.max_cloud_queue = rospy.get_param("~max_cloud_queue", 4)
        self.cloud_roi_headers_only = rospy.get_param("~cloud_roi_headers_only", True)

        self.applications = rospy.get_param("~applications")
        self.edge_server_available = True
        self.last_edge_check = 0.0
        self.edge_check_interval = rospy.get_param("~edge_check_interval", 2.0)

        self.scene_gray = None
        self.scene_embedding = None
        self.scene_profile = {"motion_score": 0.0, "scene_complexity": 0.0, "change_score": 0.0}

        self.lane_data = {}
        self.detection_results = []
        self.traffic_sign_results = []

        self.edge_state = {
            "lane_detection": {"data": None, "timestamp": 0.0, "source": "onboard", "uncertainty": 1.0},
            "collision_avoidance": {"data": [], "timestamp": 0.0, "source": "onboard", "uncertainty": 1.0},
            "traffic_sign_detection": {"data": [], "timestamp": 0.0, "source": "onboard", "uncertainty": 1.0},
        }
        self.cloud_state = {
            "lane_detection": {"data": None, "timestamp": 0.0, "source": "cloud"},
            "collision_avoidance": {"data": [], "timestamp": 0.0, "source": "cloud"},
            "traffic_sign_detection": {"data": [], "timestamp": 0.0, "source": "cloud"},
        }
        self.fused_state = {
            "lane_detection": {"data": None, "timestamp": 0.0, "source": "fused"},
            "collision_avoidance": {"data": [], "timestamp": 0.0, "source": "fused"},
            "traffic_sign_detection": {"data": [], "timestamp": 0.0, "source": "fused"},
        }

        self.cloud_queue = queue.Queue(maxsize=self.max_cloud_queue)
        self.cloud_inflight = 0
        self.cloud_lock = threading.Lock()
        threading.Thread(target=self._cloud_worker, daemon=True).start()

        log_dir = os.path.expanduser(
            "~/ros_ws/src/hiwonder_example/scripts/netx_autonomous_vehicle/load_balancer/load_balancer_logs"
        )
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(log_dir, f"lb_log_{timestamp}.csv")
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            [
                "frame_id",
                "timestamp",
                "application",
                "decision",
                "execution_location",
                "bandwidth_mbps",
                "cpu_percent",
                "gpu_percent",
                "power_mw",
                "ram_percent",
                "rtt_ms",
                "jitter_ms",
                "motion_score",
                "scene_complexity",
                "change_score",
                "edge_score",
                "cloud_score",
                "e2e_latency_sec",
            ]
        )
        rospy.loginfo("CSV logging: %s", self.csv_path)

        rospy.Subscriber("/lane_detection/result", String, self.onboard_lane_callback, queue_size=1)
        rospy.Subscriber(
            "/load_balancer/yolov5/object_detect", ObjectsInfo, self.onboard_object_callback, queue_size=1
        )
        rospy.Subscriber(
            "/load_balancer/yolov5/traffic_detect", ObjectsInfo, self.onboard_traffic_callback, queue_size=1
        )

        depth_camera = rospy.get_param("/depth_camera/camera_name", "depth_cam")
        rospy.Subscriber(
            f"/{depth_camera}/rgb/image_raw",
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**22,
        )

        self.wait_for_services()
        self.initialize_servos()
        rospy.loginfo("LoadBalancerNode initialized.")
        rospy.spin()

    def _poll_tegrastats(self):
        while self.is_running:
            try:
                proc = subprocess.Popen(
                    ["tegrastats"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    universal_newlines=True,
                )
                for line in proc.stdout:
                    if not self.is_running:
                        break

                    cache = {}
                    ram_match = re.search(r"RAM (\d+)/(\d+)MB", line)
                    if ram_match:
                        cache["ram_percent"] = round(
                            int(ram_match.group(1)) / int(ram_match.group(2)) * 100.0, 2
                        )

                    cpu_match = re.search(r"CPU \[([^\]]+)\]", line)
                    if cpu_match:
                        usages = []
                        for core in cpu_match.group(1).split(","):
                            match = re.search(r"(\d+)%", core)
                            if match:
                                usages.append(int(match.group(1)))
                        if usages:
                            cache["cpu_percent"] = round(sum(usages) / float(len(usages)), 2)

                    gpu_match = re.search(r"GR3D_FREQ (\d+)%", line)
                    if gpu_match:
                        cache["gpu_percent"] = int(gpu_match.group(1))

                    power_match = re.search(
                        r"POM_5V_IN\s+(\d+)(?:mW)?/(\d+)(?:mW)?",
                        line
                    )
                    if power_match:
                        power_now = float(power_match.group(1))
                        self._power_window.append(power_now)
                        cache["power_mw"] = power_now
                        cache["power_avg_mw"] = round(
                            sum(self._power_window) / float(len(self._power_window)), 2
                        )

                    if cache:
                        with self._resource_lock:
                            self._resource_cache.update(cache)

                proc.kill()
            except Exception as exc:
                rospy.logwarn_throttle(10.0, f"[tegrastats poller] error: {exc}")
                time.sleep(2.0)

    def _get_resource_usage(self):
        with self._resource_lock:
            return dict(self._resource_cache)

    def wait_for_services(self):
        if not rospy.get_param("~only_line_follow", False):
            while not rospy.is_shutdown():
                try:
                    if rospy.get_param("/yolov5_lb/init_finish"):
                        break
                except Exception:
                    rospy.sleep(0.1)
            try:
                rospy.ServiceProxy("/yolov5_lb/start", Trigger)()
            except Exception:
                rospy.logwarn("Failed to start YOLOv5 service")

    def initialize_servos(self):
        while not rospy.is_shutdown():
            try:
                if rospy.get_param("/hiwonder_servo_manager/init_finish") and rospy.get_param(
                    "/joint_states_publisher/init_finish"
                ):
                    break
            except Exception:
                rospy.sleep(0.1)
        set_servos(self.joints_pub, 1000, ((6, 490),))
        rospy.sleep(1)
        self.mecanum_pub.publish(Twist())

    def shutdown(self, signum, frame):
        self.is_running = False
        rospy.loginfo("Shutting down LoadBalancerNode...")
        try:
            self.csv_file.close()
        except Exception:
            pass
        try:
            self.session.close()
        except Exception:
            pass

    def image_callback(self, ros_image):
        try:
            encoding = ros_image.encoding.lower()
            if encoding not in ("rgb8", "bgr8"):
                rospy.logwarn_throttle(5.0, f"Unsupported camera encoding: {ros_image.encoding}")
                return

            raw = np.frombuffer(ros_image.data, dtype=np.uint8)
            image = raw.reshape((ros_image.height, ros_image.width, 3))
            if encoding == "bgr8":
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            self.image = image
            self.handle_frame(image, ros_image)
        except Exception as exc:
            rospy.logerr(f"Error in image_callback: {exc}")

    def bandwidth_callback(self, msg):
        data = list(msg.data)
        if len(data) >= 2:
            self.upload_speed = float(data[0])
            self.download_speed = float(data[1])
        if len(data) >= 4:
            self.rtt_ms = None if data[2] < 0 else float(data[2])
            self.jitter_ms = float(data[3])
        if len(data) >= 5:
            self.network_ok = bool(round(data[4]))
        else:
            self.network_ok = self.upload_speed > 0.0 or self.download_speed > 0.0

    def handle_frame(self, frame, ros_image):
        now = time.time()
        if now - self.last_processing_time < self.min_processing_interval:
            return
        self.last_processing_time = now

        self.frame_id += 1
        capture_time = ros_image.header.stamp.to_sec() if ros_image.header.stamp else now
        if capture_time <= 0:
            capture_time = now
        frame_uuid = f"{self.frame_id}_{int(capture_time * 1e9)}"
        self.frame_timing[frame_uuid] = {"t_capture": capture_time}
        self.start_pub.publish(capture_time)

        if len(self.frame_timing) > 500:
            oldest = sorted(self.frame_timing.keys())[:250]
            for key in oldest:
                self.frame_timing.pop(key, None)

        try:
            usage = self._get_resource_usage()
            profile = self.profile_frame(frame)
            edge_available = self.check_edge_server_available()

            for app in self.applications:
                profile_snapshot = dict(profile)
                routing = get_application_table(app)
                if routing is None:
                    rospy.logwarn(f"Application '{app}' not in table, routing onboard.")
                    self.forward_to_onboard(app, ros_image, frame_override=frame)
                    self.log_row(
                        frame_uuid,
                        app,
                        "unknown",
                        "onboard",
                        self.upload_speed,
                        usage["cpu_percent"],
                        usage["gpu_percent"],
                        usage["power_mw"],
                        usage["ram_percent"],
                        profile=profile_snapshot,
                        scores={"edge_score": 0.0, "cloud_score": 0.0},
                    )
                    continue

                decision = self.decide_processing_location(
                    routing["latency_sensitivity"],
                    routing["accuracy_priority"],
                    usage["cpu_percent"],
                    usage["ram_percent"],
                    usage["gpu_percent"],
                    self.upload_speed,
                    self.download_speed,
                    edge_available,
                    app=app,
                    profile=profile_snapshot,
                    power_mw=usage["power_avg_mw"],
                )

                if decision["publish_cached"]:
                    self.publish_cached_result(app)

                onboard_dispatched = False
                cloud_dispatched = False

                if decision["route"] in ("onboard", "dual_path"):
                    self.forward_to_onboard(app, ros_image, frame_override=frame)
                    onboard_dispatched = True

                if decision["route"] in ("offboard", "lower_resolution", "dual_path"):
                    cloud_frame = frame
                    location_label = "edge"
                    if decision["route"] == "lower_resolution":
                        cloud_frame = self.lower_resolution(frame)
                        location_label = "edge_low_res"

                    self.forward_to_edge(
                        cloud_frame,
                        app,
                        frame_uuid,
                        self.upload_speed,
                        usage["cpu_percent"],
                        usage["gpu_percent"],
                        usage["power_mw"],
                        usage["ram_percent"],
                        location_label,
                        capture_time=capture_time,
                        profile=profile_snapshot,
                        scores=decision,
                    )
                    cloud_dispatched = True

                if onboard_dispatched:
                    self.log_row(
                        frame_uuid,
                        app,
                        decision["route"],
                        "onboard",
                        self.upload_speed,
                        usage["cpu_percent"],
                        usage["gpu_percent"],
                        usage["power_mw"],
                        usage["ram_percent"],
                        profile=profile_snapshot,
                        scores=decision,
                    )

                if not onboard_dispatched and not cloud_dispatched:
                    self.forward_to_onboard(app, ros_image, frame_override=frame)
                    self.log_row(
                        frame_uuid,
                        app,
                        decision["route"],
                        "onboard_fallback",
                        self.upload_speed,
                        usage["cpu_percent"],
                        usage["gpu_percent"],
                        usage["power_mw"],
                        usage["ram_percent"],
                        profile=profile_snapshot,
                        scores=decision,
                    )

        except Exception as exc:
            rospy.logerr(f"Error in handle_frame: {exc}")

    def profile_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        flow_gray = cv2.resize(gray, (160, 90))
        embedding = cv2.resize(gray, (16, 16)).astype(np.float32) / 255.0

        motion_score = 0.0
        if self.scene_gray is not None:
            flow = cv2.calcOpticalFlowFarneback(
                self.scene_gray,
                flow_gray,
                None,
                0.5,
                3,
                15,
                3,
                5,
                1.2,
                0,
            )
            magnitude, _ = cv2.cartToPolar(flow[..., 0], flow[..., 1])
            motion_score = float(np.mean(magnitude))

        edge_density = float(np.count_nonzero(cv2.Canny(gray, 60, 160))) / float(gray.size)
        texture_density = float(cv2.Laplacian(gray, cv2.CV_32F).var())
        scene_complexity = clamp(edge_density * 2.5 + texture_density / 2500.0)

        change_score = 0.0
        if self.scene_embedding is not None:
            change_score = float(np.mean(np.abs(embedding - self.scene_embedding)))

        self.scene_gray = flow_gray
        self.scene_embedding = embedding
        self.scene_profile = {
            "motion_score": round(motion_score, 4),
            "scene_complexity": round(scene_complexity, 4),
            "change_score": round(change_score, 4),
        }
        return self.scene_profile

    def decide_processing_location(
        self,
        latency_sensitivity,
        accuracy_priority,
        cpu_usage,
        ram_usage,
        gpu_usage,
        upload_speed,
        download_speed,
        edge_available,
        app=None,
        profile=None,
        power_mw=None,
    ):
        profile = profile or self.scene_profile

        latency_critical = latency_sensitivity == "high"
        tracker_uncertainty = self.get_tracker_uncertainty(app)
        fresh_required = tracker_uncertainty > 0.55 or profile["change_score"] > 0.2 or profile["motion_score"] > 1.4

        if not self.network_ok or not edge_available:
            rospy.loginfo("[DECISION] network unavailable, forcing onboard")
            return {
                "route": "onboard",
                "edge_score": 1.0,
                "cloud_score": 0.0,
                "publish_cached": False,
                "force_fresh": True,
            }

        cpu_norm = clamp((cpu_usage or 0.0) / 100.0)
        gpu_norm = clamp((gpu_usage or 0.0) / 100.0)
        ram_norm = clamp((ram_usage or 0.0) / 100.0)
        queue_depth = clamp(float(self.get_cloud_queue_depth()) / float(max(self.max_cloud_queue, 1)))

        rtt_penalty = clamp(((self.rtt_ms or 120.0) / 150.0))
        jitter_penalty = clamp(self.jitter_ms / 80.0)
        # bandwidth_quality = clamp(
        #     min(upload_speed, download_speed) / float(max(self.bandwidth_high_threshold, 0.1))
        # )
        bandwidth_quality = clamp((1.0 - rtt_penalty) * 0.5+ (1.0 - jitter_penalty) * 0.3+ (1.0 - queue_depth) * 0.2)
        low_latency = clamp(1.0 - ((rtt_penalty * 0.65) + (jitter_penalty * 0.2) + (queue_depth * 0.15)))
        low_compute_load = clamp(1.0 - max(cpu_norm, gpu_norm, ram_norm))
        low_motion = clamp(1.0 - min(profile["motion_score"] / 4.0, 1.0))
        high_motion = clamp(1 - low_motion)
        power_saving = clamp(1.0 - ((power_mw or 0.0) / float(max(self.power_budget_mw * 1.25, 1.0))))

        accuracy_bias = 1.0 if accuracy_priority == "high" else 0.65 if accuracy_priority == "medium" else 0.35
        high_accuracy_need = clamp(
            (accuracy_bias * 0.55)
            + (profile["scene_complexity"] * 0.2)
            + (profile["change_score"] * 0.1)
            + (tracker_uncertainty * 0.15)
        )
        latency_penalty = clamp((rtt_penalty * 0.7) + (jitter_penalty * 0.2) + (queue_depth * 0.1))

        edge_score = clamp((low_latency * 0.35) + (low_compute_load * 0.20) + (low_motion * 0.15) + (power_saving * 0.2))
        cloud_score = clamp((high_accuracy_need * 0.40) + (bandwidth_quality * 0.40) - (latency_penalty * 0.20) + (profile["scene_complexity"] * 0.25))

        if latency_critical:
            edge_score = clamp(edge_score + 0.2)
            cloud_score = clamp(cloud_score - 0.15)

        if power_mw and power_mw > self.power_budget_mw:
            cloud_score = clamp(cloud_score + 0.2)

        publish_cached = (
            not fresh_required
            and tracker_uncertainty < 0.35
            and profile["change_score"] < 0.08
            and profile["motion_score"] < 0.5
        )

        if edge_score >= self.dual_path_threshold and cloud_score >= self.dual_path_threshold:
            if abs(edge_score - cloud_score) <= self.dual_path_margin or fresh_required:
                route = "dual_path"
            else:
                route = "onboard" if edge_score >= cloud_score else "offboard"
        else:
            route = "onboard" if edge_score >= cloud_score else "offboard"

        if route == "offboard" and bandwidth_quality < 0.45:
            route = "lower_resolution"

        if latency_critical and edge_score >= 0.85:
            route = "dual_path" if route in ("offboard", "lower_resolution") and fresh_required else "onboard"

        rospy.loginfo(
            "[DECISION] app=%s edge=%.3f cloud=%.3f route=%s fresh=%s rtt=%s jitter=%.2f queue=%d",
            app,
            edge_score,
            cloud_score,
            route,
            fresh_required,
            "NA" if self.rtt_ms is None else "%.2f" % self.rtt_ms,
            self.jitter_ms,
            self.get_cloud_queue_depth(),
        )
        return {
            "route": route,
            "edge_score": round(edge_score, 4),
            "cloud_score": round(cloud_score, 4),
            "publish_cached": publish_cached,
            "force_fresh": fresh_required,
        }

    def check_edge_server_available(self):
        now = time.time()
        if now - self.last_edge_check < self.edge_check_interval:
            return self.edge_server_available and self.network_ok

        if not self.network_ok:
            self.edge_server_available = False
            self.last_edge_check = now
            return False

        try:
            response = self.session.head(self.server_url, timeout=0.5)
            self.edge_server_available = response.status_code < 500
        except requests.exceptions.RequestException:
            self.edge_server_available = False

        self.last_edge_check = now
        return self.edge_server_available

    def lower_resolution(self, frame, scale_factor=None):
        scale = scale_factor if scale_factor is not None else self.low_res_scale
        return cv2.resize(frame, (int(frame.shape[1] * scale), int(frame.shape[0] * scale)))

    def safe_crop_roi(self, frame, roi):
        """
        Safely crop ROI from frame with padding.
        ROI format: (x1, y1, x2, y2)
        Falls back to full frame if invalid.
        """
        if not roi or len(roi) != 4:
            return frame

        h, w = frame.shape[:2]

        x1, y1, x2, y2 = roi

        pad = 20
        x1 -= pad
        y1 -= pad
        x2 += pad
        y2 += pad

        # Clamp values (VERY IMPORTANT)
        x1 = max(0, min(w - 1, int(x1)))
        x2 = max(0, min(w, int(x2)))
        y1 = max(0, min(h - 1, int(y1)))
        y2 = max(0, min(h, int(y2)))

        # Validate ROI
        if x2 <= x1 or y2 <= y1:
            return frame

        return frame[y1:y2, x1:x2]
    

    def log_row(
        self,
        frame_uuid,
        app,
        decision,
        location,
        bandwidth,
        cpu,
        gpu,
        power_mw,
        ram,
        profile=None,
        scores=None,
    ):
        try:
            profile = profile or self.scene_profile
            scores = scores or {}
            e2e = ""
            if frame_uuid in self.frame_timing and location != "onboard":
                e2e = round(time.time() - self.frame_timing[frame_uuid]["t_capture"], 4)
                self.frame_timing.pop(frame_uuid, None)
            elif frame_uuid in self.frame_timing and location.startswith("onboard"):
                e2e = round(time.time() - self.frame_timing[frame_uuid]["t_capture"], 4)

            self.csv_writer.writerow(
                [
                    self.frame_id,
                    round(rospy.Time.now().to_sec(), 4),
                    app,
                    decision,
                    location,
                    round(bandwidth, 2),
                    round(cpu, 2) if cpu is not None else "",
                    round(gpu, 2) if gpu is not None else "",
                    round(power_mw, 2) if power_mw is not None else "",
                    round(ram, 2) if ram is not None else "",
                    "" if self.rtt_ms is None else round(self.rtt_ms, 2),
                    round(self.jitter_ms, 2),
                    profile.get("motion_score", ""),
                    profile.get("scene_complexity", ""),
                    profile.get("change_score", ""),
                    scores.get("edge_score", ""),
                    scores.get("cloud_score", ""),
                    e2e,
                ]
            )
            self.csv_file.flush()
        except Exception as exc:
            rospy.logerr(f"Failed to write CSV row: {exc}")

    def forward_to_edge(
        self,
        frame,
        app,
        frame_uuid,
        bandwidth,
        cpu,
        gpu,
        power_mw,
        ram,
        location_label,
        capture_time=None,
        profile=None,
        scores=None,
    ):
        try:
            dispatch = self.prepare_cloud_payload(frame, app, location_label)
            request_meta = {
                "app": app,
                "frame_uuid": frame_uuid,
                "capture_time": capture_time or time.time(),
                "bandwidth": bandwidth,
                "cpu": cpu,
                "gpu": gpu,
                "power_mw": power_mw,
                "ram": ram,
                "location_label": location_label,
                "profile": profile or dict(self.scene_profile),
                "scores": scores or {},
                "headers": dispatch["headers"],
                "payload": dispatch["payload"],
            }

            if self.cloud_queue.full():
                dropped = self.cloud_queue.get_nowait()
                rospy.logwarn("Cloud queue full, dropping oldest %s request", dropped["app"])

            self.cloud_queue.put_nowait(request_meta)
        except Exception as exc:
            rospy.logwarn(f"Failed to queue cloud request for {app}: {exc}")
            self.publish_edge_fallback(app)

    def prepare_cloud_payload(self, frame, app, location_label):
        roi = self.extract_cloud_roi(app, frame.shape[1], frame.shape[0])
        cropped_frame = frame
        if roi and not self.cloud_roi_headers_only:
            cropped_frame = self.safe_crop_roi(frame, roi)
        target_width = self.edge_target_width if location_label == "edge_low_res" else self.cloud_target_width
        target_height = self.cloud_target_height

        resize_ratio = min(float(target_width) / cropped_frame.shape[1], float(target_height) / cropped_frame.shape[0])
        resize_ratio = min(resize_ratio, 0.80 if location_label == "edge_low_res" else 1.0)
        if location_label == "edge_low_res":
            resize_ratio = min(resize_ratio, self.low_res_scale)

        resized = cropped_frame
        if abs(resize_ratio - 1.0) > 1e-3:
            resized = cv2.resize(
                cropped_frame,
                (max(1, int(cropped_frame.shape[1] * resize_ratio)), max(1, int(cropped_frame.shape[0] * resize_ratio))),
            )

        bgr = cv2.cvtColor(resized, cv2.COLOR_RGB2BGR)
        jpeg_quality = 75 if min(self.upload_speed, self.download_speed) >= self.bandwidth_high_threshold else 60
        ok, buffer = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
        if not ok:
            raise RuntimeError("failed to encode cloud payload")

        headers = {
            "Content-Type": "application/octet-stream",
            "Frame-Width": str(bgr.shape[1]),
            "Frame-Height": str(bgr.shape[0]),
            "Client-Timestamp": str(time.time()),
            "detection-flag": app,
            "Cloud-Transport": "jpeg",
            "Cloud-Route": location_label,
            "Network-RTT-Ms": "NA" if self.rtt_ms is None else str(round(self.rtt_ms, 2)),
            "Network-Jitter-Ms": str(round(self.jitter_ms, 2)),
            "Suggested-ROI": ",".join(map(str, roi)) if roi else "",
            "ROI-Headers-Only": "1" if self.cloud_roi_headers_only else "0",
        }
        return {"payload": buffer.tobytes(), "headers": headers}

    def forward_to_onboard(self, app, ros_image, frame_override=None):
        try:
            frame_rgb = frame_override if frame_override is not None else self.image
            if frame_rgb is None:
                return

            if frame_rgb.shape[1] != self.edge_target_width:
                scale = float(self.edge_target_width) / float(frame_rgb.shape[1])
                frame_rgb = cv2.resize(frame_rgb, (self.edge_target_width, int(frame_rgb.shape[0] * scale)))

            output_image = self._rgb_array_to_ros_bgr8(frame_rgb)
            output_image.header = ros_image.header

            if app == "lane_detection":
                self.lane_img_pub.publish(output_image)
            elif app in ("traffic_light_detection", "traffic_sign_detection"):
                self.yolo_traffic.publish(output_image)
            elif app in ("collision_detection", "collision_avoidance", "pedestrian_detection", "pedestrian_avoidance"):
                self.yolo_obj.publish(output_image)
            else:
                rospy.logwarn_throttle(5.0, f"Unknown app '{app}', no onboard topic.")
                return

            rospy.loginfo(f"Forwarded {app} to onboard")
        except Exception as exc:
            rospy.logerr(f"Error forwarding to onboard {app}: {exc}")

    @staticmethod
    def _rgb_array_to_ros_bgr8(rgb_image):
        bgr = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        msg = Image()
        msg.height = bgr.shape[0]
        msg.width = bgr.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = bgr.tobytes()
        return msg

    def cv2_to_ros_image(self, cv_image, encoding="bgr8"):
        msg = Image()
        msg.height = cv_image.shape[0]
        msg.width = cv_image.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = msg.width if len(cv_image.shape) == 2 else msg.width * 3
        msg.data = cv_image.tobytes()
        return msg

    def _cloud_worker(self):
        while self.is_running or not self.cloud_queue.empty():
            try:
                request_meta = self.cloud_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            with self.cloud_lock:
                self.cloud_inflight += 1

            try:
                response = self.session.post(
                    self.server_url,
                    data=request_meta["payload"],
                    headers=request_meta["headers"],
                    timeout=self.edge_timeout,
                )
                if response.status_code == 200:
                    self.process_edge_response(response, request_meta["app"], request_meta)
                    rospy.loginfo("Cloud response received for %s", request_meta["app"])
                else:
                    rospy.logwarn(
                        "Edge server returned %s for %s", response.status_code, request_meta["app"]
                    )
                    self.edge_server_available = False
                    self.publish_edge_fallback(request_meta["app"])
            except requests.exceptions.RequestException as exc:
                rospy.logwarn(f"Edge request failed for {request_meta['app']}: {exc}")
                self.edge_server_available = False
                self.publish_edge_fallback(request_meta["app"])
            finally:
                with self.cloud_lock:
                    self.cloud_inflight = max(0, self.cloud_inflight - 1)

                self.log_row(
                    request_meta["frame_uuid"],
                    request_meta["app"],
                    request_meta["scores"].get("route", "offboard"),
                    request_meta["location_label"],
                    request_meta["bandwidth"],
                    request_meta["cpu"],
                    request_meta["gpu"],
                    request_meta["power_mw"],
                    request_meta["ram"],
                    profile=request_meta["profile"],
                    scores=request_meta["scores"],
                )
                self.cloud_queue.task_done()

    def process_edge_response(self, response, app, request_meta=None):
        try:
            try:
                result = response.json()
            except json.JSONDecodeError:
                text = response.text
                rospy.logwarn(f"Edge response not valid JSON: {text[:200]}")
                if text.startswith("{") and text.endswith("}"):
                    result = json.loads(text)
                else:
                    rospy.logerr("Edge response cannot be parsed as JSON.")
                    return

            recv_time = time.time()
            if "Lane_detection_model" in result:
                lane_data = result["Lane_detection_model"]
                if isinstance(lane_data, str):
                    lane_data = json.loads(lane_data)
                self.process_lane_data(lane_data, request_meta=request_meta, recv_time=recv_time)

            if "OBD_model" in result:
                obd_data = result["OBD_model"]
                if isinstance(obd_data, str):
                    obd_data = json.loads(obd_data)
                detections = self.normalize_detections(obd_data.get("detections", []), source="cloud")
                self.process_cloud_detections("collision_avoidance", detections, request_meta, recv_time)

            if "Traffic_detection_model" in result:
                traffic_data = result["Traffic_detection_model"]
                if isinstance(traffic_data, str):
                    traffic_data = json.loads(traffic_data)

                detections = []
                if traffic_data.get("success", False):
                    boxes = traffic_data.get("boxes", [])
                    classids = traffic_data.get("classids", [])
                    scores = traffic_data.get("scores", [])
                    for index, box in enumerate(boxes):
                        class_name = f"class_{classids[index]}" if index < len(classids) else "unknown"
                        detections.append(
                            {
                                "class_name": class_name,
                                "box": [int(v) for v in box],
                                "score": float(scores[index]) if index < len(scores) else 0.0,
                                "source": "cloud",
                            }
                        )
                self.process_cloud_detections("traffic_sign_detection", detections, request_meta, recv_time)
        except Exception as exc:
            rospy.logerr(f"Error processing edge response: {exc}")

    def process_lane_data(self, lane_data, request_meta=None, recv_time=None):
        try:
            if not isinstance(lane_data, dict):
                rospy.logerr(f"Lane data is not a dict: {type(lane_data)}")
                return

            parsed_lane = {
                "horizontal_y": lane_data.get("horizontal_y", 0),
                "vertical_near": {
                    "up": tuple(lane_data.get("vertical_near", {}).get("up", [0, 0])),
                    "down": tuple(lane_data.get("vertical_near", {}).get("down", [0, 0])),
                    "center_y": lane_data.get("vertical_near", {}).get("center_y", 0),
                },
                "vertical_far": {
                    "up": tuple(lane_data.get("vertical_far", {}).get("up", [0, 0])),
                    "down": tuple(lane_data.get("vertical_far", {}).get("down", [0, 0])),
                },
                "lane": {
                    "x": lane_data.get("lane", {}).get("x", -1),
                    "angle": lane_data.get("lane", {}).get("angle", None),
                },
                "source": "cloud",
                "timestamp": recv_time or time.time(),
            }

            cloud_delay = 0.0
            if request_meta is not None:
                cloud_delay = (recv_time or time.time()) - request_meta["capture_time"]

            latest_edge = self.edge_state["lane_detection"]
            if cloud_delay > self.cloud_delay_threshold and latest_edge["data"] is not None:
                fused_lane = dict(latest_edge["data"])
                fused_lane["source"] = "edge_fallback"
                fused_lane["cloud_delay"] = round(cloud_delay, 4)
            elif latest_edge["data"] is not None and (time.time() - latest_edge["timestamp"]) < 0.25:
                fused_lane = dict(parsed_lane)
                fused_lane["lane"]["x"] = latest_edge["data"]["lane"].get("x", fused_lane["lane"]["x"])
                fused_lane["lane"]["angle"] = latest_edge["data"]["lane"].get("angle", fused_lane["lane"]["angle"])
                fused_lane["source"] = "fused"
                fused_lane["cloud_delay"] = round(cloud_delay, 4)
            else:
                fused_lane = parsed_lane
                fused_lane["cloud_delay"] = round(cloud_delay, 4)

            self.lane_data = fused_lane
            now = time.time()
            self.cloud_state["lane_detection"] = {"data": parsed_lane, "timestamp": now, "source": "cloud"}
            self.fused_state["lane_detection"] = {"data": fused_lane, "timestamp": now, "source": fused_lane["source"]}
            self.lane_pub.publish(String(data=json.dumps(fused_lane)))

            if "binary_image" in lane_data:
                self._publish_base64_image(lane_data["binary_image"], self.binary_img_pub, "binary")
            if "result_image" in lane_data:
                self._publish_base64_image(lane_data["result_image"], self.result_img_pub, "result")
        except Exception as exc:
            rospy.logerr(f"Error processing lane data: {exc}")

    def process_cloud_detections(self, app, cloud_detections, request_meta, recv_time):
        cloud_delay = 0.0
        if request_meta is not None:
            cloud_delay = recv_time - request_meta["capture_time"]

        fused = self.fuse_detections(app, cloud_detections, cloud_delay)
        now = time.time()
        self.cloud_state[app] = {"data": cloud_detections, "timestamp": now, "source": "cloud"}
        self.fused_state[app] = {"data": fused, "timestamp": now, "source": "fused"}

        if app == "collision_avoidance":
            self.detection_results = fused
            self.obj_pub.publish(String(data=json.dumps(fused)))
        elif app == "traffic_sign_detection":
            self.traffic_sign_results = fused
            self.traffic_pub.publish(String(data=json.dumps(fused)))

    def _publish_base64_image(self, image_data, publisher, label):
        try:
            if not isinstance(image_data, str):
                return
            raw = base64.b64decode(image_data)
            array = np.frombuffer(raw, np.uint8)
            cv_image = cv2.imdecode(array, cv2.IMREAD_COLOR)
            if cv_image is not None:
                publisher.publish(self.cv2_to_ros_image(cv_image))
            else:
                rospy.logwarn(f"Failed to decode {label} image from edge.")
        except Exception as exc:
            rospy.logerr(f"Error publishing {label} image: {exc}")

    def onboard_lane_callback(self, msg):
        try:
            lane_data = json.loads(msg.data)
            lane_data["source"] = "onboard"
            lane_data["timestamp"] = time.time()
            self.edge_state["lane_detection"] = {
                "data": lane_data,
                "timestamp": lane_data["timestamp"],
                "source": "onboard",
                "uncertainty": self.compute_lane_uncertainty(lane_data),
            }
            self.fused_state["lane_detection"] = {
                "data": lane_data,
                "timestamp": lane_data["timestamp"],
                "source": "onboard",
            }
            self.lane_pub.publish(String(data=json.dumps(lane_data)))
        except Exception as exc:
            rospy.logerr(f"Error in onboard lane callback: {exc}")

    def onboard_object_callback(self, msg):
        detections = self.objects_info_to_list(msg, source="onboard")
        self.update_edge_detection_state("collision_avoidance", detections)
        self.obj_pub.publish(String(data=json.dumps(detections)))

    def onboard_traffic_callback(self, msg):
        detections = self.objects_info_to_list(msg, source="onboard")
        self.update_edge_detection_state("traffic_sign_detection", detections)
        self.traffic_pub.publish(String(data=json.dumps(detections)))

    def objects_info_to_list(self, msg, source="onboard"):
        detections = []
        for obj in msg.objects:
            detections.append(
                {
                    "class_name": str(obj.class_name),
                    "box": [int(v) for v in list(obj.box)],
                    "score": float(obj.score),
                    "source": source,
                }
            )
        return detections

    def normalize_detections(self, detections, source="cloud"):
        normalized = []
        for item in detections:
            if isinstance(item, dict):
                normalized.append(
                    {
                        "class_name": str(item.get("class_name", "unknown")),
                        "box": [int(v) for v in item.get("box", [0, 0, 0, 0])],
                        "score": float(item.get("score", 0.0)),
                        "source": source,
                    }
                )
        return normalized

    def update_edge_detection_state(self, app, detections):
        now = time.time()
        previous = self.edge_state.get(app, {"data": [], "timestamp": 0.0})
        dt = max(now - previous.get("timestamp", now), 1e-3)
        previous_detections = previous.get("data", [])

        updated = []
        for detection in detections:
            velocity = [0.0, 0.0]
            previous_match = self.find_best_match(detection, previous_detections)
            if previous_match is not None:
                prev_center = self.box_center(previous_match["box"])
                new_center = self.box_center(detection["box"])
                velocity = [
                    (new_center[0] - prev_center[0]) / dt,
                    (new_center[1] - prev_center[1]) / dt,
                ]
            tracked = dict(detection)
            tracked["velocity"] = [round(velocity[0], 3), round(velocity[1], 3)]
            updated.append(tracked)

        count_delta = abs(len(detections) - len(previous_detections))
        uncertainty = clamp((count_delta * 0.2) + (self.scene_profile["change_score"] * 0.5))
        self.edge_state[app] = {
            "data": updated,
            "timestamp": now,
            "source": "onboard",
            "uncertainty": uncertainty,
        }
        self.fused_state[app] = {"data": updated, "timestamp": now, "source": "onboard"}

    def get_tracker_uncertainty(self, app):
        if app == "lane_detection":
            return self.edge_state["lane_detection"].get("uncertainty", 1.0)
        if app in ("collision_avoidance", "collision_detection", "pedestrian_avoidance", "pedestrian_detection"):
            state = self.edge_state["collision_avoidance"]
        elif app in ("traffic_sign_detection", "traffic_light_detection"):
            state = self.edge_state["traffic_sign_detection"]
        else:
            return 0.5

        age = time.time() - state.get("timestamp", 0.0)
        return clamp(state.get("uncertainty", 0.5) + min(age / 0.6, 1.0) * 0.5)

    def compute_lane_uncertainty(self, lane_data):
        lane_x = lane_data.get("lane", {}).get("x", -1)
        lane_angle = lane_data.get("lane", {}).get("angle", 0.0)
        missing_lane = 1.0 if lane_x < 0 else 0.0
        angle_penalty = min(abs(lane_angle or 0.0) / 90.0, 1.0) * 0.25
        return clamp(missing_lane + angle_penalty)

    def publish_cached_result(self, app):
        if app == "lane_detection":
            state = self.fused_state["lane_detection"]
            if state["data"] is not None and (time.time() - state["timestamp"]) < 0.25:
                self.lane_pub.publish(String(data=json.dumps(state["data"])))
        elif app in ("collision_avoidance", "collision_detection", "pedestrian_avoidance", "pedestrian_detection"):
            state = self.fused_state["collision_avoidance"]
            if (time.time() - state["timestamp"]) < 0.25:
                self.obj_pub.publish(String(data=json.dumps(state["data"])))
        elif app in ("traffic_sign_detection", "traffic_light_detection"):
            state = self.fused_state["traffic_sign_detection"]
            if (time.time() - state["timestamp"]) < 0.25:
                self.traffic_pub.publish(String(data=json.dumps(state["data"])))

    def publish_edge_fallback(self, app):
        if app == "lane_detection":
            state = self.edge_state["lane_detection"]
            if state["data"] is not None:
                self.lane_pub.publish(String(data=json.dumps(state["data"])))
        elif app in ("collision_avoidance", "collision_detection", "pedestrian_avoidance", "pedestrian_detection"):
            state = self.edge_state["collision_avoidance"]
            self.obj_pub.publish(String(data=json.dumps(state["data"])))
        elif app in ("traffic_sign_detection", "traffic_light_detection"):
            state = self.edge_state["traffic_sign_detection"]
            self.traffic_pub.publish(String(data=json.dumps(state["data"])))

    def extract_cloud_roi(self, app, width, height):
        if app in ("collision_avoidance", "collision_detection", "pedestrian_avoidance", "pedestrian_detection"):
            detections = self.edge_state["collision_avoidance"]["data"]
        elif app in ("traffic_sign_detection", "traffic_light_detection"):
            detections = self.edge_state["traffic_sign_detection"]["data"]
        else:
            detections = []

        if not detections:
            return [0, 0, width, height]

        x1 = min(det["box"][0] for det in detections)
        y1 = min(det["box"][1] for det in detections)
        x2 = max(det["box"][2] for det in detections)
        y2 = max(det["box"][3] for det in detections)
        pad_x = int(0.1 * width)
        pad_y = int(0.1 * height)
        return [
            max(0, x1 - pad_x),
            max(0, y1 - pad_y),
            min(width, x2 + pad_x),
            min(height, y2 + pad_y),
        ]

    def get_cloud_queue_depth(self):
        with self.cloud_lock:
            return self.cloud_inflight + self.cloud_queue.qsize()

    def find_best_match(self, detection, candidates):
        best = None
        best_score = 0.0
        for candidate in candidates:
            iou = self.compute_iou(detection["box"], candidate["box"])
            if detection["class_name"] == candidate["class_name"]:
                iou += 0.1
            if iou > best_score:
                best_score = iou
                best = candidate
        return best if best_score > 0.05 else None

    def fuse_detections(self, app, cloud_detections, cloud_delay):
        if app == "collision_avoidance":
            edge_state = self.edge_state["collision_avoidance"]
        else:
            edge_state = self.edge_state["traffic_sign_detection"]

        edge_detections = edge_state.get("data", [])
        edge_age = time.time() - edge_state.get("timestamp", 0.0)
        if cloud_delay > self.cloud_delay_threshold and edge_detections and edge_age < 0.4:
            return edge_detections

        fused = []
        used_edge = set()
        for cloud in cloud_detections:
            best_index = None
            best_score = -1.0
            projected_box = cloud["box"]

            for index, edge in enumerate(edge_detections):
                velocity = edge.get("velocity", [0.0, 0.0])
                candidate_box = self.project_box(edge["box"], velocity, cloud_delay)
                iou = self.compute_iou(candidate_box, cloud["box"])
                center_penalty = self.center_distance(candidate_box, cloud["box"]) / 300.0
                match_score = iou - center_penalty
                if edge["class_name"] == cloud["class_name"]:
                    match_score += 0.1
                if match_score > best_score:
                    best_score = match_score
                    best_index = index
                    projected_box = candidate_box

            if best_index is not None and best_score > -0.05:
                edge = edge_detections[best_index]
                used_edge.add(best_index)
                fused.append(
                    {
                        "class_name": cloud["class_name"],
                        "box": [int(v) for v in projected_box],
                        "score": max(float(cloud["score"]), float(edge.get("score", 0.0))),
                        "source": "fused",
                        "cloud_delay": round(cloud_delay, 4),
                    }
                )
            else:
                cloud_copy = dict(cloud)
                cloud_copy["cloud_delay"] = round(cloud_delay, 4)
                fused.append(cloud_copy)

        for index, edge in enumerate(edge_detections):
            if index not in used_edge and edge_age < 0.25:
                edge_copy = dict(edge)
                edge_copy["source"] = "edge_fallback"
                edge_copy["cloud_delay"] = round(cloud_delay, 4)
                fused.append(edge_copy)

        return fused

    @staticmethod
    def box_center(box):
        return ((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0)

    def center_distance(self, box_a, box_b):
        center_a = self.box_center(box_a)
        center_b = self.box_center(box_b)
        return ((center_a[0] - center_b[0]) ** 2 + (center_a[1] - center_b[1]) ** 2) ** 0.5

    def project_box(self, box, velocity, dt):
        dx = velocity[0] * dt
        dy = velocity[1] * dt
        return [
            int(round(box[0] + dx)),
            int(round(box[1] + dy)),
            int(round(box[2] + dx)),
            int(round(box[3] + dy)),
        ]

    @staticmethod
    def compute_iou(box_a, box_b):
        x_a = max(box_a[0], box_b[0])
        y_a = max(box_a[1], box_b[1])
        x_b = min(box_a[2], box_b[2])
        y_b = min(box_a[3], box_b[3])
        inter_w = max(0, x_b - x_a)
        inter_h = max(0, y_b - y_a)
        inter_area = inter_w * inter_h
        if inter_area <= 0:
            return 0.0

        area_a = max(0, box_a[2] - box_a[0]) * max(0, box_a[3] - box_a[1])
        area_b = max(0, box_b[2] - box_b[0]) * max(0, box_b[3] - box_b[1])
        union_area = max(area_a + area_b - inter_area, 1e-6)
        return inter_area / union_area


if __name__ == "__main__":
    LoadBalancerNode("load_balancer_setb")
