#!/usr/bin/env python3
# encoding: utf-8
import os
import re
import cv2
import signal
import rospy
import time
import base64
import subprocess
import numpy as np
import requests
import threading
import json
from datetime import datetime
import csv
import collections
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from std_msgs.msg import String
from sensor_msgs.msg import Image
from hiwonder_interfaces.msg import ObjectsInfo
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from geometry_msgs.msg import Twist
from hiwonder_servo_controllers.bus_servo_control import set_servos

# Application table (inlined from lb_input.py to avoid subprocess import)
APPLICATION_TABLE = {
    "collision_avoidance":     {"latency_sensitivity": "high",   "accuracy_priority": "low"},
    "collision_detection":     {"latency_sensitivity": "high",   "accuracy_priority": "high"},
    "pedestrian_avoidance":    {"latency_sensitivity": "high",   "accuracy_priority": "high"},
    "pedestrian_detection":    {"latency_sensitivity": "high",   "accuracy_priority": "high"},
    "localization":            {"latency_sensitivity": "high",   "accuracy_priority": "high"},
    "traffic_light_detection": {"latency_sensitivity": "low",    "accuracy_priority": "high"},
    "traffic_sign_detection":  {"latency_sensitivity": "low",   "accuracy_priority": "high"},
    "lane_detection":          {"latency_sensitivity": "low",   "accuracy_priority": "high"},
    "depth_estimation":        {"latency_sensitivity": "high",   "accuracy_priority": "high"},
    "drowsiness_detection":    {"latency_sensitivity": "medium", "accuracy_priority": "high"},
    "drivable_area":           {"latency_sensitivity": "high",   "accuracy_priority": "high"},
    "infotainment":            {"latency_sensitivity": "low",    "accuracy_priority": "low"},
}

def get_application_table(application_name):
    return APPLICATION_TABLE.get(application_name)   # O(1) dict lookup


class LoadBalancerNode:
    def __init__(self, name):
        rospy.loginfo("Load balancer node initializing...")
        rospy.init_node(name, anonymous=True)
        self.name = name
        self.image = None
        self.is_running = True
        self.machine_type = os.environ.get('MACHINE_TYPE')
        self.lock = threading.Lock()

        self.frame_timing = {}

        self.upload_speed = 0
        self.download_speed = 0
        rospy.Subscriber("/network/bandwidth", Float32MultiArray, self.bandwidth_callback)

        signal.signal(signal.SIGINT, self.shutdown)

        # ── Resource cache (updated by background tegrastats thread) ──────
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

        # ── Publishers ────────────────────────────────────────────────────
        self.mecanum_pub  = rospy.Publisher('/hiwonder_controller/cmd_vel', Twist, queue_size=1)
        self.joints_pub   = rospy.Publisher('servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)

        # Onboard processing image topics
        self.lane_img_pub   = rospy.Publisher('/load_balancer/lane_image',         Image, queue_size=1)
        self.yolo_img_pub   = rospy.Publisher('/load_balancer/yolo_image',         Image, queue_size=1)
        self.yolo_traffic   = rospy.Publisher('/load_balancer/yolo/traffic_image', Image, queue_size=7)
        self.yolo_obj       = rospy.Publisher('/load_balancer/yolo/object_image',  Image, queue_size=7)
        self.binary_img_pub = rospy.Publisher('/load_balancer/lane_binary_image',  Image, queue_size=1)
        self.result_img_pub = rospy.Publisher('/load_balancer/lane_result_image',  Image, queue_size=1)

        # Result publishers
        self.lane_pub    = rospy.Publisher('/load_balancer/lane_detection',          String, queue_size=1)
        self.obj_pub     = rospy.Publisher('/load_balancer/object_detections',       String, queue_size=1)
        self.traffic_pub = rospy.Publisher('/load_balancer/traffic_sign_detections', String, queue_size=1)

        # Start time publisher
        self.start_pub = rospy.Publisher('/load_balaner/node_start_time', Float64, queue_size=1)

        self.server_url = rospy.get_param('~edge_server_url')

        # ── Configuration ─────────────────────────────────────────────────
        self.frame_width  = 640
        self.frame_height = 480
        self.frame_id     = 0
        self.edge_timeout = 2.0

        # Data storage
        self.detection_results    = []
        self.traffic_sign_results = []
        self.lane_data            = {}
        self.binary_image         = None
        self.result_image         = None

        self.applications   = rospy.get_param("~applications")
        # Decision thresholds
        self.resource_threshold       = rospy.get_param('~resource_threshold',       75)  # %
        self.bandwidth_high_threshold = rospy.get_param('~bandwidth_high_threshold',  7)  # Mbps
        self.bandwidth_low_threshold  = rospy.get_param('~bandwidth_low_threshold',   4)  # Mbps

        # Edge availability tracking — must be set BEFORE subscriber is registered
        # so handle_frame never sees missing attributes if a camera frame arrives
        # immediately after rospy.Subscriber() returns.
        self.edge_server_available = True
        self.last_edge_check       = 0
        self.edge_check_interval   = 5.0

        # Rate limiting
        self.last_processing_time    = 0
        self.min_processing_interval = 0.1

        # ── Single unified CSV log ────────────────────────────────────────
        log_dir = os.path.expanduser(
            "~/ros_ws/src/hiwonder_example/scripts/load_balancer_main/load_balancer/load_balancer_logs"
        )
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.csv_path   = os.path.join(log_dir, f"lb_log_{timestamp}.csv")
        self.csv_file   = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "frame_id", "timestamp", "application",
            "decision", "execution_location",
            "bandwidth_mbps", "cpu_percent", "gpu_percent",
            "power_mw", "ram_percent", "e2e_latency_sec"
        ])

        rospy.loginfo(f"CSV logging: {self.csv_path}")

        # ── Subscriber — registered last so callback never fires before ───
        # any attribute it touches has been initialised.
        depth_camera = rospy.get_param('/depth_camera/camera_name', 'depth_cam')
        rospy.Subscriber(
            f'/{depth_camera}/rgb/image_raw', Image,
            self.image_callback, queue_size=1, buff_size=2**24
        )

        # ── Start ─────────────────────────────────────────────────────────
        self.wait_for_services()
        self.initialize_servos()
        rospy.loginfo("LoadBalancerNode initialized.")
        rospy.spin()

    # ======================================================================
    # Background tegrastats poller  (replaces blocking get_usage_percent())
    # ======================================================================
    def _poll_tegrastats(self):
        """
        Run tegrastats continuously in the background and update
        self._resource_cache on every output line (~1 Hz by default).
        This avoids spawning a new subprocess per camera frame, which was
        causing ~1-2 s of blocking and reducing frame cadence to <1 fps.
        """
        while self.is_running:
            try:
                proc = subprocess.Popen(
                    ["tegrastats", "--interval", "100"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    universal_newlines=True
                )
                for line in proc.stdout:
                    if not self.is_running:
                        break
                    cache = {}

                    ram_m = re.search(r"RAM (\d+)/(\d+)MB", line)
                    if ram_m:
                        cache["ram_percent"] = round(
                            int(ram_m.group(1)) / int(ram_m.group(2)) * 100, 2
                        )

                    cpu_m = re.search(r"CPU \[([^\]]+)\]", line)
                    if cpu_m:
                        usages = [int(m.group(1))
                                  for core in cpu_m.group(1).split(",")
                                  for m in [re.search(r"(\d+)%", core)] if m]
                        if usages:
                            cache["cpu_percent"] = round(sum(usages) / len(usages), 2)

                    gpu_m = re.search(r"GR3D_FREQ (\d+)%", line)
                    if gpu_m:
                        cache["gpu_percent"] = int(gpu_m.group(1))

                    # POM_5V_IN current/average in mW  e.g. "POM_5V_IN 3467/2826"
                    pwr_m = re.search(
                        r"POM_5V_IN\s+(\d+)(?:mW)?/(\d+)(?:mW)?",
                        line
                    )
                    if pwr_m:
                        power_now = float(pwr_m.group(1))
                        self._power_window.append(power_now)
                        cache["power_mw"] = power_now
                        cache["power_avg_mw"] = round(
                            sum(self._power_window) / float(len(self._power_window)),
                            2
                        )

                    if cache:
                        with self._resource_lock:
                            self._resource_cache.update(cache)

                proc.kill()
            except Exception as e:
                rospy.logwarn_throttle(10, f"[tegrastats poller] error: {e}")
                time.sleep(2)

    def _get_resource_usage(self):
        """Return the latest cached resource snapshot (non-blocking)."""
        with self._resource_lock:
            return dict(self._resource_cache)

    # ======================================================================
    # Init helpers
    # ======================================================================
    def wait_for_services(self):
        if not rospy.get_param('~only_line_follow', False):
            while not rospy.is_shutdown():
                try:
                    if rospy.get_param('/yolov5_lb/init_finish'):
                        break
                except Exception:
                    rospy.sleep(0.1)
            try:
                rospy.ServiceProxy('/yolov5_lb/start', Trigger)()
            except Exception:
                rospy.logwarn("Failed to start YOLOv5 service")

    def initialize_servos(self):
        while not rospy.is_shutdown():
            try:
                if (rospy.get_param('/hiwonder_servo_manager/init_finish') and
                        rospy.get_param('/joint_states_publisher/init_finish')):
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

    # ======================================================================
    # Image callback
    # ======================================================================
    def image_callback(self, ros_image):
        """
        Convert incoming ROS Image to RGB numpy array and dispatch
        handle_frame.  Uses np.frombuffer (safe for bytes data field).
        """
        try:
            enc = ros_image.encoding.lower()
            if enc not in ('rgb8', 'bgr8'):
                rospy.logwarn_throttle(5, f"Unsupported camera encoding: {ros_image.encoding}")
                return

            raw = np.frombuffer(ros_image.data, dtype=np.uint8)
            img = raw.reshape((ros_image.height, ros_image.width, 3)).copy()

            if enc == 'bgr8':
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

            # Resize ONCE here — all downstream uses smaller frame
            img = cv2.resize(img, (320, 240))   # half resolution, 4x less memory

            self.image = img                       # stored as RGB
            self.handle_frame(self.image, ros_image)

        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")

    def bandwidth_callback(self, msg):
        self.upload_speed   = msg.data[0]
        self.download_speed = msg.data[1]

    # ======================================================================
    # Frame handling
    # ======================================================================
    def handle_frame(self, frame, ros_image):
        """Route each application to onboard or edge on every camera frame."""
        # Rate limit to 10fps
        now = time.time()
        if now - self.last_processing_time < 0.1:   # 100ms = 10fps
            return
        self.last_processing_time = now

        self.frame_id += 1
        frame_uuid = f"{self.frame_id}_{time.time() * 1e9}"
        self.frame_timing[frame_uuid] = {"t_capture": time.time()}

        self.start_pub.publish(time.time())

        # Prune old timing entries to avoid unbounded growth
        if len(self.frame_timing) > 500:
            oldest = sorted(self.frame_timing.keys())[:250]
            for k in oldest:
                self.frame_timing.pop(k, None)

        try:
            # Non-blocking resource read from background poller
            usage     = self._get_resource_usage()
            cpu_usage = usage['cpu_percent']
            ram_usage = usage['ram_percent']
            gpu_usage = usage['gpu_percent']
            power_mw  = usage['power_mw']

            upload_speed   = self.upload_speed
            download_speed = self.download_speed

            edge_available = self.check_edge_server_available()

            for app in self.applications:
                r = get_application_table(app)
                if r is None:
                    rospy.logwarn(f"Application '{app}' not in table — routing onboard.")
                    self.forward_to_onboard(app, ros_image)
                    self.log_row(frame_uuid, app, "unknown", "onboard",
                                 upload_speed, cpu_usage, gpu_usage, power_mw, ram_usage)
                    continue

                latency_sensitivity = r["latency_sensitivity"]
                accuracy_priority   = r["accuracy_priority"]

                location = self.decide_processing_location(
                    latency_sensitivity, accuracy_priority,
                    cpu_usage, ram_usage, gpu_usage,
                    upload_speed, download_speed, edge_available
                )

                if location == "onboard":
                    self.forward_to_onboard(app, ros_image)
                    self.log_row(frame_uuid, app, location, "onboard",
                                 upload_speed, cpu_usage, gpu_usage, power_mw, ram_usage)
                elif location == "offboard":
                    # log_row called inside forward_to_edge after HTTP response
                    self.forward_to_edge(frame, app, frame_uuid,
                                         upload_speed, cpu_usage, gpu_usage, power_mw, ram_usage,
                                         "edge")
                elif location == "lower_resolution":
                    low_res = self.lower_resolution(frame)
                    self.forward_to_edge(low_res, app, frame_uuid,
                                         upload_speed, cpu_usage, gpu_usage, power_mw, ram_usage,
                                         "edge_low_res")
                else:
                    self.forward_to_onboard(app, ros_image)
                    self.log_row(frame_uuid, app, location, "onboard",
                                 upload_speed, cpu_usage, gpu_usage, power_mw, ram_usage)

        except Exception as e:
            rospy.logerr(f"Error in handle_frame: {e}")

    # ======================================================================
    # Decision logic
    # ======================================================================
    def decide_processing_location(
        self, latency_sensitivity, accuracy_priority,
        cpu_usage, ram_usage, gpu_usage,
        upload_speed, download_speed, edge_available
    ):
        if not edge_available:
            rospy.loginfo("[DECISION] Edge not available → onboard")
            return "onboard"

        onboard_ok       = (cpu_usage < self.resource_threshold and
                            gpu_usage < 75)
        bandwidth_sufficient = (upload_speed   >= self.bandwidth_high_threshold and
                                download_speed >= self.bandwidth_high_threshold)
        bandwidth_low        = (upload_speed   <= self.bandwidth_low_threshold or
                                download_speed <  self.bandwidth_low_threshold)

        rospy.loginfo(
            f"[DECISION PARAMS] Latency: {latency_sensitivity}, Accuracy: {accuracy_priority}"
        )
        rospy.loginfo(
            f"[DECISION METRICS] Upload: {upload_speed:.2f} Mbps, "
            f"low={bandwidth_low}, sufficient={bandwidth_sufficient}, "
            f"resources_ok={onboard_ok}"
        )

        if latency_sensitivity == "high":
            rospy.loginfo("[DECISION PATH] High latency sensitivity")
            if onboard_ok :
                result = "onboard"
            elif bandwidth_sufficient:
                result = "offboard"
            elif bandwidth_low:
                result = "onboard"
            else:
                result = "onboard" if accuracy_priority == "high" else "lower_resolution"
        else:
            rospy.loginfo("[DECISION PATH] Low/medium latency sensitivity")
            if accuracy_priority == "high":
                result = "onboard" if not bandwidth_sufficient else "offboard"
            else:
                result = "offboard" if bandwidth_sufficient else "lower_resolution"

        rospy.loginfo(f"[DECISION FINAL] {result}")
        return result

    # ======================================================================
    # Edge availability check
    # ======================================================================
    def check_edge_server_available(self):
        now = time.time()
        if now - self.last_edge_check < self.edge_check_interval:
            return self.edge_server_available
        # ── Experiment override ──────────────────────────────────────
        # override = rospy.get_param('/experiment/edge_override', None)
        # if override is not None:
        #     self.edge_server_available = bool(override)
        #     self.last_edge_check = now   # suppress real HTTP check
        #     return self.edge_server_available
        # ─────────────────────────────────────────────────────────────  
          
        try:
            response = requests.head(self.server_url, timeout=2.0)
            self.edge_server_available = (response.status_code < 500)
        except requests.exceptions.RequestException:
            self.edge_server_available = False
        self.last_edge_check = now
        return self.edge_server_available

    def lower_resolution(self, frame, scale_factor=0.5):
        return cv2.resize(
            frame,
            (int(frame.shape[1] * scale_factor), int(frame.shape[0] * scale_factor))
        )

    # ======================================================================
    # CSV logging — single unified row per (frame, application)
    # ======================================================================
    def log_row(self, frame_uuid, app, decision, location,
                bandwidth, cpu, gpu, power_mw, ram):
        """
        Write one row to the unified CSV.
        E2E latency is computed here if the frame_uuid is still in
        frame_timing (onboard path).  For the edge path forward_to_edge
        calls log_row after the HTTP round-trip completes, so the timing
        entry is also still present and gives the full end-to-end time.
        """
        try:
            e2e = ""
            if frame_uuid in self.frame_timing:
                e2e = round(time.time() - self.frame_timing[frame_uuid]["t_capture"], 4)
                self.frame_timing.pop(frame_uuid, None)

            self.csv_writer.writerow([
                self.frame_id,
                round(rospy.Time.now().to_sec(), 4),
                app,
                decision,
                location,
                round(bandwidth, 2),
                round(cpu,  2) if cpu  is not None else "",
                round(gpu,  2) if gpu  is not None else "",
                power_mw,
                round(ram,  2) if ram  is not None else "",
                e2e
            ])
            self.csv_file.flush()
        except Exception as e:
            rospy.logerr(f"Failed to write CSV row: {e}")

    # ======================================================================
    # Edge forwarding
    # ======================================================================
    def forward_to_edge(self, frame, app, frame_uuid,
                        bandwidth, cpu, gpu, power_mw, ram, location_label):
        """
        Encode frame as JPEG, POST to edge server, then write the unified
        CSV row once the full round-trip completes so e2e_latency_sec
        covers capture → HTTP response, not just capture → dispatch.
        """
        try:
            bgr   = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            bgr   = cv2.resize(bgr, (self.frame_width, self.frame_height))
            ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                return

            headers = {
                'Content-Type':     'application/octet-stream',
                'Frame-Width':      str(self.frame_width),
                'Frame-Height':     str(self.frame_height),
                'Client-Timestamp': str(time.time()),
                'Frame-Identifier': frame_uuid,
                'detection-flag':   app
            }

            resp = requests.post(
                self.server_url, data=buf.tobytes(),
                headers=headers, timeout=self.edge_timeout
            )
            if resp.status_code == 200:
                self.process_edge_response(resp, app)
                rospy.loginfo(f"Forwarded {app} to offboard and received response")
            else:
                rospy.logwarn(f"Edge server returned {resp.status_code} for {app}")
                self.edge_server_available = False

        except requests.exceptions.RequestException as e:
            rospy.logwarn(f"Edge request failed for {app}: {e}")
            self.edge_server_available = False

        finally:
            # Always log the row — e2e captures full round-trip time including
            # any failure/timeout, giving a complete latency picture.
            self.log_row(frame_uuid, app, "offboard", location_label,
                         bandwidth, cpu, gpu, power_mw, ram)

    # ======================================================================
    # Onboard forwarding
    # ======================================================================
    def forward_to_onboard(self, app, ros_image):
        """
        Publish a BGR8 ROS Image to the appropriate onboard topic.
        self.image is stored as RGB, so convert to BGR before publishing.
        """
        try:
            if self.image is None:
                return

            output_image = self._rgb_array_to_ros_bgr8(self.image)
            output_image.header = ros_image.header

            if app == "lane_detection":
                self.lane_img_pub.publish(output_image)
            elif app in ("traffic_light_detection", "traffic_sign_detection"):
                self.yolo_traffic.publish(output_image)
            elif app in ("collision_detection", "collision_avoidance"):
                self.yolo_obj.publish(output_image)
            else:
                rospy.logwarn_throttle(5, f"Unknown app '{app}' — no onboard topic.")

            rospy.loginfo(f"Forwarded {app} to onboard")

        except Exception as e:
            rospy.logerr(f"Error forwarding to onboard {app}: {e}")

    # ======================================================================
    # Image conversion utilities
    # ======================================================================
    @staticmethod
    def _rgb_array_to_ros_bgr8(rgb_image):
        """
        Convert an RGB numpy array to a bgr8-encoded ROS Image message.
        Does NOT do a spurious second RGB→BGR conversion — the input is
        already RGB, so one cv2.cvtColor call is all that is needed.
        """
        bgr = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        msg = Image()
        msg.height     = bgr.shape[0]
        msg.width      = bgr.shape[1]
        msg.encoding   = 'bgr8'
        msg.is_bigendian = 0
        msg.step       = msg.width * 3
        msg.data       = bgr.tobytes()
        return msg

    def cv2_to_ros_image(self, cv_image, encoding='bgr8'):
        """
        General-purpose CV2 → ROS Image converter.
        cv_image is assumed to be BGR (standard OpenCV convention).
        Pass encoding='bgr8' (default) for YOLO nodes.
        """
        msg = Image()
        msg.height       = cv_image.shape[0]
        msg.width        = cv_image.shape[1]
        msg.encoding     = encoding
        msg.is_bigendian = 0
        msg.step         = msg.width * 3
        msg.data         = cv_image.tobytes()
        return msg

    # ======================================================================
    # Edge response processing
    # ======================================================================
    def process_edge_response(self, response, app):
        """Parse and publish results returned by the edge server."""
        try:
            try:
                result = response.json()
            except json.JSONDecodeError:
                text = response.text
                rospy.logwarn(f"Edge response not valid JSON: {text[:200]}")
                if text.startswith('{') and text.endswith('}'):
                    result = json.loads(text)
                else:
                    rospy.logerr("Edge response cannot be parsed as JSON.")
                    return

            with self.lock:
                if 'Lane_detection_model' in result:
                    lane_data = result['Lane_detection_model']
                    if isinstance(lane_data, str):
                        lane_data = json.loads(lane_data)
                    self.process_lane_data(lane_data)

                if 'OBD_model' in result:
                    obd_data = result['OBD_model']
                    if isinstance(obd_data, str):
                        obd_data = json.loads(obd_data)
                    self.detection_results = obd_data.get('detections', [])
                    self.obj_pub.publish(String(data=json.dumps(self.detection_results)))

                if 'Traffic_detection_model' in result:
                    traffic_data = result['Traffic_detection_model']
                    if isinstance(traffic_data, str):
                        traffic_data = json.loads(traffic_data)
                    if traffic_data.get('success', False):
                        boxes    = traffic_data.get('boxes',    [])
                        classids = traffic_data.get('classids', [])
                        scores   = traffic_data.get('scores',   [])
                        detections = [
                            {
                                'class_name': f"class_{classids[i]}" if i < len(classids) else "unknown",
                                'box':   boxes[i],
                                'score': scores[i] if i < len(scores) else 0.0
                            }
                            for i in range(len(boxes))
                        ]
                        self.traffic_sign_results = detections
                        self.traffic_pub.publish(String(data=json.dumps(detections)))
                    else:
                        rospy.logwarn("Traffic detection not successful on edge.")
                        self.traffic_sign_results = []

        except Exception as e:
            rospy.logerr(f"Error processing edge response: {e}")

    def process_lane_data(self, lane_data):
        """Unpack and publish lane detection results from edge."""
        try:
            if not isinstance(lane_data, dict):
                rospy.logerr(f"Lane data is not a dict: {type(lane_data)}")
                return

            self.lane_data = {
                'horizontal_y': lane_data.get('horizontal_y', 0),
                'vertical_near': {
                    'up':       tuple(lane_data.get('vertical_near', {}).get('up',   [0, 0])),
                    'down':     tuple(lane_data.get('vertical_near', {}).get('down', [0, 0])),
                    'center_y': lane_data.get('vertical_near', {}).get('center_y', 0)
                },
                'vertical_far': {
                    'up':   tuple(lane_data.get('vertical_far', {}).get('up',   [0, 0])),
                    'down': tuple(lane_data.get('vertical_far', {}).get('down', [0, 0]))
                },
                'lane': {
                    'x':     lane_data.get('lane', {}).get('x',     -1),
                    'angle': lane_data.get('lane', {}).get('angle', None)
                }
            }

            self.lane_pub.publish(String(data=json.dumps(self.lane_data)))

            if 'binary_image' in lane_data:
                self._publish_base64_image(lane_data['binary_image'], self.binary_img_pub, "binary")
            if 'result_image' in lane_data:
                self._publish_base64_image(lane_data['result_image'], self.result_img_pub, "result")

        except Exception as e:
            rospy.logerr(f"Error processing lane data: {e}")

    def _publish_base64_image(self, image_data, publisher, label):
        """Decode a base64-encoded JPEG/PNG and publish as a ROS Image."""
        try:
            if not isinstance(image_data, str):
                return
            raw    = base64.b64decode(image_data)
            arr    = np.frombuffer(raw, np.uint8)
            cv_img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if cv_img is not None:
                publisher.publish(self.cv2_to_ros_image(cv_img))
            else:
                rospy.logwarn(f"Failed to decode {label} image from edge.")
        except Exception as e:
            rospy.logerr(f"Error publishing {label} image: {e}")


if __name__ == "__main__":
    LoadBalancerNode('load_balancer')