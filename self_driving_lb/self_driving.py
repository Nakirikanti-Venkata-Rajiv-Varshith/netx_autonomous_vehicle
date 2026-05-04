#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/03/28
# @author:aiden
# 无人驾驶 with detailed time logging

import os
import cv2
import math
import rospy
import signal
import threading
import numpy as np
import time
import hiwonder_sdk.pid as pid
import hiwonder_sdk.misc as misc
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
import geometry_msgs.msg as geo_msg
from hiwonder_interfaces.msg import ObjectsInfo
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from hiwonder_servo_controllers.bus_servo_control import set_servos
from servo_controller_arduino import ServoController
from std_msgs.msg import String, Float64, Float32MultiArray
# REMOVED: from cv_bridge import CvBridge
import json
import csv
from datetime import datetime
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

class SelfDrivingNodeLB:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.name = name
        self.image = None
        self.depth_image = None
        self.is_running = True
        self.pid = pid.PID(0.01, 0.0, 0.0)

        # Control flags
        self.detect_far_lane = False
        self.park_x = -1
        self.start_turn_time_stamp = 0
        self.count_turn = 0
        self.start_turn = False
        self.count_right = 0
        self.count_right_miss = 0
        self.turn_right = False
        self.stop = False
        self.start_park = False
        self.count_crosswalk = 0
        self.crosswalk_distance = 0
        self.crosswalk_length = 0.1 + 0.35
        self.start_slow_down = False
        self.normal_speed = 0.15
        self.slow_down_speed = 0.1
        self.traffic_signs_status = None

        # Object detection
        self.target_class = "keyboard"
        self.object_distance = 1

        # Lane data
        self.lane_data = {
            'horizontal_y': 0,
            'vertical_near': {'up': (0,0), 'down': (0,0), 'center_y': 0},
            'vertical_far': {'up': (0,0), 'down': (0,0)},
            'lane': {'x': -1, 'angle': 0}
        }

        self.upload_speed = 0.0
        self.download_speed = 0.0
        # REMOVED: self.bridge = CvBridge()
        self.colors = None

        signal.signal(signal.SIGINT, self.shutdown)
        self.machine_type = os.environ.get('MACHINE_TYPE')

        # Publishers
        self.mecanum_pub = rospy.Publisher('/hiwonder_controller/cmd_vel', geo_msg.Twist, queue_size=1)
        self.joints_pub = rospy.Publisher('servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)

        # Subscribers - ADD buff_size to prevent cv_bridge auto-conversion
        camera = rospy.get_param('/depth_camera/camera_name', 'depth_cam')
        self.image_sub_rgb = rospy.Subscriber(
            f'/{camera}/rgb/image_raw', 
            Image, 
            self.image_callback, 
            queue_size=1,
            buff_size=65536  # ADD THIS
        )
        self.image_sub_depth = rospy.Subscriber(
            f'/{camera}/depth/image_raw', 
            Image, 
            self.depth_callback, 
            queue_size=1,
            buff_size=65536  # ADD THIS
        )
        
        rospy.Subscriber('/load_balancer/yolov5/traffic_detect', ObjectsInfo, self.get_object_callback)
        rospy.Subscriber('/load_balancer/yolov5/object_detect', ObjectsInfo, self.get_od_callback)
        rospy.Subscriber('/lane_detection/result', String, self.lane_result_callback)
        
        # FIXED: Add buff_size to these image subscribers
        rospy.Subscriber(
            '/lane_detection/result_image', 
            Image, 
            self.lane_image_callback,
            queue_size=1,
            buff_size=65536  # THIS IS CRITICAL!
        )
        rospy.Subscriber(
            '/lane_detection/binary_image', 
            Image, 
            self.binary_image_callback,
            queue_size=1,
            buff_size=65536  # THIS IS CRITICAL!
        )
        
        rospy.Subscriber('/load_balaner/node_start_time', Float64, self.startup_latency_callback, queue_size=1)
        rospy.Subscriber("/network/bandwidth", Float32MultiArray, self.bandwidth_callback)

        # Servo controller setup
        try:
            self.servo_controller = ServoController()
            self.servo_controller.connect()
            self.set_default_servo_position()
            rospy.loginfo("Servo controller initialized")
        except Exception as e:
            rospy.logwarn(f"Servo controller not available: {e}")
            self.servo_controller = None

        set_servos(self.joints_pub, 1000, ((6, 490),))
        rospy.sleep(1)
        self.mecanum_pub.publish(geo_msg.Twist())

        # Start main image processing thread
        threading.Thread(target=self.image_proc, daemon=True).start()

    def shutdown(self, signum, frame):
        self.is_running = False
        rospy.loginfo('Shutdown triggered')
        self.mecanum_pub.publish(geo_msg.Twist())
        self.set_default_servo_position()
        cv2.destroyAllWindows()

    def control_loop(self):
        rate = rospy.Rate(10)
        while self.is_running and not rospy.is_shutdown():
            rospy.sleep(0.01)

    def bandwidth_callback(self, msg):
        self.upload_speed = msg.data[0]
        self.download_speed = msg.data[1]

    def startup_latency_callback(self, msg):
        self.startup_latency = msg.data

    def image_callback(self, ros_image):
        """Convert ROS image to numpy array without cv_bridge"""
        try:
            self.frame_id = ros_image.header.frame_id
            self.image_timestamp = ros_image.header.stamp.to_sec()
            
            # Use manual conversion like your other nodes
            if ros_image.encoding == 'rgb8':
                self.image = np.ndarray(
                    shape=(ros_image.height, ros_image.width, 3),
                    dtype=np.uint8,
                    buffer=ros_image.data
                )
            elif ros_image.encoding == 'bgr8':
                bgr_image = np.ndarray(
                    shape=(ros_image.height, ros_image.width, 3),
                    dtype=np.uint8,
                    buffer=ros_image.data
                )
                self.image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
            else:
                rospy.logwarn(f"Unsupported encoding in image_callback: {ros_image.encoding}")
                return
                
            logging.info(f"[RGB] Frame received. frame_id={self.frame_id}, ros_time={self.image_timestamp}")
        except Exception as e:
            logging.error(f"RGB image callback error: {e}")

    def depth_callback(self, ros_image):
        try:
            self.depth_frame_id = ros_image.header.frame_id
            self.depth_timestamp = ros_image.header.stamp.to_sec()
            if ros_image.encoding == "16UC1":
                self.depth_image = np.frombuffer(ros_image.data, dtype=np.uint16).reshape(ros_image.height, ros_image.width)
            elif ros_image.encoding == "32FC1":
                self.depth_image = np.frombuffer(ros_image.data, dtype=np.float32).reshape(ros_image.height, ros_image.width)
            logging.info(f"[Depth] Frame received. frame_id={self.depth_frame_id}, ros_time={self.depth_timestamp}")
        except Exception as e:
            logging.error(f"Depth callback error: {e}")

    def lane_result_callback(self, msg):
        try:
            self.lane_results = json.loads(msg.data)
            if self.lane_results:
                self.lane_data = {
                    'horizontal_y': self.lane_results.get('horizontal_y', 0),
                    'vertical_near': {
                        'up': tuple(self.lane_results.get('vertical_near', {}).get('up', [0,0])),
                        'down': tuple(self.lane_results.get('vertical_near', {}).get('down', [0,0])),
                        'center_y': self.lane_results.get('vertical_near', {}).get('center_y', 0)
                    },
                    'vertical_far': {
                        'up': tuple(self.lane_results.get('vertical_far', {}).get('up', [0,0])),
                        'down': tuple(self.lane_results.get('vertical_far', {}).get('down', [0,0]))
                    },
                    'lane': {
                        'x': self.lane_results.get('lane', {}).get('x', -1),
                        'angle': self.lane_results.get('lane', {}).get('angle', 0)
                    }
                }
                logging.info(f"[Lane] Lane data updated. frame_id={getattr(self, 'frame_id', 'NA')}")
        except Exception as e:
            rospy.logerr(f"Lane result parse error: {e}")

    def lane_image_callback(self, msg):
        """Callback for processed lane image - USING MANUAL CONVERSION"""
        try:
            # Use manual conversion instead of cv_bridge
            self.lane_image = self.manual_imgmsg_to_cv2(msg, "bgr8")
            if self.lane_image is not None:
                logging.debug(f"Lane image received: {self.lane_image.shape}")
        except Exception as e:
            rospy.logerr(f"Error converting lane image: {e}")

    def binary_image_callback(self, msg):
        """Callback for binary lane image - USING MANUAL CONVERSION"""
        try:
            # Use manual conversion instead of cv_bridge
            self.binary_image = self.manual_imgmsg_to_cv2(msg, "mono8")
            if self.binary_image is not None:
                logging.debug(f"Binary image received: {self.binary_image.shape}")
                # cv2.imshow('result', self.binary_image)
                cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error converting binary image: {e}")

    def manual_imgmsg_to_cv2(self, ros_image, encoding='bgr8'):
        """Manual conversion from ROS Image to OpenCV without cv_bridge"""
        try:
            if encoding == 'bgr8':
                if ros_image.encoding == 'rgb8':
                    # Convert RGB to BGR
                    rgb_image = np.ndarray(
                        shape=(ros_image.height, ros_image.width, 3),
                        dtype=np.uint8,
                        buffer=ros_image.data
                    )
                    return cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
                elif ros_image.encoding == 'bgr8':
                    # Already BGR
                    return np.ndarray(
                        shape=(ros_image.height, ros_image.width, 3),
                        dtype=np.uint8,
                        buffer=ros_image.data
                    )
                    
            elif encoding == 'mono8':
                if ros_image.encoding == 'mono8':
                    # Already mono8
                    return np.ndarray(
                        shape=(ros_image.height, ros_image.width),
                        dtype=np.uint8,
                        buffer=ros_image.data
                    )
                elif ros_image.encoding in ['rgb8', 'bgr8']:
                    # Convert color to grayscale
                    if ros_image.encoding == 'rgb8':
                        rgb_image = np.ndarray(
                            shape=(ros_image.height, ros_image.width, 3),
                            dtype=np.uint8,
                            buffer=ros_image.data
                        )
                        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
                    else:  # bgr8
                        bgr_image = np.ndarray(
                            shape=(ros_image.height, ros_image.width, 3),
                            dtype=np.uint8,
                            buffer=ros_image.data
                        )
                    return cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
            
            rospy.logwarn(f"Unsupported conversion: {ros_image.encoding} to {encoding}")
            return None
            
        except Exception as e:
            rospy.logerr(f"Manual image conversion error: {e}")
            return None

    # ... [REST OF YOUR CODE REMAINS THE SAME] ...

    def get_coordinates(self, bbox):
        coords = []
        x_min, y_min, x_max, y_max = map(int, bbox)
        for x in range(x_min, x_max+1):
            for y in range(y_min, y_max+1):
                coords.append((x,y))
        return coords

    def get_distance(self, depth_image, coordinates):
        distance=[]
        for x,y in coordinates:
            h,w = depth_image.shape
            if 0<=x<w and 0<=y<h:
                distance.append(float(depth_image[y,x]))
        non_zero = [d for d in distance if d>0]
        min_distance = min(non_zero) if non_zero else None
        if min_distance is not None:
            distance_m = min_distance/1000.0
            logging.info(f"[Distance] min_distance={distance_m:.3f} m, frame_id={getattr(self,'frame_id','NA')}")
            return distance_m
        return None

    def set_default_servo_position(self):
        if self.servo_controller:
            self.send_servo_command(0)

    def send_servo_command(self, angular_z):
        if self.servo_controller:
            val = self.map_angular_to_digital(angular_z)
            self.servo_controller.send_commands([(1,val)])

    def map_angular_to_digital(self, angular_z):
        angular_z = max(-1, min(angular_z,1))
        return str(int(10 + ((angular_z+1)*990)/2))

    def park_action(self):
        twist = geo_msg.Twist()
        for sign in [1,-1]:
            twist.linear.x = 0.15*sign
            twist.angular.z = twist.linear.x*math.tan(-0.6)/0.213
            self.mecanum_pub.publish(twist)
            self.send_servo_command(twist.angular.z)
            rospy.sleep(2)
        self.mecanum_pub.publish(geo_msg.Twist())
        self.set_default_servo_position()

    def image_proc(self):
        while self.is_running:
            if self.image is None:
                rospy.sleep(0.01)
                continue

            frame_start = rospy.get_time()
            logging.info(f"[Process] Start frame_id={getattr(self,'frame_id','NA')}")

            # Object stop check
            distance = self.object_distance
            if distance is not None and distance <= 0.5:
                self.stop = True
                actuation = time.time()
                self.delay = actuation - frame_start
                self.mecanum_pub.publish(geo_msg.Twist())
                logging.info(f"[Actuation] Stop triggered. distance={distance:.3f}, delay={self.delay:.4f}s, frame_id={getattr(self,'frame_id','NA')}")
            else:
                self.stop = False

            # Lane following and movement control logic (simplified for brevity)
            lane_x = self.lane_data['lane']['x']
            lane_angle = self.lane_data['lane']['angle']
            twist = geo_msg.Twist()
            if lane_x >=0 and not self.stop:
                self.pid.SetPoint = 90
                self.pid.update(lane_x)
                twist.linear.x = self.normal_speed
                twist.angular.z = twist.linear.x*math.tan(misc.set_range(self.pid.output,-0.1,0.1))/0.213
                self.send_servo_command(twist.angular.z)
                self.mecanum_pub.publish(twist)

            # Save CSV log
            # self.actuation_of_stop = time.time()
            # self.delay = self.actuation_of_stop - getattr(self, 'startup_latency', frame_start)


            frame_end = rospy.get_time()
            logging.info(f"[Process] End frame_id={getattr(self,'frame_id','NA')}, duration={frame_end - frame_start:.4f}s")
            rospy.sleep(0.01)

    def get_od_callback(self, od_msg):
        od_objects_info = od_msg.objects
        if od_objects_info == []:
            print("No objects detected.")
        else:
            filtered_boxes = []
            filtered_scores = []

            for i in od_objects_info:
                class_name = i.class_name
                if class_name==self.target_class:
                    filtered_boxes.append(i.box)
                    filtered_scores.append(i.score)

            if filtered_boxes:
                for i in range(len(filtered_boxes)):
                    bbox = filtered_boxes[i]
                    
                    if bbox is not None:
                        bbox_list = list(bbox)
                        coordinates = self.get_coordinates(bbox_list)
                        if coordinates:
                            self.object_distance = self.get_distance(self.depth_image, coordinates)
                        else:
                            self.object_distance=None
            else:
                self.object_distance=None
                return None

    def get_object_callback(self, msg):
        objects_info = msg.objects
        if objects_info == []:
            self.traffic_signs_status = None
            self.crosswalk_distance = 0
        else:
            min_distance = 0
            for i in objects_info:
                class_name = i.class_name
                center = (int((i.box[0] + i.box[2])/2), int((i.box[1] + i.box[3])/2))
                
                if class_name == 'crosswalk':  
                    if center[1] > min_distance:
                        min_distance = center[1]
                elif class_name == 'right':
                    self.count_right += 1
                    self.count_right_miss = 0
                    if self.count_right >= 10:
                        self.turn_right = True
                        self.count_right = 0
                elif class_name == 'park':
                    self.park_x = center[0]
                elif class_name == 'red' or class_name == 'green':
                    self.traffic_signs_status = class_name
        
            self.crosswalk_distance = min_distance
            
if __name__ == "__main__":
    # Add environment variables to prevent cv_bridge issues
    import os
    os.environ['OPENCV_IO_ENABLE_JASPER'] = '0'
    os.environ['OPENCV_IO_ENABLE_OPENEXR'] = '0'
    
    rospy.loginfo("Starting SelfDrivingNodeLB with time logging...")
    node = SelfDrivingNodeLB('self_driving_lb')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received")
    finally:
        rospy.loginfo("Node shutdown complete")