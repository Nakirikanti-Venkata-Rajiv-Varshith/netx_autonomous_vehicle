#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/18
# @author:aiden
# yolov5目标检测
import os
import cv2
import time
import rospy
import signal
import numpy as np
from std_msgs.msg import Float64
import hiwonder_sdk.fps as fps
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
from yolov5_trt import YoLov5TRT, colors, plot_one_box
from hiwonder_interfaces.msg import ObjectInfo, ObjectsInfo

MODE_PATH = os.path.split(os.path.realpath(__file__))[0]

class Yolov5Node:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.name = name
        
        self.bgr_image = None
        self.running = True
        self.start = False

        signal.signal(signal.SIGINT, self.shutdown)

        
        #TRAFFIC_SIGN_DETCTEION
        self.fps = fps.FPS()  # fps计算器
        engine = rospy.get_param('~engine')
        lib = rospy.get_param('~lib')
        conf_thresh = rospy.get_param('~conf_thresh', 0.8)
        self.classes = rospy.get_param('~classes')
        
        self.yolov5 = YoLov5TRT(os.path.join(MODE_PATH, engine), os.path.join(MODE_PATH, lib), self.classes, conf_thresh)

        #OBJECT_DETCTEION
        od_engine = rospy.get_param('~od_engine')
        self.od_classes = rospy.get_param('~od_classes')
        
        self.yolov5_od = YoLov5TRT(os.path.join(MODE_PATH, od_engine), os.path.join(MODE_PATH, lib), self.od_classes, conf_thresh)
        

        rospy.Service('~start', Trigger, self.start_srv_callback)  # 进入玩法
        rospy.Service('~stop', Trigger, self.stop_srv_callback)  # 退出玩法

        self.traffic_image = None
        self.object_image = None

        self.traffic_sub = rospy.Subscriber("/load_balancer/yolo/traffic_image", Image, self.traffic_callback, queue_size=1)
        self.object_sub = rospy.Subscriber("/load_balancer/yolo/object_image", Image, self.object_callback, queue_size=1)
        
        #TRAFFIC SIGN DETECTION RESULTS PUBLISH
        self.object_pub = rospy.Publisher('/load_balancer/yolov5/traffic_detect', ObjectsInfo, queue_size=1)
        # self.result_image_pub = rospy.Publisher('/yolov5_k/traffic_image', Image, queue_size=1)
        
        
        #OBJECT DETECTION RESULTS PUBLISH
        self.od_object_pub = rospy.Publisher('/load_balancer/yolov5/object_detect', ObjectsInfo, queue_size=1)
        # self.od_result_image_pub = rospy.Publisher('/yolov5_k/object_image', Image, queue_size=1)
        # self.start_time = rospy.Publisher('/yolov5_k/object_detection_start_time', Float64, queue_size=1)

        rospy.set_param('~init_finish', True)
        self.image_proc()

    def start_srv_callback(self, msg):
        rospy.loginfo("start yolov5 detect")

        self.start = True

        return TriggerResponse(success=True)

    def stop_srv_callback(self, msg):
        rospy.loginfo('stop yolov5 detect')

        self.start = False

        return TriggerResponse(success=True)

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像
        self.bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        # self.bgr_image = rgb_image

    def traffic_callback(self, ros_image):
        rgb_image = np.ndarray(
            shape=(ros_image.height, ros_image.width, 3),
            dtype=np.uint8,
            buffer=ros_image.data
        )
        self.traffic_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)


    def object_callback(self, ros_image):
        rgb_image = np.ndarray(
            shape=(ros_image.height, ros_image.width, 3),
            dtype=np.uint8,
            buffer=ros_image.data
        )
        self.object_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
   
    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo('shutdown')

    def traffic_funtion(self,image):
            rospy.loginfo("traffic sign detect function")
            infer_yolo_image=image.copy()
            objects_info = []
            boxes, scores, classid = self.yolov5.infer(infer_yolo_image)
            for box, cls_conf, cls_id in zip(boxes, scores, classid):
                color = colors(cls_id, True)

                object_info = ObjectInfo()
                object_info.class_name = self.classes[cls_id]
                object_info.box = box.astype(int)
                object_info.score = cls_conf

                objects_info.append(object_info)

                plot_one_box(
                    box,
                    image,
                    color=color,
                    label="{}:{:.2f}".format(
                    self.classes[cls_id], cls_conf
                    ),
                )
            rospy.loginfo("yolov5 detect traffic sign: %d"%(len(boxes)))
            object_msg = ObjectsInfo()
            object_msg.objects = objects_info
            self.object_pub.publish(object_msg)

    def object_funtion(self,image):
            rospy.loginfo("object detect function")
            infer_yolo_image=image.copy()
            objects_info = []
            boxes, scores, classid = self.yolov5_od.infer(infer_yolo_image)
            for box, cls_conf, cls_id in zip(boxes, scores, classid):
                color = colors(cls_id, True)

                object_info = ObjectInfo()
                object_info.class_name = self.od_classes[cls_id]
                object_info.box = box.astype(int)
                object_info.score = cls_conf

                objects_info.append(object_info)

                plot_one_box(
                    box,
                    image,
                    color=color,
                    label="{}:{:.2f}".format(
                    self.od_classes[cls_id], cls_conf
                    ),
                )
            rospy.loginfo("yolov5 detect object: %d"%(len(boxes)))
            object_msg = ObjectsInfo()
            object_msg.objects = objects_info
            self.od_object_pub.publish(object_msg)

    def image_proc(self):
        rospy.loginfo("image_proc started, waiting for images...")
        while self.running:
                # image = self.bgr_image
            
            rospy.loginfo("image process if block")
            # infer_yolo_image=image.copy()
            # infer_yolo_od_image=image.copy()
            try:
                rospy.loginfo("Entered try block")
                if self.traffic_image is not None:
                    rospy.loginfo("Processing traffic image")
                    image = self.traffic_image.copy()
                    self.traffic_funtion(image)
                    self.traffic_image = None   # clear after processing

                # OBJECT DETECTION ONLY if object image received
                if self.object_image is not None:
                    rospy.loginfo("Processing object image")
                    image = self.object_image.copy()
                    self.object_funtion(image)
                    self.object_image = None   # clear after processing

            except BaseException as e:
                print(e)

            rospy.sleep(0.01)
        self.yolov5.destroy()
        self.yolov5_od.destroy()  
        rospy.signal_shutdown('shutdown')

def cv2_image2ros(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_temp = Image()
    header = Header(stamp=rospy.Time.now())
    header.frame_id = 'yolov5'
    image_temp.height = image.shape[:2][0]
    image_temp.width = image.shape[:2][1]
    image_temp.encoding = 'rgb8'
    image_temp.data = image.tostring()
    image_temp.header = header
    image_temp.step = image_temp.width*3
    
    return image_temp

if __name__ == "__main__":
    node = Yolov5Node('yolov5_k')
