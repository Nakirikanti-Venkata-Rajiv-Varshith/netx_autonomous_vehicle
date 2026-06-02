#!/usr/bin/env python3
# encoding: utf-8
import os
import signal
import time

import cv2
import numpy as np
import rospy
import hiwonder_sdk.fps as fps
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
from yolov5_trt import YoLov5TRT, colors, plot_one_box
from hiwonder_interfaces.msg import ObjectInfo, ObjectsInfo


MODE_PATH = os.path.split(os.path.realpath(__file__))[0]


class Yolov5Node:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        self.name = name
        self.running = True
        self.start = False

        signal.signal(signal.SIGINT, self.shutdown)

        self.fps = fps.FPS()
        self.frame_skip = max(1, int(rospy.get_param("~frame_skip", 1)))
        self.cache_ttl = rospy.get_param("~cache_ttl", 0.3)

        engine = rospy.get_param("~engine")
        lib = rospy.get_param("~lib")
        conf_thresh = rospy.get_param("~conf_thresh", 0.8)
        self.classes = rospy.get_param("~classes")
        self.od_engine = rospy.get_param("~od_engine")
        self.od_classes = rospy.get_param("~od_classes")

        self.yolov5 = YoLov5TRT(os.path.join(MODE_PATH, engine), os.path.join(MODE_PATH, lib), self.classes, conf_thresh)
        self.yolov5_od = YoLov5TRT(
            os.path.join(MODE_PATH, self.od_engine), os.path.join(MODE_PATH, lib), self.od_classes, conf_thresh
        )

        self.traffic_image = None
        self.object_image = None
        self.traffic_seq = 0
        self.object_seq = 0
        self.last_traffic_msg = None
        self.last_object_msg = None
        self.last_traffic_publish = 0.0
        self.last_object_publish = 0.0

        rospy.Service("~start", Trigger, self.start_srv_callback)
        rospy.Service("~stop", Trigger, self.stop_srv_callback)

        self.traffic_sub = rospy.Subscriber(
            "/load_balancer/yolo/traffic_image", Image, self.traffic_callback, queue_size=1, buff_size=2**22
        )
        self.object_sub = rospy.Subscriber(
            "/load_balancer/yolo/object_image", Image, self.object_callback, queue_size=1, buff_size=2**22
        )

        self.object_pub = rospy.Publisher("/load_balancer/yolov5/traffic_detect", ObjectsInfo, queue_size=1)
        self.od_object_pub = rospy.Publisher("/load_balancer/yolov5/object_detect", ObjectsInfo, queue_size=1)

        rospy.set_param("~init_finish", True)
        self.image_proc()

    def start_srv_callback(self, msg):
        rospy.loginfo("start yolov5 detect")
        self.start = True
        return TriggerResponse(success=True)

    def stop_srv_callback(self, msg):
        rospy.loginfo("stop yolov5 detect")
        self.start = False
        return TriggerResponse(success=True)

    def traffic_callback(self, ros_image):
        rospy.loginfo(
            "traffic_callback received frame | encoding=%s | size=%dx%d",
            ros_image.encoding,
            ros_image.width,
            ros_image.height,
        )

        self.traffic_image = self.ros_to_bgr_image(ros_image)
        self.traffic_seq += 1

        rospy.loginfo("traffic_seq=%d", self.traffic_seq)

    def object_callback(self, ros_image):
        rospy.loginfo(
            "object_callback received frame | encoding=%s | size=%dx%d",
            ros_image.encoding,
            ros_image.width,
            ros_image.height,
        )

        self.object_image = self.ros_to_bgr_image(ros_image)
        self.object_seq += 1

        rospy.loginfo("object_seq=%d", self.object_seq)

    def ros_to_bgr_image(self, ros_image):
        rospy.loginfo("received image encoding=%s", ros_image.encoding)
        if ros_image.encoding == "rgb8":
            rgb = np.ndarray(
                shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data
            )
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if ros_image.encoding == "bgr8":
            return np.ndarray(
                shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data
            ).copy()
        rospy.logwarn_throttle(5.0, f"Unsupported image encoding: {ros_image.encoding}")
        return None

    def shutdown(self, signum, frame):
        self.running = False
        rospy.loginfo("shutdown")

    def traffic_funtion(self, image):
        rospy.loginfo("traffic_funtion called")
        infer_image = image.copy()
        objects_info = []
        boxes, scores, classid = self.yolov5.infer(infer_image)

        rospy.loginfo(
            "traffic inference complete | boxes=%d",
            len(boxes)
        )
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
                label="{}:{:.2f}".format(self.classes[cls_id], cls_conf),
            )

        object_msg = ObjectsInfo()
        object_msg.objects = objects_info
        self.last_traffic_msg = object_msg
        self.last_traffic_publish = time.time()
        self.object_pub.publish(object_msg)
        rospy.loginfo("yolov5 detect traffic sign: %d", len(boxes))

    def object_funtion(self, image):
        rospy.loginfo("object_funtion called")
        infer_image = image.copy()
        objects_info = []
        boxes, scores, classid = self.yolov5_od.infer(infer_image)

        rospy.loginfo(
            "object inference complete | boxes=%d",
            len(boxes)
        )
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
                label="{}:{:.2f}".format(self.od_classes[cls_id], cls_conf),
            )

        object_msg = ObjectsInfo()
        object_msg.objects = objects_info
        self.last_object_msg = object_msg
        self.last_object_publish = time.time()
        self.od_object_pub.publish(object_msg)
        rospy.loginfo("yolov5 detect object: %d", len(boxes))

    def maybe_publish_cached(self, stream_name):
        now = time.time()
        if stream_name == "traffic" and self.last_traffic_msg is not None:
            if now - self.last_traffic_publish <= self.cache_ttl:
                self.object_pub.publish(self.last_traffic_msg)
                return True
        if stream_name == "object" and self.last_object_msg is not None:
            if now - self.last_object_publish <= self.cache_ttl:
                self.od_object_pub.publish(self.last_object_msg)
                return True
        return False

    def image_proc(self):
        rospy.loginfo("image_proc started, waiting for images...")
        rospy.logwarn_throttle(2, f"YOLO start={self.start}")
        while self.running and not rospy.is_shutdown():
            rospy.loginfo_throttle(
                5,
                "image_proc alive | start=%s traffic=%s object=%s",
                self.start,
                self.traffic_image is not None,
                self.object_image is not None,
            )
            if not self.start:
                rospy.sleep(0.02)
                continue

            try:
                if self.traffic_image is not None:
                    image = self.traffic_image.copy()
                    self.traffic_image = None
                    rospy.loginfo(
                        "processing traffic frame | seq=%d frame_skip=%d",
                        self.traffic_seq,
                        self.frame_skip,
                    )
                    if self.traffic_seq % self.frame_skip == 0 or not self.maybe_publish_cached("traffic"):
                        self.traffic_funtion(image)

                if self.object_image is not None:
                    image = self.object_image.copy()
                    self.object_image = None
                    rospy.loginfo(
                        "processing object frame | seq=%d frame_skip=%d",
                        self.object_seq,
                        self.frame_skip,
                    )
                    if self.object_seq % self.frame_skip == 0 or not self.maybe_publish_cached("object"):
                        self.object_funtion(image)
            except BaseException as exc:
                rospy.logerr(f"YOLO processing error: {exc}")

            rospy.sleep(0.01)

        self.yolov5.destroy()
        self.yolov5_od.destroy()
        rospy.signal_shutdown("shutdown")


if __name__ == "__main__":
    Yolov5Node("yolov5_lb")
