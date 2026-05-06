#!/usr/bin/env python3
# encoding: utf-8
import cv2
import math
import numpy as np
import rospy
import base64
import json
from sensor_msgs.msg import Image
from std_msgs.msg import String
# REMOVED: from cv_bridge import CvBridge
import hiwonder_sdk.common as common

lab_data = common.get_yaml_data("/home/hiwonder/software/lab_tool/lab_config.yaml")

class LaneDetector(object):
    def __init__(self, color):
        self.target_color = color
        self.rois = ((450, 480, 0, 320, 0.7), (390, 420, 0, 320, 0.2), (330, 360, 0, 320, 0.1))
        self.weight_sum = 1.0

    @staticmethod
    def get_area_max_contour(contours, threshold=100):
        contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
        contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))
        if len(contour_area) > 0:
            return max(contour_area, key=lambda c_a: c_a[1])
        return None

    def add_horizontal_line(self, image):
        #   |____  --->   |————   ---> ——
        h, w = image.shape[:2]
        roi_w_min = int(w/2)
        roi_w_max = w
        roi_h_min = 0
        roi_h_max = h
        roi = image[roi_h_min:roi_h_max, roi_w_min:roi_w_max]  # 截取右半边
        flip_binary = cv2.flip(roi, 0)  # 上下翻转
        max_y = cv2.minMaxLoc(flip_binary)[-1][1]  # 提取最上，最左数值为255的点坐标

        return h - max_y

    def add_vertical_line_far(self, image):
        h, w = image.shape[:2]
        roi_w_min = int(w/8)
        roi_w_max = int(w/2)
        roi_h_min = 0
        roi_h_max = h
        roi = image[roi_h_min:roi_h_max, roi_w_min:roi_w_max]
        flip_binary = cv2.flip(roi, -1)  # 图像左右上下翻转
        #cv2.imshow('1', flip_binary)
        # min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(ret)
        # minVal：最小值
        # maxVal：最大值
        # minLoc：最小值的位置
        # maxLoc：最大值的位置
        # 遍历的顺序，先行再列，行从左到右，列从上到下
        (x_0, y_0) = cv2.minMaxLoc(flip_binary)[-1]  # 提取最上，最左数值为255的点坐标
        y_center = y_0 + 25
        roi = flip_binary[y_center:, :]
        (x_1, y_1) = cv2.minMaxLoc(roi)[-1]
        down_p = (roi_w_max - x_1, roi_h_max - (y_1 + y_center))
        
        y_center = y_0 + 35
        roi = flip_binary[y_center:, :]
        (x_2, y_2) = cv2.minMaxLoc(roi)[-1]
        up_p = (roi_w_max - x_2, roi_h_max - (y_2 + y_center))

        up_point = (0, 0)
        down_point = (0, 0)
        if up_p[1] - down_p[1] != 0 and up_p[0] - down_p[0] != 0:
            up_point = (int(-down_p[1]/((up_p[1] - down_p[1])/(up_p[0] - down_p[0])) + down_p[0]), 0)
            down_point = (int((h - down_p[1])/((up_p[1] - down_p[1])/(up_p[0] - down_p[0])) + down_p[0]), h)

        return up_point, down_point

    def add_vertical_line_near(self, image):
        # ——|         |——        |
        #   |   --->  |     --->
        h, w = image.shape[:2]
        roi_w_min = 0
        roi_w_max = int(w/2)
        roi_h_min = int(h/2)
        roi_h_max = h
        roi = image[roi_h_min:roi_h_max, roi_w_min:roi_w_max]
        flip_binary = cv2.flip(roi, -1)  # 图像左右上下翻转
        #cv2.imshow('1', flip_binary)
        (x_0, y_0) = cv2.minMaxLoc(flip_binary)[-1]  # 提取最上，最左数值为255的点坐标
        down_p = (roi_w_max - x_0, roi_h_max - y_0)

        (x_1, y_1) = cv2.minMaxLoc(roi)[-1]
        y_center = int((roi_h_max - roi_h_min - y_1 + y_0)/2)
        roi = flip_binary[y_center:, :] 
        (x, y) = cv2.minMaxLoc(roi)[-1]
        up_p = (roi_w_max - x, roi_h_max - (y + y_center))
        
        up_point = (0, 0)
        down_point = (0, 0)
        if up_p[1] - down_p[1] != 0 and up_p[0] - down_p[0] != 0:
            up_point = (int(-down_p[1]/((up_p[1] - down_p[1])/(up_p[0] - down_p[0])) + down_p[0]), 0)
            down_point = down_p 
        return up_point, down_point, y_center
    
    def get_binary(self, image):
        rospy.loginfo("Generating binary image for lane detection...")
        img_lab = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)
        img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)
        mask = cv2.inRange(
            img_blur,
            tuple(lab_data['lab']['Stereo'][self.target_color]['min']),
            tuple(lab_data['lab']['Stereo'][self.target_color]['max'])
        )
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        return dilated

    def __call__(self, image, result_image):
        centroid_sum = 0
        h, w = image.shape[:2]
        max_center_x = -1
        center_x = []
        for roi in self.rois:
            blob = image[roi[0]:roi[1], roi[2]:roi[3]]
            contours = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]
            max_contour_area = self.get_area_max_contour(contours, 30)
            if max_contour_area is not None:
                rect = cv2.minAreaRect(max_contour_area[0])
                box = np.int0(cv2.boxPoints(rect))
                for j in range(4):
                    box[j, 1] = box[j, 1] + roi[0]
                cv2.drawContours(result_image, [box], -1, (0, 255, 255), 2)

                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2

                cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)
                center_x.append(line_center_x)
            else:
                center_x.append(-1)

        for i in range(len(center_x)):
            if center_x[i] != -1:
                if center_x[i] > max_center_x:
                    max_center_x = center_x[i]
                centroid_sum += center_x[i] * self.rois[i][-1]

        if centroid_sum == 0:
            return result_image, None, max_center_x

        center_pos = centroid_sum / self.weight_sum
        angle = math.degrees(-math.atan((center_pos - (w / 2.0)) / (h / 2.0)))
        return result_image, angle, max_center_x


class LaneNode:
    def __init__(self):
        self.lane_detect = LaneDetector('yellow')

        # Subscriber: read from load_balancer topic
        # Add buff_size parameter to prevent cv_bridge auto-conversion
        self.image_sub = rospy.Subscriber(
            '/load_balancer/lane_image', 
            Image, 
            self.image_callback, 
            queue_size=1,
            buff_size=65536  # THIS IS CRITICAL - prevents cv_bridge auto-conversion
        )

        # Publisher: publish processed lane detection results
        self.result_pub = rospy.Publisher('/lane_detection/result', String, queue_size=1)
        self.result_img_pub = rospy.Publisher('/lane_detection/result_image', Image, queue_size=1)
        self.binary_image_pub = rospy.Publisher('/lane_detection/binary_image', Image, queue_size=1)

    def image_callback(self, ros_image):
        """Callback using EXACT same method as LoadBalancerNode and YOLO nodes"""
        try:
            # METHOD 1: Exact same as LoadBalancerNode image_callback
            rospy.logdebug(f"Received image for lane detection")
            if ros_image.encoding == 'rgb8':
                rgb_image = np.ndarray(
                    shape=(ros_image.height, ros_image.width, 3),
                    dtype=np.uint8,
                    buffer=ros_image.data
                )
                cv_image = rgb_image  # Lane detection expects RGB
                
            elif ros_image.encoding == 'bgr8':
                bgr_image = np.ndarray(
                    shape=(ros_image.height, ros_image.width, 3),
                    dtype=np.uint8,
                    buffer=ros_image.data
                )
                # Convert BGR to RGB for lane detection
                cv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
                
            else:
                rospy.logwarn(f"Unsupported image encoding: {ros_image.encoding}")
                return
                
        except Exception as e:
            rospy.logerr(f"Image conversion Error: {e}")
            return

        # Process the image
        binary_image = self.lane_detect.get_binary(cv_image)
        
        # Extract detection results
        horizontal_y = self.lane_detect.add_horizontal_line(binary_image)
        near_up, near_down, near_center = self.lane_detect.add_vertical_line_near(binary_image)
        far_up, far_down = self.lane_detect.add_vertical_line_far(binary_image)
        result_image, lane_angle, lane_x = self.lane_detect(binary_image, cv_image.copy())

        # Prepare response
        response = {
            'horizontal_y': horizontal_y,
            'vertical_near': {
                'up': near_up,
                'down': near_down,
                'center_y': near_center
            },
            'vertical_far': {
                'up': far_up,
                'down': far_down
            },
            'lane': {
                'angle': lane_angle,
                'x': lane_x
            },
            'source': 'onboard',
            'timestamp': round(rospy.Time.now().to_sec(), 4),
            'quality': round(float(np.count_nonzero(binary_image)) / float(binary_image.size), 4),
            'fresh_inference': True
        }

        # Debug visualization (optional)
        # cv2.imshow("binary", binary_image)
        # cv2.imshow("lane_result", cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)

        # Convert images to ROS messages manually
        result_msg = self.cv2_to_ros_image(result_image, encoding='rgb8')
        binary_msg = self.cv2_to_ros_image(binary_image, encoding='mono8')

        # Publish result back to ROS
        self.result_pub.publish(json.dumps(response))
        self.result_img_pub.publish(result_msg)
        self.binary_image_pub.publish(binary_msg)

    def cv2_to_ros_image(self, cv_image, encoding='rgb8'):
        """Convert CV2 image to ROS Image message without cv_bridge."""
        ros_image = Image()
        ros_image.height = cv_image.shape[0]
        ros_image.width = cv_image.shape[1]
        ros_image.encoding = encoding
        ros_image.is_bigendian = 0
        ros_image.header.stamp = rospy.Time.now()
        
        # Handle different encodings
        if encoding == 'rgb8':
            if len(cv_image.shape) == 3 and cv_image.shape[2] == 3:
                if cv_image.dtype == np.uint8:
                    ros_image.data = cv_image.tobytes()
                    ros_image.step = ros_image.width * 3
                else:
                    rospy.logwarn("Unsupported image dtype for rgb8 encoding")
                    return ros_image
            else:
                rospy.logwarn("Invalid image shape for rgb8 encoding")
                return ros_image
                
        elif encoding == 'mono8':
            if len(cv_image.shape) == 2:
                # Already grayscale
                ros_image.data = cv_image.tobytes()
                ros_image.step = ros_image.width
            elif len(cv_image.shape) == 3 and cv_image.shape[2] == 3:
                # Convert color to grayscale
                gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                ros_image.data = gray_image.tobytes()
                ros_image.step = ros_image.width
            else:
                rospy.logwarn("Invalid image shape for mono8 encoding")
                return ros_image
        else:
            rospy.logwarn(f"Unsupported encoding: {encoding}")
            return ros_image
            
        return ros_image

if __name__ == '__main__':
    rospy.init_node('lane_detect_node', anonymous=True)
    
    # Optional: Add environment variables to further prevent cv_bridge issues
    import os
    os.environ['OPENCV_IO_ENABLE_JASPER'] = '0'
    os.environ['OPENCV_IO_ENABLE_OPENEXR'] = '0'
    
    rospy.loginfo("Starting Lane Detection Node without cv_bridge...")
    LaneNode()
    rospy.spin()
