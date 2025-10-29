#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import roslib
import rospy
from std_msgs.msg import Header, Bool
from sensor_msgs.msg import Image
import os
import time
import cv2
import numpy as np
import nanodet_ros.nanodet
from nanodet_ros.nanodet import my_nanodet

IMAGE_WIDTH = 320
IMAGE_HEIGHT = 320

# 保存检测结果的目录
SAVE_DIR = "/home/ming/nanodet_save"
if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR)

# === 全局状态变量 ===
best_confidence = 0.0
best_image_path = None
stop_flag = False
prev_stop_flag = False   # 记录上一次状态，用于检测 True 触发
save_enabled = False     # 当前是否正在记录最优检测图
true_count = 0


def stop_flag_callback(msg):
    """订阅 stop_flag 话题，检测状态变化"""
    global stop_flag, prev_stop_flag, best_confidence, best_image_path, save_enabled, true_count

    stop_flag = msg.data

    # 检测状态变化：False -> True
    if stop_flag and not prev_stop_flag:
        true_count += 1
        #rospy.loginfo("🟢 stop_flag=True，开始新的检测阶段")
        best_image_path = None
        save_enabled = True  # 启动保存模式
        if true_count > 2:
            rospy.signal_shutdown("stop_flag reached 2 times")
        

    # 检测状态变化：True -> False
    elif not stop_flag and prev_stop_flag:
        #rospy.loginfo("🔴 stop_flag=False，结束检测阶段")
        save_enabled = False

    prev_stop_flag = stop_flag


def image_callback_1(image):
    global nanodet_model, nanodet_result_image
    global best_confidence, best_image_path, save_enabled, true_count

    # 将 ROS 图像转为 OpenCV 格式
    ros_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)[..., ::-1]

    # 模型检测（返回绘制图 + 结果列表）
    nanodet_result_image, results = nanodet_model.detect(ros_image, return_results=True)

    # === 只有在保存模式下，且检测到目标时，才进行保存逻辑 ===
    if save_enabled and len(results) > 0:
        #current_best_conf = max(r['score'] for r in results)

        # 如果当前置信度更高 → 更新最佳图像
        #if current_best_conf > best_confidence:
            #best_confidence = current_best_conf
        #timestamp = time.strftime("%Y%m%d_%H%M%S")
        save_path = os.path.join(SAVE_DIR, f"社区人员{true_count}.jpg")

        # 删除旧图（防止重复占空间）
        if best_image_path and os.path.exists(best_image_path):
            os.remove(best_image_path)

        cv2.imwrite(save_path, nanodet_result_image)
        best_image_path = save_path

        com_count = sum(1 for r in results if r['label'] == 'Com')
        noncom_count = sum(1 for r in results if r['label'] == 'NonCom')
        rospy.loginfo(f"社区人员：{com_count}，非社区人员：{noncom_count}")
        rospy.loginfo(f"📸 已保存：{save_path}")

    # 显示检测结果（可选）
    cv2.imshow('nanodet_result', nanodet_result_image)
    cv2.waitKey(5)

    # 发布检测图像
    publish_image(nanodet_result_image)


def publish_image(imgdata):
    image_temp = Image()
    header = Header(stamp=rospy.Time.now())
    header.frame_id = 'map'
    image_temp.height = IMAGE_HEIGHT
    image_temp.width = IMAGE_WIDTH
    image_temp.encoding = 'rgb8'
    image_temp.data = np.array(imgdata).tobytes()
    image_temp.header = header
    image_temp.step = IMAGE_WIDTH * 3
    image_pub.publish(image_temp)


if __name__ == '__main__':
    model_dir = os.path.dirname(nanodet_ros.nanodet.__file__)
    model_path = os.path.join(model_dir, 'model/nanodet-3.0_3.onnx')
    clsname_path = os.path.join(model_dir, 'model/class.names')
    nanodet_result_image = np.zeros([IMAGE_HEIGHT, IMAGE_WIDTH, 3], dtype=np.uint8)

    # 初始化模型
    nanodet_model = my_nanodet(model_path=model_path, clsname_path=clsname_path)

    # 初始化 ROS 节点
    rospy.init_node('ros_nanodet')
    image_topic_1 = "/camera/rgb/image_raw"
    rospy.Subscriber(image_topic_1, Image, image_callback_1, queue_size=1, buff_size=52428800)

    # ✅ 新增 stop_flag 订阅
    rospy.Subscriber('/stop_flag', Bool, stop_flag_callback)
    rospy.loginfo("✅ 已订阅 /camera/rgb/image_raw 和 /stop_flag")

    # 发布检测结果图像
    image_pub = rospy.Publisher('/nanodet_result_out', Image, queue_size=1)

    rospy.spin()
