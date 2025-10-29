#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import yaml
import math
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool   # ✅ 新增：布尔消息类型

class WaypointFollower:
    def __init__(self, yaml_file):
        # 加载航点
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)
        self.waypoints = data['waypoints']
        self.current_pose = None

        # 发布cmd_vel
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # ✅ 新增：发布flag话题
        self.flag_pub = rospy.Publisher('/stop_flag', Bool, queue_size=10)

        self.rate = rospy.Rate(10)  # 10Hz
        self.linear_speed = 0.2     # m/s
        self.angular_speed = 0.4    # rad/s
        self.dist_tolerance = 0.05
        self.yaw_tolerance = 0.05
        self.stop_waypoints = {3, 4, 8}  # 需要停留的航点编号
        self.stop_duration = 5.0    # 停留时间（秒）

    def odom_callback(self, msg):
        """保存当前位置和朝向"""
        self.current_pose = msg.pose.pose

    def get_yaw_from_quaternion(self, orientation):
        """四元数转yaw"""
        q = orientation
        (_, _, yaw) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def stop(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def rotate_to_yaw(self, target_yaw):
        """原地旋转到目标角度"""
        while not rospy.is_shutdown() and self.current_pose:
            current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
            diff = self.normalize_angle(target_yaw - current_yaw)

            if abs(diff) < self.yaw_tolerance:
                break

            twist = Twist()
            twist.angular.z = self.angular_speed if diff > 0 else -self.angular_speed
            self.cmd_pub.publish(twist)
            self.rate.sleep()

        self.stop()

    def move_to_point(self, target_x, target_y):
        """直线移动到目标点"""
        while not rospy.is_shutdown() and self.current_pose:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            distance = math.sqrt((target_x - x)**2 + (target_y - y)**2)

            if distance < self.dist_tolerance:
                break

            angle_to_target = math.atan2(target_y - y, target_x - x)
            current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
            yaw_error = self.normalize_angle(angle_to_target - current_yaw)

            twist = Twist()

            if abs(yaw_error) > 0.2:
                twist.angular.z = 0.5 * yaw_error
            else:
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.3 * yaw_error

            self.cmd_pub.publish(twist)
            self.rate.sleep()

        self.stop()

    def normalize_angle(self, angle):
        """将角度归一化到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def run(self):
        rospy.loginfo("等待里程计数据...")
        while not rospy.is_shutdown() and self.current_pose is None:
            self.rate.sleep()
        rospy.loginfo("开始执行航点跟随")

        for i, wp in enumerate(self.waypoints):
            wp_num = i + 1
            rospy.loginfo("➡️ 前往第 %d 个点: (%.2f, %.2f, %.2f)" %
                          (wp_num, wp['x'], wp['y'], wp['yaw']))
            self.move_to_point(wp['x'], wp['y'])
            self.rotate_to_yaw(wp['yaw'])
            rospy.loginfo("✅ 到达第 %d 个点" % wp_num)

            # ✅ 发布flag消息
            is_stop = wp_num in self.stop_waypoints
            self.flag_pub.publish(Bool(data=is_stop))
            #rospy.loginfo("📡 stop_flag = %s" % str(is_stop))

            # 停留逻辑
            if is_stop:
                rospy.loginfo("⏸️ 在第 %d 个点停留 %.1f 秒" %
                              (wp_num, self.stop_duration))
                rospy.sleep(self.stop_duration)
                self.flag_pub.publish(Bool(data=False))
            else:
                rospy.sleep(1.0)

        rospy.loginfo("🎯 所有航点完成！")
        self.stop()


if __name__ == '__main__':
    rospy.init_node('waypoint_follower_cmdvel')
    yaml_file = rospy.get_param("~waypoint_file", "")
    if not yaml_file:
        rospy.logerr("没有提供参数 waypoint_file")
        exit(1)

    follower = WaypointFollower(yaml_file)
    follower.run()
