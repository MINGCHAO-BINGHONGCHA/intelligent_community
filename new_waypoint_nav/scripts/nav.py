#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped

class MultiNavNode:
    def __init__(self):
        rospy.init_node("multi_point_navigation")

        # === 订阅 AMCL 位姿 ===
        self.amcl_pose = None
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)

        # === move_base 客户端 ===
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("等待 move_base...")
        self.client.wait_for_server()
        rospy.loginfo("move_base 已连接")

        # === 读取路径文件 ===
        self.waypoints = self.load_waypoints("/home/ming/waypoints.txt")
        rospy.loginfo("加载点位数量：{}".format(len(self.waypoints)))

        # === 自动开始导航 ===
        self.run()

    def amcl_callback(self, msg):
        """实时更新 AMCL 估计位置"""
        self.amcl_pose = msg.pose.pose

    def load_waypoints(self, file_path):
        """从文件读取点位"""
        points = []
        with open(file_path, "r") as f:
            for line in f:
                if line.strip() == "":
                    continue
                x, y, yaw = map(float, line.split())
                points.append((x, y, yaw))
        return points

    def send_goal(self, x, y, yaw):
        """向 move_base 发送单个目标点"""
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        rospy.loginfo("发送目标点: x={:.2f}, y={:.2f}, yaw={:.2f}".format(x, y, yaw))
        self.client.send_goal(goal)

        self.client.wait_for_result()
        state = self.client.get_state()

        return state

    def wait_for_localization(self):
        """等待 AMCL 收敛（确保定位稳定）"""
        rospy.loginfo("等待 AMCL 位姿...")
        while self.amcl_pose is None:
            rospy.sleep(0.1)
        rospy.loginfo("AMCL 已准备")

    def run(self):
        """主逻辑：逐点导航"""
        self.wait_for_localization()

        for i, (x, y, yaw) in enumerate(self.waypoints):
            rospy.loginfo("==== 开始导航到第 {} 个点 ====".format(i + 1))

            # 发送点位
            result_state = self.send_goal(x, y, yaw)

            if result_state == 3:  # SUCCEEDED
                rospy.loginfo("到达第 {} 个点".format(i + 1))
            else:
                rospy.logwarn("第 {} 个点导航失败，跳过".format(i + 1))

        rospy.loginfo("全部点位导航完成！")


if __name__ == "__main__":
    try:
        MultiNavNode()
    except rospy.ROSInterruptException:
        pass

