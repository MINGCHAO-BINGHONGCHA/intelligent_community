#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class DistanceTracker:
    def __init__(self):
        # 初始化节点
        rospy.init_node('distance_tracker', anonymous=True)
        
        # 初始化变量
        self.total_distance = 0.0
        self.prev_x = None
        self.prev_y = None
        self.prev_z = None
        
        # 里程计话题订阅（可根据实际情况修改话题名）
        self.odom_sub = rospy.Subscriber(
            '/odom',  # 默认里程计话题，根据实际情况修改
            Odometry, 
            self.odom_callback
        )
        
        # 发布总距离话题
        self.distance_pub = rospy.Publisher(
            '/total_distance', 
            Float64, 
            queue_size=10
        )
        
        # 日志输出
        rospy.loginfo("Distance tracker initialized. Tracking vehicle distance...")
        
        # 保持节点运行
        rospy.spin()
    
    def odom_callback(self, msg):
        # 获取当前位置
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_z = msg.pose.pose.position.z  # 用于3D计算
        
        # 第一次获取位置时初始化
        if self.prev_x is None:
            self.prev_x = current_x
            self.prev_y = current_y
            self.prev_z = current_z
            return
        
        # 计算与上一位置的距离（2D或3D）
        # 2D距离计算
        dx = current_x - self.prev_x
        dy = current_y - self.prev_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # 3D距离计算（如果需要）
        # dz = current_z - self.prev_z
        # distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # 累加总距离
        self.total_distance += distance
        
        # 更新上一位置
        self.prev_x = current_x
        self.prev_y = current_y
        self.prev_z = current_z
        
        # 发布总距离
        self.distance_pub.publish(self.total_distance)
        
        # 定期打印信息（每10秒）
        if rospy.get_time() % 10 < 0.1:  # 简单的定时打印方式
            rospy.loginfo("Total distance traveled: %.2f meters", self.total_distance)

if __name__ == '__main__':
    try:
        DistanceTracker()
    except rospy.ROSInterruptException:
        rospy.loginfo("Distance tracker stopped.")
