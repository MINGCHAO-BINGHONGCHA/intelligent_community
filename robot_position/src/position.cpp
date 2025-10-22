#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>  // 用于转换四元数到欧拉角

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_position_orientation_listener");
  ros::NodeHandle nh;

  tf::TransformListener listener;
  ros::Rate rate(10.0);  // 10Hz刷新频率

  // 坐标系设置（根据实际情况修改）
  std::string base_frame = "base_link";   // 机器人基坐标系
  std::string world_frame = "map";        // 世界坐标系

  ROS_INFO("开始监听机器人位置和航向角...");
  ROS_INFO("参考坐标系: %s, 机器人坐标系: %s", world_frame.c_str(), base_frame.c_str());

  while (nh.ok()) {
    tf::StampedTransform transform;
    try {
      // 等待并获取TF变换
      listener.waitForTransform(world_frame, base_frame, ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform(world_frame, base_frame, ros::Time(0), transform);

      // 获取XYZ位置
      double x = transform.getOrigin().x();
      double y = transform.getOrigin().y();
      double z = transform.getOrigin().z();

      // 获取旋转四元数并转换为欧拉角（滚转roll, 俯仰pitch, 偏航yaw）
      tf::Quaternion quat = transform.getRotation();
      double roll, pitch, yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  // 转换为弧度

      // 可选：将Yaw角转换为角度（更直观）
      double yaw_deg = yaw * 180.0 / M_PI;

      // 打印信息
      ROS_INFO("位置: x=%.2f, y=%.2f, z=%.2f m | Yaw角: %.2f 弧度 (%.2f 度)",
               x, y, z, yaw, yaw_deg);

    } catch (tf::TransformException& ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    rate.sleep();
  }

  return 0;
}