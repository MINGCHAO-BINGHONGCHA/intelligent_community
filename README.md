# Intelligent_community

## 环境
- OS:Ubuntu 20.04
- ROS:Noetic

## 安装

```shell
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/MINGCHAO-BINGHONGCHA/intelligent_community.git
cd ..
catkin_make
source devel/setup.sh
```

## 运行

启动gazebo环境
```shell
roslaunch mycar base_test.launch
```

启动nanodet识别
```shell
rosrun rosrun nanodet_ros ros_nanodet.py
```

启动车牌号识别
```shell
roslaunch hyperlpr bringup.launch
```

## 关于Nanodet模型训练以及转换

关于自定义模型可以参考我的[博客](https://www.ming-ice-tea.top/2025/10/19/Nanodet%E6%A8%A1%E5%9E%8B%E7%9A%84%E8%AE%AD%E7%BB%83%E4%BB%A5%E5%8F%8A%E7%A7%BB%E6%A4%8D%E5%88%B0ROS%E4%B8%AD/)




## Reference
https://github.com/stunback/ros-nanodet

https://github.com/elben6exam/HyperLPR