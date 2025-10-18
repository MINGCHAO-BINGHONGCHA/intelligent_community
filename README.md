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

## Reference
https://github.com/stunback/ros-nanodet

https://github.com/elben6exam/HyperLPR