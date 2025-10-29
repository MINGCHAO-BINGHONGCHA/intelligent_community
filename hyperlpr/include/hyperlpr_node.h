#ifndef HYPERLPR_NODE_H
#define HYPERLPR_NODE_H

#include "Pipeline.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>
#include <string>

class SubPuber
{
private:
    ros::NodeHandle nodeHandle;

    // 订阅与发布
    ros::Subscriber cameraImageSub;
    ros::Subscriber stopFlagSub;
    ros::Publisher recognitionResultPub;
    ros::Publisher recognitionConfidencePub;
    ros::Publisher recognizedImagePub;

    // HyperLPR识别对象
    pr::PipelinePR prc;

    // 保存图片路径
    std::string saveDir = "/home/ming/nanodet_save";

    // 停止标志
    bool stop_flag = false;
    int stopFlagCount = 0;  // 统计stop_flag为true的次数
    const int SAVE_THRESHOLD = 3;  // 达到3次才保存
public:
    // 模型路径，从launch文件中获得
    std::string modelPath;
    SubPuber(ros::NodeHandle &nh, const std::string &model_path);
    // 构造函数
    SubPuber()
    {
        // 获取相机话题
        std::string cameraTopic;
        ros::param::get("CameraTopic", cameraTopic);

        // 订阅相机话题
        cameraImageSub = nodeHandle.subscribe(cameraTopic, 1, &SubPuber::PlateRecognitionCallback, this);

        // 发布识别结果
        recognitionResultPub = nodeHandle.advertise<std_msgs::String>("/recognition_result", 1);
        recognitionConfidencePub = nodeHandle.advertise<std_msgs::Float32>("/recognition_confidence", 1);
        recognizedImagePub = nodeHandle.advertise<sensor_msgs::Image>("/recognized_image", 1);

        // 订阅停留标志
        stopFlagSub = nodeHandle.subscribe("/stop_flag", 1, &SubPuber::StopFlagCallback, this);

        // 创建保存目录
        struct stat st;
        if (stat(saveDir.c_str(), &st) != 0)
        {
            mkdir(saveDir.c_str(), 0755);
        }
    }

    // 设置模型路径并初始化模型
    void SendPath()
    {
        prc.initialize(modelPath + "/cascade.xml",
                       modelPath + "/HorizonalFinemapping.prototxt",
                       modelPath + "/HorizonalFinemapping.caffemodel",
                       modelPath + "/Segmentation.prototxt",
                       modelPath + "/Segmentation.caffemodel",
                       modelPath + "/CharacterRecognization.prototxt",
                       modelPath + "/CharacterRecognization.caffemodel",
                       modelPath + "/SegmenationFree-Inception.prototxt",
                       modelPath + "/SegmenationFree-Inception.caffemodel");
    }

    // 图像回调
    void PlateRecognitionCallback(const sensor_msgs::ImageConstPtr &cameraImage);

    // /stop_flag回调
    void StopFlagCallback(const std_msgs::Bool::ConstPtr &msg);
};

#endif
