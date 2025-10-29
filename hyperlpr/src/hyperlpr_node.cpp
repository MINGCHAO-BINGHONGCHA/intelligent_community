#include "hyperlpr_node.h"
#include <std_msgs/Bool.h>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>

void createDirIfNotExist(const std::string &path)
{
    struct stat st;
    if (stat(path.c_str(), &st) != 0)
        mkdir(path.c_str(), 0755);
}

// ✅ 构造函数实现
SubPuber::SubPuber(ros::NodeHandle &nh, const std::string &model_path)
    : modelPath(model_path), stop_flag(false)
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

    cameraImageSub = nh.subscribe("/camera/image_raw", 1, &SubPuber::PlateRecognitionCallback, this);
    recognitionResultPub = nh.advertise<std_msgs::String>("/plate_recognition/result", 1);
    recognitionConfidencePub = nh.advertise<std_msgs::Float32>("/plate_recognition/confidence", 1);
    recognizedImagePub = nh.advertise<sensor_msgs::Image>("/plate_recognition/image", 1);
    stopFlagSub = nh.subscribe("/stop_flag", 1, &SubPuber::StopFlagCallback, this);

    saveDir = "/home/ming/nanodet_save";
    createDirIfNotExist(saveDir);
}

void SubPuber::StopFlagCallback(const std_msgs::Bool::ConstPtr &msg)
{
    if (msg->data) {
        stopFlagCount++;
        //ROS_INFO("Received stop_flag = true, current count: %d", stopFlagCount);
    }
}

void SubPuber::PlateRecognitionCallback(const sensor_msgs::ImageConstPtr &cameraImage)
{
    cv::Mat img = cv_bridge::toCvShare(cameraImage, "bgr8")->image;
    std::vector<pr::PlateInfo> res = prc.RunPiplineAsImage(img, pr::SEGMENTATION_FREE_METHOD);

    for (auto st : res)
    {
        if (st.confidence > 0.75 && stopFlagCount >= SAVE_THRESHOLD && !img.empty())
        {
            std_msgs::String recognitionResult;
            recognitionResult.data = st.getPlateName();

            std_msgs::Float32 recognitionConfidence;
            recognitionConfidence.data = st.confidence;

            setlocale(LC_CTYPE, "zh_CN.utf8");
            ROS_INFO("Plate found: %s, confidence is: %f", recognitionResult.data.c_str(), recognitionConfidence.data);

            cv::Rect region = st.getPlateRect();
            cv::rectangle(img, cv::Point(region.x, region.y),
                          cv::Point(region.x + region.width, region.y + region.height),
                          cv::Scalar(255, 255, 0), 2);

            recognitionResultPub.publish(recognitionResult);
            recognitionConfidencePub.publish(recognitionConfidence);
        }
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    recognizedImagePub.publish(*msg);

    if (stopFlagCount >= SAVE_THRESHOLD && !img.empty()){
      std::stringstream ss;
      auto t = std::time(nullptr);
      auto tm = *std::localtime(&t);
      ss << saveDir << "/车牌"<< ".jpg";
      std::string filename = ss.str();

      cv::imwrite(filename, img);
      ROS_INFO("Saved plate image to: %s", filename.c_str());

      stopFlagCount = 0;  // 重置计数

      ROS_INFO("Shutting down node after saving image.");
      ros::shutdown();  // 安全关闭 ROS 节点
    }
}
