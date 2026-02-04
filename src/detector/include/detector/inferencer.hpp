#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

// 定义关键点结构
struct KeyPoint{
    float x;
    float y;
    float score;
}__attribute__((packed));

// 定义单个输出姿态结果
struct PoseResult {
    cv::Rect box;
    float score;
    int label;
    std::vector<KeyPoint> kpts;
};


struct ModelParams{
    int input_shape_width;
    int input_shape_height;
    int num_classes;
    int num_keypoints;
    double conf_threshold;
};



class Inferencer{
public:
    Inferencer()=default;
    virtual ~Inferencer() = default;
    virtual void InitModel(const std::string& model_path)=0;
    virtual std::vector<PoseResult> Infer(const cv::Mat& src) = 0;
    virtual void PreProcess(const cv::Mat& src, cv::Mat& blob)=0;
    virtual std::vector<PoseResult> PostProcess(
        float* out_data,
        int anchors,
        int infos
        ) = 0;
};