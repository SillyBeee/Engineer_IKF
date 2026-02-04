#pragma once
#include "inferencer.hpp"
#include "inferencer_onnx.hpp"
#include "inferencer_rknn.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>



class DetectorNode : public rclcpp::Node {
public:
    explicit DetectorNode(const rclcpp::NodeOptions & options);
    ~DetectorNode();

private:
    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    // ROS
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;

    
    std::filesystem::path model_path_;
    std::string model_format_;
    
    

    // std::vector<Inferencer*> inferencers_;
    std::unique_ptr<Inferencer_RKNN> inferencer_rknn_ = nullptr;
    // 模型参数
    ModelParams model_params_ = {
        640,
        640, 
        1,  
        4,   
        0.5
    };
};