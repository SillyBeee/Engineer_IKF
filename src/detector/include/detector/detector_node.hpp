#pragma once
#include "inferencer.hpp"
#include "inferencer_onnx.hpp"
#include "inferencer_rknn.hpp"
#include "inferencer_pool.hpp"
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <vector>



class DetectorNode : public rclcpp::Node {
public:
    explicit DetectorNode(const rclcpp::NodeOptions & options);
    ~DetectorNode();

private:
    void MainLoop();
    void DrawDetections(cv::Mat& image, const std::vector<PoseResult>& results);
    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    // ROS
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;

    
    std::filesystem::path model_path_;
    std::string model_format_;
    std::unique_ptr<InferencerPool<Inferencer_RKNN, std::shared_ptr<cv::Mat>, std::vector<PoseResult>>> inferencer_pool_;
    

    // std::mutex image_mtx_;
    // std::deque<std::shared_ptr<cv::Mat>> image_queue_;
    std::thread main_loop_;


    // 模型参数
    ModelParams model_params_ = {
        640,
        640, 
        1,  
        4,   
        0.5
    };
};