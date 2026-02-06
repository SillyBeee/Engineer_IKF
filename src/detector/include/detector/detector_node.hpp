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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

typedef struct TechCorePose_{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
} TechCorePose;




class DetectorNode : public rclcpp::Node {
public:
    explicit DetectorNode(const rclcpp::NodeOptions & options);
    ~DetectorNode();

private:
    void MainLoop();
    void DrawDetections(cv::Mat& image, const std::vector<PoseResult>& results);
    void VisualizeTechCore(TechCorePose& pose);
    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    TechCorePose ProcessPNP(std::vector<KeyPoint> kpts);

    

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr techcore_marker_pub_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    

    
    std::filesystem::path model_path_;
    std::string model_format_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
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