#pragma once

// #define ONNX_MODE   //使用onnx进行推理
#define RKNN_MODE       //使用RKNN进行推理

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

#ifdef ONNX_MODE
#include <onnxruntime_cxx_api.h> 
#endif //ONNX_MODE

#ifdef RKNN_MODE
#include <rknn_api.h>
#endif //RKNN_MODE

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


class DetectorNode : public rclcpp::Node {
public:
    explicit DetectorNode(const rclcpp::NodeOptions & options);
    ~DetectorNode();

private:
    void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    //模型推理相关
    void InitModel(const std::string& model_path);
    std::vector<PoseResult> Infer(const cv::Mat& src);
    void PreProcess(const cv::Mat& src, cv::Mat& blob, float& ratio, int& dw, int& dh);
    
    // NMS
    void NMS(std::vector<PoseResult>& results, float conf_thres, float iou_thres);


    // ROS
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;

    #ifdef ONNX_MODE
    Ort::Env env_{nullptr};
    Ort::Session session_{nullptr};
    Ort::MemoryInfo allocator_info_{nullptr};
    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;
    #endif //ONNX_MODE

    


    #ifdef RKNN_MODE
    rknn_context rknn_ctx_{0};  
    rknn_input_output_num io_num_;
    std::vector<rknn_tensor_attr> input_attrs_;
    std::vector<rknn_tensor_attr> output_attrs_;  
    #endif //RKNN_MODE

    

    // 模型参数
    const int input_shape_width = 640;
    const int input_shape_height = 640;
    const int num_classes = 1;
    const int num_keypoints = 4;
    const double conf_threshold = 0.5;
};