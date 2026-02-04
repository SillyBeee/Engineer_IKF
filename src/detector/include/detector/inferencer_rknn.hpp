#pragma once
#include "inferencer.hpp"
#include <rknn_api.h>
#include "rclcpp/logger.hpp"

class Inferencer_RKNN : public Inferencer{
public:
    Inferencer_RKNN(const std::string& model_path,ModelParams model_params);
    ~Inferencer_RKNN() override;

    void InitModel(const std::string& model_path) override;
    std::vector<PoseResult> Infer(const cv::Mat& src) override;
    void PreProcess(const cv::Mat& src, cv::Mat& blob) override;
    std::vector<PoseResult> PostProcess(
        float* out_data,
        int anchors,
        int infos
        ) override;


    rclcpp::Logger get_logger(){ return logger_; }
private:
    rknn_context rknn_ctx_{0};
    rknn_input_output_num io_num_;
    std::vector<rknn_tensor_attr> input_attrs_;
    std::vector<rknn_tensor_attr> output_attrs_;

    rclcpp::Logger logger_ = rclcpp::get_logger("inferencer_rknn");


    ModelParams model_params_ = {
        640,
        640, 
        1,  
        4,   
        0.3  
    };


    float ratio_;
    int dw_;
    int dh_;
    int src_cols;
    int src_rows;

};