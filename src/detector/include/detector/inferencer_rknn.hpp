#pragma once
#include "inferencer.hpp"
#include <rknn_api.h>
#include <string>
#include "rclcpp/logger.hpp"

class Inferencer_RKNN : public Inferencer{
public:
    Inferencer_RKNN(const std::string& model_path,ModelParams model_params);
    Inferencer_RKNN(const std::string& model_path);
    ~Inferencer_RKNN() override;

    void InitModel(const std::string& model_path) override;
    std::vector<PoseResult> Infer(std::shared_ptr<cv::Mat> src) override;
    void PreProcess(std::shared_ptr<cv::Mat> src, std::shared_ptr<cv::Mat> blob) override;
    std::vector<PoseResult> PostProcess(
        float* out_data,
        int anchors,
        int infos
        ) override;
    void SetNPUCore(rknn_core_mask mask);


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
    std::string model_path_;


    float ratio_;
    int dw_;
    int dh_;
    int src_cols;
    int src_rows;

};