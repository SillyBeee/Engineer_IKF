#pragma once

#include "inferencer.hpp"
#include <onnxruntime_cxx_api.h>


class Inferencer_ONNX : public Inferencer {
public:
    Inferencer_ONNX(const std::string& model_path,ModelParams model_params);
    ~Inferencer_ONNX() override;

    void InitModel(const std::string& model_path) override;
    std::vector<PoseResult> Infer( std::shared_ptr<cv::Mat> src) override;
    void PreProcess(std::shared_ptr<cv::Mat> src, std::shared_ptr<cv::Mat> blob) override;
    std::vector<PoseResult> PostProcess(
        float* out_data,
        int anchors,
        int infos
        ) override;
    
    rclcpp::Logger get_logger(){ return logger_; }
private:
    Ort::Env env_{nullptr};
    Ort::Session session_{nullptr};
    Ort::MemoryInfo allocator_info_{nullptr};
    std::vector<const char*> input_names_;
    std::vector<const char*> output_names_;

    rclcpp::Logger logger_ = rclcpp::get_logger("inferencer_onnx");


     // 模型参数
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