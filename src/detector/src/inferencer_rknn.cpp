#include "inferencer_rknn.hpp"
#include "inferencer.hpp"
#include "rknn_api.h"
#include "scope_timer.hpp"
#include <exception>
Inferencer_RKNN::Inferencer_RKNN(const std::string& model_path,ModelParams model_params){
    this->model_params_ = model_params;
    InitModel(model_path);
}

Inferencer_RKNN::~Inferencer_RKNN(){
    if(rknn_ctx_){
        rknn_destroy(rknn_ctx_);
    }
}

void Inferencer_RKNN::InitModel(const std::string& model_path){
    try{
        FILE *fp = fopen(model_path.c_str(), "rb");
        if(fp == nullptr) {
            RCLCPP_ERROR(this->logger_, "Failed to open model file: %s", model_path.c_str());
            return;
        }
        fseek(fp, 0, SEEK_END);
        int model_len = ftell(fp);
        void *model_data = malloc(model_len);
        fseek(fp, 0, SEEK_SET);
        if(fread(model_data, 1, model_len, fp) != (size_t)model_len) {
            RCLCPP_ERROR(this->logger_, "Failed to read model data");
            free(model_data);
            fclose(fp);
            return;
        }
        fclose(fp);

        // 2. 初始化 RKNN 上下文
        int ret = rknn_init(&rknn_ctx_, model_data, model_len, 0, NULL);

        //debug模式
        // int ret = rknn_init(&rknn_ctx_, model_data, model_len,  RKNN_FLAG_COLLECT_PERF_MASK, NULL);

        free(model_data);
        if (ret < 0) { 
            RCLCPP_ERROR(this->logger_, "rknn_init fail! ret=%d", ret); 
            return; 
        }

        rknn_core_mask core_mask = RKNN_NPU_CORE_ALL;
        ret = rknn_set_core_mask(rknn_ctx_, core_mask);
        if (ret < 0) { RCLCPP_ERROR(this->logger_, "rknn_set_core_mask fail!"); return; }

        // 3. 查询输入输出数量
        ret = rknn_query(rknn_ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
        if (ret < 0) { RCLCPP_ERROR(this->logger_, "rknn_query fail!"); return; }

        // 4. 查询输入输出属性
        input_attrs_.resize(io_num_.n_input);
        output_attrs_.resize(io_num_.n_output);
        for (uint32_t i = 0; i < io_num_.n_input; i++) {
            input_attrs_[i].index = i;
            rknn_query(rknn_ctx_, RKNN_QUERY_INPUT_ATTR, &(input_attrs_[i]), sizeof(rknn_tensor_attr));
        }
        for (uint32_t i = 0; i < io_num_.n_output; i++) {
            output_attrs_[i].index = i;
            rknn_query(rknn_ctx_, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs_[i]), sizeof(rknn_tensor_attr));
        }
        
        
        RCLCPP_INFO(this->logger_, "RKNN model initialized success");
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->logger_, "Failed to init model: %s", e.what());
    }
}

std::vector<PoseResult> Inferencer_RKNN::Infer(const cv::Mat& src){
    MEASURE_TIME();
    
    cv::Mat blob;
    this->src_cols = src.cols;
    this->src_rows = src.rows;
    PreProcess(src, blob);

    float* out_data = nullptr;
    int anchors = 0;
    int infos = 0;

    if(!rknn_ctx_) return {};
    
    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index = 0;
    inputs[0].type = RKNN_TENSOR_UINT8; 
    inputs[0].size = blob.total() * blob.elemSize(); // 640*640*3
    inputs[0].fmt = RKNN_TENSOR_NHWC; 
    inputs[0].buf = blob.data; // blob 现在是 RGB UINT8 NHWC

    rknn_inputs_set(rknn_ctx_, io_num_.n_input, inputs);
    rknn_run(rknn_ctx_, NULL);

    rknn_output outputs[io_num_.n_output];
    memset(outputs, 0, sizeof(outputs));
    for (uint32_t i = 0; i < io_num_.n_output; i++) {
        outputs[i].want_float = 1; 
    }
    rknn_outputs_get(rknn_ctx_, io_num_.n_output, outputs, NULL);

    out_data = (float*)outputs[0].buf;
    anchors = output_attrs_[0].dims[1]; 
    infos = output_attrs_[0].dims[2];


    //debug输出
    // static bool first_perf_print = true;
    // if (first_perf_print) {
    //     rknn_perf_detail perf_detail;
    //     int ret = rknn_query(rknn_ctx_, RKNN_QUERY_PERF_DETAIL, &perf_detail, sizeof(perf_detail));
    //     if (ret == 0) {
    //         printf("--- RKNN Performance Detail ---\n%s\n", perf_detail.perf_data);
    //         first_perf_print = false; // 只打印一次
    //     }
    // }
    // DetectorNode::NMS(candidates, 0.7, 0.5);
    std::vector<PoseResult> candidates = PostProcess(out_data, anchors, infos);
    rknn_outputs_release(rknn_ctx_, io_num_.n_output, outputs);



    return candidates;
}

void Inferencer_RKNN::PreProcess(const cv::Mat& src, cv::Mat& blob){
    // 1. 公共逻辑：计算缩放比例和 Padding (Letterbox)
    float r = std::min((float)model_params_.input_shape_width / src.size().width, (float)model_params_.input_shape_height / src.size().height);
    int new_unpad_w = round(src.cols * r);
    int new_unpad_h = round(src.rows * r);
    
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_unpad_w, new_unpad_h));

    this->dw_ = (model_params_.input_shape_width - new_unpad_w) / 2;
    this->dh_ = (model_params_.input_shape_height - new_unpad_h) / 2;

    cv::Mat img_pad;
    cv::copyMakeBorder(resized, img_pad, this->dh_, model_params_.input_shape_height - new_unpad_h - this->dh_,
                       this->dw_, model_params_.input_shape_width - new_unpad_w - this->dw_,
                       cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    this->ratio_ = r;

    // RKNN 需要: RGB, NHWC, UINT8, [0,255] 原始值
    // 直接在 img_pad 上进行通道转换并赋值给 blob
    cv::cvtColor(img_pad, blob, cv::COLOR_BGR2RGB);
}

std::vector<PoseResult> Inferencer_RKNN::PostProcess(
        float* out_data,
        int anchors,
        int infos
        ){
    //后处理部分 (逻辑通用)
    std::vector<PoseResult> candidates;
    for (int i = 0; i < anchors; i++) {
        int base_idx = i * infos;
        float score = out_data[base_idx + 4];

        if (score > this->model_params_.conf_threshold) {
            // 获取原始输出值（假设为 x1, y1, x2, y2）
            float x1_raw = out_data[base_idx + 0];
            float y1_raw = out_data[base_idx + 1];
            float x2_raw = out_data[base_idx + 2];
            float y3_raw = out_data[base_idx + 3];

            // 核心逻辑：按 xyxy 格式计算并缩放回原图
            int left   = int((x1_raw - this->dw_) / this->ratio_);
            int top    = int((y1_raw - this->dh_) / this->ratio_);
            int right  = int((x2_raw - this->dw_) / this->ratio_);
            int bottom = int((y3_raw - this->dh_) / this->ratio_);

            // 计算宽度和高度
            int width  = right - left;
            int height = bottom - top;

            // 越界保护
            left   = std::max(0, std::min(left, this->src_cols - 1));
            top    = std::max(0, std::min(top, this->src_rows - 1));
            width  = std::max(0, std::min(width, this->src_cols - left));
            height = std::max(0, std::min(height, this->src_rows - top));

            PoseResult res = {};
            res.box = cv::Rect(left, top, width, height);
            res.score = score;
            res.label = (int)out_data[base_idx + 5];

            // 关键点逻辑保持不变（关键点通常永远是绝对坐标点）
            for (int k = 0; k < 4; ++k) {
                KeyPoint kpt;
                // 关键点从索引 6 开始，每个点占 3 个值 (x, y, score)
                float kx = out_data[base_idx + 6 + k * 3];
                float ky = out_data[base_idx + 7 + k * 3];
                float ks = out_data[base_idx + 8 + k * 3];

                //减去偏移量，再除以缩放比例
                kpt.x = (kx - this->dw_) / this->ratio_;
                kpt.y = (ky - this->dh_) / this->ratio_;
                kpt.score = ks;

                res.kpts.push_back(kpt);
            }
            // -----------------------

            candidates.push_back(res);
        }
    }

    return candidates;
}