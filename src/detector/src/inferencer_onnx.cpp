#include "inferencer_onnx.hpp"
#include "scope_timer.hpp"
Inferencer_ONNX::Inferencer_ONNX(const std::string &model_path,ModelParams model_params) {
    this->model_params_ = model_params;
    this->model_path_ = model_path;
    InitModel(model_path);
}

Inferencer_ONNX::~Inferencer_ONNX() {
    // Session 会自动释放，这里清空名称指针
    for(auto ptr : input_names_) delete[] ptr;
    for(auto ptr : output_names_) delete[] ptr;
}

void Inferencer_ONNX::InitModel(const std::string &model_path) {
    try{
        env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "YoloPose");
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(4);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

        // OrtSessionOptionsAppendExecutionProvider_CUDA(session_options, 0);
        session_ = Ort::Session(env_, model_path.c_str(), session_options);
        allocator_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

        // 获取输入输出层名称
        Ort::AllocatorWithDefaultOptions allocator;
        size_t num_input_nodes = session_.GetInputCount();
        for (size_t i = 0; i < num_input_nodes; i++) {
            auto input_name = session_.GetInputNameAllocated(i, allocator);
            
            // 修复点 1: 先获取原始指针
            const char* raw_name = input_name.get();
            
            // 修复点 2: 使用 strlen 获取长度，而不是 .length()
            char* name_str = new char[strlen(raw_name) + 1];
            
            // 修复点 3: 直接复制，不需要 .c_str()
            strcpy(name_str, raw_name);
            RCLCPP_INFO(this->get_logger(), "Input name[%zu]: %s", i, name_str);
            input_names_.push_back(name_str);
        }

        size_t num_output_nodes = session_.GetOutputCount();
        for (size_t i = 0; i < num_output_nodes; i++) {
            auto output_name = session_.GetOutputNameAllocated(i, allocator);
            const char* raw_name = output_name.get();
            char* name_str = new char[strlen(raw_name) + 1];
            strcpy(name_str, raw_name);
            RCLCPP_INFO(this->get_logger(), "Output name[%zu]: %s", i, name_str);
            
            output_names_.push_back(name_str);
        }
    }
    catch(const std::exception& e){
        RCLCPP_ERROR(this->get_logger(), "Failed to init model: %s", e.what());
    }
    
}

std::vector<PoseResult> Inferencer_ONNX::Infer(std::shared_ptr<cv::Mat> src) {
    MEASURE_TIME();

    this->src_cols = src->cols;
    this->src_rows = src->rows;
    std::shared_ptr<cv::Mat> blob = std::make_shared<cv::Mat>();   
    PreProcess(src, blob);

    float* out_data = nullptr;
    int anchors = 0;
    int infos = 0;

    if(!session_) return {};
    int64_t input_dims[] = {1, 3, this->model_params_.input_shape_height, this->model_params_.input_shape_width};
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    // blob 现在是 NCHW FP32
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, blob->ptr<float>(), blob->total(), input_dims, 4);

    auto output_tensors = session_.Run(Ort::RunOptions{nullptr}, 
                                       input_names_.data(), &input_tensor, 1, 
                                       output_names_.data(), output_names_.size());

    out_data = output_tensors[0].GetTensorMutableData<float>();
    auto shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
    anchors = shape[1];
    infos = shape[2];

    auto candidates = PostProcess(out_data, anchors, infos);
    for (auto& c : candidates) {
        c.src = src;
    }

    return candidates;
}

void Inferencer_ONNX::PreProcess(std::shared_ptr<cv::Mat> src, std::shared_ptr<cv::Mat> blob) {
    // 1. 公共逻辑：计算缩放比例和 Padding (Letterbox)
    float r = std::min((float)model_params_.input_shape_width / src->size().width, (float)model_params_.input_shape_height / src->size().height);
    int new_unpad_w = round(src->cols * r);
    int new_unpad_h = round(src->rows * r);
    
    cv::Mat resized;
    cv::resize(*src, resized, cv::Size(new_unpad_w, new_unpad_h));

    this->dw_ = (model_params_.input_shape_width - new_unpad_w) / 2;
    this->dh_ = (model_params_.input_shape_height - new_unpad_h) / 2;

    cv::Mat img_pad;
    cv::copyMakeBorder(resized, img_pad, dh_, model_params_.input_shape_height - new_unpad_h - dh_, 
                       dw_, model_params_.input_shape_width - new_unpad_w - dw_, 
                       cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    this->ratio_ = r;

    // ONNX 需要: RGB, NCHW, FP32, [0,1] 归一化
    cv::dnn::blobFromImage(img_pad, *blob, 1.0/255.0, cv::Size(), cv::Scalar(), true, false);
}

std::vector<PoseResult> Inferencer_ONNX::PostProcess(
    float *out_data,
    int anchors, 
    int infos) 
{
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



void NMS(std::vector<PoseResult>& results, float conf_thres, float iou_thres) {
    //YOLO26消除了NMS,不需要NMS后处理了
    std::vector<int> indices;
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    for(const auto& r : results){
        boxes.push_back(r.box);
        scores.push_back(r.score);
    }
    
    cv::dnn::NMSBoxes(boxes, scores, conf_thres, iou_thres, indices);
    
    std::vector<PoseResult> nms_results;
    for(int idx : indices){
        nms_results.push_back(results[idx]);
    }
    results = nms_results;
}