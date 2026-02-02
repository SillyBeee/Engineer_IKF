#include "detector/detector_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <algorithm>
#include <filesystem>
#include <scope_timer.hpp>
using namespace std;


DetectorNode::DetectorNode(const rclcpp::NodeOptions & options)
    : Node("detector_node", options) {
    #ifdef ONNX_MODE
    this->declare_parameter("model_path", "model/best_s.onnx");
    #endif //ONNX_MODE

    #ifdef RKNN_MODE
    this->declare_parameter("model_path", "model/best_n.rknn");
    #endif //RKNN_MODE

    string model_path = this->get_parameter("model_path").as_string();

    RCLCPP_INFO(this->get_logger(), "Loading model: %s", model_path.c_str());
    InitModel(model_path);

    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image", 10,
        std::bind(&DetectorNode::ImageCallback, this, std::placeholders::_1));

    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("/detector/result", 10);
}

DetectorNode::~DetectorNode() {
    // Session 会自动释放，这里清空名称指针
    #ifdef ONNX_MODE
    for(auto ptr : input_names_) delete[] ptr;
    for(auto ptr : output_names_) delete[] ptr;
    #endif //ONNX_MODE
}

void DetectorNode::InitModel(const string& model_path) {
    try {
        #ifdef ONNX_MODE
        env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "YoloPose");
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(4);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

        // 如果需要GPU加速（前提是安装了 CUDA 版 ORT）：
        // OrtSessionOptionsAppendExecutionProvider_CUDA(session_options, 0);
        std::filesystem::path pkg_path= std::filesystem::path(PKG_SOURCE_DIR); 
        session_ = Ort::Session(env_, (pkg_path / model_path).c_str(), session_options);
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
            
            // 同样的修复逻辑
            const char* raw_name = output_name.get();
            char* name_str = new char[strlen(raw_name) + 1];
            strcpy(name_str, raw_name);
            RCLCPP_INFO(this->get_logger(), "Output name[%zu]: %s", i, name_str);
            
            output_names_.push_back(name_str);
        }
        #endif //ONNX_MODE

        #ifdef RKNN_MODE
        // 1. 读取模型文件
        std::filesystem::path pkg_path= std::filesystem::path(PKG_SOURCE_DIR); 
        std::string model_path_full = (pkg_path / model_path).string();
        FILE *fp = fopen(model_path_full.c_str(), "rb");
        if(fp == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open model file: %s", model_path.c_str());
            return;
        }
        fseek(fp, 0, SEEK_END);
        int model_len = ftell(fp);
        void *model_data = malloc(model_len);
        fseek(fp, 0, SEEK_SET);
        if(fread(model_data, 1, model_len, fp) != (size_t)model_len) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read model data");
            free(model_data);
            fclose(fp);
            return;
        }
        fclose(fp);

        // 2. 初始化 RKNN 上下文
        int ret = rknn_init(&rknn_ctx_, model_data, model_len, 0, NULL);
        free(model_data);
        if (ret < 0) { 
            RCLCPP_ERROR(this->get_logger(), "rknn_init fail! ret=%d", ret); 
            return; 
        }

        // 3. 查询输入输出数量
        ret = rknn_query(rknn_ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
        if (ret < 0) { RCLCPP_ERROR(this->get_logger(), "rknn_query fail!"); return; }

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
        RCLCPP_INFO(this->get_logger(), "RKNN model initialized success");
        #endif 
    } catch (const exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to init model: %s", e.what());
    }
    RCLCPP_INFO(this->get_logger(), "model initialized success");
}

void DetectorNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Received image");
    cv::Mat frame;
    try {
        frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    if(frame.empty()) return;

    auto results = Infer(frame);
    RCLCPP_INFO(this->get_logger(), "Detect %zu target", results.size());
    
    // 定义不同关键点的颜色列表 (BGR 顺序)
    static const std::vector<cv::Scalar> kpt_colors = {
        cv::Scalar(255, 0, 0),   // 点 0: 蓝色
        cv::Scalar(0, 0, 255),   // 点 1: 红色
        cv::Scalar(0, 255, 255), // 点 2: 黄色
        cv::Scalar(255, 0, 255)  // 点 3: 紫色
    };

    // 简单绘制
    for(const auto& res : results){
        cv::rectangle(frame, res.box, cv::Scalar(0, 255, 0), 2);
        
        // 改用索引遍历，以便区分不同的点
        for(size_t i = 0; i < res.kpts.size(); ++i){
            const auto& kp = res.kpts[i];
            if(kp.score > 0.5){
                // 根据索引选择颜色，如果点数超过预设颜色则默认绿色
                cv::Scalar color = (i < kpt_colors.size()) ? kpt_colors[i] : cv::Scalar(0, 255, 0);
                cv::circle(frame, cv::Point((int)kp.x, (int)kp.y), 4, color, -1);
                
                // (可选) 在点旁边绘制索引号，方便确认点的顺序
                // cv::putText(frame, std::to_string(i), cv::Point((int)kp.x, (int)kp.y), 
                //             cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            }
        }
    }

    sensor_msgs::msg::Image::SharedPtr out_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
    pub_image_->publish(*out_msg);
}

void DetectorNode::PreProcess(const cv::Mat& src, cv::Mat& blob, float& ratio, int& dw, int& dh) {
    // 1. 公共逻辑：计算缩放比例和 Padding (Letterbox)
    float r = std::min((float)input_shape_width / src.size().width, (float)input_shape_height / src.size().height);
    int new_unpad_w = round(src.cols * r);
    int new_unpad_h = round(src.rows * r);
    
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_unpad_w, new_unpad_h));

    dw = (input_shape_width - new_unpad_w) / 2;
    dh = (input_shape_height - new_unpad_h) / 2;

    cv::Mat img_pad;
    cv::copyMakeBorder(resized, img_pad, dh, input_shape_height - new_unpad_h - dh, 
                       dw, input_shape_width - new_unpad_w - dw, 
                       cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    ratio = r;

    // 2. 分模式处理数据格式
    #ifdef ONNX_MODE
    // ONNX 需要: RGB, NCHW, FP32, [0,1] 归一化
    cv::dnn::blobFromImage(img_pad, blob, 1.0/255.0, cv::Size(), cv::Scalar(), true, false);
    #endif

    #ifdef RKNN_MODE
    // RKNN 需要: RGB, NHWC, UINT8, [0,255] 原始值
    // 直接在 img_pad 上进行通道转换并赋值给 blob
    cv::cvtColor(img_pad, blob, cv::COLOR_BGR2RGB);
    #endif
}

vector<PoseResult> DetectorNode::Infer(const cv::Mat& src) {
    MEASURE_TIME();
    
    cv::Mat blob;
    float ratio; 
    int dw, dh;
    PreProcess(src, blob, ratio, dw, dh);

    float* out_data = nullptr;
    int anchors = 0;
    int infos = 0;

    #ifdef ONNX_MODE
    if(!session_) return {};
    int64_t input_dims[] = {1, 3, this->input_shape_height, this->input_shape_width};
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    // blob 现在是 NCHW FP32
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, blob.ptr<float>(), blob.total(), input_dims, 4);

    auto output_tensors = session_.Run(Ort::RunOptions{nullptr}, 
                                       input_names_.data(), &input_tensor, 1, 
                                       output_names_.data(), output_names_.size());

    out_data = output_tensors[0].GetTensorMutableData<float>();
    auto shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
    anchors = shape[1];
    infos = shape[2];
    #endif 

    #ifdef RKNN_MODE
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
    #endif 
    
    //后处理部分 (逻辑通用)
    vector<PoseResult> candidates;
    for (int i = 0; i < anchors; i++) {
        int base_idx = i * infos;
        float score = out_data[base_idx + 4];

        if (score > this->conf_threshold) {
            // 获取原始输出值（假设为 x1, y1, x2, y2）
            float x1_raw = out_data[base_idx + 0];
            float y1_raw = out_data[base_idx + 1];
            float x2_raw = out_data[base_idx + 2];
            float y3_raw = out_data[base_idx + 3];

            // 核心逻辑：按 xyxy 格式计算并缩放回原图
            int left   = int((x1_raw - dw) / ratio);
            int top    = int((y1_raw - dh) / ratio);
            int right  = int((x2_raw - dw) / ratio);
            int bottom = int((y3_raw - dh) / ratio);

            // 计算宽度和高度
            int width  = right - left;
            int height = bottom - top;

            // 越界保护
            left   = std::max(0, std::min(left, src.cols - 1));
            top    = std::max(0, std::min(top, src.rows - 1));
            width  = std::max(0, std::min(width, src.cols - left));
            height = std::max(0, std::min(height, src.rows - top));

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

                // 核心修复：减去偏移量，再除以缩放比例
                kpt.x = (kx - dw) / ratio;
                kpt.y = (ky - dh) / ratio;
                kpt.score = ks;

                res.kpts.push_back(kpt);
            }
            // -----------------------

            candidates.push_back(res);
        }
    }
    
    #ifdef RKNN_MODE
    rknn_outputs_release(rknn_ctx_, io_num_.n_output, outputs);
    #endif

    return candidates;
}

void DetectorNode::NMS(vector<PoseResult>& results, float conf_thres, float iou_thres) {
    //YOLO26消除了NMS,不需要NMS后处理了
    vector<int> indices;
    vector<cv::Rect> boxes;
    vector<float> scores;
    for(const auto& r : results){
        boxes.push_back(r.box);
        scores.push_back(r.score);
    }
    
    cv::dnn::NMSBoxes(boxes, scores, conf_thres, iou_thres, indices);
    
    vector<PoseResult> nms_results;
    for(int idx : indices){
        nms_results.push_back(results[idx]);
    }
    results = nms_results;
}

RCLCPP_COMPONENTS_REGISTER_NODE(DetectorNode)