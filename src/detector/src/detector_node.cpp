#include "detector/detector_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <algorithm>
#include <filesystem>
#include <scope_timer.hpp>
using namespace std;

DetectorNode::DetectorNode(const rclcpp::NodeOptions & options)
    : Node("detector_node", options) {
    
    this->declare_parameter("model_path", "model/best_s.onnx");
    string model_path = this->get_parameter("model_path").as_string();

    RCLCPP_INFO(this->get_logger(), "Loading ONNX model: %s", model_path.c_str());
    InitModel(model_path);

    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image", 10,
        std::bind(&DetectorNode::ImageCallback, this, std::placeholders::_1));

    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("/detector/result", 10);
}

DetectorNode::~DetectorNode() {
    // Session 会自动释放，这里清空名称指针
    for(auto ptr : input_names_) delete[] ptr;
    for(auto ptr : output_names_) delete[] ptr;
}

void DetectorNode::InitModel(const string& model_path) {
    try {
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
        
    } catch (const exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to init ONNX model: %s", e.what());
    }
    RCLCPP_INFO(this->get_logger(), "ONNX model initialized success");
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
    
    // 简单绘制
    for(const auto& res : results){
        cv::rectangle(frame, res.box, cv::Scalar(0, 255, 0), 2);
        for(const auto& kp : res.kpts){
            if(kp.score > 0.5){
                cv::circle(frame, cv::Point((int)kp.x, (int)kp.y), 3, cv::Scalar(0, 0, 255), -1);
            }
        }
    }

    sensor_msgs::msg::Image::SharedPtr out_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
    pub_image_->publish(*out_msg);
}

void DetectorNode::PreProcess(const cv::Mat& src, cv::Mat& blob, float& ratio, int& dw, int& dh) {
    // Letterbox: 保持长宽比缩放
    float r = std::min((float)input_shape_width / src.size().width, (float)input_shape_height / src.size().height);
    int new_unpad_w = round(src.cols * r);
    int new_unpad_h = round(src.rows * r);
    
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_unpad_w, new_unpad_h));

    // 计算 Padding
    dw = (input_shape_width - new_unpad_w) / 2;
    dh = (input_shape_height - new_unpad_h) / 2;

    // 加边框
    cv::copyMakeBorder(resized, blob, dh, input_shape_height - new_unpad_h - dh, dw, input_shape_width - new_unpad_w - dw, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

    // HWC -> CHW, BGR -> RGB, /255.0
    cv::dnn::blobFromImage(blob, blob, 1.0/255.0, cv::Size(), cv::Scalar(), true, false);
    ratio = r;
}

vector<PoseResult> DetectorNode::Infer(const cv::Mat& src) {
    MEASURE_TIME();
    if(!session_) return {};

    cv::Mat blob;
    float ratio; 
    int dw, dh;
    PreProcess(src, blob, ratio, dw, dh);

    int64_t input_dims[] = {1, 3, this->input_shape_height, this->input_shape_width};
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    
    // cv::dnn::blobFromImage 结果是连续的浮点数，可以直接使用
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, blob.ptr<float>(), blob.total(), input_dims, 4);

    auto output_tensors = session_.Run(Ort::RunOptions{nullptr}, 
                                       input_names_.data(), &input_tensor, 1, 
                                       output_names_.data(), output_names_.size());

    // YOLO26 Pose 输出是 [1, 300, 18] (channel + candidates + infos)
    // 18(infos) = 4(box) + 1(confidence) + 1(class) + 4*3(kpts)
    float* out_data = output_tensors[0].GetTensorMutableData<float>();
    auto tensor_info = output_tensors[0].GetTensorTypeAndShapeInfo();
    auto shape = tensor_info.GetShape();
    // RCLCPP_INFO(this->get_logger(),"shape_size:%ld", shape.size());
    // RCLCPP_INFO(this->get_logger(), "Output shape: [%ld, %ld, %ld]", shape[0], shape[1], shape[2]);
    
    int anchors = shape[1]; // 300
    int infos = shape[2];  // 18

    vector<PoseResult> candidates;
    // 遍历每一个 anchor
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
    
    NMS(candidates, 0.45, 0.5);
    return candidates;
}

void DetectorNode::NMS(vector<PoseResult>& results, float conf_thres, float iou_thres) {
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