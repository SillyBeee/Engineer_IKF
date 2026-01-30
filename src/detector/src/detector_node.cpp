#include "detector/detector_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <algorithm>
#include <filesystem>
using namespace std;

DetectorNode::DetectorNode(const rclcpp::NodeOptions & options)
    : Node("detector_node", options) {
    
    this->declare_parameter("model_path", "model/best.onnx");
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
            
            input_names_.push_back(name_str);
        }

        size_t num_output_nodes = session_.GetOutputCount();
        for (size_t i = 0; i < num_output_nodes; i++) {
            auto output_name = session_.GetOutputNameAllocated(i, allocator);
            
            // 同样的修复逻辑
            const char* raw_name = output_name.get();
            char* name_str = new char[strlen(raw_name) + 1];
            strcpy(name_str, raw_name);
            
            output_names_.push_back(name_str);
        }
        
    } catch (const exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to init ONNX model: %s", e.what());
    }
    RCLCPP_INFO(this->get_logger(), "ONNX model initialized success");
}

void DetectorNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received image");
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
    float r = min((float)kInputW / src.cols, (float)kInputH / src.rows);
    int new_unpad_w = round(src.cols * r);
    int new_unpad_h = round(src.rows * r);
    
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(new_unpad_w, new_unpad_h));

    // 计算 Padding
    dw = (kInputW - new_unpad_w) / 2;
    dh = (kInputH - new_unpad_h) / 2;
    
    // 加边框
    cv::copyMakeBorder(resized, blob, dh, kInputH - new_unpad_h - dh, dw, kInputW - new_unpad_w - dw, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
    
    // HWC -> CHW, BGR -> RGB, /255.0
    cv::dnn::blobFromImage(blob, blob, 1.0/255.0, cv::Size(), cv::Scalar(), true, false);
    ratio = r;
}

vector<PoseResult> DetectorNode::Infer(const cv::Mat& src) {
    if(!session_) return {};

    cv::Mat blob;
    float ratio; 
    int dw, dh;
    PreProcess(src, blob, ratio, dw, dh);

    int64_t input_dims[] = {1, 3, kInputH, kInputW};
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    
    // cv::dnn::blobFromImage 结果是连续的浮点数，可以直接使用
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, blob.ptr<float>(), blob.total(), input_dims, 4);

    auto output_tensors = session_.Run(Ort::RunOptions{nullptr}, 
                                       input_names_.data(), &input_tensor, 1, 
                                       output_names_.data(), output_names_.size());

    // 解析输出: YOLOv8 Pose 输出一般是 [1, 56, 8400] (class + box + kpts)
    // 56 = 4(box) + 1(score) + 17*3(kpts)
    float* out_data = output_tensors[0].GetTensorMutableData<float>();
    auto tensor_info = output_tensors[0].GetTensorTypeAndShapeInfo();
    auto shape = tensor_info.GetShape();
    
    // 注意：YOLOv8 输出通常是 [Batch, Channels, Anchors]，需要转置解析
    int channels = shape[1]; // 56
    int anchors = shape[2];  // 8400

    vector<PoseResult> candidates;
    // 遍历每一个 anchor
    for (int i = 0; i < anchors; i++) {
        // 获取置信度 (score 通常在索引 4)
        // [cx, cy, w, h, score, kpt1_x, kpt1_y, kpt1_s, ...]
        // 这一步取决于你的模型输出结构，YOLOv8 可能是 score 在 box 后面
        float score = out_data[4 * anchors + i]; 
        
        if (score > 0.45) { // Confidence Threshold
            float cx = out_data[0 * anchors + i];
            float cy = out_data[1 * anchors + i];
            float w  = out_data[2 * anchors + i];
            float h  = out_data[3 * anchors + i];

            // 还原到 letterbox 下的坐标
            int left = int((cx - 0.5 * w) - dw) / ratio;
            int top = int((cy - 0.5 * h) - dh) / ratio;
            int width = int(w) / ratio;
            int height = int(h) / ratio;

            PoseResult res;
            res.box = cv::Rect(left, top, width, height);
            res.score = score;
            
            // 解析关键点
            for(int k=0; k<kNumKpts; ++k){
                // 关键点数据在 score 之后，每个点 3 个值 (x, y, conf)
                int kpt_idx_base = (5 + k * 3) * anchors + i;
                float kx = (out_data[kpt_idx_base] - dw) / ratio;
                float ky = (out_data[kpt_idx_base + 1] - dh) / ratio;
                float ks = out_data[kpt_idx_base + 2];
                res.kpts.push_back({kx, ky, ks});
            }
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