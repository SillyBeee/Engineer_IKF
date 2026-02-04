#include "detector/detector_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <filesystem>
#include <scope_timer.hpp>
using namespace std;


DetectorNode::DetectorNode(const rclcpp::NodeOptions & options)
    : Node("detector_node", options) {
    this->declare_parameter("model_path", "model/best_prune.rknn");
    this->declare_parameter("model_format", "rknn");

    model_path_ = this->get_parameter("model_path").as_string();
    std::string model_full_path = PKG_SOURCE_DIR+model_path_.string();
    RCLCPP_INFO(this->get_logger(), "Loading model: %s", model_full_path.c_str());

    this->inferencer_rknn_ = std::make_unique<Inferencer_RKNN>(model_full_path, model_params_);

    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image", 10,
        std::bind(&DetectorNode::ImageCallback, this, std::placeholders::_1));
    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("/detector/result", 10);
}

DetectorNode::~DetectorNode() {
    this->inferencer_rknn_.reset();
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

    auto results = this->inferencer_rknn_->Infer(frame);
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




RCLCPP_COMPONENTS_REGISTER_NODE(DetectorNode)