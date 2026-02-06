#include "detector/detector_node.hpp"
#include <memory>
#include <rclcpp/qos.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <filesystem>
#include <scope_timer.hpp>
#include <thread>
#include "cv_bridge/cv_bridge.hpp"
using namespace std;


DetectorNode::DetectorNode(const rclcpp::NodeOptions & options)
    : Node("detector_node", options) {
    this->declare_parameter("model_path", "model/best_n.rknn");
    this->declare_parameter("model_format", "rknn");
    this->declare_parameter("thread_num", 6);

    this->camera_matrix_ = [this]()->cv::Mat{
        auto para = this->declare_parameter("camera_matrix", std::vector<double>{0,0,0,0,0,0,0,0,0});
        cv::Mat mat = cv::Mat(3, 3, CV_64F, para.data()).clone();
        return mat;
    }();

    this->dist_coeffs_ = [this]()->cv::Mat{
        auto para = this->declare_parameter("dist_coeffs", std::vector<double>{0,0,0,0,0,0,0,0,0});
        cv::Mat mat = cv::Mat(5, 1, CV_64F, para.data()).clone();
        return mat;
    }();

    model_path_ = this->get_parameter("model_path").as_string();
    std::string model_full_path = PKG_SOURCE_DIR+model_path_.string();
    RCLCPP_INFO(this->get_logger(), "Loading model: %s", model_full_path.c_str());

    this->inferencer_pool_ = std::make_unique<InferencerPool
                                    <Inferencer_RKNN, 
                                    std::shared_ptr<cv::Mat>, 
                                    std::vector<PoseResult> >>
                                    (model_full_path,
                                    this->get_parameter("thread_num").as_int());

    if(this->inferencer_pool_->Init() != 0){
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize InferencerPool");
        return;
    }

    auto inferencers = this->inferencer_pool_->GetInferencers();
    inferencers[0]->SetNPUCore(RKNN_NPU_CORE_0);
    inferencers[1]->SetNPUCore(RKNN_NPU_CORE_1);
    inferencers[2]->SetNPUCore(RKNN_NPU_CORE_2);
    inferencers[3]->SetNPUCore(RKNN_NPU_CORE_ALL);
    inferencers[4]->SetNPUCore(RKNN_NPU_CORE_ALL);
    inferencers[5]->SetNPUCore(RKNN_NPU_CORE_ALL);
                                                                                                      

    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    auto sensor_qos = rclcpp::QoS(10);
    sensor_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image", 
        sensor_qos,
        std::bind(&DetectorNode::ImageCallback, this, std::placeholders::_1),
        sub_options);

    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("/detector/result", 10);


    this->main_loop_ = std::thread(&DetectorNode::MainLoop, this);

}

DetectorNode::~DetectorNode() {
    if (this->main_loop_.joinable()) {
        this->main_loop_.join();
    }
    this->inferencer_pool_.reset();
    
}

void DetectorNode::MainLoop() {
    static int frame_count = 0;
    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {

        //从线程池获取推理结果
        std::vector<PoseResult> results;
        if (this->inferencer_pool_->Get(results) != 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            // RCLCPP_ERROR(this->get_logger(), "Failed to get inference result");
            continue;
        }

        RCLCPP_INFO(this->get_logger(), "Got inference result with %zu detections", results.size());

        frame_count++;
        if(frame_count == 120){
            auto end_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            double fps = frame_count * 1000.0 / elapsed;
            RCLCPP_WARN(this->get_logger(), "Processing FPS: %.2f", fps);
            frame_count = 0;
            start_time = std::chrono::steady_clock::now();
        }
        

        if (results.empty()) {
            continue;
        }

        //绘制并发布
        DrawDetections(*(results[0].src), results);
        
        // 绘制检测结果
        cv::Mat draw_img = *results[0].src;
        DrawDetections(draw_img, results);

        // 发布结果图像
        cv_bridge::CvImage  cv_image = cv_bridge::CvImage();
        cv_image.encoding = "bgr8";
        cv_image.image = draw_img;

        auto output_msg = cv_image.toImageMsg();
        pub_image_->publish(*output_msg);
    }
}



void DetectorNode::DrawDetections(cv::Mat& image, const std::vector<PoseResult>& results) {
    // 定义不同关键点的颜色列表 (BGR 顺序)
    static const std::vector<cv::Scalar> kpt_colors = {
        cv::Scalar(255, 0, 0),   // 点 0: 蓝色
        cv::Scalar(0, 0, 255),   // 点 1: 红色
        cv::Scalar(0, 255, 255), // 点 2: 黄色
        cv::Scalar(255, 0, 255)  // 点 3: 紫色
    };

    // 简单绘制
    for(const auto& res : results){
        cv::rectangle(image, res.box, cv::Scalar(0, 255, 0), 2);
        
        // 改用索引遍历，以便区分不同的点
        for(size_t i = 0; i < res.kpts.size(); ++i){
            const auto& kp = res.kpts[i];
            if(kp.score > 0.5){
                // 根据索引选择颜色，如果点数超过预设颜色则默认绿色
                cv::Scalar color = (i < kpt_colors.size()) ? kpt_colors[i] : cv::Scalar(0, 255, 0);
                cv::circle(image, cv::Point((int)kp.x, (int)kp.y), 4, color, -1);
                
                // (可选) 在点旁边绘制索引号，方便确认点的顺序
                // cv::putText(frame, std::to_string(i), cv::Point((int)kp.x, (int)kp.y), 
                //             cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            }
        }
    }
}


void DetectorNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Received image");


    static int frame_count = 0;
    static auto start_time = std::chrono::steady_clock::now();
    frame_count++;
        if(frame_count == 120){
            auto end_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            double fps = frame_count * 1000.0 / elapsed;
            RCLCPP_WARN(this->get_logger(), "Receive FPS: %.2f", fps);
            frame_count = 0;
            start_time = std::chrono::steady_clock::now();
    }


    std::shared_ptr<cv::Mat> frame;
    try {
        frame = std::make_shared<cv::Mat>(cv_bridge::toCvCopy(msg)->image);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    if(frame->empty()) return;

    //图像放入线程池
    if(this->inferencer_pool_->Put(frame)!=0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to put image into inferencer pool");
        return;
    }

}


TechCorePose DetectorNode::ProcessPNP(std::vector<KeyPoint> kpts){
    if (kpts.size() < 4) {
        RCLCPP_DEBUG(this->get_logger(), "Not enough keypoints for PnP");
        return {};
    }

    // 提取 2D 点
    std::vector<cv::Point2f> image_points;
    for (const auto& kp : kpts) {
        image_points.emplace_back(kp.x, kp.y);
    }

    // 假设我们有对应的 3D 点
    std::vector<cv::Point3f> object_points = {
        {0, 0, 0}, {0, 1, 0}, {1, 1, 0}, {1, 0, 0}
    };

    cv::Mat rvec, tvec;
    cv::solvePnP(object_points, image_points, camera_matrix_, dist_coeffs_, rvec, tvec);

    TechCorePose pose;
    pose.x = tvec.at<double>(0);
    pose.y = tvec.at<double>(1);
    pose.z = tvec.at<double>(2);
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    double sy = sqrt(R.at<double>(0,0)*R.at<double>(0,0) + R.at<double>(1,0)*R.at<double>(1,0));
    bool singular = sy < 1e-6;
    pose.roll  = atan2(R.at<double>(2,1), R.at<double>(2,2));
    pose.pitch = singular ? atan2(-R.at<double>(2,0), sy)
                          : atan2(-R.at<double>(2,0), sy);
    pose.yaw   = atan2(R.at<double>(1,0), R.at<double>(0,0));

    return pose;
}

void DetectorNode::VisualizeTechCore(TechCorePose& pose){
    if (!techcore_marker_pub_) {
        return;
    }
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = this->now();
    marker.header.frame_id = techcore_marker_frame_;
    marker.ns = "techcore_pose";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = pose.x;
    marker.pose.position.y = pose.y;
    marker.pose.position.z = pose.z;
    tf2::Quaternion q;
    q.setRPY(pose.roll, pose.pitch, pose.yaw);
    marker.pose.orientation = tf2::toMsg(q);
    marker.scale.x = 0.3;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    techcore_marker_pub_->publish(marker);
}

RCLCPP_COMPONENTS_REGISTER_NODE(DetectorNode)
