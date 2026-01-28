#include "interfaces/srv/pixel_to_base.hpp"
#include "libobsensor/ObSensor.hpp"
#include "libobsensor/hpp/Frame.hpp"
#include "libobsensor/hpp/Utils.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "utils.hpp"
#include "utils_opencv.hpp"
#include <libobsensor/ObSensor.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
// #include <libobsensor/hpp/Utils.hpp>
class CameraNode: public rclcpp::Node {
public:
    explicit CameraNode(const rclcpp::NodeOptions& options);

    ~CameraNode();

    /// @brief 图片发布线程
    void LoopForPublish();

    /// @brief 相机信息发布线程
    void LoopForCameraInfo();


    /// @brief 彩色图发布函数
    /// @param image 传入图片
    void PublishImage(cv::Mat& image);


    /// @brief 深度图发布函数
    /// @param image 传入图片
    void PublishDepth(cv::Mat& image);


    /// @brief 像素点解算回调
    void PixelToBaseCallback(
        const std::shared_ptr<interfaces::srv::PixelToBase::Request> request,
        std::shared_ptr<interfaces::srv::PixelToBase::Response> response
    );


    /// @brief 获取深度图像素点的深度值
    /// @param x x轴坐标
    /// @param y y轴坐标
    /// @return 深度数值
    double GetPixelDepth(int x, int y);


    /// @brief 填补深度图空洞
    /// @param depth 深度图
    /// @param maxDepth 计入填补中值的最大深度值
    /// @param minDepth 计入填补中值的最小深度值
    void FillDepthHoles(cv::Mat& depth, int maxDepth, int minDepth ,int kernel);

    //保存一张image至本地
    void SaveImage(cv::Mat& image);

    /// @brief 将一个ob::ColorFrame彩色图转换为cv::Mat的妙妙小工具
    cv::Mat ConvertColor2Mat(std::shared_ptr<const ob::ColorFrame> frame) {
        if (frame == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Given Color frame is nullptr");
            return cv::Mat();
        }
        cv::Mat rstMat;
        if (frame->getFormat() == OB_FORMAT_MJPG) {
            cv::Mat rawMat(1, frame->getDataSize(), CV_8UC1, frame->getData());
            rstMat = cv::imdecode(rawMat, 1);
        } else if (frame->getFormat() == OB_FORMAT_RGB) {
            cv::Mat rawMat(frame->getHeight(), frame->getWidth(), CV_8UC3, frame->getData());
            cv::cvtColor(rawMat, rstMat, cv::COLOR_RGB2BGR);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Given frame is not a MJPG or RGB frame");
        }
        return rstMat;
    }

    /// @brief 将一个ob::DepthFrame深度图转换为cv::Mat的妙妙小工具
    cv::Mat ConvertDepth2Mat(std::shared_ptr<const ob::DepthFrame> frame) {
        if (frame == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Given Depth frame is nullptr");
            return cv::Mat();
        }
        cv::Mat rstMat;
        if (frame->getType() == OB_FRAME_DEPTH) {
            if (frame->getFormat() == OB_FORMAT_Y16 || frame->getFormat() == OB_FORMAT_Z16) {
                cv::Mat rawMat = cv::Mat(frame->getHeight(), frame->getWidth(), CV_16UC1, frame->getData());
                // rstMat = rawMat;
                //depth frame pixel value multiply scale to get distance in millimeter
                float scale = frame->as<ob::DepthFrame>()->getValueScale();

                cv::Mat cvtMat;
                // normalization to 0-255. 0.032f is 256/8000, to limit the range of depth to 8000mm
                rawMat.convertTo(cvtMat, CV_32F, scale * 0.032f);

                // apply gamma correction to enhance the contrast for near objects
                cv::pow(cvtMat, 0.6f, cvtMat);

                //  convert to 8-bit
                cvtMat.convertTo(cvtMat, CV_8UC1, 10); // multiplier 10 is to normalize to 0-255 (nearly) after applying gamma correction

                // apply colormap
                cv::applyColorMap(cvtMat, rstMat, cv::COLORMAP_JET);
            }
        };
        return rstMat;
    }

private:
    std::shared_ptr<ob::Pipeline> pipe_;

    std::shared_ptr<ob::Config> config_;

    std::shared_ptr<ob::Align> color2depthAlign_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub;

    rclcpp::Service<interfaces::srv::PixelToBase>::SharedPtr pixel2base_srv;

    tf2_ros::Buffer tf_buffer_;

    tf2_ros::TransformListener tf_listener_;

    std::thread PublishThread_;

    std::thread CameraInfoThread_;

    std::mutex framesetMutex;
    std::shared_ptr<ob::FrameSet> frameset = nullptr;

    std::shared_ptr<ob::FrameSet> align_frameset = nullptr;

    //服务用
    int srv_flag = 0; //用于判断是否第一次调用服务，如过是第一次，则保存该帧图片
    // std::shared_ptr<ob::FrameSet> frameset_for_srv = nullptr; //用于保存第一次调用服务时的帧图片
    std::shared_ptr<ob::DepthFrame> depth_for_srv = nullptr; //用于保存第一次调用服务时的深度图
    cv::Mat color_for_srv;                                   //用于保存第一次调用服务时的深度图
    geometry_msgs::msg::TransformStamped transform_for_srv;  //用于保存第一次调用服务时的变换矩阵
    double dis_for_srv = 0.0; //用于保存第一次调用服务时的深度值;

    // 几个坐标系的名称
    std::string base_coordinate,
        camera_coordinate,
        sucker_coordinate,
        tool_coordinate;
};
