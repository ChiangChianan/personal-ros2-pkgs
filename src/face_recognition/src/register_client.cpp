#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "face_recognition/srv/register_face.hpp"
/**
 * @brief 人脸注册客户端节点类
 *
 * 该类封装了与 face_recognition/srv/RegisterFace 服务通信的逻辑，
 * 提供发送注册请求的方法，并处理请求的异步调用。
 */

class FaceRegisterClient : public rclcpp::Node {
 public:
  using RegisterFace = face_recognition::srv::RegisterFace;
  using RegisterFuture = rclcpp::Client<RegisterFace>::SharedFuture;

  FaceRegisterClient() : Node("register_client_node") {
    client_ = this->create_client<RegisterFace>("register_face");
  }

  /**
   * @brief 发送人脸注册请求（异步）
   *
   * @param img_path 包含人脸的图像文件路径
   * @param name     要注册的人名
   * @return RegisterFuture 可用于等待并获取服务响应的 future 对象
   *         如果服务不可用或图像读取失败，返回空的 future
   */
  RegisterFuture SendRegistrationRequest(const std::string& img_path,
                                         const std::string& name) {
    // 等待服务启动，超时5秒
    if (!client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Server not found!");
      return RegisterFuture();  // 返回空 future 表示失败
    }

    // 使用 OpenCV 读取图像文件
    cv::Mat img = cv::imread(img_path);
    if (img.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Could not read image!");
      return RegisterFuture();
    }

    // 创建服务请求对象
    auto request = std::make_shared<RegisterFace::Request>();
    request->name = name;  // 设置人名

    // 将 OpenCV 图像转换为 ROS 图像消息（sensor_msgs::msg::Image）
    // cv_bridge::CvImage 用于封装头信息、编码格式和图像数据
    // .toImageMsg() 将 CvImage 转换为共享指针指向的 sensor_msgs::msg::Image
    cv_bridge::CvImage img_bridge(std_msgs::msg::Header(), "bgr8", img);
    request->image = *(img_bridge.toImageMsg());

    RCLCPP_INFO(this->get_logger(), "Sending registration request for %s...",
                name.c_str());
    // 异步发送请求，返回 future 对象
    return client_->async_send_request(request).future.share();
  }

 private:
  rclcpp::Client<RegisterFace>::SharedPtr client_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  if (argc < 3) {
    std::cout
        << "Usage: ros2 run face_recognition register_client <img_path> <name>"
        << std::endl;
    return 1;
  }
  // 创建人脸注册客户端节点实例
  auto node = std::make_shared<FaceRegisterClient>();
  // 发送注册请求，获取 future 对象
  auto future_result = node->SendRegistrationRequest(argv[1], argv[2]);

  // 如果 future 有效（即请求已成功发送）
  if (future_result.valid()) {
    // 进入事件循环并等待服务响应，直到成功、失败或中断
    if (rclcpp::spin_until_future_complete(node, future_result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto res = future_result.get();
      if (res->success) {
        RCLCPP_INFO(node->get_logger(), "Success: %s", res->message.c_str());
      } else {
        RCLCPP_ERROR(node->get_logger(), "Failed: %s", res->message.c_str());
      }
    } else {
      RCLCPP_ERROR(node->get_logger(), "Service call failed or interrupted.");
    }
  }
  // 关闭 ROS2
  rclcpp::shutdown();
  return 0;
}