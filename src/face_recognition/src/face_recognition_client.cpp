#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <face_recognition/srv/recognize_face.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;
using RecognizeFace = face_recognition::srv::RecognizeFace;

class FaceRecognitionClient : public rclcpp::Node {
 public:
  FaceRecognitionClient() : Node("face_recognition_client") {
    // 1. 声明并初始化参数
    this->declare_parameter<std::string>("input_source", "0");
    this->declare_parameter<int>("camera_width", 640);
    this->declare_parameter<int>("camera_height", 480);
    this->declare_parameter<double>("capture_interval", 0.03);  // 默认约30fps

    // 2. 预先创建 Service Client，避免重复创建开销
    client_ = this->create_client<RecognizeFace>("recognize_face");

    RCLCPP_INFO(this->get_logger(), "人脸识别客户端已启动");
  }

  void run() {
    std::string source = this->get_parameter("input_source").as_string();

    // 判断是图片路径还是摄像头索引
    if (source.find(".jpg") != std::string::npos ||
        source.find(".png") != std::string::npos) {
      processImage(source);
    } else {
      try {
        int cam_id = std::stoi(source);
        runCamera(cam_id);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "无效的输入源: %s", source.c_str());
      }
    }
  }

 private:
  rclcpp::Client<RecognizeFace>::SharedPtr client_;

  void processImage(const std::string& filepath) {
    cv::Mat img = cv::imread(filepath);
    if (img.empty()) {
      RCLCPP_ERROR(this->get_logger(), "无法读取图片: %s", filepath.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "正在处理图片: %s", filepath.c_str());
    cv::Mat result_img = callRecognizeService(img);

    cv::imshow("Recognition Result", result_img);
    RCLCPP_INFO(this->get_logger(), "按任意键退出...");
    cv::waitKey(0);
  }

  void runCamera(int cam_id) {
    cv::VideoCapture cap(cam_id);
    if (!cap.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开摄像头: %d", cam_id);
      return;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH,
            this->get_parameter("camera_width").as_int());
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,
            this->get_parameter("camera_height").as_int());

    double interval = this->get_parameter("capture_interval").as_double();
    rclcpp::Rate rate(1.0 / interval);

    RCLCPP_INFO(this->get_logger(),
                "摄像头已就绪。操作提示: [空格]: 识别当前帧, [q]: 退出");

    while (rclcpp::ok()) {
      cv::Mat frame;
      cap >> frame;
      if (frame.empty()) break;

      cv::imshow("Camera Feed", frame);

      char key = (char)cv::waitKey(1);
      if (key == 'q' || key == 27) {  // q 或 ESC 退出
        break;
      } else if (key == ' ') {
        RCLCPP_INFO(this->get_logger(), "正在请求识别...");
        cv::Mat res = callRecognizeService(frame);
        cv::imshow("Recognition Result", res);  // 在独立窗口显示结果
      }

      rate.sleep();
    }
  }

  cv::Mat callRecognizeService(const cv::Mat& img_in) {
    // 等待服务可用
    if (!client_->wait_for_service(2s)) {
      RCLCPP_ERROR(this->get_logger(), "服务未就绪，请检查服务端是否启动。");
      return img_in;
    }

    // 准备请求
    auto request = std::make_shared<RecognizeFace::Request>();
    auto header = std_msgs::msg::Header();
    header.stamp = this->now();
    header.frame_id = "camera_frame";

    cv_bridge::CvImage img_bridge(header, "bgr8", img_in);
    img_bridge.toImageMsg(request->image);

    // 发送异步请求并阻塞等待结果（在简单 Client 节点中可以接受）
    auto result_future = client_->async_send_request(request);

    // 使用 spin_until_future_complete 以便在等待时处理 ROS 内部事务
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto response = result_future.get();
      cv::Mat display_img = img_in.clone();

      if (response->authorized) {
        RCLCPP_INFO(this->get_logger(), "成功识别: %s", response->name.c_str());
        drawOverlay(display_img, "User: " + response->name,
                    cv::Scalar(0, 255, 0));
      } else {
        RCLCPP_WARN(this->get_logger(), "识别失败: 未授权用户");
        drawOverlay(display_img, "Unauthorized", cv::Scalar(0, 0, 255));
      }
      return display_img;
    } else {
      RCLCPP_ERROR(this->get_logger(), "服务调用失败");
      return img_in;
    }
  }

  void drawOverlay(cv::Mat& img, const std::string& text,
                   const cv::Scalar& color) {
    cv::putText(img, text, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                color, 2);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FaceRecognitionClient>();

  node->run();

  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}