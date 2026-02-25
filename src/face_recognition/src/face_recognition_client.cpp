#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "face_recognition/srv/recognize_face.hpp"

using namespace std::chrono_literals;

/**
 * @brief 人脸识别客户端节点类
 *
 * 该类继承自 rclcpp::Node，负责从摄像头或图像文件获取图像，
 * 调用远程人脸识别服务，并在图像上显示识别结果。
 */
class FaceRecognitionClient : public rclcpp::Node {
 public:
  using RecognizeFace = face_recognition::srv::RecognizeFace;

  /**
   * @brief 构造函数：初始化节点，声明参数，创建服务客户端
   */
  FaceRecognitionClient() : Node("face_recognition_client") {
    // 1. 声明并初始化节点参数（可在运行时通过 launch 文件或命令行修改）
    this->declare_parameter("input_source",
                            "0");  // 输入源：摄像头索引或图片路径
    this->declare_parameter("camera_width", 640);   // 摄像头采集宽度
    this->declare_parameter("camera_height", 480);  //  摄像头采集高度
    this->declare_parameter("capture_interval",
                            0.03);  // 图像采集间隔（秒），默认约 30fps

    // 2. 创建服务客户端，用于调用 "recognize_face" 服务
    //    这里预先创建，避免每次请求时重复创建客户端
    client_ = this->create_client<RecognizeFace>("recognize_face");
    RCLCPP_INFO(this->get_logger(), "人脸识别客户端已启动");
  }

  /**
   * @brief 运行主逻辑：根据输入源类型选择处理方式
   */
  void Run() {
    std::string source = this->get_parameter("input_source").as_string();
    // 判断输入源是图片文件还是摄像头索引
    if (source.find(".jpg") != std::string::npos ||
        source.find(".png") != std::string::npos) {
      ProcessImage(source);  // 处理单张图片
    } else {
      try {
        int cam_id = std::stoi(source);
        RunCamera(cam_id);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "无效的输入源: %s", source.c_str());
      }
    }
  }

 private:
  // 服务客户端指针，用于发送请求
  rclcpp::Client<RecognizeFace>::SharedPtr client_;

  /**
   * @brief 处理单张图片文件
   * @param filepath 图片文件路径
   */
  void ProcessImage(const std::string& filepath) {
    cv::Mat img = cv::imread(filepath);
    if (img.empty()) {
      RCLCPP_ERROR(this->get_logger(), "无法读取图片: %s", filepath.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "正在处理图片: %s", filepath.c_str());
    cv::Mat result_img = CallRecognizeService(img);

    // 显示结果图像
    cv::imshow("Recognition Result", result_img);
    RCLCPP_INFO(this->get_logger(), "按任意键退出...");
    cv::waitKey(0);  // 等待用户按键，保持窗口显示
  }

  /**
   * @brief 处理摄像头实时视频流
   * @param cam_id 摄像头设备索引（例如 0 表示 /dev/video0）
   */
  void RunCamera(int cam_id) {
    cv::VideoCapture cap(cam_id);  // 打开摄像头
    if (!cap.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开摄像头: %d", cam_id);
      return;
    }
    // 设置摄像头采集分辨率（根据参数）
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,
            this->get_parameter("camera_height").as_int());
    cap.set(cv::CAP_PROP_FRAME_WIDTH,
            this->get_parameter("camera_width").as_int());
    double interval = this->get_parameter("capture_interval").as_double();

    rclcpp::Rate rate(1.0 / interval);  // 控制循环频率
    RCLCPP_INFO(this->get_logger(),
                "摄像头已就绪。操作提示: [空格]: 识别当前帧, [q]: 退出");
    // 主循环：持续读取视频帧，显示实时画面，并响应按键事件
    while (rclcpp::ok()) {
      cv::Mat frame;
      cap >> frame;
      if (frame.empty()) break;          // 若获取失败则退出
      cv::imshow("Camera Feed", frame);  // 显示原始视频流
      char key = (char)cv::waitKey(1);
      if (key == 'q' || key == 27) {
        break;
      } else if (key == ' ') {
        RCLCPP_INFO(this->get_logger(), "正在请求识别...");
        cv::Mat res = CallRecognizeService(frame);  // 调用服务并获取标注图像
        cv::imshow("Recognition Result", res);
      }
      rate.sleep();
    }
  }

  /**
   * @brief 调用人脸识别服务
   * @param img_in 输入图像（OpenCV Mat 格式）
   * @return 标注了识别结果的图像
   *
   * 该函数将 OpenCV 图像转换为 ROS 图像消息，发送服务请求，
   * 等待响应，然后根据响应结果在图像上绘制文字。
   */
  cv::Mat CallRecognizeService(cv::Mat img_in) {
    // 等待服务可用，超时 2 秒
    if (!client_->wait_for_service(2s)) {
      RCLCPP_ERROR(this->get_logger(), "服务未就绪，请检查服务端是否启动。");
      return img_in;
    }

    // 准备服务请求
    auto request = std::make_shared<RecognizeFace::Request>();
    auto header = std_msgs::msg::Header();
    header.stamp = this->now();
    header.frame_id = "camera_frame";
    // 将 OpenCV 图像转换为 ROS 图像消息（编码为 bgr8）
    cv_bridge::CvImage img_bridge(header, "bgr8", img_in);
    img_bridge.toImageMsg(request->image);

    // 异步发送请求，返回一个 future 对象
    auto result_future = client_->async_send_request(request);

    // 使用 spin_until_future_complete 等待服务响应，同时保持 ROS 事件循环处理
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      // 请求成功，获取响应
      auto response = result_future.get();
      cv::Mat display_img = img_in.clone();  // 克隆原图，用于绘制标注
      if (response->authorized) {
        RCLCPP_INFO(this->get_logger(), "成功识别: %s", response->name.c_str());
        DrawOverlay(display_img, "User: " + response->name,
                    cv::Scalar(0, 255, 0));  // 绿色文字

      } else {
        RCLCPP_WARN(this->get_logger(), "识别失败: 未授权用户");
        DrawOverlay(display_img, "Unauthorized",
                    cv::Scalar(0, 0, 255));  // 红色文字
      }
      return display_img;
    } else {
      RCLCPP_ERROR(this->get_logger(), "服务调用失败");
      return img_in;  // 服务调用失败，返回原图
    }
  }

  /**
   * @brief 在图像上绘制文字覆盖层（用于显示识别结果）
   * @param img 要绘制的图像（会被修改）
   * @param text 要绘制的文字
   * @param color 文字颜色（BGR 格式）
   */
  void DrawOverlay(cv::Mat& img, const std::string& text,
                   const cv::Scalar& color) {
    cv::putText(img, text, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                color, 2);
  }
};

/**
 * @brief 主函数：初始化 ROS2，创建节点对象，运行客户端逻辑，最后清理
 */

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FaceRecognitionClient>();
  node->Run();
  cv::destroyAllWindows();  // 关闭所有 OpenCV 窗口
  rclcpp::shutdown();
  return 0;
}