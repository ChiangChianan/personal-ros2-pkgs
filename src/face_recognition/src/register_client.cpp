#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "face_recognition/srv/register_face.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  if (argc < 3) {
    std::cout
        << "Usage: ros2 run face_recognition register_client <img_path> <name>"
        << std::endl;
    return 1;
  }

  auto node = rclcpp::Node::make_shared("register_client_node");
  auto client =
      node->create_client<face_recognition::srv::RegisterFace>("register_face");

  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Server not found!");
    return 1;
  }

  cv::Mat img = cv::imread(argv[1]);
  if (img.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Could not read image!");
    return 1;
  }

  auto request =
      std::make_shared<face_recognition::srv::RegisterFace::Request>();
  request->name = argv[2];
  request->image =
      *(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg());

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto res = result.get();
    if (res->success)
      RCLCPP_INFO(node->get_logger(), "Success: %s", res->message.c_str());
    else
      RCLCPP_ERROR(node->get_logger(), "Failed: %s", res->message.c_str());
  }

  rclcpp::shutdown();
  return 0;
}