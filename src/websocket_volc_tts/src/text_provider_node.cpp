#include "websocket_volc_tts/text_provider_node.hpp"

#include <iostream>

using namespace std::literals;

TextProvider::TextProvider(std::string node_name)
    : Node(node_name),
      file_opened_(false),
      is_finished_(false),
      waiting_for_feedback_(false) {
  // 声明参数
  this->declare_parameter<std::string>("file_path", "");
  file_path_ = this->get_parameter("file_path").as_string();

  // 创建发布者和订阅者
  text_publisher_ =
      this->create_publisher<std_msgs::msg::String>("text_provider", 10);
  feedback_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
      "tts_feedback", 10,
      std::bind(&TextProvider::FeedbackCallback, this, std::placeholders::_1));

  // 打开文件
  if (!OpenFile()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s",
                 file_path_.c_str());
    return;
  }
  // 发布第一行文本（触发流程开始）
  PublishNextLine();
}

bool TextProvider::OpenFile() {
  file_.open(file_path_);
  if (!file_.is_open()) {
    return false;
  }
  char bom[3] = {0};
  file_.read(bom, 3);
  if (!(bom[0] == (char)0xEF && bom[1] == (char)0XBB && bom[2] == (char)0xBF)) {
    file_.seekg(0);
  }
  file_opened_ = true;
  return true;
}

void TextProvider::CloseFile() {
  if (file_.is_open()) {
    file_.close();
  }
  file_opened_ = false;
}

void TextProvider::PublishNextLine() {
  if (!file_.is_open() || !file_opened_ || is_finished_) {
    return;
  }
  RCLCPP_INFO(this->get_logger(), "start publish");
  std::string line;
  if (std::getline(file_, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    // 跳过空行
    if (line.empty()) {
      PublishNextLine();
      return;
    }

    auto msg = std_msgs::msg::String();
    msg.data = line;
    text_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published: %s", line.c_str());

    waiting_for_feedback_ = true;
  } else {  //文件读取完毕
    is_finished_ = true;
    CloseFile();
    waiting_for_feedback_ = false;
    RCLCPP_INFO(this->get_logger(), "Finished reading file");
  }
}

void TextProvider::FeedbackCallback(
    const std_msgs::msg::Empty::SharedPtr /*msg*/) {
  // 只有正在等待反馈时才处理
  if (!waiting_for_feedback_ || is_finished_) {
    return;
  }
  waiting_for_feedback_ = false;
  PublishNextLine();
}

// ---------- main 函数 ----------
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto text_publisher = std::make_shared<TextProvider>("text_provider");
  rclcpp::spin(text_publisher);
  rclcpp::shutdown();
  return 0;
}
