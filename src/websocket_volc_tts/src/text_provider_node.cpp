#include "websocket_volc_tts/text_provider_node.hpp"

#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

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
      this->create_publisher<std_msgs::msg::String>("text_to_speak", 10);
  feedback_subscription_ = this->create_subscription<std_msgs::msg::Empty>(
      "tts_feedback", 10,
      std::bind(&TextProvider::FeedbackCallback, this, std::placeholders::_1));

  // 打开文件
  if (!OpenFile()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s",
                 file_path_.c_str());
    return;
  }

  // 延迟启动：等待订阅者就绪后再发布第一行
  auto timer = this->create_wall_timer(1s, [this]() {
    timer_->cancel();  // 只执行一次
    if (this->text_publisher_->get_subscription_count() == 0) {
      RCLCPP_WARN(this->get_logger(),
                  "No subscriber for 'text_to_speak' yet, waiting...");
      // 可以继续等待或直接发布（消息仍可能丢失，但至少给订阅者时间注册）
    }
    // 发布第一行文本，启动流程
    PublishNextLine();
  });
  timer_ = timer;  // 保存定时器，防止析构
}

bool TextProvider::OpenFile() {
  file_.open(file_path_);
  if (!file_.is_open()) {
    return false;
  }
  // 跳过 UTF-8 BOM
  char bom[3] = {0};
  file_.read(bom, 3);
  if (!(bom[0] == (char)0xEF && bom[1] == (char)0xBB && bom[2] == (char)0xBF)) {
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

  std::string line;
  if (std::getline(file_, line)) {
    // 去除 Windows 行尾的 '\r'
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
  } else {
    // 文件读取完毕
    is_finished_ = true;
    CloseFile();
    waiting_for_feedback_ = false;
    RCLCPP_INFO(this->get_logger(), "Finished reading file");
  }
}

void TextProvider::FeedbackCallback(
    const std_msgs::msg::Empty::SharedPtr /*msg*/) {
  // 只有正在等待反馈且未结束时才处理
  if (!waiting_for_feedback_ || is_finished_) {
    return;
  }
  waiting_for_feedback_ = false;
  PublishNextLine();
}

// ---------- main ----------
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TextProvider>("text_provider");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}