#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

class TextProvider : public rclcpp::Node {
 public:
  explicit TextProvider(std::string node_name);

 private:
  // 发布下一行文本（如果文件未结束）
  void PublishNextLine();
  // 反馈订阅回调：收到反馈后发布下一行
  void FeedbackCallback(const std_msgs::msg::Empty::SharedPtr /*msg*/);
  // 打开文件并跳过 BOM
  bool OpenFile();
  // 关闭文件
  void CloseFile();

  // 发布者：文本消息
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_publisher_;

  // 订阅者：TTS 完成反馈
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr feedback_subscription_;

  std::string file_path_;
  std::ifstream file_;
  bool file_opened_;
  bool is_finished_;
  bool waiting_for_feedback_;
};