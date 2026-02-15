#ifndef WEBSOCKET_CLIENT_NODE_H
#define WEBSOCKET_CLIENT_NODE_H

#include <atomic>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_client.hpp>

using json = nlohmann::json;

// WebSocket 客户端类型定义（使用 TLS）
typedef websocketpp::client<websocketpp::config::asio_tls_client>
    WebSocketClient;
typedef websocketpp::lib::shared_ptr<websocketpp::lib::asio::ssl::context>
    ContextPtr;

// 消息类型枚举（与 Python 协议一致）
enum class MsgType : uint8_t {
  FullClientRequest = 0b0001,
  AudioOnlyClient = 0b0010,
  FullServerResponse = 0b1001,
  AudioOnlyServer = 0b1011,
  FrontEndResultServer = 0b1100,
  Error = 0b1111
};

// 标志位
enum class MsgFlag : uint8_t {
  NoSeq = 0b0000,
  PositiveSeq = 0b0001,
  LastNoSeq = 0b0010,
  NegativeSeq = 0b0011,
  WithEvent = 0b0100
};

// 序列化方式
enum class Serialization : uint8_t {
  Raw = 0b0000,
  JSON = 0b0001,
  Thrift = 0b0011,
  Custom = 0b1111
};

// 压缩方式
enum class Compression : uint8_t {
  None = 0b0000,
  Gzip = 0b0001,
  Custom = 0b1111
};

// 事件类型（只列出用到的）
enum class EventType : int32_t {
  None = 0,
  StartConnection = 1,
  FinishConnection = 2,
  ConnectionStarted = 50,
  ConnectionFailed = 51,
  ConnectionFinished = 52,
  StartSession = 100,
  CancelSession = 101,
  FinishSession = 102,
  SessionStarted = 150,
  SessionCanceled = 151,
  SessionFinished = 152,
  SessionFailed = 153,
  TaskRequest = 200
};

class VolcTTSNode : public rclcpp::Node {
 public:
  VolcTTSNode();
  ~VolcTTSNode();

 private:
  // ROS 2 订阅者和发布者
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr feedback_pub_;

  // 参数
  std::string appid_;
  std::string access_token_;
  std::string resource_id_;
  std::string voice_type_;
  std::string audio_format_;
  int sample_rate_;
  std::string endpoint_;
  bool enable_timestamp_;

  // WebSocket 连接管理
  void init_websocket();
  void connect_websocket();
  void close_websocket();
  ContextPtr on_tls_init();
  void on_open(websocketpp::connection_hdl hdl);
  void on_message(websocketpp::connection_hdl hdl,
                  WebSocketClient::message_ptr msg);
  void on_close(websocketpp::connection_hdl hdl);
  void on_fail(websocketpp::connection_hdl hdl);

  WebSocketClient client_;
  websocketpp::connection_hdl connection_hdl_;
  std::mutex ws_mutex_;  // 保护共享数据
  std::queue<std::vector<uint8_t>> message_queue_;
  std::condition_variable message_cv_;
  bool connected_ = false;
  std::atomic<bool> stop_thread_{false};
  std::unique_ptr<std::thread> io_thread_;

  // 消息序列化/反序列化
  std::vector<uint8_t> serialize_message(MsgType type, MsgFlag flag,
                                         EventType event,
                                         const std::string& session_id,
                                         const std::vector<uint8_t>& payload);

  bool parse_message(const std::vector<uint8_t>& data, MsgType& type,
                     MsgFlag& flag, EventType& event, std::string& session_id,
                     std::vector<uint8_t>& payload, int32_t& error_code);

  bool send_message(const std::vector<uint8_t>& msg);
  bool receive_message(std::vector<uint8_t>& out, int timeout_ms = 5000);
  bool wait_for_event(EventType expected_event,
                      std::vector<uint8_t>& out_payload,
                      std::string& out_session_id, int timeout_ms = 10000);

  // 处理单个文本
  void text_callback(const std_msgs::msg::String::SharedPtr msg);
  bool process_text(const std::string& text, int index);

  // 辅助函数
  std::string generate_uuid();
  std::string get_resource_id(const std::string& voice);
  void save_audio(const std::vector<uint8_t>& audio_data, int index);
  void load_config_from_file(const std::string& file_path);
};

#endif