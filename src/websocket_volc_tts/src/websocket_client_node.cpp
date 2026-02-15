#include "websocket_volc_tts/websocket_client_node.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <websocketpp/common/connection_hdl.hpp>
#include <websocketpp/common/memory.hpp>

// 辅助：整数转大端字节序
inline void write_uint32_be(uint32_t value, std::vector<uint8_t>& buffer) {
  buffer.push_back(static_cast<uint8_t>(value >> 24));
  buffer.push_back(static_cast<uint8_t>(value >> 16));
  buffer.push_back(static_cast<uint8_t>(value >> 8));
  buffer.push_back(static_cast<uint8_t>(value));
}

inline void write_int32_be(int32_t value, std::vector<uint8_t>& buffer) {
  uint32_t uval = static_cast<uint32_t>(value);
  buffer.push_back(static_cast<uint8_t>(uval >> 24));
  buffer.push_back(static_cast<uint8_t>(uval >> 16));
  buffer.push_back(static_cast<uint8_t>(uval >> 8));
  buffer.push_back(static_cast<uint8_t>(uval));
}

inline uint32_t read_uint32_be(const uint8_t* data) {
  return (static_cast<uint32_t>(data[0]) << 24) |
         (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) | static_cast<uint32_t>(data[3]);
}

inline int32_t read_int32_be(const uint8_t* data) {
  return static_cast<int32_t>(read_uint32_be(data));
}

VolcTTSNode::VolcTTSNode() : Node("volc_tts_node") {
  // 声明参数（包括 config_file）
  this->declare_parameter<std::string>("config_file", "");
  this->declare_parameter<std::string>("appid", "");
  this->declare_parameter<std::string>("access_token", "");
  this->declare_parameter<std::string>("resource_id", "");
  this->declare_parameter<std::string>("voice_type",
                                       "zh_female_cancan_mars_bigtts");
  this->declare_parameter<std::string>("audio_format", "mp3");
  this->declare_parameter<int>("sample_rate", 24000);
  this->declare_parameter<std::string>(
      "endpoint", "wss://openspeech.bytedance.com/api/v3/tts/bidirection");
  this->declare_parameter<bool>("enable_timestamp", true);

  // 从配置文件加载（若指定）
  std::string config_file = this->get_parameter("config_file").as_string();
  if (!config_file.empty()) {
    load_config_from_file(config_file);
  }

  // 读取参数
  appid_ = this->get_parameter("appid").as_string();
  access_token_ = this->get_parameter("access_token").as_string();
  resource_id_ = this->get_parameter("resource_id").as_string();
  voice_type_ = this->get_parameter("voice_type").as_string();
  audio_format_ = this->get_parameter("audio_format").as_string();
  sample_rate_ = this->get_parameter("sample_rate").as_int();
  endpoint_ = this->get_parameter("endpoint").as_string();
  enable_timestamp_ = this->get_parameter("enable_timestamp").as_bool();

  // 创建订阅者和发布者
  text_sub_ = this->create_subscription<std_msgs::msg::String>(
      "text_to_speak", 10,
      std::bind(&VolcTTSNode::text_callback, this, std::placeholders::_1));
  feedback_pub_ =
      this->create_publisher<std_msgs::msg::Empty>("tts_feedback", 10);

  // 初始化 WebSocket 并连接
  init_websocket();
  connect_websocket();

  RCLCPP_INFO(this->get_logger(), "VolcTTSNode initialized");
}

VolcTTSNode::~VolcTTSNode() {
  stop_thread_ = true;
  if (io_thread_ && io_thread_->joinable()) {
    io_thread_->join();
  }
  close_websocket();
}

// ------------------------------------------------------------
// WebSocket 初始化与回调
// ------------------------------------------------------------
void VolcTTSNode::init_websocket() {
  client_.clear_access_channels(websocketpp::log::alevel::all);
  client_.clear_error_channels(websocketpp::log::elevel::all);

  client_.init_asio();

  client_.set_tls_init_handler(std::bind(&VolcTTSNode::on_tls_init, this));

  client_.set_open_handler(
      std::bind(&VolcTTSNode::on_open, this, std::placeholders::_1));
  client_.set_message_handler(std::bind(&VolcTTSNode::on_message, this,
                                        std::placeholders::_1,
                                        std::placeholders::_2));
  client_.set_close_handler(
      std::bind(&VolcTTSNode::on_close, this, std::placeholders::_1));
  client_.set_fail_handler(
      std::bind(&VolcTTSNode::on_fail, this, std::placeholders::_1));
}

ContextPtr VolcTTSNode::on_tls_init() {
  // 使用 websocketpp::lib::asio::ssl::context
  ContextPtr ctx =
      websocketpp::lib::make_shared<websocketpp::lib::asio::ssl::context>(
          websocketpp::lib::asio::ssl::context::tlsv12);
  try {
    ctx->set_default_verify_paths();
  } catch (std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "TLS init error: %s", e.what());
  }
  return ctx;
}

void VolcTTSNode::connect_websocket() {
  websocketpp::lib::error_code ec;
  auto con = client_.get_connection(endpoint_, ec);
  if (ec) {
    RCLCPP_ERROR(this->get_logger(), "Get connection error: %s",
                 ec.message().c_str());
    return;
  }

  // 设置 HTTP 头
  con->append_header("X-Api-App-Key", appid_);
  con->append_header("X-Api-Access-Key", access_token_);
  std::string rid =
      resource_id_.empty() ? get_resource_id(voice_type_) : resource_id_;
  con->append_header("X-Api-Resource-Id", rid);
  con->append_header("X-Api-Connect-Id", generate_uuid());

  client_.connect(con);

  // 启动 ASIO 运行线程
  io_thread_ = std::make_unique<std::thread>([this]() { client_.run(); });
}

void VolcTTSNode::close_websocket() {
  std::lock_guard<std::mutex> lock(ws_mutex_);
  if (connected_) {
    websocketpp::lib::error_code ec;
    client_.close(connection_hdl_, websocketpp::close::status::normal,
                  "Shutdown", ec);
    if (ec) {
      RCLCPP_ERROR(this->get_logger(), "Close error: %s", ec.message().c_str());
    }
    connected_ = false;
  }
}

void VolcTTSNode::on_open(websocketpp::connection_hdl hdl) {
  std::lock_guard<std::mutex> lock(ws_mutex_);
  connection_hdl_ = hdl;
  connected_ = true;
  RCLCPP_INFO(this->get_logger(), "WebSocket connected");
  message_cv_.notify_all();
}

void VolcTTSNode::on_message(websocketpp::connection_hdl,
                             WebSocketClient::message_ptr msg) {
  if (msg->get_opcode() == websocketpp::frame::opcode::binary) {
    std::lock_guard<std::mutex> lock(ws_mutex_);
    const auto& payload = msg->get_payload();
    std::vector<uint8_t> data(payload.begin(), payload.end());
    message_queue_.push(data);
    message_cv_.notify_one();
  } else {
    RCLCPP_WARN(this->get_logger(), "Received text message, ignoring");
  }
}

void VolcTTSNode::on_close(websocketpp::connection_hdl) {
  std::lock_guard<std::mutex> lock(ws_mutex_);
  connected_ = false;
  RCLCPP_WARN(this->get_logger(), "WebSocket closed");
  message_cv_.notify_all();
}

void VolcTTSNode::on_fail(websocketpp::connection_hdl) {
  std::lock_guard<std::mutex> lock(ws_mutex_);
  connected_ = false;
  RCLCPP_ERROR(this->get_logger(), "WebSocket connection failed");
  message_cv_.notify_all();
}

// ------------------------------------------------------------
// 消息序列化/反序列化
// ------------------------------------------------------------
std::vector<uint8_t> VolcTTSNode::serialize_message(
    MsgType type, MsgFlag flag, EventType event, const std::string& session_id,
    const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> buffer;

  // 固定头部 4 字节
  uint8_t byte0 = (static_cast<uint8_t>(1) << 4) |
                  0x01;  // version=1, header_size=1 (4字节)
  uint8_t byte1 =
      (static_cast<uint8_t>(type) << 4) | static_cast<uint8_t>(flag);
  uint8_t byte2 = (static_cast<uint8_t>(Serialization::JSON) << 4) |
                  static_cast<uint8_t>(Compression::None);
  uint8_t byte3 = 0;  // reserved
  buffer.push_back(byte0);
  buffer.push_back(byte1);
  buffer.push_back(byte2);
  buffer.push_back(byte3);

  // 根据 flag 添加可选字段
  if (flag == MsgFlag::WithEvent) {
    // event (int32)
    write_int32_be(static_cast<int32_t>(event), buffer);

    // 判断是否需要写入 session_id
    // Connection 类事件不需要 session_id，Session 类事件需要
    bool need_session_id = true;
    if (event == EventType::StartConnection ||
        event == EventType::FinishConnection ||
        event == EventType::ConnectionStarted ||
        event == EventType::ConnectionFailed ||
        event == EventType::ConnectionFinished) {
      need_session_id = false;
    }

    if (need_session_id) {
      uint32_t session_len = static_cast<uint32_t>(session_id.size());
      write_uint32_be(session_len, buffer);
      if (session_len > 0) {
        buffer.insert(buffer.end(), session_id.begin(), session_id.end());
      }
    }
  }

  // payload length + payload
  uint32_t payload_len = static_cast<uint32_t>(payload.size());
  write_uint32_be(payload_len, buffer);
  if (payload_len > 0) {
    buffer.insert(buffer.end(), payload.begin(), payload.end());
  }

  return buffer;
}

bool VolcTTSNode::parse_message(const std::vector<uint8_t>& data, MsgType& type,
                                MsgFlag& flag, EventType& event,
                                std::string& session_id,
                                std::vector<uint8_t>& payload,
                                int32_t& error_code) {
    if (data.size() < 4) return false;

    size_t pos = 0;
    uint8_t byte0 = data[pos++];
    uint8_t byte1 = data[pos++];
    uint8_t byte2 = data[pos++];
    uint8_t byte3 = data[pos++];  // reserved

    uint8_t version = byte0 >> 4;
    uint8_t header_size = byte0 & 0x0F;
    type = static_cast<MsgType>(byte1 >> 4);
    flag = static_cast<MsgFlag>(byte1 & 0x0F);
    // Serialization ser = static_cast<Serialization>(byte2 >> 4);
    // Compression comp = static_cast<Compression>(byte2 & 0x0F);

    // 解析可选字段
    if (type == MsgType::Error) {
        if (data.size() - pos < 4) return false;
        error_code = read_int32_be(&data[pos]);
        pos += 4;
    } else {
        if (flag == MsgFlag::WithEvent) {
            if (data.size() - pos < 4) return false;
            event = static_cast<EventType>(read_int32_be(&data[pos]));
            pos += 4;

            // 根据事件类型决定读取 session_id 还是 connect_id
            if (event == EventType::ConnectionStarted ||
                event == EventType::ConnectionFailed ||
                event == EventType::ConnectionFinished) {
                // 读取 connect_id（只需移动指针，无需保存）
                if (data.size() - pos < 4) return false;
                uint32_t id_len = read_uint32_be(&data[pos]);
                pos += 4;
                if (id_len > 0) {
                    if (data.size() - pos < id_len) return false;
                    pos += id_len;  // 跳过 connect_id 内容
                }
            } else {
                // 读取 session_id
                if (data.size() - pos < 4) return false;
                uint32_t session_len = read_uint32_be(&data[pos]);
                pos += 4;
                if (session_len > 0) {
                    if (data.size() - pos < session_len) return false;
                    session_id.assign(reinterpret_cast<const char*>(&data[pos]), session_len);
                    pos += session_len;
                }
            }
        }
        // 如果有 sequence 字段，这里需要处理（本示例未使用，忽略）
    }

    // 读取 payload
    if (data.size() - pos < 4) return false;
    uint32_t payload_len = read_uint32_be(&data[pos]);
    pos += 4;
    if (payload_len > 0) {
        if (data.size() - pos < payload_len) return false;
        payload.assign(data.begin() + pos, data.begin() + pos + payload_len);
        pos += payload_len;
    }

    return pos == data.size();
}

bool VolcTTSNode::send_message(const std::vector<uint8_t>& msg) {
  std::lock_guard<std::mutex> lock(ws_mutex_);
  if (!connected_) {
    RCLCPP_ERROR(this->get_logger(), "Cannot send: WebSocket not connected");
    return false;
  }
  websocketpp::lib::error_code ec;
  client_.send(connection_hdl_, msg.data(), msg.size(),
               websocketpp::frame::opcode::binary, ec);
  if (ec) {
    RCLCPP_ERROR(this->get_logger(), "Send error: %s", ec.message().c_str());
    return false;
  }
  return true;
}

bool VolcTTSNode::receive_message(std::vector<uint8_t>& out, int timeout_ms) {
  std::unique_lock<std::mutex> lock(ws_mutex_);
  if (!message_queue_.empty()) {
    out = message_queue_.front();
    message_queue_.pop();
    return true;
  }
  // 等待新消息或超时
  if (message_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] {
        return !message_queue_.empty() || !connected_;
      })) {
    if (!connected_) {
      return false;
    }
    if (!message_queue_.empty()) {
      out = message_queue_.front();
      message_queue_.pop();
      return true;
    }
  }
  return false;  // 超时
}

bool VolcTTSNode::wait_for_event(EventType expected_event,
                                 std::vector<uint8_t>& out_payload,
                                 std::string& out_session_id, int timeout_ms) {
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time <
         std::chrono::milliseconds(timeout_ms)) {
    std::vector<uint8_t> raw;
    if (!receive_message(raw, 100)) {
      continue;
    }
    MsgType type;
    MsgFlag flag;
    EventType event;
    std::string session_id;
    std::vector<uint8_t> payload;
    int32_t error_code = 0;
    if (parse_message(raw, type, flag, event, session_id, payload,
                      error_code)) {
      if (type == MsgType::FullServerResponse && event == expected_event) {
        out_payload = payload;
        out_session_id = session_id;
        return true;
      } else if (type == MsgType::Error) {
        RCLCPP_ERROR(this->get_logger(), "Received error: code=%d, payload=%s",
                     error_code,
                     std::string(payload.begin(), payload.end()).c_str());
        return false;
      } else {
        // 忽略其他事件（如音频）
        // 注意：音频数据是 AudioOnlyServer 类型，不会进入此等待循环
      }
    }
  }
  return false;
}

// ------------------------------------------------------------
// 文本处理
// ------------------------------------------------------------
void VolcTTSNode::text_callback(const std_msgs::msg::String::SharedPtr msg) {
  static int sentence_index = 0;
  bool success = false;
  try {
    success = process_text(msg->data, sentence_index++);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in process_text: %s", e.what());
  }

  // 无论成功与否，都发送反馈，确保 TextProvider 继续
  feedback_pub_->publish(std_msgs::msg::Empty());

  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to process text: %s",
                 msg->data.c_str());
  }
}

bool VolcTTSNode::process_text(const std::string& text, int index) {
  // 检查连接状态
  if (!connected_) {
    RCLCPP_ERROR(this->get_logger(),
                 "WebSocket not connected, cannot process text");
    return false;
  }

  // 连接启动（仅首次执行）
  static bool connection_started = false;
  if (!connection_started) {
    std::vector<uint8_t> empty_payload = {'{', '}'};
    auto msg = serialize_message(MsgType::FullClientRequest, MsgFlag::WithEvent,
                                 EventType::StartConnection, "", empty_payload);
    if (!send_message(msg)) {
      return false;
    }
    std::vector<uint8_t> payload;
    std::string session_id;
    if (!wait_for_event(EventType::ConnectionStarted, payload, session_id)) {
      RCLCPP_ERROR(this->get_logger(), "Did not receive ConnectionStarted");
      return false;
    }
    connection_started = true;
  }

  // 生成 Session ID
  std::string session_id = generate_uuid();

  // 构建 StartSession 请求
  json start_req;
  start_req["user"] = {{"uid", generate_uuid()}};
  start_req["namespace"] = "BidirectionalTTS";
  start_req["req_params"] = {
      {"speaker", voice_type_},
      {"audio_params",
       {{"format", audio_format_},
        {"sample_rate", sample_rate_},
        {"enable_timestamp", enable_timestamp_}}},
      {"additions", "{\"disable_markdown_filter\": false}"}};
  start_req["event"] = static_cast<int>(EventType::StartSession);

  std::string start_json = start_req.dump();
  std::vector<uint8_t> start_payload(start_json.begin(), start_json.end());

  auto start_msg =
      serialize_message(MsgType::FullClientRequest, MsgFlag::WithEvent,
                        EventType::StartSession, session_id, start_payload);
  if (!send_message(start_msg)) {
    return false;
  }

  // 等待 SessionStarted
  std::vector<uint8_t> session_started_payload;
  std::string session_started_id;
  if (!wait_for_event(EventType::SessionStarted, session_started_payload,
                      session_started_id)) {
    RCLCPP_ERROR(this->get_logger(), "Did not receive SessionStarted");
    return false;
  }

  // 发送 TaskRequest
  json task_req;
  task_req["user"] = {{"uid", generate_uuid()}};
  task_req["namespace"] = "BidirectionalTTS";
  task_req["req_params"] = {
      {"text", text},
      {"speaker", voice_type_},
      {"audio_params",
       {{"format", audio_format_},
        {"sample_rate", sample_rate_},
        {"enable_timestamp", enable_timestamp_}}},
      {"additions", "{\"disable_markdown_filter\": false}"}};
  task_req["event"] = static_cast<int>(EventType::TaskRequest);

  std::string task_json = task_req.dump();
  std::vector<uint8_t> task_payload(task_json.begin(), task_json.end());

  auto task_msg =
      serialize_message(MsgType::FullClientRequest, MsgFlag::WithEvent,
                        EventType::TaskRequest, session_id, task_payload);
  if (!send_message(task_msg)) {
    return false;
  }

  // 发送 FinishSession
  std::vector<uint8_t> empty_payload = {'{', '}'};
  auto finish_msg =
      serialize_message(MsgType::FullClientRequest, MsgFlag::WithEvent,
                        EventType::FinishSession, session_id, empty_payload);
  if (!send_message(finish_msg)) {
    return false;
  }

  // 接收音频数据，直到 SessionFinished
  std::vector<uint8_t> audio_data;
  bool session_finished = false;
  while (!session_finished) {
    std::vector<uint8_t> raw;
    if (!receive_message(raw, 5000)) {
      RCLCPP_ERROR(this->get_logger(),
                   "Timeout while waiting for audio/session end");
      return false;
    }

    MsgType type;
    MsgFlag flag;
    EventType event;
    std::string rcv_session_id;
    std::vector<uint8_t> payload;
    int32_t error_code = 0;

    if (!parse_message(raw, type, flag, event, rcv_session_id, payload,
                       error_code)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse incoming message");
      continue;
    }

    if (type == MsgType::AudioOnlyServer) {
      audio_data.insert(audio_data.end(), payload.begin(), payload.end());
      RCLCPP_DEBUG(this->get_logger(), "Received audio chunk, size=%zu",
                   payload.size());
    } else if (type == MsgType::FullServerResponse &&
               event == EventType::SessionFinished) {
      session_finished = true;
      RCLCPP_INFO(this->get_logger(), "Session finished");
    } else {
      RCLCPP_WARN(this->get_logger(), "Unexpected message: type=%d, event=%d",
                  static_cast<int>(type), static_cast<int>(event));
    }
  }

  // 保存音频
  if (!audio_data.empty()) {
    save_audio(audio_data, index);
  } else {
    RCLCPP_WARN(this->get_logger(), "No audio data received for sentence %d",
                index);
  }

  return true;
}

// ------------------------------------------------------------
// 辅助函数
// ------------------------------------------------------------
std::string VolcTTSNode::generate_uuid() {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_int_distribution<uint64_t> dis;

  uint64_t part1 = dis(gen);
  uint64_t part2 = dis(gen);
  std::stringstream ss;
  ss << std::hex << std::setfill('0') << std::setw(8) << (part1 >> 32) << "-"
     << std::setw(4) << ((part1 >> 16) & 0xFFFF) << "-" << std::setw(4)
     << (part1 & 0xFFFF) << "-" << std::setw(4) << ((part2 >> 48) & 0xFFFF)
     << "-" << std::setw(12) << (part2 & 0xFFFFFFFFFFFF);
  return ss.str();
}

std::string VolcTTSNode::get_resource_id(const std::string& voice) {
  if (voice.rfind("S_", 0) == 0) {
    return "volc.megatts.default";
  }
  return "volc.service_type.10029";
}

void VolcTTSNode::save_audio(const std::vector<uint8_t>& audio_data,
                             int index) {
  std::string filename =
      voice_type_ + "_session_" + std::to_string(index) + "." + audio_format_;
  std::ofstream file(filename, std::ios::binary);
  if (!file) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing: %s",
                 filename.c_str());
    return;
  }
  file.write(reinterpret_cast<const char*>(audio_data.data()),
             audio_data.size());
  file.close();
  RCLCPP_INFO(this->get_logger(), "Audio saved to %s (size=%zu)",
              filename.c_str(), audio_data.size());
}

void VolcTTSNode::load_config_from_file(const std::string& file_path) {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open config file: %s",
                 file_path.c_str());
    return;
  }

  try {
    nlohmann::json j;
    file >> j;

    for (auto it = j.begin(); it != j.end(); ++it) {
      const std::string& key = it.key();
      const nlohmann::json& value = it.value();

      // 根据 JSON 值的类型设置对应的 ROS 参数
      if (value.is_string()) {
        this->set_parameter(rclcpp::Parameter(key, value.get<std::string>()));
      } else if (value.is_number_integer()) {
        this->set_parameter(rclcpp::Parameter(key, value.get<int>()));
      } else if (value.is_number_float()) {
        this->set_parameter(rclcpp::Parameter(key, value.get<double>()));
      } else if (value.is_boolean()) {
        this->set_parameter(rclcpp::Parameter(key, value.get<bool>()));
      } else {
        RCLCPP_WARN(this->get_logger(), "Unsupported JSON type for key '%s'",
                    key.c_str());
      }
    }
    RCLCPP_INFO(this->get_logger(), "Loaded configuration from %s",
                file_path.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON config: %s",
                 e.what());
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VolcTTSNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}