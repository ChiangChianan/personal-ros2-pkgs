#include "websocket_volc_tts/websocket_client_node.hpp"

// ==================== 辅助工具函数（大端字节序读写）====================
/**
 * @brief 将 32 位无符号整数按大端字节序写入 vector
 *
 * WebSocket 二进制协议通常使用网络字节序（大端），
 * 这里将 value 拆分成 4 个字节，高位在前，依次 push 到 buffer。
 *
 * @param value  要写入的 32 位无符号整数值
 * @param buffer 目标字节容器
 */
inline void WriteUint32Be(uint32_t value, std::vector<uint8_t>& buffer) {
  buffer.push_back(static_cast<uint8_t>(value >> 24));
  buffer.push_back(static_cast<uint8_t>(value >> 16));
  buffer.push_back(static_cast<uint8_t>(value >> 8));
  buffer.push_back(static_cast<uint8_t>(value));
}

/**
 * @brief 将 32 位有符号整数按大端字节序写入 vector
 *
 * 先将有符号整数转为无符号（保持位模式不变），再调用大端写入。
 *
 * @param value  要写入的 32 位有符号整数值
 * @param buffer 目标字节容器
 */

inline void WriteInt32Be(int32_t value, std::vector<uint8_t>& buffer) {
  uint32_t uval = static_cast<uint32_t>(value);
  buffer.push_back(static_cast<uint8_t>(uval >> 24));
  buffer.push_back(static_cast<uint8_t>(uval >> 16));
  buffer.push_back(static_cast<uint8_t>(uval >> 8));
  buffer.push_back(static_cast<uint8_t>(uval));
}

/**
 * @brief 从内存中读取大端序的 32 位无符号整数
 *
 * 将 data[0]~data[3] 按大端组装成 uint32_t。
 *
 * @param data 指向至少 4 字节内存的指针
 * @return uint32_t 转换后的值
 */

inline uint32_t ReadUint32Be(const uint8_t* data) {
  return (static_cast<uint32_t>(data[0]) << 24 |
          static_cast<uint32_t>(data[1]) << 16 |
          static_cast<uint32_t>(data[2]) << 8 | static_cast<uint32_t>(data[3]));
}

/**
 * @brief 从内存中读取大端序的 32 位有符号整数
 *
 * 先读取无符号，再强制转换为有符号（保持位模式）。
 *
 * @param data 指向至少 4 字节内存的指针
 * @return int32_t 转换后的值
 */

inline int32_t ReadInt32Be(const uint8_t* data) {
  return static_cast<int32_t>(ReadUint32Be(data));
}

// ==================== 构造函数 ====================
VolcTTSNode::VolcTTSNode() : Node("volc_tts_node") {
  // 声明 ROS2 参数，允许从 launch 文件或命令行传递
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

  // 如果指定了配置文件，则从文件加载参数（文件内容会覆盖默认值）
  std::string config_file = this->get_parameter("config_file").as_string();
  if (!config_file.empty()) {
    LoadConfigFromFile(config_file);
  }

  // 读取所有参数到成员变量
  appid_ = this->get_parameter("appid").as_string();
  access_token_ = this->get_parameter("access_token").as_string();
  resource_id_ = this->get_parameter("resource_id").as_string();
  voice_type_ = this->get_parameter("voice_type").as_string();
  audio_format_ = this->get_parameter("audio_format").as_string();
  sample_rate_ = this->get_parameter("sample_rate").as_int();
  endpoint_ = this->get_parameter("endpoint").as_string();
  enable_timestamp_ = this->get_parameter("enable_timestamp").as_bool();

  // 创建订阅者和发布者
  // 订阅 "text_to_speak" 话题，收到文本后触发 text_callback
  text_sub_ = this->create_subscription<std_msgs::msg::String>(
      "text_to_speak", 10,
      std::bind(&VolcTTSNode::TextCallback, this, std::placeholders::_1));
  feedback_pub_ =
      this->create_publisher<std_msgs::msg::Empty>("tts_feedback", 0);

  // 初始化 WebSocket 客户端并建立连接
  InitWebSocket();
  ConnectWebSocket();

  RCLCPP_INFO(this->get_logger(), "VolcTTSNode initialized");
}

// ==================== 析构函数 ====================
VolcTTSNode::~VolcTTSNode() {
  // 通知 I/O 线程停止
  stop_thread_ = true;
  if (io_thread_ && io_thread_->joinable()) {
    io_thread_->join();
  }
  // 关闭 WebSocket 连接
  CloseWebSocket();
}

// ==================== WebSocket 初始化与回调 ====================
void VolcTTSNode::InitWebSocket() {
  // 清除所有访问日志和错误日志通道，避免输出冗余信息
  client_.clear_access_channels(websocketpp::log::alevel::all);
  client_.clear_error_channels(websocketpp::log::alevel::all);

  // 初始化 Asio（网络 I/O 后端）
  client_.init_asio();

  // 设置 TLS 初始化回调，用于配置 SSL 上下文（如加载 CA 证书）
  client_.set_tls_init_handler(std::bind(&VolcTTSNode::OnTlsInit, this));

  // 设置连接打开、收到消息、连接关闭、连接失败的回调
  client_.set_open_handler(
      std::bind(&VolcTTSNode::OnOpen, this, std::placeholders::_1));
  client_.set_message_handler(std::bind(&VolcTTSNode::OnMessage, this,
                                        std::placeholders::_1,
                                        std::placeholders::_2));
  client_.set_close_handler(
      std::bind(&VolcTTSNode::OnClose, this, std::placeholders::_1));
  client_.set_fail_handler(
      std::bind(&VolcTTSNode::OnFail, this, std::placeholders::_1));
}

/**
 * @brief TLS 初始化回调
 *
 * 当 WebSocket 客户端准备建立 TLS 连接时调用，返回配置好的 SSL 上下文。
 * 这里使用默认的系统证书路径进行验证（生产环境通常需要验证服务器证书）。
 *
 * @return ContextPtr 包含 SSL 上下文的共享指针
 */

ContextPtr VolcTTSNode::OnTlsInit() {
  // 创建一个 TLSv1.2 的 SSL 上下文
  ContextPtr ctx =
      websocketpp::lib::make_shared<websocketpp::lib::asio::ssl::context>(
          websocketpp::lib::asio::ssl::context::tlsv12);
  try {
    // 设置默认的证书验证路径（从系统加载 CA 证书）
    ctx->set_default_verify_paths();
    // 如果需要更严格的验证，可以添加 ctx->set_verify_mode(...);
    // 这里保留默认验证，服务器证书必须有效

  } catch (std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "TLS init error: %s", e.what());
  }
  return ctx;
}

/**
 * @brief 建立 WebSocket 连接
 *
 * 根据 endpoint_ 创建连接，设置必要的 HTTP 头部（包含认证信息、资源 ID、连接
 * ID）， 然后发起连接，并启动一个独立线程运行 Asio 事件循环。
 */

void VolcTTSNode::ConnectWebSocket() {
  websocketpp::lib::error_code ec;
  // 通过 endpoint_ 获取连接对象，并传入错误码
  auto conn = client_.get_connection(endpoint_, ec);
  if (ec) {
    RCLCPP_ERROR(this->get_logger(), "Get connection error: %s",
                 ec.message().c_str());
    return;
  }

  // 添加 HTTP 头部，这些是火山引擎 TTS 服务要求的认证与元数据
  conn->append_header("X-Api-App-Key", appid_);
  conn->append_header("X-Api-Access-Key", access_token_);
  // 资源 ID：若未显式指定，则根据音色类型自动选择
  std::string rid =
      resource_id_.empty() ? GetResourceId(voice_type_) : resource_id_;
  conn->append_header("X-Api-Resource-Id", rid);
  conn->append_header("X-Api-Connect-Id", GenerateUuid());

  // 发起连接（非阻塞）
  client_.connect(conn);
  // 启动一个线程运行 Asio 事件循环，client_.run() 会阻塞直到所有连接关闭
  io_thread_ = std::make_unique<std::thread>([this]() { client_.run(); });
}

/**
 * @brief 关闭 WebSocket 连接
 *
 * 如果当前已连接，则发送正常关闭帧，并更新连接状态。
 */
void VolcTTSNode::CloseWebSocket() {
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

/**
 * @brief WebSocket 连接成功打开的回调
 *
 * 保存连接句柄，标记已连接状态，并通知可能正在等待连接建立的线程。
 */
void VolcTTSNode::OnOpen(websocketpp::connection_hdl hdl) {
  std::lock_guard<std::mutex> lock(ws_mutex_);
  connection_hdl_ = hdl;
  connected_ = true;
  RCLCPP_INFO(this->get_logger(), "WebSocket connected");
  message_cv_.notify_all();
}

/**
 * @brief 收到 WebSocket 消息的回调
 *
 * 只处理二进制消息，将消息负载存入队列，并通知等待线程。
 * 文本消息被忽略（服务端可能发送文本作为心跳或调试信息，这里简单忽略）。
 */
void VolcTTSNode::OnMessage(websocketpp::connection_hdl,
                            WebSocketClient::message_ptr msg) {
  if (msg->get_opcode() == websocketpp::frame::opcode::BINARY) {
    std::lock_guard<std::mutex> lock(ws_mutex_);
    const auto& payload = msg->get_payload();
    std::vector<uint8_t> data(payload.begin(), payload.end());
    message_queue_.push(data);
    message_cv_.notify_one();
  } else {
    RCLCPP_WARN(this->get_logger(), "Received text message, ignoring");
  }
}

/**
 * @brief 连接关闭的回调
 *
 * 更新连接状态，并通知所有等待线程（避免永久阻塞）。
 */
void VolcTTSNode::OnClose(websocketpp::connection_hdl) {
  std::lock_guard<std::mutex> lock(ws_mutex_);
  connected_ = false;
  RCLCPP_WARN(this->get_logger(), "WebSocket closed");
  message_cv_.notify_all();
}

/**
 * @brief 连接失败的回调
 *
 * 更新连接状态，并通知等待线程。
 */
void VolcTTSNode::OnFail(websocketpp::connection_hdl) {
  std::lock_guard<std::mutex> lock(ws_mutex_);
  connected_ = false;
  RCLCPP_WARN(this->get_logger(), "WebSocket connection failed");
  message_cv_.notify_all();
}

// ==================== 消息序列化/反序列化（自定义协议）====================

/**
 * @brief 将消息内容序列化为二进制格式（按协议添加头部）
 *
 * 协议格式（根据火山引擎官方文档或逆向得出）：
 *   Byte0: 高4位版本号(1)，低4位头部长度(固定为1，表示头部总长4字节)
 *   Byte1: 高4位 MsgType，低4位 MsgFlag
 *   Byte2: 高4位 Serialization，低4位 Compression
 *   Byte3: 保留字段（全0）
 *   若 flag 为 WithEvent，则后跟：
 *     - event (int32 大端)
 *     - 如果需要 session_id（根据事件类型判断）：
 *         session_id_len (uint32 大端)
 *         session_id (UTF-8 字符串)
 *   接着是：
 *     - payload_len (uint32 大端)
 *     - payload (二进制数据)
 *
 * @param type       消息类型
 * @param flag       标志位（是否带事件）
 * @param event      事件类型（仅当 flag=WithEvent 时有效）
 * @param session_id 会话 ID（仅当 flag=WithEvent 且事件需要时有效）
 * @param payload    负载数据
 * @return std::vector<uint8_t> 序列化后的二进制数据
 */

std::vector<uint8_t> VolcTTSNode::SerializeMessage(
    MsgType type, MsgFlag flag, EventType event, const std::string& session_id,
    const std::vector<uint8_t>& payload) {
  std::vector<uint8_t> buffer;
  // 固定头部 4 字节
  // version=1, header_size=1 (表示头部总长4字节)
  uint8_t byte0 = (static_cast<uint8_t>(1) << 4) | 0x01;
  uint8_t byte1 =
      (static_cast<uint8_t>(type) << 4) | static_cast<uint8_t>(flag);
  uint8_t byte2 = (static_cast<uint8_t>(Serialization::JSON) << 4) |
                  static_cast<uint8_t>(Compression::None);
  uint8_t byte3 = 0;
  buffer.push_back(byte0);
  buffer.push_back(byte1);
  buffer.push_back(byte2);
  buffer.push_back(byte3);
  // 根据 flag 添加可选字段
  if (flag == MsgFlag::WithEvent) {
    WriteInt32Be(static_cast<int32_t>(event), buffer);
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
      WriteUint32Be(session_len, buffer);
      if (session_len > 0) {
        buffer.insert(buffer.end(), session_id.begin(), session_id.end());
      }
    }
  }
  uint32_t payload_len = static_cast<uint32_t>(payload.size());
  WriteUint32Be(payload_len, buffer);
  if (payload_len > 0) {
    buffer.insert(buffer.end(), payload.begin(), payload.end());
  }
  // TODO : delete the test code
  // std::string hex_str;
  // for (uint8_t b : buffer) {
  //   char buf[3];
  //   snprintf(buf, sizeof(buf), "%02x", b);
  //   hex_str += buf;
  // }
  // RCLCPP_INFO(this->get_logger(), "buffer hex: %s", hex_str.c_str());
  return buffer;
}

/**
 * @brief 解析接收到的二进制数据，提取各字段
 *
 * 与 serialize_message 对应，解析头部、可选字段和负载。
 *
 * @param data        原始二进制数据
 * @param type        [out] 消息类型
 * @param flag        [out] 标志位
 * @param event       [out] 事件类型（如果存在）
 * @param session_id  [out] 会话 ID（如果存在）
 * @param payload     [out] 负载数据
 * @param error_code  [out] 错误码（当 type 为 Error 时有效）
 * @return true 解析成功，false 数据格式错误
 */
bool VolcTTSNode::ParseMessage(const std::vector<uint8_t>& data, MsgType& type,
                               MsgFlag& flag, EventType& event,
                               std::string& session_id,
                               std::vector<uint8_t>& payload,
                               int32_t& error_code) {
  if (data.size() < 4) {
    return false;
  }

  size_t pos = 0;
  uint8_t byte0 = data[pos++];
  uint8_t byte1 = data[pos++];
  uint8_t byte2 = data[pos++];
  (void)byte2;
  uint8_t byte3 = data[pos++];
  (void)byte3;

  uint8_t version = byte0 >> 4;  // 版本号（当前为1）
  (void)version;
  uint8_t header_size = byte0 & 0x0F;  // 头部长度（忽略，假定为1）
  (void)header_size;
  type = static_cast<MsgType>(byte1 >> 4);
  flag = static_cast<MsgFlag>(byte1 & 0x0F);
  // Serialization ser = static_cast<Serialization>(byte2 >> 4);
  // Compression comp = static_cast<Compression>(byte2 & 0x0F);

  // 解析可选字段
  if (type == MsgType::Error) {
    // 错误消息：紧接着 4 字节错误码
    if (data.size() - pos < 4) {
      return false;
    }
    error_code = ReadInt32Be(&data[pos]);
    pos += 4;
  } else {
    if (flag == MsgFlag::WithEvent) {
      if (data.size() - pos < 4) {
        return false;
      }
      event = static_cast<EventType>(ReadInt32Be(&data[pos]));
      pos += 4;

      // 根据事件类型决定读取 session_id 还是 connect_id
      if (event == EventType::ConnectionStarted ||
          event == EventType::ConnectionFailed ||
          event == EventType::ConnectionFinished) {
        // 这类事件携带的是 connect_id（连接标识符），我们只需跳过，不需要保存
        if (data.size() - pos < 4) {
          return false;
        }
        uint32_t id_len = ReadUint32Be(&data[pos]);
        pos += 4;
        if (id_len > 0) {
          if (data.size() - pos < id_len) return false;
          pos += id_len;  // 跳过 connect_id 内容
        }
      } else {
        // 其他事件携带 session_id
        if (data.size() - pos < 4) {
          return false;
        }
        uint32_t session_len = ReadUint32Be(&data[pos]);
        pos += 4;
        if (session_len > 0) {
          if (data.size() - pos < session_len) {
            return false;
          }
          session_id.assign(reinterpret_cast<const char*>(&data[pos]),
                            session_len);
          pos += session_len;
        }
      }
    }
    // 如果有 sequence 字段，这里需要处理（本示例未使用，忽略）
  }

  // 读取 payload 长度及数据
  if (data.size() - pos < 4) {
    return false;
  }
  uint32_t payload_len = ReadUint32Be(&data[pos]);
  pos += 4;
  if (payload_len > 0) {
    if (data.size() - pos < payload_len) {
      return false;
    }
    payload.assign(data.begin() + pos, data.begin() + pos + payload_len);
    pos += payload_len;
  }
  return pos == data.size();
}

/**
 * @brief 发送一条二进制消息到 WebSocket 服务端
 *
 * @param msg 已序列化的消息数据
 * @return true 发送成功（放入发送队列），false 失败（如未连接或发送错误）
 */

bool VolcTTSNode::SendMessage(const std::vector<uint8_t>& msg) {
  std::lock_guard<std::mutex> lock(ws_mutex_);
  if (!connected_) {
    RCLCPP_ERROR(this->get_logger(), "Cannot send: WebSocket not connected");
    return false;
  }
  websocketpp::lib::error_code ec;
  // TODO:delete the test code
  // std::string hex_str;
  // for (uint8_t b : msg) {
  //   char buf[3];
  //   snprintf(buf, sizeof(buf), "%02x", b);
  //   hex_str += buf;
  // }
  // RCLCPP_INFO(this->get_logger(), "msg in SendMessge hex: %s",
  // hex_str.c_str());
  client_.send(connection_hdl_, msg.data(), msg.size(),
               websocketpp::frame::opcode::binary, ec);
  if (ec) {
    RCLCPP_ERROR(this->get_logger(), "Send error: %s", ec.message().c_str());
    return false;
  }
  return true;
}

/**
 * @brief 从接收队列中取出一条消息（阻塞，带超时）
 *
 * 如果队列为空，等待条件变量通知或超时。
 *
 * @param out        输出参数：接收到的消息数据
 * @param timeout_ms 超时时间（毫秒）
 * @return true 成功取出消息，false 超时或连接断开
 */

bool VolcTTSNode::ReceiveMessage(std::vector<uint8_t>& out, int timeout_ms) {
  std::unique_lock<std::mutex> lock(ws_mutex_);
  if (!message_queue_.empty()) {
    out = message_queue_.front();
    message_queue_.pop();
    return true;
  }
  // 等待新消息或超时，同时检测连接状态（若连接断开则提前返回）
  if (message_cv_.wait_for(
          lock, std::chrono::milliseconds(timeout_ms),
          [this]() { return !message_queue_.empty() || !connected_; })) {
    if (!connected_) {
      return false;
    }
    if (!message_queue_.empty()) {
      out = message_queue_.front();
      message_queue_.pop();
      return true;
    }
  }
  return false;
}

/**
 * @brief 等待特定事件类型的消息
 *
 * 循环接收消息，解析后检查事件类型是否为 expected_event，
 * 直到超时或收到期望事件。
 *
 * @param expected_event  期望的事件类型
 * @param out_payload     输出参数：匹配消息的负载
 * @param out_session_id  输出参数：匹配消息的会话 ID
 * @param timeout_ms      总超时时间（毫秒）
 * @return true 成功收到期望事件，false 超时或错误
 */
bool VolcTTSNode::WaitForEvent(EventType expected_event,
                               std::vector<uint8_t>& out_payload,
                               std::string& out_session_id, int timeout_ms) {
  auto start_time = std::chrono::steady_clock::now();
  // fixbug：循环条件出错，导致直接跳出
  while (std::chrono::steady_clock::now() - start_time <
         std::chrono::milliseconds(timeout_ms)) {
    std::vector<uint8_t> raw;
    // 调用 receive_message 阻塞 100ms 轮询，避免 CPU 空转
    if (!ReceiveMessage(raw, 100)) {
      continue;
    }
    MsgType type;
    MsgFlag flag;
    EventType event;
    std::string session_id;
    std::vector<uint8_t> payload;
    int32_t error_code = 0;
    if (ParseMessage(raw, type, flag, event, session_id, payload, error_code)) {
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
        // 忽略其他事件（如音频数据），因为音频是 AudioOnlyServer 类型，
        // 不会进入此等待循环（循环只处理 FullServerResponse）
      }
    }
  }
  return false;
}

// ==================== 文本处理核心逻辑 ====================

/**
 * @brief ROS2 文本订阅回调
 *
 * 每当有新的文本消息到达时调用，递增句子索引，调用 process_text 处理，
 * 无论成功与否都发布一个空反馈消息，以便上游继续发送下一条文本。
 */
void VolcTTSNode::TextCallback(const std_msgs::msg::String::SharedPtr msg) {
  static int sentence_index = 0;
  bool success = false;
  try {
    success = ProcessText(msg->data, sentence_index++);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in process_text: %s", e.what());
  }

  // 发布空反馈，通知 TextProvider 节点当前文本已处理（无论成功与否）
  feedback_pub_->publish(std_msgs::msg::Empty());
  if (!success) {
    RCLCPP_ERROR(this->get_logger(), "Failed to process text: %s",
                 msg->data.c_str());
  }
}

/**
 * @brief 处理单个文本的 TTS 合成
 *
 * 执行完整的会话流程：
 *   1. 若首次处理，先发送 StartConnection 并等待 ConnectionStarted。
 *   2. 生成会话 ID，发送 StartSession 并等待 SessionStarted。
 *   3. 发送 TaskRequest（包含待合成文本）。
 *   4. 发送 FinishSession。
 *   5. 循环接收 AudioOnlyServer 消息（音频数据），直到收到 SessionFinished。
 *   6. 保存所有音频数据到文件。
 *
 * @param text  待合成的文本
 * @param index 序号（用于生成文件名）
 * @return true 处理成功，false 失败
 */

bool VolcTTSNode::ProcessText(const std::string& text, int index) {
  // 检查连接状态
  if (!connected_) {
    RCLCPP_ERROR(this->get_logger(),
                 "WebSocket not connected, cannot process text");
    return false;
  }
  // 静态局部变量，确保整个节点生命周期内只发送一次 StartConnection
  static bool connection_started = false;
  if (!connection_started) {
    // 发送 StartConnection 事件，空负载使用 "{}"（JSON 格式）
    std::vector<uint8_t> empty_payload = {'{', '}'};
    auto msg = SerializeMessage(MsgType::FullClientRequest, MsgFlag::WithEvent,
                                EventType::StartConnection, "", empty_payload);
    if (!SendMessage(msg)) {
      return false;
    }
    std::vector<uint8_t> payload;
    std::string session_id;
    // 等待服务端返回 ConnectionStarted 事件
    if (!WaitForEvent(EventType::ConnectionStarted, payload, session_id)) {
      RCLCPP_ERROR(this->get_logger(), "Did not receive ConnectionStarted");
      return false;
    }
    connection_started = true;
  }
  // 生成本次会话的唯一 ID
  std::string session_id = GenerateUuid();

  // ===== 构建 StartSession 请求 =====
  json start_req;
  start_req["user"] = {{"uid", GenerateUuid()}};  //用户标识
  start_req["namespace"] = "BidirectionalTTS";    // 命名空间（固定）
  start_req["req_params"] = {
      {"speaker", voice_type_},
      {"audio_params",
       // fixbug json构造出错
       {{"format", audio_format_},
        {"sample_rate", sample_rate_},
        {"enable_timestamp", enable_timestamp_}}},             // 音频参数
      {"additions", "{\"disable_markdown_filter\": false}"}};  // 附加参数
  start_req["event"] = static_cast<int>(EventType::StartSession);  // 事件类型
  RCLCPP_INFO(this->get_logger(), "StartRequest JSON: %s",
              start_req.dump().c_str());
  std::string start_json = start_req.dump();  // JSON 序列化为字符串
  std::vector<uint8_t> start_payload(start_json.begin(), start_json.end());
  auto start_msg =
      SerializeMessage(MsgType::FullClientRequest, MsgFlag::WithEvent,
                       EventType::StartSession, session_id, start_payload);
  if (!SendMessage(start_msg)) {
    return false;
  }

  // 等待 SessionStarted 事件
  std::vector<uint8_t> session_started_payload;
  std::string session_started_id;
  if (!WaitForEvent(EventType::SessionStarted, session_started_payload,
                    session_started_id)) {
    RCLCPP_ERROR(this->get_logger(), "Did not receive SessionStarted");
    return false;
  }

  // ===== 发送 TaskRequest（包含实际合成文本） =====
  json task_req;
  task_req["user"] = {{"uid", GenerateUuid()}};
  task_req["namespace"] = "BidirectionalTTS";
  task_req["req_params"] = {
      {"text", text},  // 待合成的文本
      {"speaker", voice_type_},
      {"audio_params",
       {{"format", audio_format_},
        {"sample_rate", sample_rate_},
        {"enable_timestamp", enable_timestamp_}}},
      {"additions", "{\"disable_markdown_filter\": false}"}};
  task_req["event"] = static_cast<int>(EventType::TaskRequest);
  RCLCPP_INFO(this->get_logger(), "TaskRequest JSON: %s",
              task_req.dump().c_str());
  std::string task_json = task_req.dump();
  std::vector<uint8_t> task_payload(task_json.begin(), task_json.end());
  auto task_msg =
      SerializeMessage(MsgType::FullClientRequest, MsgFlag::WithEvent,
                       EventType::TaskRequest, session_id, task_payload);
  if (!SendMessage(task_msg)) {
    return false;
  }

  // ===== 发送 FinishSession（声明会话结束） =====
  std::vector<uint8_t> empty_payload = {'{', '}'};
  auto finish_msg =
      SerializeMessage(MsgType::FullClientRequest, MsgFlag::WithEvent,
                       EventType::FinishSession, session_id, empty_payload);
  if (!SendMessage(finish_msg)) {
    return false;
  }

  // ===== 接收音频数据，直到收到 SessionFinished =====
  std::vector<uint8_t> audio_data;
  bool session_finished = false;
  while (!session_finished) {
    std::vector<uint8_t> raw;
    if (!ReceiveMessage(raw, 5000)) {  // 单条消息超时 5 秒
      RCLCPP_ERROR(this->get_logger(),
                   "Timeout while waiting for audio/session end");
      return false;
    }
    MsgType type;
    MsgFlag flag;
    EventType event;
    std::string rcv_session_id;
    std::vector<uint8_t> payload;
    int error_code = 0;
    if (!ParseMessage(raw, type, flag, event, rcv_session_id, payload,
                      error_code)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse incoming message");
      continue;
    }

    if (type == MsgType::AudioOnlyServer) {
      // 音频数据块，追加到总缓冲区
      audio_data.insert(audio_data.end(), payload.begin(), payload.end());
      RCLCPP_DEBUG(this->get_logger(), "Received audio chunk, size=%zu",
                   payload.size());
    } else if (type == MsgType::FullServerResponse &&
               event == EventType::SessionFinished) {
      // 会话结束事件
      session_finished = true;
      RCLCPP_INFO(this->get_logger(), "Session finished");
    }
    // TODO:新增错误处理分支
    else if (type == MsgType::Error) {
      // 错误帧，payload 应为 JSON 字符串
      std::string error_str(payload.begin(), payload.end());
      RCLCPP_ERROR(this->get_logger(), "Raw error payload: %s",
                   error_str.c_str());  // 添加这行
    } else {
      RCLCPP_WARN(this->get_logger(), "Unexpected message: type=%d, event=%d",
                  static_cast<int>(type), static_cast<int>(event));
    }
  }
  // 保存音频到文件
  if (!audio_data.empty()) {
    SaveAudio(audio_data, index);
  } else {
    RCLCPP_INFO(this->get_logger(), "No audio data received for sentence %d",
                index);
  }
  return true;
}

// ==================== 辅助函数 ====================
/**
 * @brief 生成一个随机 UUID（版本 4 风格，不完全符合标准但足够唯一）
 *
 * 使用随机数生成器产生两个 64 位整数，组合成类似 UUID 的字符串格式。
 *
 * @return std::string 形如 "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx" 的字符串
 */
std::string VolcTTSNode::GenerateUuid() {
  //随机数生成器初始化
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

/**
 * @brief 根据音色类型获取对应的资源 ID
 *
 * 这是火山引擎 TTS 服务的资源映射规则，根据官方文档或经验设定。
 *
 * @param voice 音色名称
 * @return std::string 资源 ID
 */
std::string VolcTTSNode::GetResourceId(const std::string& voice) {
  if (voice.rfind("S_", 0) == 0) {
    return "volc.megatts.default";
  }
  return "volc.service_type.10029";
}

/**
 * @brief 将接收到的音频数据保存到文件
 *
 * 文件名格式：<音色>_session_<序号>.<音频格式>
 *
 * @param audio_data 二进制音频数据
 * @param index      序号（来自句子索引）
 */
void VolcTTSNode::SaveAudio(const std::vector<uint8_t>& audio_data, int index) {
  std::string filename =
      voice_type_ + "_session_" + std::to_string(index) + "." + audio_format_;
  std::ofstream file(filename, std::ios_base::binary);
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

/**
 * @brief 从 JSON 配置文件加载参数
 *
 * 读取文件内容，解析为 JSON，并将每个键值对设置为 ROS2 参数。
 * 支持字符串、整数、浮点数、布尔类型。
 *
 * @param file_path JSON 文件路径
 */
void VolcTTSNode::LoadConfigFromFile(const std::string& file_path) {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open config file: %s",
                 file_path.c_str());
    return;
  }

  try {
    json j;
    file >> j;
    for (auto it = j.begin(); it != j.end(); it++) {
      const std::string& key = it.key();
      const json& value = it.value();
      // 根据 JSON 值的类型设置对应的 ROS 参数
      if (value.is_string()) {
        this->set_parameter(rclcpp::Parameter(key, value.get<std::string>()));
      } else if (value.is_number_integer()) {
        this->set_parameter(rclcpp::Parameter(key, value.get<int>()));
      } else if (value.is_number_float()) {
        this->set_parameter(rclcpp::Parameter(key, value.get<float>()));
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

// ==================== 主函数 ====================
int main(int argc, char** argv) {
  // 初始化 ROS2
  rclcpp::init(argc, argv);
  // 创建节点实例
  auto websocket_node = std::make_shared<VolcTTSNode>();
  // 进入事件循环，等待回调
  rclcpp::spin(websocket_node);
  // 关闭 ROS2
  rclcpp::shutdown();
  return 0;
}