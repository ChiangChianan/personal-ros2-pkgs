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

// WebSocketClient 是一个使用 Asio 和 TLS/SSL 的 WebSocket 客户端类型。
// websocketpp::client 是 WebSocket++ 提供的客户端模板类，
// websocketpp::config::asio_tls_client 是一个预定义的配置类，
// 它告诉客户端使用 Asio 作为网络后端，并启用 TLS/SSL 加密（即 wss:// 协议）。
using WebSocketClient =
    websocketpp::client<websocketpp::config::asio_tls_client>;

// ContextPtr 是一个指向 asio::ssl::context 的共享指针别名。
// websocketpp::lib::shared_ptr 是 WebSocket++ 内部对 std::shared_ptr 或
// boost::shared_ptr 的封装， 用于跨平台和库的兼容性。
// asio::ssl::context 是 Boost.Asio 中用于保存
// SSL/TLS配置（如证书、私钥、验证模式等）的类。 使用 shared_ptr 管理
// context的生命周期，可以确保在多个异步操作（如多个客户端连接）期间， SSL
// 上下文对象不会被意外销毁，直到所有使用它的对象都释放为止。

using ContextPtr =
    websocketpp::lib::shared_ptr<websocketpp::lib::asio::ssl::context>;

// 消息类型枚举（与 Python 协议一致）
enum class MsgType : uint8_t {
  // 客户端 -> 服务器：完整请求（含元数据和/或音频）
  FullClientRequest = 0b0001,
  // 客户端 -> 服务器：纯音频数据（流式上传）
  AudioOnlyClient = 0b0010,
  // 服务器 -> 客户端：完整响应（含元数据和/或音频）
  FullServerResponse = 0b1001,
  // 服务器 -> 客户端：纯音频数据（流式下发）
  AudioOnlyServer = 0b1011,
  // 服务器 -> 客户端：前端处理结果（仅文本，如识别文本、意图等）
  FrontEndResultServer = 0b1100,
  // 服务器 -> 客户端：错误信息
  Error = 0b1111
};

// 标志位
enum class MsgFlag : uint8_t {
  NoSeq = 0b0000,
  PositiveSeq = 0b0001,
  LastNoSeq = 0b0010,
  NegativeSeq = 0b0011,
  // 存在事件号
  WithEvent = 0b0100
};

// 序列化方式
enum class Serialization : uint8_t {
  // Raw（无特殊序列化方式，主要针对二进制音频数据）
  Raw = 0b0000,
  // JSON（主要针对文本类型消息）
  JSON = 0b0001,
  Thrift = 0b0011,
  Custom = 0b1111
};

// 压缩方式
enum class Compression : uint8_t {
  // 无压缩
  None = 0b0000,
  // gzip
  Gzip = 0b0001,
  Custom = 0b1111
};

// 事件类型（只列出用到的）
enum class EventType : int32_t {
  None = 0,
  // Websocket 阶段申明创建连接
  StartConnection = 1,
  // 结束连接
  FinishConnection = 2,
  // 成功建连
  ConnectionStarted = 50,
  // 建连失败
  ConnectionFailed = 51,
  //结束连接成功
  ConnectionFinished = 52,
  // Websocket 阶段申明创建会话
  StartSession = 100,
  // 取消会话（上行）
  CancelSession = 101,
  // 声明结束会话（上行）
  FinishSession = 102,
  // 成功开始会话
  SessionStarted = 150,
  // 已取消会话
  SessionCanceled = 151,
  // 会话已结束（上行&下行）
  SessionFinished = 152,
  // 会话失败
  SessionFailed = 153,
  // 传输任务请求（即具体的 TTS 合成请求）
  TaskRequest = 200
};

/**
 * @brief 火山引擎 TTS 的 ROS2 节点类
 *
 * 该类继承自 rclcpp::Node，负责订阅文本话题，将文本通过 WebSocket 连接发送到
 * 火山引擎的 TTS 服务，接收合成的音频数据并保存到本地文件。
 * 同时通过发布空消息反馈处理进度。
 */
class VolcTTSNode : public rclcpp::Node {
 public:
  VolcTTSNode();
  ~VolcTTSNode();

 private:
  // ROS 2 订阅者和发布者
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr feedback_pub_;

  // 参数
  std::string appid_;         // 应用 ID
  std::string access_token_;  // 访问令牌
  std::string resource_id_;   // 资源 ID（标识使用的音色等）
  std::string voice_type_;    // 音色类型
  std::string audio_format_;  // 音频格式（如 pcm、mp3）
  int sample_rate_;           // 音频采样率
  std::string endpoint_;      // WebSocket 服务端地址（wss://...）
  bool enable_timestamp_;     // 是否启用时间戳

  // -------------------- WebSocket 连接管理 --------------------
  /**
   * @brief 初始化 WebSocket 客户端
   *
   * 配置日志级别（这里全部关闭，避免控制台被刷屏），
   * 初始化 Asio 网络库，并绑定各种事件处理器。
   */
  void InitWebSocket();

  /**
   * @brief 发起 WebSocket 连接
   *
   * 根据 endpoint_ 创建连接并启动异步 I/O 线程。
   */
  void ConnectWebSocket();

  /**
   * @brief 关闭 WebSocket 连接
   *
   * 停止 I/O 线程，关闭底层连接。
   */
  void CloseWebSocket();

  /**
   * @brief TLS 初始化回调函数
   *
   * 该函数由 WebSocket++ 在建立 TLS 连接前调用，用于配置 SSL 上下文。
   * 例如设置验证模式（是否验证服务器证书）、加载 CA 证书等。
   *
   * @return ContextPtr 配置好的 SSL 上下文共享指针
   */
  ContextPtr OnTlsInit();

  /**
   * @brief WebSocket 连接成功打开时的回调
   *
   * 当连接建立成功后触发，此时可以开始发送数据。
   */
  void OnOpen(websocketpp::connection_hdl hdl);

  /**
   * @brief 收到 WebSocket 消息时的回调
   *
   * 解析二进制消息，根据协议提取类型、负载等，并进行相应处理（如保存音频）。
   */
  void OnMessage(websocketpp::connection_hdl hdl,
                 WebSocketClient::message_ptr msg);

  /**
   * @brief WebSocket 连接关闭时的回调
   */
  void OnClose(websocketpp::connection_hdl hdl);

  /**
   * @brief WebSocket 连接失败时的回调
   */
  void OnFail(websocketpp::connection_hdl hdl);

  WebSocketClient client_;                          // WebSocket 客户端实例
  websocketpp::connection_hdl connection_hdl_;      // 当前连接句柄
  std::mutex ws_mutex_;                             // 保护共享数据
  std::queue<std::vector<uint8_t>> message_queue_;  // 接收到的消息队列
  std::condition_variable message_cv_;  // 用于等待新消息的条件变量
  bool connected_ = false;              // 连接状态标志
  std::atomic<bool> stop_thread_{false};  // 通知 I/O 线程停止
  std::unique_ptr<std::thread> io_thread_;  // 运行 WebSocket I/O 事件的线程

  // -------------------- 协议序列化/反序列化 --------------------
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
  std::vector<uint8_t> SerializeMessage(MsgType type, MsgFlag flag,
                                        EventType event,
                                        const std::string& session_id,
                                        const std::vector<uint8_t>& payload);

  /**
   * @brief 解析接收到的二进制数据，提取消息各字段
   *
   * @param data 原始数据
   * @param type 输出参数：消息类型
   * @param flag 输出参数：标志位
   * @param event 输出参数：事件类型
   * @param session_id 输出参数：会话 ID
   * @param payload 输出参数：负载数据
   * @param error_code 输出参数：错误码（如果是错误消息）
   * @return true 解析成功，false 解析失败
   */
  bool ParseMessage(const std::vector<uint8_t>& data, MsgType& type,
                    MsgFlag& flag, EventType& event, std::string& session_id,
                    std::vector<uint8_t>& payload, int32_t& error_code);

  /**
   * @brief 发送一条二进制消息到 WebSocket 服务端
   *
   * @param msg 已序列化的消息数据
   * @return true 发送成功（放入发送队列），false 失败（如未连接）
   */
  bool SendMessage(const std::vector<uint8_t>& msg);

  /**
   * @brief 从接收队列中取出一条消息（阻塞，带超时）
   *
   * @param out 输出参数：接收到的消息数据
   * @param timeout_ms 超时时间（毫秒）
   * @return true 成功取出消息，false 超时或失败
   */
  bool ReceiveMessage(std::vector<uint8_t>& out, int timeout_ms = 5000);

  /**
   * @brief 等待特定事件类型的消息到来
   *
   * 循环接收消息，解析后检查事件类型是否为 expected_event，直到超时。
   *
   * @param expected_event 期望的事件类型
   * @param out_payload 输出参数：匹配消息的负载
   * @param out_session_id 输出参数：匹配消息的会话 ID
   * @param timeout_ms 超时时间（毫秒）
   * @return true 成功收到期望事件，false 超时或错误
   */
  bool WaitForEvent(EventType expected_event, std::vector<uint8_t>& out_payload,
                    std::string& out_session_id, int timeout_ms = 10000);

  // -------------------- 核心处理逻辑 --------------------
  /**
   * @brief ROS2 文本订阅回调
   *
   * 当收到新的文本消息时，调用 process_text 进行处理。
   */
  void TextCallback(const std_msgs::msg::String::SharedPtr msg);

  /**
   * @brief 处理单个文本的 TTS 合成
   *
   * 负责建立会话、发送文本、接收音频并保存。
   *
   * @param text 待合成的文本
   * @param index 序号（用于生成文件名）
   * @return true 处理成功，false 失败
   */
  bool ProcessText(const std::string& text, int index);

  // -------------------- 辅助函数 --------------------
  /**
   * @brief 生成一个 UUID（用作会话 ID）
   */
  std::string GenerateUuid();

  /**
   * @brief 根据音色类型获取资源 ID
   *
   * 可能根据 voice_type_ 映射到具体的 resource_id_。
   */
  std::string GetResourceId(const std::string& voice);

  /**
   * @brief 将音频数据保存到文件
   *
   * @param audio_data 二进制音频数据
   * @param index 序号，用于文件名
   */
  void SaveAudio(const std::vector<uint8_t>& audio_data, int index);

  /**
   * @brief 从配置文件加载参数（如 appid、access_token 等）
   *
   * @param file_path JSON 配置文件路径
   */
  void LoadConfigFromFile(const std::string& file_path);
};

#endif