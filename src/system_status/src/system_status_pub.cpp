#include <limits.h>

#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "status_interfaces/msg/system_status.hpp"

/**
 * @brief 系统状态发布节点
 *
 * 定时（1秒）发布以下信息：
 * - 时间戳
 * - 主机名
 * - CPU 使用率（%）
 * - 内存总量、空闲量、使用率（%）
 * - 网络累计发送/接收字节数
 */

class SysStatusSub : public rclcpp::Node {
 public:
  SysStatusSub() : Node("sys_status_pub") {
    // 创建发布者，话题名 "sys_status"，队列长度 10
    publisher_ = this->create_publisher<status_interfaces::msg::SystemStatus>(
        "sys_status", 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&SysStatusSub::TimerCallback, this));
    // 初始化上一次的 CPU 统计数据，用于计算使用率
    auto [total, idle] = ReadCpuStats();
    prev_cpu_total_ = total;
    prev_cpu_idle_ = idle;
  }

 private:
  // ---------- 辅助函数：读取 /proc/stat 获取 CPU 总时间和空闲时间 ----------
  std::pair<unsigned long long, unsigned long long> ReadCpuStats() {
    std::ifstream file("/proc/stat");
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开 /proc/stat");
      return {0, 0};
    }
    std::string line;
    std::getline(file, line);  // 第一行以 "cpu " 开头
    std::istringstream iss(line);
    std::string cpu;
    unsigned long long user, nice, system, idle, iowait, irq, softirq, steal,
        guest, guest_nice;
    iss >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >>
        steal >> guest >> guest_nice;
    // 总时间 = 所有模式时间之和
    unsigned long long total = user + nice + system + idle + iowait + irq +
                               softirq + steal + guest + guest_nice;
    return {total, idle};
  }

  // ---------- 辅助函数：获取主机名 ----------
  std::string GetHostName() {
    char hostname[HOST_NAME_MAX];
    if (gethostname(hostname, HOST_NAME_MAX) == 0) {
      return std::string(hostname);
    }
    return "unknown";
  }

  // 内存信息结构体
  struct MemoryInfo {
    unsigned long long total;  // 总内存（字节）
    unsigned long long free;   // 空闲内存（字节）
    double usage_percent;      // 内存使用率（%）
  };
  // ---------- 辅助函数：读取 /proc/meminfo 获取内存信息 ----------
  MemoryInfo GetMemoryInfo() {
    std::ifstream file("/proc/meminfo");
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开 /proc/meminfo");
      return {0, 0, 0.0};
    }
    std::string line;
    unsigned long long total = 0;
    unsigned long long free = 0;
    unsigned long long available = 0;
    while (std::getline(file, line)) {
      if (line.compare(0, 9, "MemTotal:") == 0) {
        std::istringstream iss(line.substr(9));
        iss >> total;
      } else if (line.compare(0, 8, "MemFree:") == 0) {
        std::istringstream iss(line.substr(8));
        iss >> free;
      } else if (line.compare(0, 13, "MemAvailable:") == 0) {
        std::istringstream iss(line.substr(13));
        iss >> available;
      }
    }
    double usage = 0.0;
    if (total > 0) {
      // 优先使用 MemAvailable 计算实际可用内存，否则用 MemFree
      unsigned long long used = total - (available ? available : free);
      usage = static_cast<double>(used) / total * 100.0;
    }
    return {total, free, usage};
  }

  // 网络 IO 结构体
  struct NetIO {
    unsigned long long bytes_sent;  // 累计发送字节数
    unsigned long long bytes_recv;  // 累计接收字节数
  };

  // ---------- 辅助函数：读取 /proc/net/dev 获取网络累计流量 ----------
  NetIO GetNetIO() {
    std::ifstream file("/proc/net/dev");
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开 /proc/net/dev");
      return {0, 0};
    }
    std::string line;
    unsigned long long total_recv = 0;
    unsigned long long total_sent = 0;
    // 跳过前两行标题
    std::getline(file, line);
    std::getline(file, line);
    while (std::getline(file, line)) {
      // fixbug: iss未初始化
      std::istringstream iss(line);
      std::string iface;
      iss >> iface;
      // 去掉接口名后面的冒号
      if (!iface.empty() && iface.back() == ':') {
        iface.pop_back();
      }
      // 不跳过回环接口 lo
      //   if (iface == "lo") continue;
      unsigned long long recv_bytes, sent_bytes;
      // 格式：接口名 接收字节  ... (中间跳过7个字段) ... 发送字节
      iss >> recv_bytes;
      for (int i = 0; i < 7; i++) {
        std::string dummy;
        iss >> dummy;
      }
      iss >> sent_bytes;
      total_recv += recv_bytes;
      total_sent += sent_bytes;
    }
    return {total_sent, total_recv};
  }

  // ---------- 定时器回调：采集数据并发布 ----------
  void TimerCallback() {
    auto msg = status_interfaces::msg::SystemStatus();
    // 1. 时间戳
    msg.stamp = this->now();
    // 2. 主机名
    msg.host_name = GetHostName();
    // 3. CPU 使用率（基于两次采样差值）
    auto [curr_total, curr_idle] = ReadCpuStats();
    unsigned long long differ_total = curr_total - prev_cpu_total_;
    unsigned long long differ_idle = curr_idle - prev_cpu_idle_;
    if (differ_total > 0) {
      msg.cpu_usage =
          (1.0 - static_cast<double>(differ_idle) / differ_total) * 100.0;
    } else {
      msg.cpu_usage = 0.0;
    }
    // 更新上一次的值供下次使用
    prev_cpu_total_ = curr_total;
    prev_cpu_idle_ = curr_idle;

    // 4. 内存信息
    auto mem = GetMemoryInfo();
    msg.memory_total = mem.total * 1024;
    msg.memory_free = mem.free * 1024;
    msg.memory_usage = mem.usage_percent;

    // 5. 网络累计流量
    auto net = GetNetIO();
    msg.network_sent = net.bytes_sent;
    msg.network_received = net.bytes_recv;

    // 打印日志
    RCLCPP_INFO(
        this->get_logger(),
        "发布系统状态: stamp=%d.%09d, host=%s, cpu=%.1f%%, mem_total=%lu, "
        "mem_free=%lu, mem_usage=%.1f%%, net_sent=%lu, net_recv=%lu",
        msg.stamp.sec, msg.stamp.nanosec, msg.host_name.c_str(), msg.cpu_usage,
        msg.memory_total, msg.memory_free, msg.memory_usage, msg.network_sent,
        msg.network_received);
    // 发布消息
    publisher_->publish(msg);
  }

  // 上一次采样的 CPU 总时间和空闲时间（用于计算使用率）
  unsigned long long prev_cpu_total_ = 0;
  unsigned long long prev_cpu_idle_ = 0;
  // ROS 2 发布者句柄
  rclcpp::Publisher<status_interfaces::msg::SystemStatus>::SharedPtr publisher_;
  // ROS 2 定时器句柄
  rclcpp::TimerBase::SharedPtr timer_;
};

// ---------- 主函数 ----------
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SysStatusSub>());
  rclcpp::shutdown();
  return 0;
}