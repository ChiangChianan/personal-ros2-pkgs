#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QLabel>
#include <QMainWindow>
#include <QMetaType>
#include <QProgressBar>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>

#include "status_interfaces/msg/system_status.hpp"

/**
 * 声明自定义类型为 Qt 元类型。
 * 这使得 status_interfaces::msg::SystemStatus::SharedPtr
 * 类型能够用于跨线程的信号/槽连接，
 * 因为 Qt 的信号槽机制需要事先知道自定义类型，以便正确地序列化参数。
 */

Q_DECLARE_METATYPE(status_interfaces::msg::SystemStatus::SharedPtr)

/**
 * SystemStatusNode 类：继承自 QObject 和 rclcpp::Node，同时拥有 Qt 元对象能力和
 * ROS2 节点功能。 它作为一个 ROS2 订阅者，监听系统状态话题，并通过 Qt
 * 信号将数据传递给主线程的 UI 更新函数。
 */
class SystemStatusNode : public QObject, public rclcpp::Node {
  Q_OBJECT
 public:
  SystemStatusNode();
  ~SystemStatusNode();

 signals:
  /**
   * 当收到新的系统状态消息时，发射此信号，将消息共享指针传递给订阅者（通常是主窗口）。
   * 该信号会跨线程传递，因此需要 Q_DECLARE_METATYPE 注册。
   */
  void dataReceived(const status_interfaces::msg::SystemStatus::SharedPtr msg);

 private:
  /**
   * 话题回调函数：每当有新的消息到达时被调用。
   * 它通过发射 dataReceived 信号，将消息转发给主线程处理。
   */
  void TopicCallback(const status_interfaces::msg::SystemStatus::SharedPtr msg);

  rclcpp::Subscription<status_interfaces::msg::SystemStatus>::SharedPtr
      subscription_;
};

/**
 * MainWindow 类：应用程序的主窗口，负责显示系统状态信息。
 * 它包含各种 UI 组件（标签、进度条、按钮），并通过槽函数接收来自
 * SystemStatusNode 的数据更新界面。
 * 同时提供暂停/继续功能，允许用户冻结界面更新，便于观察某一时刻的状态。
 */
class MainWindow : public QMainWindow {
  Q_OBJECT
 public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

 public slots:
  void updateUI(const status_interfaces::msg::SystemStatus::SharedPtr msg);
 private slots:
  void togglePause();  // 切换暂停/继续状态

 private:
  // UI 组件指针（均在堆上创建，由 Qt 对象树管理）
  QLabel* host_label;         // 显示主机名
  QLabel* timestamp_label;    // 显示消息时间戳
  QProgressBar* cpu_bar;      // CPU 使用率进度条
  QLabel* cpu_label;          // CPU 使用率文本
  QProgressBar* mem_bar;      // 内存使用率进度条
  QLabel* mem_label;          // 内存使用率文本
  QLabel* mem_total_label;    // 总内存大小
  QLabel* mem_free_label;     // 空闲内存大小
  QLabel* net_sent_label;     // 网络发送字节数
  QLabel* net_recv_label;     // 网络接收字节数
  QPushButton* pause_button;  // 暂停/继续按钮
  bool paused_ = false;
};

#endif