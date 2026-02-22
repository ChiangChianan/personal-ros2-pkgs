#include <QApplication>
#include <QObject>
#include <QThread>
#include <rclcpp/rclcpp.hpp>

#include "system_status/main_window.hpp"

// 单独的线程运行 ROS2 spin
void SpinThread(rclcpp::Node::SharedPtr node) { rclcpp::spin(node); }

int main(int argc, char** argv) {
  // 初始化 Qt
  QApplication app(argc, argv);

  // 初始化 ROS2
  rclcpp::init(argc, argv);

  // 创建节点和窗口
  auto node = std::make_shared<SystemStatusNode>();
  MainWindow window;

  // 注册跨线程传递的自定义类型
  qRegisterMetaType<status_interfaces::msg::SystemStatus::SharedPtr>();

  // 连接节点的信号到窗口的槽（跨线程连接，自动使用 Qt::QueuedConnection）
  QObject::connect(node.get(), &SystemStatusNode::dataReceived, &window,
                   &MainWindow::updateUI);

  // 在新线程中运行 ROS2 spin
  QThread rosThread;
  QObject::connect(&rosThread, &QThread::started,
                   [node]() { SpinThread(node); });
  rosThread.start();

  // 显示窗口
  window.show();

  // 运行 Qt 事件循环
  int result = app.exec();

  // 退出前清理
  rclcpp::shutdown();
  rosThread.wait();  // 等待 spin 线程结束

  return result;
}