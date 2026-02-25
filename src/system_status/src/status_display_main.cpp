#include <QApplication>
#include <QObject>
#include <QThread>
#include <rclcpp/rclcpp.hpp>

#include "system_status/main_window.hpp"

// 单独的线程运行 ROS2 spin
// fixbug:
// QThread 启动后默认执行的 run() 函数大概是这样的逻辑：
// void QThread::run() {
//     emit started(); // 触发你绑定的 Lambda 表达式，也就是你的 SpinThread
//     (void) exec();  // 等 Lambda 结束后，开始运行 Qt
//     的线程事件循环，并阻塞在这里！
// }
// 主线程执行到 rclcpp::shutdown()。这个函数非常尽职，它会立刻向 ROS
// 系统发送关闭信号，并且成功唤醒并打断了子线程里正在阻塞的 rclcpp::spin(node)。
// rclcpp::spin(node) 被成功打断并返回， spinThread 函数结束，绑定在 started
// 信号上的 Lambda 表达式 [node]() { spinThread(node); } 也顺利执行完毕。
// 当Lambda 表达式执行完后，控制权交还给了 QThread 底层的默认 run()
// 函数。默认的 run() 函数在发射完 started 信号后，紧接着就会调用 exec()。
// 此时子线程虽然结束了 ROS 的工作，但一头扎进了 Qt
// 的事件循环里，开始无休止地等待 Qt 信号！ 主线程死锁：主线程执行到
// rosThread.wait()，死死等待子线程完全退出。因为此时子线程正困在自己的事件循环（exec()）里出不来，于是死锁再次发生。

// void spinThread(rclcpp::Node::SharedPtr node) { rclcpp::spin(node); }

void SpinFunc(rclcpp::Node::SharedPtr node) {
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::cout << "ROS Spin Thread started." << std::endl;

  // 当 rclcpp::shutdown() 被调用时，rclcpp::ok() 会变为 false
  // 从而自然退出循环
  while (rclcpp::ok()) {
    executor.spin_once(std::chrono::milliseconds(50));
  }

  std::cout << "SpinThread exiting normally..." << std::endl;
}

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

  auto rosThread =
      std::unique_ptr<QThread>(QThread::create([node]() { SpinFunc(node); }));
  rosThread->setObjectName("ROS_Spin_Thread");
  rosThread->start();
  // 显示窗口
  window.show();
  int result = app.exec();

  rclcpp::shutdown();
  if (rosThread) {
    if (!rosThread->wait(3000)) {
      std::cerr << "Thread stuck, terminating..." << std::endl;
      rosThread->terminate();
      rosThread->wait();
    }
  }
  std::cout << "Finished." << std::endl;
  return result;
}
