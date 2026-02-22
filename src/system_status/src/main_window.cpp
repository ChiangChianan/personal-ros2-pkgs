#include "system_status/main_window.hpp"

#include <QDateTime>
#include <QFormLayout>
#include <QGroupBox>
#include <QVBoxLayout>

// ------------------- SystemStatusNode 实现 -------------------
SystemStatusNode::SystemStatusNode() : Node("qt_system_status_sub") {
  subscription_ = create_subscription<status_interfaces::msg::SystemStatus>(
      "sys_status", 10,
      std::bind(&SystemStatusNode::TopicCallback, this, std::placeholders::_1));
}

SystemStatusNode::~SystemStatusNode() {}

void SystemStatusNode::TopicCallback(
    const status_interfaces::msg::SystemStatus::SharedPtr msg) {
  emit dataReceived(msg);  // 跨线程发送信号
}

// ------------------- MainWindow 实现 -------------------
MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), paused_(false) {
  setWindowTitle("ROS2 系统状态监控");
  resize(400, 550);  // 增加高度以容纳按钮

  QWidget* central = new QWidget(this);
  setCentralWidget(central);
  QVBoxLayout* main_layout = new QVBoxLayout(central);

  // 基本信息组
  QGroupBox* info_group = new QGroupBox("基本信息");
  QFormLayout* info_layout = new QFormLayout(info_group);
  host_label = new QLabel("-");
  timestamp_label = new QLabel("-");
  info_layout->addRow("主机名:", host_label);
  info_layout->addRow("时间戳:", timestamp_label);
  main_layout->addWidget(info_group);

  // CPU 组
  QGroupBox* cpu_group = new QGroupBox("CPU");
  QFormLayout* cpu_layout = new QFormLayout(cpu_group);
  cpu_bar = new QProgressBar;
  cpu_bar->setRange(0, 100);
  cpu_bar->setTextVisible(false);
  cpu_label = new QLabel("0%");
  cpu_label->setAlignment(Qt::AlignCenter);
  cpu_layout->addWidget(cpu_bar);
  cpu_layout->addWidget(cpu_label);
  main_layout->addWidget(cpu_group);

  // 内存组
  QGroupBox* mem_group = new QGroupBox("内存");
  QVBoxLayout* mem_layout = new QVBoxLayout(mem_group);
  mem_bar = new QProgressBar;
  mem_bar->setRange(0, 100);
  mem_bar->setTextVisible(false);
  mem_label = new QLabel("0%");
  mem_label->setAlignment(Qt::AlignCenter);
  mem_layout->addWidget(mem_bar);
  mem_layout->addWidget(mem_label);

  QHBoxLayout* mem_info_layout = new QHBoxLayout;
  mem_total_label = new QLabel("总：-");
  mem_free_label = new QLabel("剩余：-");
  mem_info_layout->addWidget(mem_total_label);
  mem_info_layout->addWidget(mem_free_label);
  mem_layout->addLayout(mem_info_layout);
  main_layout->addWidget(mem_group);

  // 网络组
  QGroupBox* net_group = new QGroupBox("网络 (累计)");
  QFormLayout* net_layout = new QFormLayout(net_group);
  net_sent_label = new QLabel("-");
  net_recv_label = new QLabel("-");
  net_layout->addRow("发送", net_sent_label);
  net_layout->addRow("接收", net_recv_label);
  main_layout->addWidget(net_group);

  // 按钮区域
  QHBoxLayout* button_layout = new QHBoxLayout;
  pause_button = new QPushButton("暂停更新", this);
  pause_button->setCheckable(true);  // 可保持按下状态，提供视觉反馈
  button_layout->addStretch();       // 按钮右对齐
  button_layout->addWidget(pause_button);
  main_layout->addLayout(button_layout);

  // 连接按钮点击信号到槽函数
  connect(pause_button, &QPushButton::clicked, this, &MainWindow::togglePause);

  // 底部提示
  QLabel* note_label = new QLabel("数据来自 ROS2 话题 /sys_status");
  note_label->setAlignment(Qt::AlignCenter);
  note_label->setStyleSheet("color: gray;");

  main_layout->addWidget(note_label);
  main_layout->addStretch();
}

MainWindow::~MainWindow() {}

void MainWindow::togglePause() {
  paused_ = !paused_;
  if (paused_) {
    pause_button->setText("恢复更新");
    pause_button->setChecked(true);
  } else {
    pause_button->setText("暂停更新");
    pause_button->setChecked(false);
  }
}

void MainWindow::updateUI(
    const status_interfaces::msg::SystemStatus::SharedPtr msg) {
  if (paused_) {
    // 暂停状态：忽略更新
    return;
  }
  // 更新主机名
  host_label->setText(QString::fromStdString(msg->host_name));

  // 更新时间戳（精确到毫秒）
  qint64 sec = msg->stamp.sec;
  qint64 nsec = msg->stamp.nanosec;
  QDateTime dt = QDateTime::fromSecsSinceEpoch(sec);
  dt = dt.addMSecs(nsec / 1000000);
  timestamp_label->setText(dt.toString("HH:mm:ss.zzz"));

  // CPU
  cpu_bar->setValue(static_cast<int>(msg->cpu_usage));
  cpu_label->setText(QString::number(msg->cpu_usage, 'f', 1) + "%");

  // 内存
  mem_bar->setValue(static_cast<int>(msg->memory_usage));
  mem_label->setText(QString::number(msg->memory_usage, 'f', 1) + "%");

  // 格式化内存大小
  auto format_bytes = [](uint64_t bytes) -> QString {
    const char* uints[] = {"B", "KB", "MB", "GB", "TB"};
    int i = 0;
    double sizes = bytes;
    while (sizes >= 1024 && i < 4) {
      sizes /= 1024;
      i++;
    }
    return QString::number(sizes, 'f', 2) + " " + uints[i];
  };

  mem_total_label->setText("总: " + format_bytes(msg->memory_total));
  mem_free_label->setText("剩余: " + format_bytes(msg->memory_free));

  // 网络
  net_sent_label->setText(format_bytes(msg->network_sent));
  net_recv_label->setText(format_bytes(msg->network_received));
}