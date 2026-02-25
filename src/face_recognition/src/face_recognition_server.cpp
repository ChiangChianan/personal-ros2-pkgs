#include "face_recognition/face_recognition_server.hpp"

#include <fstream>

FaceRecognitionServer::FaceRecognitionServer()
    : Node("face_recognition_server") {
  // 声明 ROS2 参数，允许在启动时通过命令行或 launch 文件配置

  // 人脸关键点检测模型路径
  this->declare_parameter<std::string>("shape_predictor", "");
  // 人脸识别 ResNet 模型路径
  this->declare_parameter<std::string>("face_recognition_model", "");
  // 人脸数据库文件路径
  this->declare_parameter<std::string>("face_db_file", "face_db.bin");
  // 识别阈值（距离小于该值认为匹配）
  this->declare_parameter<double>("recognition_threshold", 0.3);

  // 加载 Dlib 模型（检测器、关键点检测器、识别网络）和已有数据库
  if (!LoadResources()) {
    throw std::runtime_error("Failed to initialize models. Check paths.");
  }

  // 创建两个服务，分别绑定对应的处理函数
  register_service_ = this->create_service<RegisterFace>(
      "register_face", std::bind(&FaceRecognitionServer::HandleRegister, this,
                                 std::placeholders::_1, std::placeholders::_2));

  recognize_service_ = this->create_service<RecognizeFace>(
      "recognize_face",
      std::bind(&FaceRecognitionServer::HandleRecognize, this,
                std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Face Recognition Server is ready.");
}

bool FaceRecognitionServer::LoadResources() {
  try {
    std::string sp_path = this->get_parameter("shape_predictor").as_string();
    std::string model_path =
        this->get_parameter("face_recognition_model").as_string();
    // 初始化人脸检测器（Dlib 自带的 HOG 检测器，无需加载文件）
    detector_ = dlib::get_frontal_face_detector();
    // 反序列化加载关键点模型和识别模型
    dlib::deserialize(sp_path) >> sp_;
    dlib::deserialize(model_path) >> net_;
    // 加载已有数据库
    LoadFaceDB();
    return true;
  } catch (const std::exception& e) {
    return false;
  }
}

void FaceRecognitionServer::LoadFaceDB() {
  std::string db_file = this->get_parameter("face_db_file").as_string();
  std::ifstream file(db_file, std::ios::binary);
  if (!file) {
    RCLCPP_WARN(this->get_logger(), "No DB found, starting fresh.");
    return;
  }
  size_t count;
  file.read(reinterpret_cast<char*>(&count), sizeof(count));
  for (size_t i = 0; i < count; i++) {
    size_t name_len;
    file.read(reinterpret_cast<char*>(&name_len), sizeof(name_len));
    std::string name(name_len, '\0');
    file.read(&name[0], name_len);

    size_t feat_size;
    file.read(reinterpret_cast<char*>(&feat_size), sizeof(feat_size));
    dlib::matrix<float, 0, 1> feat(feat_size);
    file.read(reinterpret_cast<char*>(&feat(0)), feat_size * sizeof(float));
    face_db_[name] = feat;
  }
  RCLCPP_INFO(this->get_logger(), "DB loaded with %zu entries.",
              face_db_.size());
}

void FaceRecognitionServer::SaveFaceDB() {
  std::string db_file = this->get_parameter("face_db_file").as_string();
  std::ofstream file(db_file, std::ios::binary);
  size_t count = face_db_.size();
  file.write(reinterpret_cast<const char*>(&count), sizeof(count));
  for (const auto& [name, feat] : face_db_) {
    size_t n_len = name.size();
    file.write(reinterpret_cast<const char*>(&n_len), sizeof(n_len));
    file.write(name.c_str(), n_len);
    size_t f_size = feat.size();
    file.write(reinterpret_cast<const char*>(&f_size), sizeof(f_size));
    file.write(reinterpret_cast<const char*>(&feat(0)), f_size * sizeof(float));
  }
}

bool FaceRecognitionServer::ExtractFeature(const sensor_msgs::msg::Image& msg,
                                           dlib::matrix<float, 0, 1>& feat,
                                           std::string& err) {
  try {
    // ROS -> OpenCV (BGR 格式)
    cv::Mat cv_img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    // OpenCV -> Dlib (bgr_pixel 类型)
    dlib::cv_image<dlib::bgr_pixel> dlib_img(cv_img);
    // 人脸检测
    auto faces = detector_(dlib_img);
    if (faces.empty()) {
      err = "No face detected";
      return false;
    }
    // 选择面积最大的人脸（通常为主人脸）
    auto max_face = *std::max_element(
        faces.begin(), faces.end(),
        [](const dlib::rectangle& a, const dlib::rectangle& b) {
          return a.area() < b.area();
        });
    // 获取人脸关键点
    auto shape = sp_(dlib_img, max_face);
    // 根据关键点对齐人脸，得到 150x150 的人脸图像
    dlib::matrix<dlib::rgb_pixel> face_chip;
    dlib::extract_image_chip(
        dlib_img, dlib::get_face_chip_details(shape, 150, 0.25), face_chip);
    // 通过网络提取特征
    feat = net_(face_chip);
    return true;
  } catch (const std::exception& e) {
    err = e.what();
    return false;
  }
}

void FaceRecognitionServer::HandleRegister(
    const std::shared_ptr<RegisterFace::Request> req,
    std::shared_ptr<RegisterFace::Response> res) {
  dlib::matrix<float, 0, 1> feat;
  std::string err;
  if (ExtractFeature(req->image, feat, err)) {
    face_db_[req->name] = feat;
    SaveFaceDB();
    res->success = true;
    res->message = "successfully";
  } else {
    res->success = false;
    res->message = err;
  }
}

void FaceRecognitionServer::HandleRecognize(
    const std::shared_ptr<RecognizeFace::Request> req,
    std::shared_ptr<RecognizeFace::Response> res) {
  dlib::matrix<float, 0, 1> query;
  std::string err;
  if (!ExtractFeature(req->image, query, err)) {
    res->authorized = false;  // 检测失败，不授权
    return;
  }
  double threshold = this->get_parameter("recognition_threshold").as_double();
  double best_dist = 1e6;
  std::string best_name = "";

  for (const auto& [name, feat] : face_db_) {
    double dist = dlib::length(feat - query);
    if (dist < best_dist) {
      best_dist = dist;
      best_name = name;
    }
    res->authorized = (best_dist < threshold);
    res->name = res->authorized ? best_name : "Unknown";
    RCLCPP_INFO(this->get_logger(), "Match: %s (dist: %.3f)", res->name.c_str(),
                best_dist);
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    // 创建并运行节点
    rclcpp::spin(std::make_shared<FaceRecognitionServer>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("server"), "Fatal: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}