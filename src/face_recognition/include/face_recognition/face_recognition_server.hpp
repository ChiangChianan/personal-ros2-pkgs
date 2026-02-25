#ifndef FACE_RECOGNITON_SERVER_H
#define FACE_RECOGNITION_SERVER_H

#include <cv_bridge/cv_bridge.h>
#include <dlib/dnn.h>
#include <dlib/image_processing.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/opencv.h>

#include <rclcpp/rclcpp.hpp>

#include "face_recognition/srv/recognize_face.hpp"
#include "face_recognition/srv/register_face.hpp"

// ----------------------------------------------------------------------------------------
// Dlib ResNet 神经网络定义 (必须与官方 .dat 模型文件完全匹配)
// ----------------------------------------------------------------------------------------

// 基本残差块定义：输入经过卷积、BN、ReLU后，再与原始输入相加
template <template <int, template <typename> class, int, typename> class block,
          int N, template <typename> class BN, typename SUBNET>
using residual = dlib::add_prev1<block<N, BN, 1, dlib::tag1<SUBNET>>>;

// 带下采样的残差块（通过步长为2的卷积或池化降低空间尺寸）
template <template <int, template <typename> class, int, typename> class block,
          int N, template <typename> class BN, typename SUBNET>
using residual_down = dlib::add_prev2<dlib::avg_pool<
    2, 2, 2, 2, dlib::skip1<dlib::tag2<block<N, BN, 2, dlib::tag1<SUBNET>>>>>>;

// 卷积块：包含卷积、BN、ReLU，stride参数决定是否下采样
template <int N, template <typename> class BN, int stride, typename SUBNET>
using block =
    BN<dlib::con<N, 3, 3, 1, 1,
                 dlib::relu<BN<dlib::con<N, 3, 3, stride, stride, SUBNET>>>>>;

// 使用仿射变换（affine）而非BN，适用于 fine-tuning 或固定特征提取
template <int N, typename SUBNET>
using ares = dlib::relu<residual<block, N, dlib::affine, SUBNET>>;
template <int N, typename SUBNET>
using ares_down = dlib::relu<residual_down<block, N, dlib::affine, SUBNET>>;

// 网络层次结构定义（从高层到底层）
template <typename SUBNET>
using alevel0 = ares_down<256, SUBNET>;
template <typename SUBNET>
using alevel1 = ares<256, ares<256, ares_down<256, SUBNET>>>;
template <typename SUBNET>
using alevel2 = ares<128, ares<128, ares_down<128, SUBNET>>>;
template <typename SUBNET>
using alevel3 = ares<64, ares<64, ares<64, ares_down<64, SUBNET>>>>;
template <typename SUBNET>
using alevel4 = ares<32, ares<32, ares<32, SUBNET>>>;

// 神经网络中，浅层（靠近输入的层）通道数少、分辨率高；深层（靠近输出的层）通道数多、分辨率低。
// 最终的网络类型：输入为 150x150 的 RGB 图像，输出 128 维特征向量
using anet_type = dlib::loss_metric<dlib::fc_no_bias<
    128,
    dlib::avg_pool_everything<
        alevel0<alevel1<alevel2<alevel3<alevel4<dlib::max_pool<
            3, 3, 2, 2,
            dlib::relu<dlib::affine<dlib::con<
                32, 7, 7, 2, 2, dlib::input_rgb_image_sized<150>>>>>>>>>>>>>;

// ----------------------------------------------------------------------------------------

// 为方便使用，定义服务类型的别名

/**
 * @brief 人脸识别服务端节点
 *
 * 提供两个服务：
 *   - register_face：注册人脸，将名字和对应特征存入数据库
 *   - recognize_face：识别人脸，返回是否授权及匹配的名字
 */
class FaceRecognitionServer : public rclcpp::Node {
 public:
  // --- 类型别名 ---
  using RegisterFace = face_recognition::srv::RegisterFace;
  using RecognizeFace = face_recognition::srv::RecognizeFace;

  FaceRecognitionServer();

 private:
  // Dlib 对象
  dlib::frontal_face_detector detector_;  // HOG 人脸检测器
  dlib::shape_predictor sp_;              // 人脸关键点检测器（68点）
  anet_type net_;                         // ResNet 人脸识别网络

  // 人脸数据库：名字 -> 128维特征向量（用 matrix<float,0,1>
  // 存储，0表示动态维度）
  std::map<std::string, dlib::matrix<float, 0, 1>> face_db_;
  // 服务句柄
  rclcpp::Service<face_recognition::srv::RegisterFace>::SharedPtr
      register_service_;
  rclcpp::Service<face_recognition::srv::RecognizeFace>::SharedPtr
      recognize_service_;
  /**
   * @brief 加载 Dlib 模型和已有数据库
   * @return 是否成功
   */
  bool LoadResources();

  /**
   * @brief 从二进制文件加载人脸数据库
   *
   * 文件格式：
   *   [条目数量: size_t]
   *   对每个条目：
   *     [名字长度: size_t] [名字数据: char数组]
   *     [特征向量长度: size_t] [特征数据: float数组]
   */
  void LoadFaceDB();

  /**
   * @brief 将当前人脸数据库保存到二进制文件
   *
   * 格式与 LoadFaceDB 一致
   */
  void SaveFaceDB();

  /**
   * @brief 从 ROS 图像消息中提取人脸特征
   *
   * 步骤：
   *   1. 将 ROS 图像转为 OpenCV 图像，再转为 Dlib 图像格式
   *   2. 使用人脸检测器检测人脸（取面积最大的）
   *   3. 使用关键点检测器获取 68 个关键点
   *   4. 根据关键点对齐并裁剪出人脸区域（150x150）
   *   5. 通过 ResNet 网络提取 128 维特征向量
   *
   * @param msg ROS 图像消息
   * @param feat 输出特征向量（引用）
   * @param err 输出错误信息
   * @return 是否成功
   */
  bool ExtractFeature(const sensor_msgs::msg::Image& msg,
                      dlib::matrix<float, 0, 1>& feat, std::string& err);

  /**
   * @brief 处理注册请求
   *
   * 从图像中提取特征，将名字和特征存入数据库，并保存到文件。
   */
  void HandleRegister(const std::shared_ptr<RegisterFace::Request> req,
                      std::shared_ptr<RegisterFace::Response> res);

  /**
   * @brief 处理识别请求
   *
   * 从图像中提取特征，与数据库中所有特征计算欧氏距离，找到最近的名字。
   * 如果最近距离小于阈值，则授权通过，否则返回 Unknown。
   */
  void HandleRecognize(const std::shared_ptr<RecognizeFace::Request> req,
                       std::shared_ptr<RecognizeFace::Response> res);
};

#endif