#include "speed_decision_adapter.hpp"
#include "stop_shape.hpp"
#include "log.hpp"
#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {
  CVTE_BABOT::initGoogleLog("speed_decision", "info");
  LOG(INFO) << "start";

  cv::Mat map(500, 500, CV_8UC1);

  map.setTo(0);

  CVTE_BABOT::StopShape stop_shap(0.01);
  CVTE_BABOT::StopShape recover_shap(0.05);
  CVTE_BABOT::StopShape danger_shap(0.05);
  double angle = M_PI / 4;
  double radius = 0.8;

  std::vector<double> point_list_stop = {0.60,  0.36,  -0.33, 0.33,
                                         -0.33, -0.33, 0.60,  -0.36};
  double resolution = 0.005;

  // std::vector<double> point_list_dang = {
  //     0.25,  0.103,  0.103,  0.25,  -0.103, 0.25,  -0.25, 0.103,
  //     -0.25, -0.103, -0.103, -0.25, 0.103,  -0.25, 0.25,  -0.103};

  // std::vector<double> point_list_recover = {
  //     0.27,  0.118,  0.118,  0.27,  -0.118, 0.27,  -0.27, 0.118,
  //     -0.27, -0.118, -0.118, -0.27, 0.118,  -0.27, 0.27,  -0.118};

  // for (int i = 0; i < 9; ++i) {
  //   Eigen::Vector2d point;
  //   point(0) = radius * cos(((double) i + 0.5) * -angle);
  //   point(1) = radius * sin(((double) i + 0.5) * -angle);
  //   point_list.push_back(point(0));
  //   point_list.push_back(point(1));
  // }
  // stop_shap.push("all", {0.80, -0.45, -0.50, -0.45, -0.50, 0.45, 0.80,
  // 0.45});

  stop_shap.push("all", point_list_stop);
  // danger_shap.push("all", point_list_dang);
  // recover_shap.push("all", point_list_recover);
  // stop_shap.setInc(SHAPE_LEFT, 0.4);
  // stop_shap.setInc("front", 0.2);
  // for (double x = -3; x < 3; x += resolution) {
  //   for (double y = -3; y < 3; y += resolution) {
  //     if (recover_shap.checkPonitInside(x, y)) {
  //       int x_ = (int) ((x / resolution)) + 250;
  //       int y_ = (int) (y / resolution) + 250;
  //       // LOG(INFO) << "find point " << x_ << " " << y_;
  //       map.at<uint8_t>(x_, y_) = 100;
  //     }
  //   }
  // }

  for (double x = -3; x < 3; x += resolution) {
    for (double y = -3; y < 3; y += resolution) {
      if (stop_shap.checkPonitInside(x, y)) {
        int x_ = (int) ((x / resolution)) + 250;
        int y_ = (int) (y / resolution) + 250;
        // LOG(INFO) << "find point " << x_ << " " << y_;
        map.at<uint8_t>(x_, y_) = 254;
      }
    }
  }

  // for (double x = -3; x < 3; x += resolution) {
  //   for (double y = -3; y < 3; y += resolution) {
  //     if (danger_shap.checkPonitInside(x, y)) {
  //       int x_ = (int) ((x / resolution)) + 250;
  //       int y_ = (int) (y / resolution) + 250;
  //       // LOG(INFO) << "find point " << x_ << " " << y_;
  //       map.at<uint8_t>(x_, y_) = 0;
  //     }
  //   }
  // }

  // for (double x = -3; x < 3; x += 0.01) {
  //   for (double y = -3; y < 3; y += 0.01) {
  //     double x_, y_;
  //     if (stop_shap.checkPonitInside(x, y, "front")) {
  //       int x_ = (int) ((x / 0.01)) + 250;
  //       int y_ = (int) (y / 0.01) + 250;
  //       // LOG(INFO) << "find point " << x_ << " " << y_;
  //       map.at<uint8_t>(x_, y_) = 100;
  //     }
  //   }
  // }

  // std::vector<Eigen::Vector2d> temp = stop_shap.getStopShape();
  // LOG(INFO) << "size " << temp.size();
  // for (auto point : temp) {
  //   double x = point(0);
  //   double y = point(1);
  //   int x_ = (int) (x / resolution) + 250;
  //   int y_ = (int) (y / resolution) + 250;
  //   cv::Point p(y_, x_);
  //   cv::circle(map, p, 3, 100, -1);
  //   map.at<uint8_t>(x_, y_) = 100;
  // }

  // clearFov参数读取
  std::vector<double> v_clear_fov_value = {0.4, 0.034, 0.0, 0.0, 0.4, -0.034};
  std::vector<double> tf_temp = {0.471, 0.28, 0.938};

  // 计算端点向量
  std::vector<Eigen::Vector2d> v_endpoint_;
  std::vector<Eigen::Vector2d> v_product;
  std::vector<Eigen::Vector2d> v_origin;

  for (int i = 0; i < v_clear_fov_value.size() - 1; i += 2) {
    v_endpoint_.emplace_back(v_clear_fov_value[i], v_clear_fov_value[i + 1]);
    LOG(INFO) << "x " << v_endpoint_.back()(0) << " y "
              << v_endpoint_.back()(1);
  }

  // 计算法向量
  Eigen::Matrix2d R;
  R << 0.0, 1.0, -1.0, 0.0;  //{cos,-sin,sin,cos} -90°旋转矩阵
  for (int i = 0; i < v_endpoint_.size() - 1; i++) {
    // 法向量
    Eigen::Vector2d vector = R * (v_endpoint_[i + 1] - v_endpoint_[i]);
    Eigen::Vector2d origin = (v_endpoint_[i + 1] + v_endpoint_[i]) / 2;
    vector += origin;

    v_product.emplace_back(vector);
    v_origin.emplace_back(origin);
    LOG(INFO) << "local vector x " << v_product.back()(0) << " y "
              << v_product.back()(1);
    LOG(INFO) << "local origin x " << v_origin.back()(0) << " y "
              << v_origin.back()(1);
  }
  // 构建最后一个法向量
  Eigen::Vector2d vector = R * (v_endpoint_[0] - v_endpoint_.back());
  Eigen::Vector2d origin = (v_endpoint_.back() + v_endpoint_[0]) / 2;
  vector += origin;
  v_product.emplace_back(vector);
  v_origin.emplace_back(origin);
  LOG(INFO) << "v_endpoint_ 0 " << v_endpoint_[0](0) << " y "
            << v_endpoint_[0](1);
  LOG(INFO) << "v_endpoint_ back " << v_endpoint_.back()(0) << " y "
            << v_endpoint_.back()(1);
  LOG(INFO) << "local vector x1 " << v_product.back()(0) << " y "
            << v_product.back()(1);
  LOG(INFO) << "local origin x1 " << v_origin.back()(0) << " y "
            << v_origin.back()(1);

  Eigen::Matrix3d T_;
  T_ << cos(tf_temp[2]), -sin(tf_temp[2]), tf_temp[0],  //
      sin(tf_temp[2]), cos(tf_temp[2]), tf_temp[1],     //
      0, 0, 1;

  // 变换至baselink
  for (int i = 0; i < v_product.size(); ++i) {
    Eigen::Vector3d product(v_product[i](0), v_product[i](1), 1);
    Eigen::Vector3d origin(v_origin[i](0), v_origin[i](1), 1);
    product = T_ * product;
    origin = T_ * origin;

    v_product[i] = Eigen::Vector2d(product(0), product(1));
    v_origin[i] = Eigen::Vector2d(origin(0), origin(1));

    LOG(INFO) << "base_link vector x " << v_product[i](0) << " y "
              << v_product[i](1);
    LOG(INFO) << "base_link origin x " << v_origin[i](0) << " y "
              << v_origin[i](1);
  }

  // 计算到世界坐标的变换矩阵
  Eigen::Vector3d pose = {0, 0, 0};
  Eigen::Matrix3d T;
  T << cos(pose(2)), -sin(pose(2)), pose(0),  //
      sin(pose(2)), cos(pose(2)), pose(1),    //
      0, 0, 1;

  // 将clear_fov 法向量投影到当前世界坐标下
  std::vector<Eigen::Vector2d> v_product_;
  std::vector<Eigen::Vector2d> v_origin_;
  for (int i = 0; i < v_product.size(); i++) {
    Eigen::Vector3d product(v_product[i](0), v_product[i](1), 1);
    product = T * product;
    v_product_.emplace_back(product(0), product(1));

    Eigen::Vector3d origin(v_origin[i](0), v_origin[i](1), 1);
    origin = T * origin;
    v_origin_.emplace_back(origin(0), origin(1));
  }

  for (double x = -3; x < 3; x += resolution) {
    for (double y = -3; y < 3; y += resolution) {
      int inside_size = 0;
      for (int i = 0; i < v_product_.size(); ++i) {
        Eigen::Vector2d sensor_pose(x, y);
        sensor_pose = sensor_pose - v_origin_[i];
        if (sensor_pose.dot(v_product_[i] - v_origin_[i]) <= 0) {
          ++inside_size;
        }
        if (fabs(x - 0.01) < 0.0001 && fabs(y) < 0.0001) {
          LOG(ERROR) << "v_product_ " << v_product_[i](0) << " "
                     << v_product_[i](1);
          LOG(ERROR) << "v_origin_ " << v_origin_[i](0) << " "
                     << v_origin_[i](1);
        }
      }
      if (fabs(x - 0.01) < 0.0001 && fabs(y) < 0.0001) {
        LOG(ERROR) << "inside_size " << inside_size;
      }
      if (inside_size >= v_product.size()) {
        int x_ = (int) ((x / resolution)) + 250;
        int y_ = (int) (y / resolution) + 250;
        map.at<uint8_t>(x_, y_) = 100;
      }
    }
  }
  cv::Point p((0.36 / resolution + 250), (0.6 / resolution + 250));
  cv::circle(map, p, 5, 50, -1);

  cv::Point p1((250), (250));
  cv::circle(map, p1, 5, 50, -1);

  cv::imshow("speedDecisionTest1.png", map);
  // cv::imwrite("speedDecisionTest.png", map);
  cv::waitKey(0);
  return 0;
}
