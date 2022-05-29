#include "amcl/motion_model.hpp"
#include <glog/logging.h>
#include <random>

namespace slam2d_core {
namespace amcl {

double MotionModel::randomGaussian(double sigma) {
  double x1, x2, w, r;

  do {
    do { r = drand48(); } while (r == 0.0);
    x1 = 2.0 * r - 1.0;
    do { r = drand48(); } while (r == 0.0);
    x2 = 2.0 * r - 1.0;
    w = x1 * x1 + x2 * x2;
  } while (w > 1.0 || w == 0.0);

  return (sigma * x2 * sqrt(-2.0 * log(w) / w));
}

bool MotionModel::setInitStatus(std::vector<Sample> &samples,
                                const unsigned int sample_count,
                                const Eigen::Vector3d &mean,
                                const Eigen::Vector3d &cov) {
  if (sample_count > samples.size()) {
    LOG(ERROR) << "parameter sample: " << sample_count
               << " was big then size of samples " << samples.size();
  }

  if (cov[0] < 0 || cov[1] < 0 || cov[2] < 0) {
    LOG(FATAL) << "cov: " << cov[0] << " " << cov[1] << " " << cov[2] << " <0";
  }

  // Initialize the filter using a guassian
  std::default_random_engine random_engine;
  std::normal_distribution<double> dis_x(mean[0], std::sqrt(cov[0]));
  std::normal_distribution<double> dis_y(mean[1], std::sqrt(cov[1]));
  std::normal_distribution<double> dis_yaw(mean[2], std::sqrt(cov[2]));

  double weight = 1.0 / sample_count;
  for (unsigned int i = 0; i < sample_count; i++) {
    samples[i].weight = weight;
    samples[i].pose = common::Rigid3{
        Eigen::Vector3d(dis_x(random_engine), dis_y(random_engine), 0.0),
        common::yawToQuaternion(0.0, 0.0, dis_yaw(random_engine))};
  }

  return true;
}

bool MotionModel::computeStatus(const std::vector<Sample> &samples,
                                const unsigned int sample_count,
                                Eigen::Vector3d &pose_mean,
                                Eigen::Vector3d &pose_cov) {
  if (sample_count > samples.size()) {
    LOG(ERROR) << "parameter sample: " << sample_count
               << "was big then size of samples " << samples.size();
  }

  if (sample_count <= 1) {
    return false;
  }
  unsigned int i = 0;

  double m[4];
  for (i = 0; i < 4; i++) { m[i] = 0.0; }

  double sum_weight = 0.0;
  for (i = 0; i < sample_count; i++) {
    sum_weight += samples[i].weight;

    double yaw = common::quaternionToYaw(samples[i].pose.rotation());
    m[0] += samples[i].weight * samples[i].pose.translation().x();
    m[1] += samples[i].weight * samples[i].pose.translation().y();
    m[2] += samples[i].weight * std::cos(yaw);
    m[3] += samples[i].weight * std::sin(yaw);
  }

  pose_mean[0] = m[0] / sum_weight;
  pose_mean[1] = m[1] / sum_weight;
  pose_mean[2] = std::atan2(m[3], m[2]);

  double cov_xx = 0;
  double cov_yy = 0;
  for (i = 0; i < sample_count; i++) {
    cov_xx += samples[i].weight *
              (samples[i].pose.translation().x() - pose_mean[0]) *
              (samples[i].pose.translation().x() - pose_mean[0]);
    cov_yy += samples[i].weight *
              (samples[i].pose.translation().y() - pose_mean[1]) *
              (samples[i].pose.translation().y() - pose_mean[1]);
  }

  pose_cov[0] = cov_xx;
  pose_cov[1] = cov_yy;
  pose_cov[2] = -2 * std::log(std::sqrt(m[2] * m[2] + m[3] * m[3]));
  return true;
}

bool MotionModel::updateAction(std::vector<Sample> &samples,
                               const unsigned int sample_count,
                               const common::Rigid3 &new_system_status,
                               const common::Rigid3 &last_system_status) {
  if (!init_) {
    LOG(WARNING) << "update action error, not init odom model";
    return false;
  }
  if (sample_count > samples.size()) {
    LOG(ERROR) << "parameter sample: " << sample_count
               << "was big then size of samples " << samples.size();
  }

  common::Rigid3 action_data;

  action_data = last_system_status.inverse() * new_system_status;
  double deta_yaw = common::quaternionToYaw(action_data.rotation());
  double deta_x = new_system_status.translation().x() -
                  last_system_status.translation().x();
  double deta_y = new_system_status.translation().y() -
                  last_system_status.translation().y();
  double delta_trans = hypot(deta_x, deta_y);

  double last_system_yaw =
      common::quaternionToYaw(last_system_status.rotation());

  double delta_rot1;
  if (delta_trans < 0.03) {
    delta_rot1 = 0.0;
  } else {
    delta_rot1 = common::angleDiff(std::atan2(deta_y, deta_x), last_system_yaw);
  }

  double delta_rot2 = common::angleDiff(deta_yaw, delta_rot1);

  double delta_rot1_noise = std::min(fabs(common::angleDiff(delta_rot1, 0.0)),
                                     fabs(common::angleDiff(delta_rot1, M_PI)));
  double delta_rot2_noise = std::min(fabs(common::angleDiff(delta_rot2, 0.0)),
                                     fabs(common::angleDiff(delta_rot2, M_PI)));

  double delta_rot1_hat;
  double delta_trans_hat;
  double delta_rot2_hat;
  for (unsigned int i = 0; i < sample_count; i++) {
    delta_rot1_hat = common::angleDiff(
        delta_rot1,
        randomGaussian(alpha1_ * delta_rot1_noise * delta_rot1_noise +
                       alpha2_ * delta_trans * delta_trans));
    delta_trans_hat =
        delta_trans -
        randomGaussian(alpha3_ * delta_trans * delta_trans +
                       alpha4_ * delta_rot1_noise * delta_rot1_noise +
                       alpha4_ * delta_rot2_noise * delta_rot2_noise);
    delta_rot2_hat = common::angleDiff(
        delta_rot2,
        randomGaussian(alpha1_ * delta_rot2_noise * delta_rot2_noise +
                       alpha2_ * delta_trans * delta_trans));

    double yaw = common::quaternionToYaw(samples[i].pose.rotation());
    double d_x = samples[i].pose.translation().x() +
                 delta_trans_hat * std::cos(yaw + delta_rot1_hat);
    double d_y = samples[i].pose.translation().y() +
                 delta_trans_hat * std::sin(yaw + delta_rot1_hat);

    yaw += delta_rot1_hat + delta_rot2_hat;

    samples[i].pose = common::Rigid3{
        Eigen::Vector3d(d_x, d_y, 0.0),
        common::yawToQuaternion(0.0, 0.0, common::normalize_angle(yaw))};
  }
  return true;
}

void MotionModel::addHistogramSample(const common::Rigid3 &sample_pose) {
  HistogramIndex index;
  double yaw = common::quaternionToYaw(sample_pose.rotation());
  index.index_[0] = std::floor(sample_pose.translation().x() / cell_size_x_);
  index.index_[1] = std::floor(sample_pose.translation().y() / cell_size_y_);
  index.index_[2] = std::floor(yaw / cell_size_yaw_);

  auto iter = histogram_.find(index);
  if (iter == histogram_.end()) {
    histogram_.insert(std::pair<HistogramIndex, unsigned int>(index, 1));
  } else {
    iter->second++;
  }
}

void MotionModel::setHistogramCellSize(const double x, const double y,
                                       const double yaw) {
  if (0 == x || 0 == y || 0 == yaw) {
    LOG(FATAL) << "set histogtam cell size wrong: <0";
  }
  cell_size_x_ = x;
  cell_size_y_ = y;
  cell_size_yaw_ = yaw;
}

}  // namespace amcl
}  // namespace slam2d_core