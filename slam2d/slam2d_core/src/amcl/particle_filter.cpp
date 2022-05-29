#include "amcl/particle_filter.hpp"
namespace slam2d_core {
namespace amcl {

ParticleFilter::ParticleFilter(const ParticleFilterOptions &options)
    : options_(options) {
  initialization();
}

ParticleFilter::~ParticleFilter() {
  clear();
}

void ParticleFilter::setOptions(const ParticleFilterOptions &options) {
  options_ = options;
  initialization();
}

void ParticleFilter::clear() {
  samples_.sample_count[0] = 0;
  samples_.sample_count[1] = 0;
  samples_.converged[0] = false;
  samples_.converged[1] = false;
  samples_.current_set = 0;
}

void ParticleFilter::initialization() {
  samples_.min_samples = options_.min_samples;
  samples_.max_samples = options_.max_samples;
  samples_.pop_err = options_.pop_err;
  samples_.pop_z = options_.pop_z;
  samples_.dist_treshold = options_.dist_treshold;

  samples_.current_set = 0;

  samples_.set[0].resize(samples_.max_samples);
  samples_.set[1].resize(samples_.max_samples);
  samples_.sample_count[0] = 0;
  samples_.sample_count[1] = 0;
  samples_.converged[0] = false;
  samples_.converged[1] = false;

  samples_.w_slow = 0.0;
  samples_.w_fast = 0.0;

  samples_.alpha_slow = options_.alpha_slow;
  samples_.alpha_fast = options_.alpha_fast;

  probalility_table_.resize(options_.max_samples);

  init_ = true;
}

bool ParticleFilter::setInitStatus(MotionModel &model,
                                   const Eigen::Vector3d &mean,
                                   const Eigen::Vector3d &cov) {
  if (!init_) {
    LOG(WARNING) << "set init-status error, not init-pf";
    return false;
  }

  clear();
  model.setInitStatus(samples_.set[0], samples_.max_samples, mean, cov);

  samples_.current_set = 0;
  samples_.sample_count[0] = samples_.max_samples;
  samples_.sample_count[1] = 0;
  samples_.w_slow = 0.0;
  samples_.w_fast = 0.0;
  return true;
}

bool ParticleFilter::computeStatus(MotionModel &model, Eigen::Vector3d &mean,
                                   Eigen::Vector3d &cov) {
  if (!init_) {
    LOG(WARNING) << " compute-status error, not init-pf";
    return false;
  }
  return model.computeStatus(samples_.set[samples_.current_set % 2],
                             samples_.sample_count[samples_.current_set % 2],
                             mean, cov);
}

bool ParticleFilter::updateAction(MotionModel &action_model,
                                  const common::Rigid3 &action_data,
                                  const common::Rigid3 &last_system_status) {
  if (!init_) {
    LOG(WARNING) << "update action error, not init-pf";
    return false;
  }
  return action_model.updateAction(
      samples_.set[samples_.current_set % 2],
      samples_.sample_count[samples_.current_set % 2], action_data,
      last_system_status);
}

bool ParticleFilter::updateSensor(LaserModel &laser_mode,
                                  const LaserScanData &sensor_data) {
  if (!init_) {
    LOG(WARNING) << "update sensor error, not init-pf";
    return false;
  }
  unsigned int current_set = samples_.current_set % 2;
  unsigned int sample_count = samples_.sample_count[current_set];

  double total = laser_mode.updateSensor(samples_.set[current_set],
                                         sample_count, sensor_data);

  if (total > 0.0) {
    for (unsigned int i = 0; i < sample_count; i++) {
      samples_.set[current_set][i].weight =
          samples_.set[current_set][i].weight / total;
    }
  } else {
    double new_weight = 1.0 / sample_count;
    for (unsigned int i = 0; i < sample_count; i++) {
      samples_.set[current_set][i].weight = new_weight;
    }
  }
  return true;
}

bool ParticleFilter::updateResample(MotionModel &model) {
  probalility_table_[0] = 0.0;
  unsigned int current_set = samples_.current_set % 2;
  unsigned int new_set = (samples_.current_set + 1) % 2;
  unsigned int sample_count = samples_.sample_count[current_set];

  assert(sample_count <= samples_.max_samples);
  for (unsigned int i = 0; i < sample_count; i++) {
    probalility_table_[i + 1] =
        probalility_table_[i] + samples_.set[current_set][i].weight;
  }

  unsigned new_samples_count = 0;
  model.clearHistogram();

  double r = 0;
  unsigned int i = 0;
  while (new_samples_count < samples_.max_samples) {
    r = drand48();
    for (i = 0; i < sample_count; i++) {
      if (probalility_table_[i] <= r && probalility_table_[i + 1] > r) {
        break;
      }
    }
    assert(i < sample_count);
    samples_.set[new_set][new_samples_count++].pose =
        samples_.set[current_set][i].pose;
    assert(samples_.set[current_set][i].weight > 0);

    model.addHistogramSample(samples_.set[current_set][i].pose);

    if (new_samples_count > resampleLimit(model.getHistogramCount())) {
      break;
    }
  }
  samples_.sample_count[new_set] = new_samples_count;

  double new_weight = 1.0 / static_cast<double>(new_samples_count);
  for (i = 0; i < new_samples_count; i++) {
    samples_.set[new_set][i].weight = new_weight;
  }
  samples_.current_set = new_set;
  return true;
}

void ParticleFilter::getCurrentPFSet(std::vector<Sample> &current_set) {
  current_set.resize(samples_.sample_count[samples_.current_set]);
  for (size_t i = 0; i < samples_.sample_count[samples_.current_set]; i++) {
    current_set[i] = samples_.set[samples_.current_set][i];
  }
}

unsigned int ParticleFilter::resampleLimit(const unsigned int k) {
  auto limit_iter_ = resample_limit_map_.find(k);
  if (limit_iter_ != resample_limit_map_.end()) {
    return limit_iter_->second;
  }
  unsigned int limit = 0;
  if (k < 1) {
    limit = samples_.max_samples;
  }

  if (k == 1) {
    limit = samples_.min_samples;
  }

  double a = 1.0;
  double b = 2.0 / (9 * (static_cast<double>(k) - 1.0));
  double c = std::sqrt(b) * samples_.pop_z;
  double x = a - b + c;

  unsigned int n = static_cast<unsigned int>(
      std::ceil((k - 1) / (2 * samples_.pop_err) * x * x * x));

  if (n < samples_.min_samples) {
    limit = samples_.min_samples;
  } else if (n > samples_.max_samples) {
    limit = samples_.max_samples;
  } else {
    limit = n;
  }

  resample_limit_map_.insert(std::pair<unsigned int, unsigned int>(k, limit));
  return limit;
}

}  // namespace amcl
}  // namespace slam2d_core