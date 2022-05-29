#include "probability_voxel_grid.hpp"

#include <glog/logging.h>
// #include <iterator>
namespace CVTE_BABOT {

ProbabilityVoxelGrid::ProbabilityVoxelGrid(const unsigned int size_x,
                                           const unsigned int size_y,
                                           const unsigned int size_z,
                                           const double res_x,
                                           const double res_y,
                                           const double res_z)
    : size_x_(size_x),
      size_y_(size_y),
      size_z_(size_z),
      res_x_(res_x),
      res_y_(res_y),
      res_z_(res_z) {
  ptr_voxel_grid_data_ = std::make_shared<std::vector<unsigned int>>();
  ptr_voxel_grid_data_->resize(size_z_ * size_y_ * size_x_, define_value);

  ptr_voxel_cell_update_time_ = std::make_shared<std::vector<time_t>>();
  time_t time_now = time(NULL);
  ptr_voxel_cell_update_time_->resize(size_y_ * size_x_, time_now);
  ptr_probability_table_ = std::make_shared<ProbabilityTable>(0.65, 0.40);
  // define_value = 16384;
  define_value = 0;
}

ProbabilityVoxelGrid::~ProbabilityVoxelGrid() {
  ptr_voxel_grid_data_.reset();
  ptr_voxel_cell_update_time_.reset();
}

void ProbabilityVoxelGrid::reset() {
  if (ptr_voxel_grid_data_->empty()) {
    LOG(INFO) << " voxel grid data is empty.";
    return;
  }

  std::fill(ptr_voxel_grid_data_->begin(), ptr_voxel_grid_data_->end(),
            define_value);

  time_t time_now = time(NULL);
  std::fill(ptr_voxel_cell_update_time_->begin(),
            ptr_voxel_cell_update_time_->end(), time_now);
  LOG(INFO) << "ProbabilityVoxelGrid::reset()";
}

void ProbabilityVoxelGrid::resize(const unsigned int size_x,
                                  const unsigned int size_y,
                                  const unsigned int size_z) {
  ptr_voxel_grid_data_.reset();
  ptr_voxel_grid_data_ = std::make_shared<std::vector<unsigned int>>();
  ptr_voxel_grid_data_->resize(size_z_ * size_y_ * size_x_, define_value);
  ptr_voxel_cell_update_time_.reset();
  ptr_voxel_cell_update_time_ = std::make_shared<std::vector<time_t>>();
  time_t time_now = time(NULL);
  ptr_voxel_cell_update_time_->resize(size_y_ * size_x_, time_now);
  LOG(INFO) << "ProbabilityVoxelGrid::resize()";
}

void ProbabilityVoxelGrid::unkownVoxel(const unsigned int x,
                                       const unsigned int y,
                                       const unsigned int z, const time_t& t) {
  if (x >= size_x_ || y >= size_y_ || z >= size_z_) {
    LOG(ERROR) << "voxel index out of bounds. (" << x << ", " << y << ", " << z
               << ") size:(" << size_x_ << ", " << size_y_ << ", " << size_z_
               << ")";
  } else {
    int index_t = y * size_x_ + x;
    int index_m = index_t * size_z_ + z;
    // if ((*ptr_voxel_grid_data_)[index_m] < 5) {
    (*ptr_voxel_grid_data_)[index_m] =
        ptr_probability_table_->unknow((*ptr_voxel_grid_data_)[index_m]);
    // }
    (*ptr_voxel_cell_update_time_)[index_t] = t;
    // LOG(INFO) << "mark (" << x << ", " << y << ", " << z << ") - "
    //           << (*ptr_voxel_grid_data_)[index_m];
  }
}

void ProbabilityVoxelGrid::markVoxel(const unsigned int x, const unsigned int y,
                                     const unsigned int z, const time_t& t) {
  if (x >= size_x_ || y >= size_y_ || z >= size_z_) {
    LOG(ERROR) << "voxel index out of bounds. (" << x << ", " << y << ", " << z
               << ") size:(" << size_x_ << ", " << size_y_ << ", " << size_z_
               << ")";
  } else {
    int index_t = y * size_x_ + x;
    int index_m = index_t * size_z_ + z;
    // if ((*ptr_voxel_grid_data_)[index_m] < 5) {
    (*ptr_voxel_grid_data_)[index_m] =
        ptr_probability_table_->hit((*ptr_voxel_grid_data_)[index_m]);
    // }
    (*ptr_voxel_cell_update_time_)[index_t] = t;
    // LOG(INFO) << "mark (" << x << ", " << y << ", " << z << ") - "
    //           << (*ptr_voxel_grid_data_)[index_m];
    size_t hash_value = 137 * x + 149 * y + 163 * z;
    has_update_hash_.insert(hash_value);
  }
}

void ProbabilityVoxelGrid::unmarkVoxel(const unsigned int x,
                                       const unsigned int y,
                                       const unsigned int z, const time_t& t) {
  if (x >= size_x_ || y >= size_y_ || z >= size_z_) {
    LOG(ERROR) << "voxel index out of bounds. (" << x << ", " << y << ", " << z
               << ") size:(" << size_x_ << ", " << size_y_ << ", " << size_z_
               << ")";
  } else {
    size_t hash_value = 137 * x + 149 * y + 163 * z;
    if (has_update_hash_.find(hash_value) == has_update_hash_.end()) {
      has_update_hash_.insert(hash_value);
      int index_t = y * size_x_ + x;
      int index_m = index_t * size_z_ + z;
      (*ptr_voxel_grid_data_)[index_m] =
          ptr_probability_table_->miss((*ptr_voxel_grid_data_)[index_m]);
      (*ptr_voxel_cell_update_time_)[index_t] = t;
    }
  }
}

void ProbabilityVoxelGrid::unmarkVoxel(const unsigned int x,
                                       const unsigned int y, const time_t& t) {
  if (x >= size_x_ || y >= size_y_) {
    LOG(ERROR) << "voxel index out of bounds. (" << x << ", " << y << ") size:("
               << size_x_ << ", " << size_y_ << ")";
  } else {
    int index_t = y * size_x_ + x;
    for (unsigned z = 0; z < size_z_; z++) {
      size_t hash_value = 137 * x + 149 * y + 163 * z;
      if (has_update_hash_.find(hash_value) == has_update_hash_.end()) {
        has_update_hash_.insert(hash_value);
        int index_m = index_t * size_z_ + z;
        (*ptr_voxel_grid_data_)[index_m] =
            ptr_probability_table_->miss((*ptr_voxel_grid_data_)[index_m]);
        (*ptr_voxel_cell_update_time_)[index_t] = t;
      }
    }
  }
}

void ProbabilityVoxelGrid::updateTime(const unsigned int x,
                                      const unsigned int y, const time_t& t) {
  if (x >= size_x_ || y >= size_y_) {
    LOG(ERROR) << "voxel index out of bounds. (" << x << ", " << y << ") size:("
               << size_x_ << ", " << size_y_ << ")";
  } else {
    int index_t = y * size_x_ + x;
    (*ptr_voxel_cell_update_time_)[index_t] = t;
  }
}

void ProbabilityVoxelGrid::clearVoxel(const unsigned int x,
                                      const unsigned int y,
                                      const unsigned int z) {
  if (x >= size_x_ || y >= size_y_ || z >= size_z_) {
    LOG(ERROR) << "voxel index out of bounds. (" << x << ", " << y << ", " << z
               << ") size:(" << size_x_ << ", " << size_y_ << ", " << size_z_
               << ")";
  } else {
    int index_t = y * size_x_ + x;
    int index_m = index_t * size_z_ + z;
    (*ptr_voxel_grid_data_)[index_m] = 0;
  }
}

void ProbabilityVoxelGrid::clearVoxel(const unsigned int x,
                                      const unsigned int y) {
  if (x >= size_x_ || y >= size_y_) {
    LOG(ERROR) << "voxel index out of bounds. (" << x << ", " << y << ") size:("
               << size_x_ << ", " << size_y_ << ")";
  } else {
    int index_t = y * size_x_ + x;
    int index_xyz = index_t * size_z_;
    std::fill(ptr_voxel_grid_data_->begin() + index_xyz,
              ptr_voxel_grid_data_->begin() + index_xyz + size_z_,
              define_value);
  }
}

void ProbabilityVoxelGrid::clearByTime(const time_t& now_time,
                                       const double& clearing_time_threshold) {
  time_t clear_time = now_time - clearing_time_threshold;
  size_t index_y;
  size_t index_x;
  size_t index_z;
  for (unsigned int iy = 0; iy < size_y_; iy++) {
    index_y = iy * size_x_;
    for (unsigned int ix = 0; ix < size_x_; ix++) {
      index_x = index_y + ix;
      if ((*ptr_voxel_cell_update_time_)[index_x] < clear_time) {
        index_z = index_x * size_z_;
        std::fill(ptr_voxel_grid_data_->begin() + index_z,
                  ptr_voxel_grid_data_->begin() + index_z + size_z_,
                  define_value);
        (*ptr_voxel_cell_update_time_)[index_x] = now_time;
      }
    }
  }
}

float ProbabilityVoxelGrid::getValue(const unsigned int x, const unsigned int y,
                                     const unsigned int z) {
  if (x >= size_x_ || y >= size_y_ || z >= size_z_) {
    LOG(ERROR) << "voxel index out of bounds. (" << x << ", " << y << ", " << z
               << ") size:(" << size_x_ << ", " << size_y_ << ", " << size_z_
               << ")";
  } else {
    return ptr_probability_table_->valueToProbability(
        (*ptr_voxel_grid_data_)[(y * size_x_ + x) * size_z_ + z]);
  }
}

float ProbabilityVoxelGrid::getValue(const unsigned int x,
                                     const unsigned int y) {
  if (x >= size_x_ || y >= size_y_) {
    LOG(ERROR) << "voxel index out of bounds. (" << x << ", " << y << ") size:("
               << size_x_ << ", " << size_y_ << ")";
  } else {
    int index = (y * size_x_ + x) * size_z_;
    unsigned int max_value = 0;
    int index_m;
    for (unsigned z = 0; z < size_z_; z++) {
      index_m = index + z;
      if ((*ptr_voxel_grid_data_)[index_m] > max_value) {
        max_value = (*ptr_voxel_grid_data_)[index_m];
      }
    }
    // LOG(INFO) << "probability: " << max_value << " - "
    //           << ptr_probability_table_->valueToProbability(max_value);
    return ptr_probability_table_->valueToProbability(max_value);
  }
}

bool ProbabilityVoxelGrid::updateCostMap(
    boost::shared_array<unsigned char>& ptr_costmap,
    const unsigned char occ_status, const unsigned char unknow_status,
    const unsigned char free_status) {
  // if (ptr_costmap.size() != size_x_ * size_y_) {
  //   LOG(ERROR) << "costmap size != grid size";
  //   return false;
  // }
  unsigned int occ_value = ptr_probability_table_->probabilityToValue(0.65);
  unsigned int unknow_value = ptr_probability_table_->probabilityToValue(0.40);

  size_t index_x, index_y, index_z;
  unsigned int max_value;
  unsigned int value;
  for (unsigned int iy = 0; iy < size_y_; iy++) {
    index_y = iy * size_x_;
    for (unsigned int ix = 0; ix < size_x_; ix++) {
      index_x = index_y + ix;
      index_z = index_x * size_z_;

      max_value = 0;
      for (unsigned int iz = 0; iz < size_z_; iz++) {
        value = (*ptr_voxel_grid_data_)[index_z + iz];
        if (value > max_value) {
          max_value = value;
        }
      }
      if (max_value > occ_value) {
        ptr_costmap[index_x] = occ_status;
      } else if (value > unknow_value) {
        ptr_costmap[index_x] = unknow_status;
      } else {
        ptr_costmap[index_x] = free_status;
      }
    }
  }
  return true;
}

bool ProbabilityVoxelGrid::isViewed(const unsigned int x, const unsigned int y,
                                    const unsigned int z) {
  if (x >= size_x_ || y >= size_y_ || z >= size_z_) {
    LOG(ERROR) << "voxel index out of bounds. (" << x << ", " << y << ", " << z
               << ") size:(" << size_x_ << ", " << size_y_ << ", " << size_z_
               << ")";
  } else {
    if ((*ptr_voxel_grid_data_)[(y * size_x_ + x) * size_z_ + z] == 0) {
      return false;
    } else {
      return true;
    }
  }
  return false;
}

bool ProbabilityVoxelGrid::isViewed(const unsigned int x,
                                    const unsigned int y) {
  if (x >= size_x_ || y >= size_y_) {
    LOG(ERROR) << "voxel index out of bounds. (" << x << ", " << y << ") size:("
               << size_x_ << ", " << size_y_ << ")";
  } else {
    int index = (y * size_x_ + x) * size_z_;
    for (unsigned z = 0; z < size_z_; z++) {
      int index_m = index + z;
      if ((*ptr_voxel_grid_data_)[index_m] == 0) {
        return false;
      }
    }

    return true;
  }
}

time_t ProbabilityVoxelGrid::getUpdateTime(const unsigned int x,
                                           const unsigned int y) {
  if (x >= size_x_ || y >= size_y_) {
    LOG(ERROR) << "voxel index out of bounds. (" << x << ", " << y << ") size:("
               << size_x_ << ", " << size_y_ << ")";
    return time(NULL);
  } else {
    return (*ptr_voxel_cell_update_time_)[y * size_x_ + x];
  }
}

bool ProbabilityVoxelGrid::shifting(const int shift_x, const int shift_y) {
  if (shift_x == 0 && shift_y == 0) {
    return true;
  }
  if (std::abs(shift_y) >= size_y_ || std::abs(shift_x) >= size_x_) {
    LOG(ERROR) << " shifting error: shifting(" << shift_x << ", " << shift_y
               << ") size(" << size_x_ << ", " << size_y_ << ")";
    return false;
  }
  std::shared_ptr<std::vector<unsigned int>> ptr_tmp_voxel_grid_data =
      std::make_shared<std::vector<unsigned int>>();
  ptr_tmp_voxel_grid_data->resize(size_z_ * size_y_ * size_x_, define_value);

  std::shared_ptr<std::vector<time_t>> ptr_tmp_voxel_cell_update_time =
      std::make_shared<std::vector<time_t>>();
  time_t time_now = time(NULL);
  ptr_tmp_voxel_cell_update_time->resize(size_y_ * size_x_, time_now);

  for (unsigned int iy = 0; iy < size_y_; iy++) {
    int sy = iy + shift_y;
    if (sy >= 0 && sy < size_y_) {
      if (shift_x < 0) {
        std::copy(ptr_voxel_grid_data_->begin() + (sy * size_x_) * size_z_,
                  ptr_voxel_grid_data_->begin() +
                      (sy * size_x_ + size_x_ + shift_x) * size_z_,
                  ptr_tmp_voxel_grid_data->begin() +
                      (iy * size_x_ - shift_x) * size_z_);

        std::copy(
            ptr_voxel_cell_update_time_->begin() + sy * size_x_,
            ptr_voxel_cell_update_time_->begin() + sy * size_x_ + size_x_ +
                shift_x,
            ptr_tmp_voxel_cell_update_time->begin() + iy * size_x_ - shift_x);

      } else {
        std::copy(
            ptr_voxel_grid_data_->begin() + (sy * size_x_ + shift_x) * size_z_,
            ptr_voxel_grid_data_->begin() + (sy * size_x_ + size_x_) * size_z_,
            ptr_tmp_voxel_grid_data->begin() + (iy * size_x_) * size_z_);

        std::copy(ptr_voxel_cell_update_time_->begin() + sy * size_x_ + shift_x,
                  ptr_voxel_cell_update_time_->begin() + sy * size_x_ + size_x_,
                  ptr_tmp_voxel_cell_update_time->begin() + iy * size_x_);
      }
    }
  }
  ptr_voxel_grid_data_.reset();
  ptr_voxel_grid_data_ = ptr_tmp_voxel_grid_data;
  ptr_voxel_cell_update_time_.reset();
  ptr_voxel_cell_update_time_ = ptr_tmp_voxel_cell_update_time;
}

// TODO: 此处可优化, 增加每一个行x是否更新, 每个个xy是否更新,
// 每个xzy是否更新,多级判断, 减少多余遍历
void ProbabilityVoxelGrid::startUpate() { has_update_hash_.clear(); }

}  // namespace CVTE_BABOT