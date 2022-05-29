#ifndef PROBABILITY_VOXEL_GRID_HPP
#define PROBABILITY_VOXEL_GRID_HPP
#include <time.h>

#include <boost/smart_ptr.hpp>
#include <memory>
#include <probability_table.hpp>
#include <unordered_set>
#include <vector>
namespace CVTE_BABOT {

// TODO: 写注释
class ProbabilityVoxelGrid {
 public:
  ProbabilityVoxelGrid(const unsigned int size_x, const unsigned int size_y,
                       const unsigned int size_z, const double res_x,
                       const double res_y, const double res_z);
  ~ProbabilityVoxelGrid();

  ProbabilityVoxelGrid(const ProbabilityVoxelGrid &obj) = delete;
  ProbabilityVoxelGrid &operator=(const ProbabilityVoxelGrid &obj) = delete;

  void reset();

  void resize(const unsigned int size_x, const unsigned int size_y,
              const unsigned int size_z);
  void unkownVoxel(const unsigned int x, const unsigned int y,
                   const unsigned int z, const time_t &t);
  void markVoxel(const unsigned int x, const unsigned int y,
                 const unsigned int z, const time_t &t);
  void updateTime(const unsigned int x, const unsigned int y, const time_t &t);
  void unmarkVoxel(const unsigned int x, const unsigned int y,
                   const unsigned int z, const time_t &t);

  void unmarkVoxel(const unsigned int x, const unsigned int y, const time_t &t);

  void clearVoxel(const unsigned int x, const unsigned int y,
                  const unsigned int z);
  void clearVoxel(const unsigned int x, const unsigned int y);

  float getValue(const unsigned int x, const unsigned int y,
                 const unsigned int z);
  float getValue(const unsigned int x, const unsigned int y);

  bool isViewed(const unsigned int x, const unsigned int y);
  bool isViewed(const unsigned int x, const unsigned int y,
                const unsigned int z);

  bool shifting(const int shift_x, const int shift_y);

  void startUpate();

  bool updateCostMap(boost::shared_array<unsigned char> &ptr_costmap,
                     const unsigned char occ_status,
                     const unsigned char unknow_status,
                     const unsigned char free_status);

  void clearByTime(const time_t &now_time,
                   const double &clearing_time_threshold);

  time_t getUpdateTime(const unsigned int x, const unsigned int y);

  inline unsigned int sizeX() { return size_x_; }
  inline unsigned int sizeY() { return size_y_; }
  inline unsigned int sizeZ() { return size_z_; }
  inline double resX() { return res_x_; }
  inline double resY() { return res_y_; }
  inline double resZ() { return res_z_; }

 private:
  std::shared_ptr<std::vector<unsigned int>> ptr_voxel_grid_data_;
  std::shared_ptr<std::vector<time_t>> ptr_voxel_cell_update_time_;
  std::shared_ptr<ProbabilityTable> ptr_probability_table_;

  double res_x_;
  double res_y_;
  double res_z_;
  unsigned int size_x_;
  unsigned int size_y_;
  unsigned int size_z_;

  std::unordered_set<size_t> has_update_hash_;

  unsigned int define_value = 0;
};
}  // namespace CVTE_BABOT
#endif