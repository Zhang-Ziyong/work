#ifndef _OCCUPANCY_GRID_MAP_
#define _OCCUPANCY_GRID_MAP_
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>
namespace slam2d_core {
namespace amcl {

/**
 * Cell
 * @brief 地图栅格元素
 **/
class Cell {
 public:
  int occ_state;    ///< 占据状态 -1-free, 0-unknown, 1-occupy
  double occ_dist;  ///< 离最近的被占据的点的距离
};

/**
 * CellData
 * @brief 用于计算 cache distance map
 **/
class CellData {
 public:
  unsigned int i;      ///< 更新目标点地图坐标
  unsigned int j;      ///< 更新目标点地图坐标
  unsigned int src_i;  ///< 比较点地图坐标
  unsigned int src_j;  ///< 比较点地图坐标
  double occ_dist;     ///< 比较距离

  /**
   * operator<
   * @brief
   *比较连个CellData的大小,由于在此部分需要使用降序排序，而保存的容器默认使用升序，所以大小相反
   * @return true- a最近被占据点距离大于b点，false- a最近被占据点距离小于b点
   **/
  bool operator<(const CellData &a) const {
    return this->occ_dist > a.occ_dist;
  }
};

/**
 * OccupancyGrid
 * @brief 2D占据栅格冬天
 * 1. 从文件读取地图和地图参数
 * 2. 计算cache distance map
 * 3. 地图中每个元素包括占据信息和与最近的被占据的点的距离
 *
 **/
class OccupancyGrid {
 public:
  /**
   * OccupancyGrid
   * @brief 构造函数
   **/
  OccupancyGrid();

  /**
   * OccupancyGrid
   * @brief 构造函数
   * @param[in] map_path-地图文件的文件地址
   * @param[in] max_occ_dist-计算cache distance
   * map的最大距离
   **/
  OccupancyGrid(const std::string &map_path, const double max_occ_dist);

  /**
   *  ~OccupancyGrid
   * @brief 析构函数
   **/
  ~OccupancyGrid();

  /**
   * readFromFile
   * @brief 从文件读取地图和地图参数
   * @param[in] map_path-地图文件的文件地址
   * @return true-读取成功，false-读取失败，可能原因：地图路径错误
   **/
  bool readFromFile(const std::string &map_path);

  /**
   * getCell
   * @brief 获得对应坐标的地图栅格元素；
   * @param[in] x-世界坐标系下的坐标x
   * @param[in] y-世界坐标系下的坐标y
   * @return 栅格地图上对应点栅格元素
   **/
  Cell getCell(const double x, const double y);

  /**
   * updataCspace
   * @brief 更新cache distance map
   * @param[in] max_occ_dist-每个栅格元素的最大搜索距离
   * @return true-更新成功，false-更新失败，可能原因：参数错误
   **/
  bool updataCspace(const double max_occ_dist);

  /**
   * mapClacRange
   * @brief 获得点(x,y)在yaw方向上最近的被占据的栅格点的距离
   * @param[in] x-世界坐标系下的坐标x
   * @param[in] y-世界坐标系下的坐标y
   * @param[in] yaw-世界坐标系下的角度yaw
   * @param[in] max_range-最大距离
   * @return 与最近的被占据的栅格点的距离
   **/
  double mapClacRange(const double x, const double y, const double yaw,
                      const double max_range);

  /**
   * isInFreeSpace
   * @brief 判断点(x,y)是否是free的状态
   * @param[in] x-世界坐标系下的坐标x
   * @param[in] y-世界坐标系下的坐标y
   * @return true-该点为free, false-该点为unknown或occupy
   **/
  bool isInFreeSpace(const double x, const double y);

  /**
   * clear
   * @brief 清除当前状态
   **/
  void clear();

  /**
   * getMaxOccDist
   * @brief 返回max_occ_dist_参数
   * @return 返回max_occ_dist_参数
   **/
  double getMaxOccDist();

  /**
   * worldValid
   * @brief 判断此世界坐标是否有效坐标
   * @param[in] i-世界坐标上的x轴坐标
   * @param[in] j-世界坐标上的y轴坐标
   * @return true-有效，false-无效，原因：超过地图有效范围
   **/
  inline bool worldValid(const double x, const double y) {
    unsigned int i = getMapCoordX(x);
    unsigned int j = getMapCoordY(y);
    return mapValid(i, j);
  }

  /**
   * getResolution
   * @brief 获得地图分辨率
   * @return 地图分辨率
   **/
  inline double getResolution() { return scale_; }

  /**
   * getSizeX
   * @brief 获得地图X方向大小
   * @return 地图X方向大小
   **/
  inline unsigned int getSizeX() { return size_x_; }

  /**
   * getSizeX
   * @brief 获得地图Y方向大小
   * @return 地图Y方向大小
   **/
  inline unsigned int getSizeY() { return size_y_; }

  /**
   * getOriginX
   * @brief 获得地图原点对应的世界坐标x
   * @return 地图原点对应的世界坐标x
   **/
  inline double getOriginX() { return origin_x_; }

  /**
   * getOriginY
   * @brief 获得地图原点对应的世界坐标y
   * @return 地图原点对应的世界坐标y
   **/
  inline double getOriginY() { return origin_y_; }

  /**
   * getOriginYaw
   * @brief 获得地图原点对应的世界坐标yaw
   * @return 地图原点对应的世界坐标yaw
   **/
  inline double getOriginYaw() { return origin_a_; }

  /**
   * getCells
   * @brief 获得地图数据
   * @return 地图数据
   **/
  inline const std::vector<std::vector<Cell>> &getCells() { return cells_; }

  inline double getOccDist(const double x, const double y) {
    unsigned int index_x = getMapCoordX(x);
    unsigned int index_y = getMapCoordY(y);
    if (mapValid(index_x, index_y)) {
      return cells_[index_x][index_y].occ_dist;
    }

    return max_occ_dist_;
  }

  inline void setResolution(const double &scale) { scale_ = scale; }
  inline void setSizeX(const double &size_x) { size_x_ = size_x; }
  inline void setSizeY(const double &size_y) { size_y_ = size_y; }
  inline void setOriginX(const double &origin_x) { origin_x_ = origin_x; }
  inline void setOriginY(const double &origin_y) { origin_y_ = origin_y; }
  inline void setOriginYaw(const double &origin_a) { origin_a_ = origin_a; }
  inline void setMaxoccdist(const double &max_occ_dist) {
    max_occ_dist_ = max_occ_dist;
  }

  // reset cells_
  inline void reseizeMapCells() {
    cells_.resize(size_x_);
    for (size_t i = 0; i < size_x_; ++i) { cells_[i].resize(size_y_); }
  }

  // 设置
  inline void setMapCells(const double &cell_x, const double &cell_y,
                          const Cell &cell) {
    unsigned int localmap_cell_x = getMapCoordX(cell_x);
    unsigned int localmap_cell_y = getMapCoordY(cell_y);

    if (mapValid(localmap_cell_x, localmap_cell_y)) {
      cells_[localmap_cell_x][localmap_cell_y] = cell;
    }
  }

 private:
  /**
   * getWorldCoordX
   * @brief 将地图的x轴坐标转换为世界坐标
   * @param[in] mx-地图上的x轴坐标
   * @return 世界坐标系下x轴坐标
   **/
  inline double getWorldCoordX(const unsigned int mx) {
    // return (origin_x_ + (mx - size_x_ / 2)) * scale_;
    return (origin_x_ + mx * scale_);
  }

  /**
   * getWorldCoordY
   * @brief 将地图的y轴坐标转换为世界坐标
   * @param[in] my-地图上的x轴坐标
   * @return 世界坐标系下y轴坐标
   **/
  inline double getWorldCoordY(const unsigned int my) {
    // return (origin_y_ + (my - size_y_ / 2)) * scale_;
    return (origin_y_ + my * scale_);
  }

  /**
   * getMapCoordX
   * @brief 将世界坐标下的x轴坐标转换为地图坐标系下的x轴坐标
   * @param[in] wx-世界坐标系中的x轴坐标
   * @return 地图坐标系下x轴坐标
   **/
  inline unsigned int getMapCoordX(const double wx) {
    // return std::floor((wx - origin_x_) / scale_ - 0.5) + size_x_ / 2;
    return std::floor((wx - origin_x_) / scale_);
  }

  /**
   * getMapCoordY
   * @brief 将世界坐标下的y轴坐标转换为地图坐标系下的y轴坐标
   * @param[in] wy-世界坐标系中的x轴坐标
   * @return 地图坐标系下y轴坐标
   **/
  inline unsigned int getMapCoordY(const double wy) {
    // return std::floor((wy - origin_y_) / scale_ - 0.5) + size_y_ / 2;
    return std::floor((wy - origin_y_) / scale_);
  }

  /**
   * mapValid
   * @brief 判断此地图坐标是否有效坐标
   * @param[in] i-地图上的x轴坐标
   * @param[in] j-地图上的y轴坐标
   * @return true-有效，false-无效，原因：超过地图有效范围
   **/
  inline bool mapValid(const unsigned int i, const unsigned int j) {
    return (i < size_x_) && (j < size_y_);
  }

  /**
   * enqueue
   * @brief 计算栅格点的里最近被占据点距离
   * @param[in] i-目标点在地图上的x轴坐标
   * @param[in] j-目标点在地图上的y轴坐标
   * @param[in] src_i-比较点在地图上的x轴坐标
   * @param[in] src_j-比较点在地图上的y轴坐标
   * @param[in] cdm-存储距离计算的缓存，用于查询两个栅格点的距离
   * @param[in] Q-未完成比较的数据
   * @param[in] marked-标记栅格点是否已经被处理过
   **/
  void enqueue(const unsigned int i, const unsigned int j,
               const unsigned int src_i, const unsigned int src_j,
               std::vector<std::vector<double>> &cdm,
               std::priority_queue<CellData> &Q,
               std::vector<std::vector<unsigned char>> &marked);

  double origin_x_;  ///< 地图0点对应的世界坐标系下x轴坐标
  double origin_y_;  ///< 地图0点对应的世界坐标系下y轴坐标
  double origin_a_;  ///< 地图坐标系与世界坐标系的转换角度，不使用为0

  double scale_;         ///< 地图分辨率
  unsigned int size_x_;  ///< 地图在x方向的大小
  unsigned int size_y_;  ///< 地图在y方向的大小
  double max_occ_dist_;  ///< 计算cache distance map的最大比较距离
  int cell_redius_;  ///< 计算cache distance map的最大比较的栅格距离

  std::string map_filename_;  ///< 地图文件路径

  std::vector<std::vector<Cell>> cells_;  ///< 保存地图栅格点的容器
};

}  // namespace amcl
}  // namespace slam2d_core

#endif