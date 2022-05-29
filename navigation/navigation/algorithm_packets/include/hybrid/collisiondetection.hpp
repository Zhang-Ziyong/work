#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <boost/smart_ptr.hpp>
#include "constants.hpp"
#include "lookup.hpp"
#include "node2d.hpp"
#include "node3d.hpp"

namespace CVTE_BABOT {
/*!
   \brief The CollisionDetection class determines whether a given configuration
   q of the robot will result in a collision with the environment.

   It is supposed to return a boolean value that returns true for collisions and
   false in the case of a safe node.
*/
class CollisionDetection {
 public:
  /// Constructor
  CollisionDetection(const int &width, const int &height);

  /*!
\brief evaluates whether the configuration is safe
\return true if it is traversable, else false
*/
  template <typename T>
  bool isTraversable(const T *node) {
    /* Depending on the used collision checking mechanism this needs to be
adjusted standard: collision checking using the spatial occupancy enumeration
other: collision checking using the 2d costmap and the navigation stack
*/
    float cost = 0;
    float x;
    float y;
    float t;
    // assign values to the configuration
    getConfiguration(node, x, y, t);

    // 2D collision test
    if (t == 99) {
      return !costmap_[node->getIdx()];
    }

    if (true) {
      cost = configurationTest(x, y, t) ? 0 : 1;
    } else {
      cost = configurationCost();
    }

    return cost <= 0;
  }

  /*!
  \brief Calculates the cost of the robot taking a specific configuration q int
  the World W \param x the x position \param y the y position \param t the
  theta angle \return the cost of the configuration q of W(q) \todo needs to be
  implemented correctly
  */
  float configurationCost() { return 0; }

  /*!
  \brief Tests whether the configuration q of the robot is in C_free
  \param x the x position
  \param y the y position
  \param t the theta angle
  \return true if it is in C_free, else false
  */
  bool configurationTest(float x, float y, float t);

  /*!
  \brief updates the grid with the world map
  */
  void updateGrid(boost::shared_array<unsigned char> &costs) {
    costmap_ = costs;
  }

 private:
  void getConfiguration(const Node2D *node, float &x, float &y, float &t) {
    x = node->getX();
    y = node->getY();
    // avoid 2D collision checking
    t = 99;
  }

  void getConfiguration(const Node3D *node, float &x, float &y, float &t) {
    x = node->getX();
    y = node->getY();
    t = node->getT();
  }

  /// The occupancy grid
  boost::shared_array<unsigned char> costmap_;

  /// The collision lookup table
  Constants::config collisionLookup[Constants::headings * Constants::positions];

  int width_ = 0;
  int height_ = 0;
};
}  // namespace CVTE_BABOT
#endif  // COLLISIONDETECTION_H
