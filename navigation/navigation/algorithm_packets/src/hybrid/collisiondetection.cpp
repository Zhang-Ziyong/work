#include "hybrid/collisiondetection.hpp"

using namespace CVTE_BABOT;

CollisionDetection::CollisionDetection(const int &width, const int &height) {
  width_ = width;
  height_ = height;
  this->costmap_ = nullptr;
  Lookup::collisionLookup(collisionLookup);
}

bool CollisionDetection::configurationTest(float x, float y, float t) {
  int X = (int)x;
  int Y = (int)y;
  int iX = (int)((x - (long)x) * Constants::positionResolution);
  iX = iX > 0 ? iX : 0;
  int iY = (int)((y - (long)y) * Constants::positionResolution);
  iY = iY > 0 ? iY : 0;
  int iT = (int)(t / Constants::deltaHeadingRad);
  int idx = iY * Constants::positionResolution * Constants::headings +
            iX * Constants::headings + iT;
  int cX;
  int cY;

  for (int i = 0; i < collisionLookup[idx].length; ++i) {
    cX = (X + collisionLookup[idx].pos[i].x);
    cY = (Y + collisionLookup[idx].pos[i].y);

    // make sure the configuration coordinates are actually on the grid
    if (cX >= 0 && cX < width_ && cY >= 0 && cY < height_) {
      if (costmap_[cY * width_ + cX]) {
        return false;
      }
    }
  }

  return true;
}
