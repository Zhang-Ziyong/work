/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, CVTE.
 * All rights reserved.
 *
 *@file test_voxel_grid.cpp
 *
 *@brief test voxel grid.
 *
 *@modified by wuhuabo(wuhuabo@cvte.com)
 *
 *@author wuhuabo(wuhuabo@cvte.com)
 *@version current_algo.dev.1.3
 *@data 2019-08-23
 ************************************************************************/
#include <gtest/gtest.h>
#include <voxel_grid.hpp>

namespace CVTE_BABOT {

TEST(voxel_grid, basicMarkingAndClearing) {
  int size_x = 50, size_y = 10, size_z = 16;
  VoxelGrid vg(size_x, size_y, size_z);

  // Put a "tabletop" into the scene.  A flat rectangle of set voxels at z = 12.
  int table_z = 12;
  int table_x_min = 5, table_x_max = 15;
  int table_y_min = 0, table_y_max = 3;
  for (int x = table_x_min; x <= table_x_max; x++) {
    vg.markVoxelLine({x, table_y_min, table_z}, {x, table_y_max, table_z});
  }

  for (int i = table_x_min; i <= table_x_max; ++i) {
    for (int j = table_y_min; j <= table_y_max; ++j) {
      // check that each cell of the table is marked
      ASSERT_EQ(MARKED, vg.getVoxel({i, j, table_z}));
    }
  }

  int mark_count = 0;
  unsigned int unknown_count = 0;
  // go through each cell in the voxel grid and make sure that only 44 are
  // filled in
  for (unsigned int i = 0; i < vg.sizeX(); ++i) {
    for (unsigned int j = 0; j < vg.sizeY(); ++j) {
      for (unsigned int k = 0; k < vg.sizeZ(); ++k) {
        if (vg.getVoxel({i, j, k}) == MARKED) {
          mark_count++;
        } else if (vg.getVoxel({i, j, k}) == UNKNOWN) {
          unknown_count++;
        }
      }
    }
  }
  ASSERT_EQ(mark_count, 44);

  // the rest of the cells should be unknown
  ASSERT_EQ(unknown_count, vg.sizeX() * vg.sizeY() * vg.sizeZ() - 44);

  // now, let's clear one of the rows of the table
  vg.clearVoxelLine({table_x_min, table_y_min, table_z},
                    {table_x_max, table_y_min, table_z});

  mark_count = 0;
  unknown_count = 0;
  int free_count = 0;
  // go through each cell in the voxel grid and make sure that only 33 are now
  // filled in
  for (unsigned int i = 0; i < vg.sizeX(); ++i) {
    for (unsigned int j = 0; j < vg.sizeY(); ++j) {
      for (unsigned int k = 0; k < vg.sizeZ(); ++k) {
        if (vg.getVoxel({i, j, k}) == MARKED) {
          mark_count++;
        } else if (vg.getVoxel({i, j, k}) == FREE) {
          free_count++;
        } else if (vg.getVoxel({i, j, k}) == UNKNOWN) {
          unknown_count++;
        }
      }
    }
  }

  // we've switched 11 cells from marked to free
  ASSERT_EQ(mark_count, 33);

  // we've just explicitly seen through 11 cells
  ASSERT_EQ(free_count, 11);

  // the rest of the cells should still be unknown
  ASSERT_EQ(unknown_count, vg.sizeX() * vg.sizeY() * vg.sizeZ() - 44);

  // now let's put in a vertical column manually to test markVoxel
  for (unsigned int i = 0; i < vg.sizeZ(); ++i) {
    vg.markVoxel({0, 0, i});
    ASSERT_EQ(vg.getVoxel({0, 0, i}), MARKED);
  }

  vg.printColumnGrid();
  vg.printVoxelGrid();

  // now, let's clear that line of voxels and make sure that they clear out OK
  vg.clearVoxelLine({0, 0, 0}, {0, 0, vg.sizeZ() - 1});

  for (unsigned int i = 0; i < vg.sizeZ(); ++i) {
    ASSERT_EQ(vg.getVoxel({0, 0, i}), FREE);
  }

  mark_count = 0;

  // Visualize the output
  /*
     v->printVoxelGrid();
     v->printColumnGrid();

     printf("CostMap:\n===========\n");
     for(int y = 0; y < size_y; y++){
     for(int x = 0; x < size_x; x++){
     printf((costMap[y * size_x + x] > 0 ? "#" : " "));
     }printf("|\n");
     }
     */
}
}  // namespace CVTE_BABOT
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
