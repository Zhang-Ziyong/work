/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *
 *@file state_utils.hpp
 *
 *@brief hpp模板
 *
 *@author huabo wu(wuhuabo@cvte.com)
 *@modified wuhuabo (wuhuabo@cvte.com)
 *@version current_algo.dev.3.0
 *@data 2020-09-24
 ************************************************************************/
#ifndef __STATE_UTILS_HPP
#define __STATE_UTILS_HPP
#include "state.hpp"

namespace CVTE_BABOT {
std::shared_ptr<State> getStateInstance(const PathFollowerStates &state);
}  // namespace CVTE_BABOT
#endif
