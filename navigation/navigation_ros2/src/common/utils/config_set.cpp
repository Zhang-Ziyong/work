#include "common/utils/config_set.hpp"

#include <glog/logging.h>

ConfigSet::ConfigSet() : gsettings_("org.babot.setting") {
  initSimulationConfig();
}

ConfigSet &ConfigSet::getInstance() {
  static ConfigSet config;
  return config;
}

bool ConfigSet::isSimulation() {
  std::lock_guard<std::mutex> lock(mutex_);
  return is_simulation_;
}

void ConfigSet::initSimulationConfig() {
  std::lock_guard<std::mutex> lock(mutex_);

  try {
    gsettings_.getSetting("enable-simulation", is_simulation_);
  } catch (...) { LOG(ERROR) << "initSimulationConfig failed!"; }

  LOG(INFO) << "is_simulation=" << is_simulation_;
}