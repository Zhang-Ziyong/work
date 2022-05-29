#ifndef __CONFIG_SET_HPP_
#define __CONFIG_SET_HPP_

#include "data_storage/gsetting/GSettingsWraper.hpp"

#include <mutex>

class ConfigSet {
 public:
  static ConfigSet &getInstance();

  bool isSimulation();

 private:
  ConfigSet();
  ~ConfigSet() {}

  void initAutoChargeConfig();
  void initSimulationConfig();

  ConfigSet(const ConfigSet &) = delete;
  ConfigSet &operator=(const ConfigSet &) = delete;

 private:
  GSettingsWraper gsettings_;
  bool is_simulation_;

  std::mutex mutex_;
};

#endif