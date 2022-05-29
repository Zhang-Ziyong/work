/************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2020, CVTE.
 * All rights reserved.
 *

 * @FilePath: /full_coverage_planner/ros_adater/glog_setup.h
 * @brief:
 * @
 * @
 * @author: Wei Zhang(zhangwei@cvte.com)
 * @version: v 1.0
 * @Date: 2020-11-19 15:55:04
 ************************************************************************/

#include <pwd.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <iostream>
#include <user_glog.hpp>
static void initGoogleLog(const std::string &module,
                          const std::string &log_level) {
  if (log_level.empty()) {
    std::cout << "please set log level.(error/warn/info)" << std::endl;
    return;
  }
  FLAGS_colorlogtostderr = true;

  google::InitGoogleLogging(module.c_str());
  google::InstallFailureSignalHandler();

  std::cout << "glog level: " << log_level << std::endl;
  if (!log_level.compare("error")) {
    google::SetStderrLogging(google::ERROR);
  } else if (!log_level.compare("warn")) {
    google::SetStderrLogging(google::WARNING);
  } else if (!log_level.compare("info")) {
    google::SetStderrLogging(google::INFO);
  } else if (!log_level.compare("fatal")) {
    google::SetStderrLogging(google::FATAL);
  } else {
    std::cout << "error log mode. (error/warn/info/fatal)" << std::endl;
  }

  std::string LogHomeDir = "log_info/";

  char current_path[500] = {'\0'};
  if (getcwd(current_path, 500) == NULL) {
    LOG(FATAL) << "get pwd fail!!";
  }

  if (strncmp(current_path, "/tmp", 4) == 0) {  // appimage部署方式
    struct passwd *pw;
    pw = getpwuid(getuid());
    std::string data_folder_path = std::string("/home/") +
                                   std::string(pw->pw_name) +
                                   std::string("/Development/");

    if (access(data_folder_path.c_str(), F_OK) != 0) {
      if (mkdir(data_folder_path.c_str(), S_IRWXU) != 0) {
        LOG(FATAL) << "mkdir " << data_folder_path << " fail!!";
      }
    }

    LogHomeDir = data_folder_path + LogHomeDir;
  }

  if (access(LogHomeDir.c_str(), F_OK)) {
    std::cerr << "loghomedir not exist, will create!" << std::endl;
    mkdir(LogHomeDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  }

  const std::string LogDir = LogHomeDir + module + "/";
  if (access(LogDir.c_str(), F_OK)) {
    std::cerr << "logdir not exist, will create!" << std::endl;
    mkdir(LogDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  }

  const std::string infoLogDir = LogHomeDir + module + "/info/";
  if (access(infoLogDir.c_str(), F_OK)) {
    std::cerr << "infologdir not exist, will create!" << std::endl;
    mkdir(infoLogDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  }

  const std::string warnLogDir = LogHomeDir + module + "/warn/";
  if (access(warnLogDir.c_str(), F_OK)) {
    std::cerr << "warnlogdir not exist, will create!" << std::endl;
    mkdir(warnLogDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  }

  const std::string errorLogDir = LogHomeDir + module + "/error/";
  if (access(errorLogDir.c_str(), F_OK)) {
    std::cerr << "errorlogdir not exist, will create!" << std::endl;
    mkdir(errorLogDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  }

  const std::string fatalLogDir = LogHomeDir + module + "/fatal/";
  if (access(fatalLogDir.c_str(), F_OK)) {
    std::cerr << "fatallogdir not exist, will create!" << std::endl;
    mkdir(fatalLogDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  }

  google::SetLogDestination(google::GLOG_INFO, infoLogDir.c_str());
  google::SetLogDestination(google::GLOG_WARNING, warnLogDir.c_str());
  google::SetLogDestination(google::GLOG_ERROR, errorLogDir.c_str());
  google::SetLogDestination(google::GLOG_FATAL, fatalLogDir.c_str());
}