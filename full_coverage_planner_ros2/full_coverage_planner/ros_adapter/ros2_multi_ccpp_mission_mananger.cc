#include "ros2_multi_ccpp_mission_mananger.hpp"
#include <jsoncpp/json/json.h>
#include <unistd.h>
#include <boost/filesystem.hpp>

#include "cpp_interface/edge_path_planner.hpp"
#include "cpp_interface/multi_areas_ccpp.hpp"
#include "cpp_interface/welt_path_planner.hpp"
#include "glog/logging.h"

MultiCCPPMissionManager::MultiCCPPMissionManager(rclcpp::Node::SharedPtr node) {
  node_ = node;
  mission_manager_server_ =
      node_->create_service<mission_manager_msgs::srv::MissionManager>(
          "multi_area_ccpp_mission_manager",
          std::bind(&MultiCCPPMissionManager::missionManagerCallback, this,
                    std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3));
  loadParamters();

  LOG(INFO) << "start multi area ccpp mission manager";
}
void MultiCCPPMissionManager::loadParamters() {
  double wall_distance;
  int decomposition_type_int;
  int cost_function_type_int;
  double v_max;
  double a_max;
  bool offset_polygons;
  bool sweep_single_direction;
  int sensor_model_type;
  double lateral_overlap;
  double lateral_footprint;
  double lateral_fov;
  double waypoint_resolution;
  double altitude;
  double changeColosCost;
  double turn_offset;
  double obs_offset;
  double inflation_dis;
  double dis_of_path;

  node_->get_parameter_or("decomposition_type", decomposition_type_int, 0);
  node_->get_parameter_or("cost_function_type", cost_function_type_int, 1);
  node_->get_parameter_or("v_max", v_max, 1.0);
  node_->get_parameter_or("a_max", a_max, 1.0);
  node_->get_parameter_or("wall_distance", wall_distance, 0.5);
  node_->get_parameter_or("offset_polygons", offset_polygons, false);
  node_->get_parameter_or("sweep_single_direction", sweep_single_direction,
                          false);
  node_->get_parameter_or("sensor_model_type", sensor_model_type, 0);
  node_->get_parameter_or("lateral_overlap", lateral_overlap, 0.1);
  node_->get_parameter_or("lateral_footprint", lateral_footprint, 0.5);
  node_->get_parameter_or("lateral_fov", lateral_fov, 0.5);
  node_->get_parameter_or("waypoint_resolution", waypoint_resolution, 0.5);
  node_->get_parameter_or("altitude", altitude, 0.0);
  node_->get_parameter_or("changeColosCost", changeColosCost, 10.0);
  node_->get_parameter_or("turn_offset", turn_offset, 0.9);
  node_->get_parameter_or("obs_offset", obs_offset, 0.1);
  node_->get_parameter_or("inflation_dis", inflation_dis, 0.5);
  node_->get_parameter_or("dis_of_path", dis_of_path, 0.4);

  LOG(INFO) << "decomposition_type_int: " << decomposition_type_int;
  LOG(INFO) << "cost_function_type_int: " << cost_function_type_int;
  LOG(INFO) << "v_max: " << v_max;
  LOG(INFO) << "a_max: " << a_max;
  LOG(INFO) << "wall_distance: " << wall_distance;
  LOG(INFO) << "offset_polygons: " << offset_polygons;
  LOG(INFO) << "sweep_single_direction: " << sweep_single_direction;
  LOG(INFO) << "sensor_model_type: " << sensor_model_type;
  LOG(INFO) << "lateral_overlap: " << lateral_overlap;
  LOG(INFO) << "lateral_footprint: " << lateral_footprint;
  LOG(INFO) << "lateral_fov: " << lateral_fov;
  LOG(INFO) << "waypoint_resolution: " << waypoint_resolution;
  LOG(INFO) << "altitude: " << altitude;
  LOG(INFO) << "changeColosCost: " << changeColosCost;
  LOG(INFO) << "turn_offset: " << turn_offset;
  LOG(INFO) << "obs_offset: " << obs_offset;
  LOG(INFO) << "inflation_dis: " << inflation_dis;
  LOG(INFO) << "dis_of_path: " << dis_of_path;

  // config_.offset_polygons_ = offset_polygons;
  // config_.sweep_single_direction_ = sweep_single_direction;
  // config_.lateral_footprint_ = std::make_optional<double>(lateral_footprint);
  // config_.lateral_overlap_ = std::make_optional<double>(lateral_overlap);
  // config_.lateral_fov_ = std::make_optional<double>(lateral_fov);
  // config_.decomposition_type_int_ = decomposition_type_int;
  // config_.sensor_model_type_int_ = sensor_model_type;
  // config_.cost_type_int_ = cost_function_type_int;
  // config_.wall_distance_ = wall_distance;
  // config_.waypoint_resolution_ = waypoint_resolution;
  // config_.altitude_ = std::make_optional<double>(altitude);
  // config_.v_max_ = std::make_optional<double>(v_max);
  // config_.a_max_ = std::make_optional<double>(a_max);
  // config_.save_trajectry_ = true;
  // config_.optimaze_trajectry_ = false;
  config_.changeColosCost = changeColosCost;
  config_.turn_offset = turn_offset;
  config_.obs_offset = obs_offset;
  config_.inflation_dis = inflation_dis;
  config_.dis_of_path = dis_of_path;
}

void MultiCCPPMissionManager::missionManagerCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<mission_manager_msgs::srv::MissionManager::Request>
        request,
    const std::shared_ptr<mission_manager_msgs::srv::MissionManager::Response>
        response) {
  missionHandler(request->send_data, response->ack_data);
}

void MultiCCPPMissionManager::missionHandler(std::string rev_data,
                                             std::string &ack_data) {
  LOG(INFO) << "get cmd: " << rev_data;
  Json::Reader json_reader;
  Json::Value json_rev_data;
  Json::Value json_ack_data;

  if (!json_reader.parse(rev_data, json_rev_data)) {
    LOG(ERROR) << "rev_data is illegal!!";
    json_ack_data["error_msgs"] = " rev_data is illegal!!";
    json_ack_data["succeed"] = false;
    ack_data = json_ack_data.toStyledString();
    return;
  }
  if (json_rev_data["cmd"].asString() == "coverage_path_planning") {
  } else if (json_rev_data["cmd"].asString() == "close_path_planning") {
  } else if (json_rev_data["cmd"].asString() ==
             "coverage_path_and_close_path_planning") {
    fullCoveragerPlanner(rev_data, ack_data);
  } else {
    json_ack_data["error_msgs"] = " cmd wrong";
    json_ack_data["succeed"] = false;
    ack_data = json_ack_data.toStyledString();
  }
  LOG(INFO) << "send result: " << ack_data;
}

void MultiCCPPMissionManager::fullCoveragerPlanner(std::string rev_data,
                                                   std::string &ack_data) {
  Json::Reader json_reader;
  Json::Value json_rev_data;
  Json::Value json_ack_data;

  if (!json_reader.parse(rev_data, json_rev_data)) {
    LOG(ERROR) << "rev_data is illegal!!";
    json_ack_data["error_msgs"] = " rev_data is illegal!!";
    json_ack_data["succeed"] = false;
    ack_data = json_ack_data.toStyledString();
    return;
  }

  std::string map_path = json_rev_data["map_path"].asString();
  std::string save_path = json_rev_data["coverage_path_save_path"].asString();
  std::string close_path_save_path =
      json_rev_data["close_path_save_path"].asString();

  // chack map
  if (access(map_path.c_str(), F_OK)) {
    LOG(ERROR) << "map: " << map_path << " didn`t exist!!";
    json_ack_data["error_msgs"] = "map didn`t exist!!";
    json_ack_data["succeed"] = false;
    ack_data = json_ack_data.toStyledString();
    return;
  }

  int model_type = json_rev_data["type"].asInt();

  std::vector<PathPointWithType> close_welt_path;
  std::vector<Point> path_points;
  switch (model_type) {
    case 0: {
      LOG(INFO) << "makePlan end，fault case 0";
    } break;
    case 1: {
      LOG(INFO) << "makePlan end，fault case 1";
    } break;

    case 2: {  // multi map areas bons ccpp

      std::vector<std::vector<AreaVertex>> areas;
      std::vector<AreaVertex> area_points;
      AreaVertex area_vertex;

      for (Json::ArrayIndex i = 0; i < json_rev_data["areas"].size(); i++) {
        area_points.clear();
        for (Json::ArrayIndex j = 0;
             j < json_rev_data["areas"][i]["point"].size(); j++) {
          area_vertex.x = json_rev_data["areas"][i]["point"][j]["x"].asDouble();
          area_vertex.y = json_rev_data["areas"][i]["point"][j]["y"].asDouble();
          area_points.push_back(area_vertex);
        }
        areas.push_back(area_points);
      }
      std::vector<std::vector<AreaVertex>> welts;
      std::vector<AreaVertex> welt;
      AreaVertex welt_vertex;
      for (Json::ArrayIndex i = 0; i < json_rev_data["welts"].size(); i++) {
        welt.clear();
        for (Json::ArrayIndex j = 0;
             j < json_rev_data["welts"][i]["point"].size(); j++) {
          welt_vertex.x = json_rev_data["welts"][i]["point"][j]["x"].asDouble();
          welt_vertex.y = json_rev_data["welts"][i]["point"][j]["y"].asDouble();
          welt.push_back(welt_vertex);
        }
        welts.push_back(welt);
      }

      MultiAreasCCPP ccpp;
      ccpp.readMap(map_path);
      ccpp.addWelts(welts);
      ccpp.generateCoveragePathByAreas(path_points, areas, config_);
      // ccpp.savaAsImage(path_points, "ccpp.png");
      ccpp.savaAsYaml(path_points, save_path);

      // WeltPathPlanner wpp;
      // wpp.readMap(map_path);
      // wpp.addWelts(welts);
      // wpp.generateWeltPathByAreas(close_welt_path, areas);
      // wpp.saveAsImage(close_welt_path);
      // wpp.savePath(close_welt_path, close_path_save_path);

      EdgePlanner epp;
      epp.readMap(map_path);
      epp.addWelts(welts);
      epp.generateEdgePathByAreas(close_welt_path, areas);
      epp.saveAsImage(close_welt_path);
      epp.savePath(close_welt_path, close_path_save_path);
    } break;

    default: {
      LOG(ERROR) << "type  wrong!!";
      json_ack_data["error_msgs"] = " cmd  wrong!!";
      json_ack_data["succeed"] = false;
      ack_data = json_ack_data.toStyledString();
      return;
    } break;
  }

  if (path_points.size() > 2) {
    json_ack_data["coverage_path_start_point_x"] = path_points[0].x;
    json_ack_data["coverage_path_start_point_y"] = path_points[0].y;
    json_ack_data["coverage_path_start_point_yaw"] = path_points[0].yaw;
    json_ack_data["coverage_path_start_point_index"] = 0;
    json_ack_data["coverage_path_end_point_x"] = path_points.back().x;
    json_ack_data["coverage_path_end_point_y"] = path_points.back().y;
    json_ack_data["coverage_path_end_point_yaw"] = path_points.back().yaw;
    json_ack_data["coverage_path_end_point_index"] =
        int(path_points.size() - 1);

    if (close_welt_path.size() > 0) {
      json_ack_data["close_path_start_point_x"] = close_welt_path[0].x;
      json_ack_data["close_path_start_point_y"] = close_welt_path[0].y;
      json_ack_data["close_path_start_point_yaw"] = close_welt_path[0].yaw;
      json_ack_data["close_path_start_point_index"] = 0;
      json_ack_data["close_path_end_point_x"] = close_welt_path.back().x;
      json_ack_data["close_path_end_point_y"] = close_welt_path.back().y;
      json_ack_data["close_path_end_point_yaw"] = close_welt_path.back().yaw;
      json_ack_data["close_path_end_point_index"] =
          int(close_welt_path.size() - 1);
    }

    json_ack_data["succeed"] = true;
    ack_data = json_ack_data.toStyledString();
    return;
  } else {
    LOG(ERROR) << "plan size <=2";
    json_ack_data["error_msgs"] = "plan size <=2";
    json_ack_data["succeed"] = false;
    ack_data = json_ack_data.toStyledString();
    return;
  }
}

// TODO: 多条路径拼接, 路径点增加贴边属性字段
void MultiCCPPMissionManager::weltPathPlanner(std::string rev_data,
                                              std::string &ack_data) {
  Json::Reader json_reader;
  Json::Value json_rev_data;
  Json::Value json_ack_data;

  if (!json_reader.parse(rev_data, json_rev_data)) {
    LOG(ERROR) << "rev_data is illegal!!";
    json_ack_data["error_msgs"] = " rev_data is illegal!!";
    json_ack_data["succeed"] = false;
    ack_data = json_ack_data.toStyledString();
    return;
  }
  std::string map_path = json_rev_data["map_path"].asString();
  std::string save_path = json_rev_data["close_path_save_path"].asString();
  // chack map
  if (access(map_path.c_str(), F_OK)) {
    LOG(ERROR) << "map: " << map_path << " didn`t exist!!";
    json_ack_data["error_msgs"] = "map didn`t exist!!";
    json_ack_data["succeed"] = false;
    ack_data = json_ack_data.toStyledString();
    return;
  }
  // WeltPathPlanner wpp;
  // wpp.readMap(map_path);

  EdgePlanner epp;
  epp.readMap(map_path);

  std::vector<PathPointWithType> path;
  int model_type = json_rev_data["type"].asInt();
  switch (model_type) {
    case 0: {
      // if (!wpp.generateWeltPath(path)) {
      if (!epp.generateEdgePath(path)) {
        LOG(ERROR) << "generate welt path wrong!!";
        json_ack_data["error_msgs"] = "generate welt path wrong!!";
        json_ack_data["succeed"] = false;
        ack_data = json_ack_data.toStyledString();
        return;
      }
    } break;
    // TODO: 补充 1的方式
    case 2: {
      std::vector<std::vector<AreaVertex>> areas;
      std::vector<AreaVertex> area;
      AreaVertex area_vertex;
      for (Json::ArrayIndex i = 0; i < json_rev_data["areas"].size(); i++) {
        for (Json::ArrayIndex j = 0;
             j < json_rev_data["areas"][i]["points"].size(); j++) {
          area_vertex.x =
              json_rev_data["areas"][i]["points"][j]["x"].asDouble();
          area_vertex.y =
              json_rev_data["areas"][i]["points"][j]["y"].asDouble();
          area.push_back(area_vertex);
        }
        areas.push_back(area);
      }

      // if (!wpp.generateWeltPathByAreas(path, areas)) {
      if (!epp.generateEdgePathByAreas(path, areas)) {
        LOG(ERROR) << "generate areas welt path wrong!!";
        json_ack_data["error_msgs"] = "generate areas welt path  wrong!!";
        json_ack_data["succeed"] = false;
        ack_data = json_ack_data.toStyledString();
        return;
      }
    } break;

    default: {
      LOG(ERROR) << "type  wrong!!";
      json_ack_data["error_msgs"] = " cmd  wrong!!";
      json_ack_data["succeed"] = false;
      ack_data = json_ack_data.toStyledString();
      return;
    } break;
  }

  if (path.size() > 2) {
    // std::vector<PathPointWithType> filter_path;
    // wpp.pathFilter(path, filter_path);
    // wpp.saveAsImage(path);
    // wpp.savePath(path, save_path);
    epp.saveAsImage(path);
    epp.savePath(path, save_path);
    json_ack_data["close_path_start_point_x"] = path[0].x;
    json_ack_data["close_path_start_point_y"] = path[0].y;
    json_ack_data["close_path_start_point_yaw"] = path[0].yaw;
    json_ack_data["close_path_start_point_index"] = 0;
    json_ack_data["close_path_end_point_x"] = path.back().x;
    json_ack_data["close_path_end_point_y"] = path.back().y;
    json_ack_data["close_path_end_point_yaw"] = path.back().yaw;
    json_ack_data["close_path_end_point_index"] = int(path.size() - 1);
    json_ack_data["succeed"] = true;
  } else {
    LOG(ERROR) << "path generate wrong!!";
    json_ack_data["error_msgs"] = " path generate  wrong!!";
    json_ack_data["succeed"] = false;
  }
  ack_data = json_ack_data.toStyledString();
}

void initGoogleLog(const std::string &module, const int &log_level) {
  FLAGS_colorlogtostderr = true;
  FLAGS_max_log_size = 100;  // 单个日志文件大小上限（MB）, 如果设置为0将默认为1
  FLAGS_logbufsecs = 0;  //设置可以缓冲日志的最大秒数,glog默认30秒,0指实时输出
  google::InitGoogleLogging(module.c_str());
  google::InstallFailureSignalHandler();
  google::SetStderrLogging(log_level);

  boost::filesystem::path path("log_info/" + module);
  if (!boost::filesystem::exists(path)) {
    boost::filesystem::create_directories(path);
  }

  boost::filesystem::path tmp_path;
  tmp_path = path / "info/";
  if (!boost::filesystem::exists(tmp_path)) {
    boost::filesystem::create_directory(tmp_path);
  }
  google::SetLogDestination(google::GLOG_INFO, tmp_path.c_str());

  tmp_path = path / "warn/";
  if (!boost::filesystem::exists(tmp_path)) {
    boost::filesystem::create_directory(tmp_path);
  }
  google::SetLogDestination(google::GLOG_WARNING, tmp_path.c_str());

  tmp_path = path / "error/";
  if (!boost::filesystem::exists(tmp_path)) {
    boost::filesystem::create_directory(tmp_path);
  }
  google::SetLogDestination(google::GLOG_ERROR, tmp_path.c_str());

  tmp_path = path / "fatal/";
  if (!boost::filesystem::exists(tmp_path)) {
    boost::filesystem::create_directory(tmp_path);
  }
  google::SetLogDestination(google::GLOG_FATAL, tmp_path.c_str());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  initGoogleLog("multi_area_ccpp", google::WARNING);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(
      "multi_area_ccpp_mission_manager", options);
  MultiCCPPMissionManager mccpp(node);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  exec.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
