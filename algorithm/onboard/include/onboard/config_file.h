#ifndef _ONBOARD_CONFIG_FILE_H_
#define _ONBOARD_CONFIG_FILE_H_

// 代码里的结构体默认值只是兜底，实机一律以 YAML 为准
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include "onboard/ego_planner_adapter.h"
#include "onboard/offboard_pilot.h"
#include "onboard/mission_input.h"
#include "onboard/livox_source.h"

namespace ego_onboard
{

  struct AppConfig
  {
    OffboardPilot::Config pilot;
    MissionInputConfig mission_input;
    LivoxSource::Config lidar;
    EgoPlannerAdapterConfig adapter;

    std::string uplink;             

    // 机体包络半径
    double vehicle_radius = 0.85;
    double min_obstacle_gap = 1.5; 
  };

  // 读 YAML 并填充 out；缺省的键保持结构体默认值。
  bool loadConfigFile(const std::string &path, AppConfig &out, std::string &err);

} // namespace ego_onboard

#endif
