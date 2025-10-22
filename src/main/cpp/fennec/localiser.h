#pragma once

#include "maths.h"
#include "drivetrain.h"

constexpr v2 ROBOT_LOCAL_FORWARD = { 0, 1 };
constexpr v2 ROBOT_LOCAL_RIGHT   = { 1, 0 };

struct RobotData;

void stepLocaliser(RobotData* robot);

struct Localiser_FirstOrderLag
{
  bool first = true;
  Pose pose_estimate;
  float prev_imu;
  float starting_rotation;
};

void initLocaliser(RobotData* robot);


