#pragma once

#include "pid.h"
#include "drivetrain.h"

#include "sliding_average.h"

struct RobotData;

enum DrivetrainControllerMode
{
	DRIVECTRL_VELOCITY = 0,
	DRIVECTRL_THROTTLE,
	DRIVECTRL_WAYPOINT,
};

struct DrivetrainController
{

	// linear velocity
	PID normal_pid  = { .kP = 0.1, .kI = 0, .kD = 0 };
	PID tangent_pid = { .kP = 0.1, .kI = 0, .kD = 0 };

	// angular velocity
	PID angular_pid = { .kP = 0.1, .kI = 0, .kD = 0 };

	// aligner
	PID linear_x_pid { .kP = 2.0, .kI = 0.0, .kD = 0.0 };
	PID linear_y_pid { .kP = 2.0, .kI = 0.0, .kD = 0.0 };
	PID aligner_pid { .kP = 5.0, .kI = 0.01, .kD = 0.0 };

	// Tag aligner pid
	PID tag_aligner_pid = { .kP = 0.7f, .kI = 0.3, .kD = 0.01f,  .errorAccum = 0, .lastError = 0 };
	PID tag_aligner_pid_auto = { .kP = 0.4f, .kI = 0, .kD = 0.0f,  .errorAccum = 0, .lastError = 0 };
	// PID tag_aligner_pid_auto = { .kP = 0.7f, .kI = 0.3, .kD = 0.01f,  .errorAccum = 0, .lastError = 0 };

	bool drivetrain_ovveride = false;


	// input
	struct {
		struct {
			v2 		velocity 				 = {};
			float angular_velocity = 0;
		} velocity;

		struct {
			v2 		throttle 				 = {};
			float angular_throttle = 0;
		} throttle;

		struct {
			Pose pose;
			float speed;
			float speed_rot;
		} waypoint;
		float timeout = 0;
	} ctrl;

	SlidingAvg<v2, 5> velocity_avg;
	SlidingAvg<float, 5> ang_velocity_avg;

  DrivetrainControllerMode mode = DRIVECTRL_VELOCITY;
};

void initDrivetrainController(DrivetrainController* controller);

void updateDrivetrainController(RobotData* r, DrivetrainController* controller, Drivetrain* drivetrain, OdometryFrame prev_odo, float dt);