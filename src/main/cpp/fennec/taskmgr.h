#pragma once
#include <stdint.h>
#include <stddef.h>

#include "maths.h"
#include "pid.h"

struct RobotData; // forward declaration


#define TASKMGR_MAX_TASKS (1024)

enum TaskType {
	TASK_NONE = 0,

	TASK_LIST,

	TASK_DELAY,

	TASK_FOLLOW_LINE,

	TASK_WAYPOINT,
	TASK_WAYPOINT_PULLER,
	TASK_AUTO_AWAIT_PULLER,
	TASK_AUTO_AWAIT_PULLER_INVERT,
	TASK_DRIVETRAIN_VELOCITY,
	
	TASK_SHOOTER_POSITIONING,

    TASK_MIDDLE_THE_WHEELS,

	TASK_INTAKE_PULLER,

	TASK_CHECK_INTAKE_FREE,

	TASK_SHOOTER_PULLER,

	TASK_INTAKE_WITHOUT_BB,

	TASK_SHOOTER_FIRE,

	TASK_SHOOTER_STOP,

	TASK_SEAT_RING,
	
	TASK_SEAT_RING_WITH_BEAMBREAK,

	TASK_ANGLE_TO_TAG,

	TASK_ANGLE_TO_TAG_AUTO,

	TASK_WAIT_FOR_FIRING_RPM,

	TASK_AMP_READY,

	TASK_SHOOTER_PULLER_START,

	TASK_SHOOTER_PULLER_STOP,

	TASK_SHOOTER_POSITIONING_NO_RETURN,
	
	TASK_ANGLE_FOR_CLIMB, //Comment out if it doesn't work - Nethra

	TASK_EXTEND_FOR_CLIMB, //Comment out if it doesn't work - Nethra

	TASK_RETRACT_FOR_CLIMB, //Comment out if it doesn't work - Nethra

	TASK_DRIVETRAIN_OVERRIDE,

	TASK_AUTO_AIM_ACTIVATION,

	TASK_PHOTON_AIM_TIMER_RESET,

	// TASK_FUNCTIONPTR, // TODO
};


struct TaskMgr;

struct TaskData_FiringMotor
{
	float direction;
};

struct TaskData_AutoAim
{
	bool activated;
};

struct TaskData_Delay {
	float length;

	//state
	float timer;
};

struct TaskData_Elevator
{
    float target_height;
	float ideal_height; //Comment out if it doesn't work - Nethra
	float retract_height; //Comment out if it doesn't work - Nethra
    float epsilon;
};

struct TaskData_Wait_For_RPM
{
	float rpm;
	float timer;
};


struct TaskData_Waypoint {
	Pose  target_pose;

	float speed;
	float speed_rot;
	float epsilon;
    float epsilon_rot;

	bool tag_aligner;
};

struct TaskData_FollowLine {
	Pose starting_pose;
	Pose ending_pose;

	float start_speed;
	float mid_speed;
	float end_speed;

	float speed_rot;

	float max_accel;

	float epsilon;
    float epsilon_rot;
};

struct TaskData_DrivetrainVelocity {
	v2 target_velocity;
	float target_angular_velocity;
    float timer;
    float length;
};

struct TaskData_PhotonAligner {
	int align_tag_id;
	float angular_throttle;
	float shooter_align_epsilon;
	bool timer_first;
	float timer;
	float delay_length;
	float angular_throttle_timer;
};

struct TaskData_Hank {
	v2 target_state;
	float target_wrist;
	float epsilon;
	float epsilon_wrist;
	float timeout_time;
	float timeout_length;
};

struct TaskData_Stag {
	float intake_speed;
    float manual_offset;
    bool overrider;
};

struct TaskData_Shrek {
	float epsilon;
};

struct TaskData_Shooter {
	float target_angle;
	float epsilon;
	float delay_timer;
	float delay_length;
	float seat_speed_firing;
	float seat_speed_control;
	float seat_speed_intake;
	float seat_prior_firing_throttle;
	bool seat_first;
	float fire_direction;
	bool maintain_prev_throttle;
};

struct TaskData_IntakePuller {
	float throttle;
};

struct TaskData_MiddleWheels {
    bool enabled;
};


struct Task {
	TaskType type = TASK_NONE;
	bool started = false;

	union {
		TaskMgr* list;
		TaskData_Delay delay;
		TaskData_Waypoint waypoint;
		TaskData_DrivetrainVelocity drivetrain_velocity;
		TaskData_Shooter shooter;
		TaskData_IntakePuller intake_puller;
        TaskData_MiddleWheels middle_wheels;
        TaskData_FollowLine follow_line;
		TaskData_FiringMotor firing_motor;
		TaskData_PhotonAligner photon_aligner;
		TaskData_Elevator elevator;
		TaskData_Wait_For_RPM wait_rpm;
		TaskData_AutoAim auto_aim;
	};
};



struct TaskMgr
{
	uint64_t write_head = 0;
	uint64_t read_head = 0;

	// this is a ring buffer, keep that in mind
	Task task_buffer[TASKMGR_MAX_TASKS] = {};

	bool is_parallel = false;
};

bool pushTask(TaskMgr* mgr, Task task);
void updateManager(TaskMgr* mgr, RobotData* robot);

// util
inline Task genTaskDefault(TaskType type)
{
	Task t;
	t.type = type;
	return t;
}

Task genTaskList(TaskMgr* list);

inline Task genTaskDelay(float delay)
{
	Task t;
	t.type = TASK_DELAY;
	t.delay.timer = 0;
	t.delay.length = delay;
	return t;
}

inline Task genTaskWaypoint(TaskData_Waypoint waypoint)
{
	Task t;
	t.type = TASK_WAYPOINT;
	t.waypoint = waypoint;
	return t;
}

inline Task genTaskDrivetrainVelocity(TaskData_DrivetrainVelocity drivetrain_velocity)
{
	Task t;
	t.type = TASK_DRIVETRAIN_VELOCITY;
	t.drivetrain_velocity = drivetrain_velocity;
	return t;
}





