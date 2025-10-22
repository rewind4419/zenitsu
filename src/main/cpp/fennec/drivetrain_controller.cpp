#include "drivetrain_controller.h"

#include "config.h"
#include "../Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>



void initDrivetrainController(DrivetrainController* controller)
{
	initPid(&controller->normal_pid);
	initPid(&controller->tangent_pid);
	initPid(&controller->angular_pid);

    controller->mode = DRIVECTRL_VELOCITY;
	controller->ctrl.velocity.velocity 		 = v2 { 0, 0 };
	controller->ctrl.velocity.angular_velocity = 0;
}



// TODO
// have throttle in the drivetrain controller
// then have a function that adds on PID functionally on loops that its called
// otherwise, just use throttle
// similarly, have another layer of function for specific target rotation control

void updateDrivetrainController(RobotData* r, DrivetrainController* controller, Drivetrain* drivetrain, OdometryFrame prev_odo, float dt)

{
    switch (controller->mode)
    {
	    case DRIVECTRL_THROTTLE:
    	{
    		drivetrainUpdate(drivetrain, controller->ctrl.throttle.throttle, controller->ctrl.throttle.angular_throttle, dt);
    	} break;

    	case DRIVECTRL_VELOCITY:
		{
			v2 current_velocity_raw = prev_odo.delta_position / dt;
			float current_angular_velocity_raw = prev_odo.delta_rotation / dt; // @MEGATODO maybe use the IMU for velocity

			update(&controller->velocity_avg, current_velocity_raw);
			update(&controller->ang_velocity_avg, current_angular_velocity_raw);

			v2 current_velocity = eval(controller->velocity_avg);
			float current_angular_velocity = eval(controller->ang_velocity_avg);

			v2 translation = { 
				.x = evalPid(&controller->tangent_pid, controller->ctrl.velocity.velocity.x - current_velocity.x, dt),
				.y = evalPid(&controller->normal_pid,  controller->ctrl.velocity.velocity.y - current_velocity.y, dt)
			};

			float rotation = evalPid(&controller->angular_pid, controller->ctrl.velocity.angular_velocity - current_angular_velocity, dt);

			translation.x -= rotation * CFG_DRIVETRAIN_ANTIDRIFT;

			drivetrainUpdate(drivetrain, translation, rotation, dt);
		} break;


		case DRIVECTRL_WAYPOINT:
		{
			// Drives towards a point using the localiser
			Pose target_pose = controller->ctrl.waypoint.pose;

			Pose current_pose = r->localiser.pose_estimate;
			v2 move_to = target_pose.position - current_pose.position;
			// frc::SmartDashboard::PutNumber("Waypoint Difference x", move_to.x);
			// frc::SmartDashboard::PutNumber("Waypoint Difference y", move_to.y);


			// frc::SmartDashboard::PutNumber("Side", r->side);


			v2 current_facing = rotate(v2{ 0, 1 }, current_pose.rotation);
			v2 target_facing  = rotate(v2{ 0, 1 }, target_pose.rotation);

			// float error = acos(dot(current_facing, target_facing));
			// if (dot(target_facing, rightPerpendicular(current_facing)) < 0)
			// 	error *= -1;

			
			
			float error = leastAngularError(current_pose.rotation, target_pose.rotation);


			move_to.x = evalPid(&controller->linear_x_pid, move_to.x, r->delta_time);
			move_to.y = evalPid(&controller->linear_y_pid, move_to.y, r->delta_time);

			// printf("%f movetoX\n", move_to.x);
			// printf("%f movetoY\n", move_to.y);

			if (length(move_to) > 1)
				move_to = normalize(move_to);

			move_to = move_to * controller->ctrl.waypoint.speed;

			if(r->side == 1)
			{
				move_to = rotate(move_to, current_pose.rotation + r->localiser.starting_rotation + (3 * M_PI) / 2);
			}
			else if (r->side == 0) 
			{
				// frc::SmartDashboard::PutNumber("Move to Offset", current_pose.rotation + r->localiser.starting_rotation + M_PI / 2);
				move_to = rotate(move_to, current_pose.rotation + r->localiser.starting_rotation + M_PI / 2);
			}


			float rot = error;

			rot = evalPid(&controller->aligner_pid, rot * controller->ctrl.waypoint.speed_rot, r->delta_time);

		    {
	    		v2 current_velocity_raw = prev_odo.delta_position / dt;
				float current_angular_velocity_raw = prev_odo.delta_rotation / dt; // @MEGATODO maybe use the IMU for velocity

				update(&controller->velocity_avg, current_velocity_raw);
				update(&controller->ang_velocity_avg, current_angular_velocity_raw);



				v2 current_velocity = eval(controller->velocity_avg);
				float current_angular_velocity = eval(controller->ang_velocity_avg);

				v2 translation = { 
					.x = evalPid(&controller->tangent_pid, move_to.x - current_velocity.x, dt),
					.y = evalPid(&controller->normal_pid,  move_to.y - current_velocity.y, dt)
				};

				float rotation = evalPid(&controller->angular_pid, rot - current_angular_velocity, dt);

				translation.x -= rotation * CFG_DRIVETRAIN_ANTIDRIFT;

				// frc::SmartDashboard::PutNumber("Translation X", translation.x);
				// frc::SmartDashboard::PutNumber("Translation Y", translation.y);

				drivetrainUpdate(drivetrain, translation, rotation, dt);
				drivetrainUpdate(drivetrain, translation, rotation, dt);
				// drivetrainUpdate(drivetrain, translation, 0, dt);
		    }

		} break;
    }

}