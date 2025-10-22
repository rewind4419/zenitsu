#include "drivetrain.h"
#include <frc/Joystick.h>
#include <rev/SparkMax.h>
#include<frc/smartdashboard/SmartDashboard.h>
#include "config.h"

#include "pid.h"

#include <math.h>

// Dont even bother w/ deinitialization tbh,
// the roborio literally closes and reopens the program
// the only memory leak stuff we need to worry about is constant allocation, 
// this should be fine tho



#define NEW_SPARK_MAX(can) new rev::SparkMax(can, rev::SparkMaxLowLevel::MotorType::kBrushless)

void initDrivetrain(Drivetrain* drivetrain)
{
    SwerveDriveModule fl;
    fl.direction_encoder = new ctre::phoenix6::hardware::CANcoder(CFG_CAN_DRIVETRAIN_STEER_ENCODER_FL);
    fl.drive_motor       = NEW_SPARK_MAX(CFG_CAN_DRIVETRAIN_DRIVE_MOTOR_FL);
    fl.steer_motor       = NEW_SPARK_MAX(CFG_CAN_DRIVETRAIN_STEER_MOTOR_FL);
    fl.initial_rotation_offset = -CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_FL;
    fl.tangent_vector = v2 { -10, 10 };
    fl.center_offset = v2 { -CFG_DRIVETRAIN_DISTANCE_BETWEEN_WHEELS_HORIZONTAL / 2.0f, CFG_DRIVETRAIN_DISTANCE_BETWEEN_WHEELS_VERTICAL / 2.0f };
    drivetrain->swerve_drives[DrivetrainSwerve_FL] = fl;

    SwerveDriveModule fr;
    fr.direction_encoder = new ctre::phoenix6::hardware::CANcoder(CFG_CAN_DRIVETRAIN_STEER_ENCODER_FR);
    fr.drive_motor       = NEW_SPARK_MAX(CFG_CAN_DRIVETRAIN_DRIVE_MOTOR_FR);
    fr.steer_motor       = NEW_SPARK_MAX(CFG_CAN_DRIVETRAIN_STEER_MOTOR_FR);
    fr.initial_rotation_offset = -CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_FR;
    fr.tangent_vector = v2 { 10, 10 };
    fr.center_offset = v2 { CFG_DRIVETRAIN_DISTANCE_BETWEEN_WHEELS_HORIZONTAL / 2.0f, CFG_DRIVETRAIN_DISTANCE_BETWEEN_WHEELS_VERTICAL / 2.0f };
    drivetrain->swerve_drives[DrivetrainSwerve_FR] = fr;

    SwerveDriveModule bl;
    bl.direction_encoder = new ctre::phoenix6::hardware::CANcoder(CFG_CAN_DRIVETRAIN_STEER_ENCODER_BL);
    bl.drive_motor       = NEW_SPARK_MAX(CFG_CAN_DRIVETRAIN_DRIVE_MOTOR_BL);
    bl.steer_motor       = NEW_SPARK_MAX(CFG_CAN_DRIVETRAIN_STEER_MOTOR_BL);
    bl.initial_rotation_offset = -CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_BL;
    bl.tangent_vector = v2 { -10, -10 };
    bl.center_offset = v2 { -CFG_DRIVETRAIN_DISTANCE_BETWEEN_WHEELS_HORIZONTAL / 2.0f, -CFG_DRIVETRAIN_DISTANCE_BETWEEN_WHEELS_VERTICAL / 2.0f };
    drivetrain->swerve_drives[DrivetrainSwerve_BL] = bl;

    SwerveDriveModule br;
    br.direction_encoder = new ctre::phoenix6::hardware::CANcoder(CFG_CAN_DRIVETRAIN_STEER_ENCODER_BR);
    br.drive_motor       = NEW_SPARK_MAX(CFG_CAN_DRIVETRAIN_DRIVE_MOTOR_BR);
    br.steer_motor       = NEW_SPARK_MAX(CFG_CAN_DRIVETRAIN_STEER_MOTOR_BR);
    br.initial_rotation_offset = -CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_BR;
    br.tangent_vector = v2 {  10, -10 };
    br.center_offset = v2 { CFG_DRIVETRAIN_DISTANCE_BETWEEN_WHEELS_HORIZONTAL / 2.0f, -CFG_DRIVETRAIN_DISTANCE_BETWEEN_WHEELS_VERTICAL / 2.0f };
    drivetrain->swerve_drives[DrivetrainSwerve_BR] = br;


    // NOTE hopefully the cancoders are initialized by this point
    // otherwise we should figure out when to zero

    for (int i = 0; i < DrivetrainSwerve_Count; i++)
    {
        SwerveDriveModule* module = &drivetrain->swerve_drives[i];
        
        module->steer_encoder = new rev::SparkRelativeEncoder(module->steer_motor->GetEncoder());
        module->drive_encoder = new rev::SparkRelativeEncoder(module->drive_motor->GetEncoder());

        float initial_position = -module->direction_encoder->GetAbsolutePosition().GetValue().value() * M_2_PI - module->initial_rotation_offset;
        
        //-module->direction_encoder->GetAbsolutePosition() * M_PI / 180 - module->initial_rotation_offset;

        // Convert the zero position into a vector to not worry about 
        // @Incomplete: figure out which way the 0 angle is of the CANCoders and if positive is clockwise or not
        module->initial_vector = rotate(v2 { 0, 1 }, initial_position);

        // printf("Initial vector ,, %f", -module->direction_encoder->GetAbsolutePosition() * M_PI / 180 - module->initial_rotation_offset);

        // zero the relative encoders
        module->steer_encoder->SetPosition(0);
        module->drive_encoder->SetPosition(0);
    }
}

void drivetrainUpdate(Drivetrain* drivetrain, v2 translation, float rotation, float delta_time)
{
    
    v2 target_vectors[DrivetrainSwerve_Count];

    // Calculate the target vectors of each wheel
    // (Magnitude is the speed of the wheel and direction is the direction)
    for (int i = 0; i < DrivetrainSwerve_Count; i++)
    {
        SwerveDriveModule* module = &drivetrain->swerve_drives[i];
        v2 tangent_vector = normalize(rightPerpendicular(module->tangent_vector));

        target_vectors[i] = -translation + tangent_vector * v2 { rotation, rotation };
    }

    drivetrainUpdateRawVectors(drivetrain, target_vectors, delta_time, false);
}

void drivetrainUpdateRawVectors(Drivetrain* drivetrain, v2* target_vectors, float delta_time, bool steering_only)
{
    // Normalize the wheel target vectors by dividing by them by the longest vector's magnitude


	// float wheel_speed_maximum = 0;
	// for (int i = 0; i < DrivetrainSwerve_Count; i++)
	// {
	// 	float speed = length(target_vectors[i]);
	// 	if (wheel_speed_maximum < speed) 
	// 		wheel_speed_maximum = speed;
	// }

    for (int i = 0; i < DrivetrainSwerve_Count; i++)
    {
        // int i = DrivetrainSwerve_FL;

        SwerveDriveModule* module = &drivetrain->swerve_drives[i];


        // @Todo: figure out what the deal with gear ratios is
        float steerRotation = -module->steer_encoder->GetPosition() / CFG_DRIVETRAIN_STEER_RATIO * (M_PI * 2);
        module->current_vector = rotate(module->initial_vector, steerRotation);


        // @Incomplete: add a PID loop!
        float turn_angle = 0;
        float rightness = 0;


        float direction_flipper = 1; // this will become negative 1 if the closer arc to rotate is the reverse one


        // if the move vector is below a certain amount, dont bother moving the wheel to avoid super small numbers causing weird rotations
        // TARGET_VECTOR_EPSILON should be pretty small
        if (length(target_vectors[i]) > CFG_DRIVETRAIN_TARGET_VECTOR_EPSILON)
        { 
            v2 direction_vector = normalize(target_vectors[i]); 

            if (dot(direction_vector, module->current_vector) < 0.0)
            {
                direction_flipper = -1;
                direction_vector = direction_vector * v2 { -1, -1 };
            }


            turn_angle = acos(dot(direction_vector, normalize(module->current_vector)));
            rightness = dot(direction_vector, rightPerpendicular(module->current_vector));

            // if we are going left,
            // sign(rightness) == -1
            // so the turn angle becomes negative
            turn_angle *= sign(rightness);
        }


		// Debug thing
        // printf("%f\n", module->drive_encoder->GetVelocity());
        // printf("initial_vector-> %f\n", length(module->initial_vector));
        // printf("current_vector-> %f\n", length(module->current_vector));
        // printf("steerRotation-> %f\n", steerRotation);


        // @Calibration
        // printf("%d -- %f\n", i, module->direction_encoder->GetAbsolutePosition() * M_PI / 180);


        // simple P loop
        float steer_throttle = -turn_angle / M_PI; 
        // if (steer_throttle >  .5) steer_throttle =  .5;
        // if (steer_throttle < -.5) steer_throttle = -.5;
        if(drivetrain->drivetrain_override) steer_throttle = 0;
        module->steer_motor->Set( steer_throttle);

        // frc::SmartDashboard::PutNumber("Final Steer", steer_throttle);

        // printf("steer speed = %f\n", steer_throttle);
		
		float driveSpeed = 1.0;

        // if (wheel_speed_maximum != 0)
        //     driveSpeed /= wheel_speed_maximum;



        float v = length(target_vectors[i]);
        if (v > 1) v = 1;
        if (v < -1) v = -1;
        v = v*v;
        v *= direction_flipper;
        if (steering_only)
        {
            v = 0;
        }

        float drive_throttle = v / driveSpeed;

        if(drivetrain->drivetrain_override) drive_throttle = 0;
        module->drive_motor->Set(drive_throttle);
        // frc::SmartDashboard::PutNumber("Final Drive", drive_throttle);


        // printf("drive speed = %f\n", drive_throttle);



        // float cancoder_position = module->direction_encoder->GetAbsolutePosition() * M_PI / 180;

        // if (i == DrivetrainSwerve_BL) printf("BL: ");
        // if (i == DrivetrainSwerve_FL) printf("FL: ");
        // if (i == DrivetrainSwerve_BR) printf("BR: ");
        // if (i == DrivetrainSwerve_FR) printf("FR: ");
        // printf("%f\n", cancoder_position - module->initial_rotation_offset);
    }

}




OdometryFrame getDrivetrainOdometry(Drivetrain* drivetrain)
{
    v2 travel_vectors[DrivetrainSwerve_Count];

    // read the travel vectors for each swerve module (wheel)
    for (int i = 0; i < DrivetrainSwerve_Count; i++)
    {
        SwerveDriveModule* module = &drivetrain->swerve_drives[i];

        float SWERVE_DRIVE_DRIVE_RATIO = 6.75f;
        float DRIVE_WHEEL_RADIUS = 4 * INCH_TO_METER; // meters

        float drivePosition = module->drive_encoder->GetPosition();

        // float driveDelta = (((drivePosition - module->previous_drive_encoder) / (2 * M_PI)) * DRIVE_WHEEL_RADIUS) / SWERVE_DRIVE_DRIVE_RATIO;
        float driveDelta = (drivePosition - module->previous_drive_encoder) / SWERVE_DRIVE_DRIVE_RATIO * (2 * M_PI * DRIVE_WHEEL_RADIUS) / 2;
        travel_vectors[i] = module->current_vector * v2 { driveDelta, driveDelta };
        module->previous_drive_encoder = drivePosition;
    }


    OdometryFrame odometry;

    // localization
    {
        v2 delta_pos {};

        for (int i = 0; i < DrivetrainSwerve_Count; i++)
        {
            SwerveDriveModule* module = &drivetrain->swerve_drives[i];

            delta_pos = delta_pos + (module->center_offset + travel_vectors[i]) / v2 { DrivetrainSwerve_Count, DrivetrainSwerve_Count };
        }

        v2 forward0 = (drivetrain->swerve_drives[DrivetrainSwerve_FL].center_offset + travel_vectors[DrivetrainSwerve_FL]) - (drivetrain->swerve_drives[DrivetrainSwerve_BL].center_offset + travel_vectors[DrivetrainSwerve_BL]);
        v2 forward1 = (drivetrain->swerve_drives[DrivetrainSwerve_FR].center_offset + travel_vectors[DrivetrainSwerve_FR]) - (drivetrain->swerve_drives[DrivetrainSwerve_BR].center_offset + travel_vectors[DrivetrainSwerve_BR]);

        v2 right0 = (drivetrain->swerve_drives[DrivetrainSwerve_FR].center_offset + travel_vectors[DrivetrainSwerve_FR]) - (drivetrain->swerve_drives[DrivetrainSwerve_FL].center_offset + travel_vectors[DrivetrainSwerve_FL]);
        v2 right1 = (drivetrain->swerve_drives[DrivetrainSwerve_BR].center_offset + travel_vectors[DrivetrainSwerve_BR]) - (drivetrain->swerve_drives[DrivetrainSwerve_BL].center_offset + travel_vectors[DrivetrainSwerve_BL]);
        
        v2 forward = normalize((forward0 + forward1) / v2 { 2.0, 2.0 });
        v2 right = normalize((right0 + right1) / v2 { 2.0, 2.0 });
        
        float forward_based_angle = acos(dot(forward, v2 {0, 1}));
        float right_based_angle = acos(dot(right, v2 {1, 0}));
        if (forward.x < 0)
            forward_based_angle *= -1;
        if (right.y > 0)
            right_based_angle *= -1;
        
        float angle = (forward_based_angle + right_based_angle) / 2;

        odometry.delta_position = -delta_pos * CFG_DRIVETRAIN_ODO_MULTIPLIER;
        odometry.delta_rotation = angle;
        // printf("rot: %f, %f\n", forward_based_angle, right_based_angle);
    }

    return odometry;
}



void printCalibrationData(Drivetrain* drivetrain)
{
    printf("==== SWERVE CALIBRATE ====\n");
    printf("Warning: Disable this before use!\n");
    
// mjh comment entire print block    
     for (int i = 0; i < DrivetrainSwerve_Count; i++)
    {
        SwerveDriveModule* module = &drivetrain->swerve_drives[i];

        // @Calibration
        switch (i)
        {
            case DrivetrainSwerve_BL: printf("bl"); break;
            case DrivetrainSwerve_FL: printf("fl"); break;
            case DrivetrainSwerve_BR: printf("br"); break;
            case DrivetrainSwerve_FR: printf("fr"); break;

            default: printf("what the heck"); break;
        }
        printf(" -- %f\n", module->direction_encoder->GetAbsolutePosition().GetValue().value() * M_2_PI);
    }
    //printf("==========================\n");
    
}