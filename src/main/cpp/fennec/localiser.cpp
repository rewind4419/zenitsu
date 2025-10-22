#include "localiser.h"
#include "../Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>

void initLocaliser(RobotData* robot)
{
  Localiser_FirstOrderLag* localiser = &robot->localiser;
  // robot->side = 0;
  if(robot->side == 0)
  {
    float rotation = - M_PI / 2;
    localiser->pose_estimate.rotation = rotation;
    localiser->starting_rotation = rotation;
  }
  else if(robot->side == 1)
  {
    float rotation = M_PI / 2;
    localiser->pose_estimate.rotation = rotation;
    localiser->starting_rotation = rotation;
  }

  // Bottom of speaker should be 36.17 inches from april tag
  // Bottom of speaker should be 36.17 inches from april tag
  v2 init_pose = {1.6, 4.8};
  localiser->pose_estimate.position = init_pose;
}

// void stepLocaliser(RobotData* robot)
// {
//   Localiser_FirstOrderLag* localiser = &robot->localiser;
//   OdometryFrame odometry_frame = robot->latest_odometry_frame;
//   float imu_rotation = degToRad(robot->sensor_imu->GetYaw());

//   if (localiser->first)
//   {
//     localiser->first = false;
//     localiser->prev_imu = imu_rotation;
//     robot->side = 0;
//     initLocaliser(robot);
//   }  



//   // frc::SmartDashboard::PutNumber("Localiser X", localiser->pose_estimate.position.x);
//   // frc::SmartDashboard::PutNumber("Localiser Y", localiser->pose_estimate.position.y);
//   // frc::SmartDashboard::PutNumber("Localiser Rotation", localiser->pose_estimate.rotation);

//   float delta_rot = imu_rotation - localiser->prev_imu;
//   localiser->prev_imu = imu_rotation;


//   float apriltag_first_order_lag_damping = 0.6;
//   // float apriltag_first_order_lag_damping = 0.0f;
//   for (int i = 0; i < robot->photon.global_tags.size(); i++)
//   {
//     v2 curr_tag_pose = { static_cast<float>(robot->photon.global_tags[i].pose.X()), static_cast<float>(robot->photon.global_tags[i].pose.Y()) };

//     // frc::SmartDashboard::PutNumber("April Tag Global Pose X", curr_tag_pose.x);
//     // frc::SmartDashboard::PutNumber("April Tag Global Pose Y", curr_tag_pose.y);

//     // float curr_tag_rotation = static_cast<float>(robot->photon.global_tags[i].pose.Rotation().Z()) ;
//     float curr_tag_rotation = static_cast<float>(robot->photon.global_tags[i].pose.Rotation().Z()) + M_PI/ 2;

//     // frc::SmartDashboard::PutNumber("April Tag Global Rotation", curr_tag_rotation);

//     localiser->pose_estimate.position = mix(localiser->pose_estimate.position, 
//                                             curr_tag_pose, 
//                                             apriltag_first_order_lag_damping);

//     localiser->pose_estimate.rotation = mix(localiser->pose_estimate.rotation, 
//                                             curr_tag_rotation,
//                                             apriltag_first_order_lag_damping);
    
//   }
//   if(robot->photon.n_tags == 0) 
//   {
//     // printf("No tags seen\n");


//     if(delta_rot > M_PI) localiser->pose_estimate.rotation += 2 * M_PI;
//     if(delta_rot < -M_PI) localiser->pose_estimate.rotation -= 2 * M_PI;

//     // Uncomment if it doesnt work
//     if (localiser->pose_estimate.rotation < 0.0) {localiser->pose_estimate.rotation += M_PI * 2;}
//     localiser->pose_estimate.rotation = fmod(localiser->pose_estimate.rotation, M_PI * 2);

//     localiser->pose_estimate.rotation +=  -1 * delta_rot;
//     localiser->pose_estimate.position = localiser->pose_estimate.position + rotate(odometry_frame.delta_position, -localiser->pose_estimate.rotation);
//   }

//   //Comment if it doesnt work
//   // if (localiser->pose_estimate.rotation < 0.0) {localiser->pose_estimate.rotation += M_PI * 2;}
//   // localiser->pose_estimate.rotation = fmod(localiser->pose_estimate.rotation, M_PI * 2);

//   robot->photon.global_tags.clear();
// }

bool tagIdExists(std::vector<TagPosition> tag_vector, int index)
{
  bool current_exists = false;
  for(int i = 0; i < tag_vector.size(); i++)
  {
    if(tag_vector[i].tag_id == index) current_exists = true;
  }
  return current_exists;
}


void stepLocaliser(RobotData* robot)
{
  Localiser_FirstOrderLag* localiser = &robot->localiser;
  OdometryFrame odometry_frame = robot->latest_odometry_frame;
  float imu_rotation = degToRad(robot->sensor_imu->GetYaw());

  if (localiser->first)
  {
    localiser->first = false;
    localiser->prev_imu = imu_rotation;
    initLocaliser(robot);
  }  



  // frc::SmartDashboard::PutNumber("Localiser X", localiser->pose_estimate.position.x);
  // frc::SmartDashboard::PutNumber("Localiser Y", localiser->pose_estimate.position.y);
  // frc::SmartDashboard::PutNumber("Localiser Rotation", localiser->pose_estimate.rotation);

  float delta_rot = imu_rotation - localiser->prev_imu;
  localiser->prev_imu = imu_rotation;


  float apriltag_first_order_lag_damping;

  for (int i = 0; i < robot->photon.global_tags.size(); i++)
  {
    int tag_id = robot->photon.global_tags[i].tag_id;

    bool current_exists = tagIdExists(robot->photon.global_tags, tag_id);
    bool prev_exists = tagIdExists(robot->photon.global_tags_prev, tag_id);

    // printf("%d, %d, %d \n", i, current_exists, prev_exists);

    if(current_exists && prev_exists)
    {
      frc::Translation3d diff_between_tags = robot->photon.global_tags[i].pose.Translation() - robot->photon.global_tags_prev[i].pose.Translation();

      float magnitude_from_prev = static_cast<float>(diff_between_tags.Norm());

      if(magnitude_from_prev < 0.3f)
      {
        apriltag_first_order_lag_damping = 0.6;
      }
      else apriltag_first_order_lag_damping = 0;
    }
    else apriltag_first_order_lag_damping = 0;

    v2 curr_tag_pose = { static_cast<float>(robot->photon.global_tags[i].pose.X()), static_cast<float>(robot->photon.global_tags[i].pose.Y()) };

    // frc::SmartDashboard::PutNumber("April Tag Global Pose X", curr_tag_pose.x);
    // frc::SmartDashboard::PutNumber("April Tag Global Pose Y", curr_tag_pose.y);

    // float curr_tag_rotation = static_cast<float>(robot->photon.global_tags[i].pose.Rotation().Z()) ;
    float curr_tag_rotation = static_cast<float>(robot->photon.global_tags[i].pose.Rotation().Z()) + M_PI/ 2;

    // frc::SmartDashboard::PutNumber("April Tag Global Rotation", curr_tag_rotation);

    localiser->pose_estimate.position = mix(localiser->pose_estimate.position, 
                                            curr_tag_pose, 
                                            apriltag_first_order_lag_damping);

    localiser->pose_estimate.rotation = mix(localiser->pose_estimate.rotation, 
                                            curr_tag_rotation,
                                            apriltag_first_order_lag_damping);
  }
  if(robot->photon.n_tags == 0) 
  {
    // printf("No tags seen\n");


    if(delta_rot > M_PI) localiser->pose_estimate.rotation += 2 * M_PI;
    if(delta_rot < -M_PI) localiser->pose_estimate.rotation -= 2 * M_PI;

    // Uncomment if it doesnt work

    localiser->pose_estimate.rotation +=  -1 * delta_rot;
    localiser->pose_estimate.position = localiser->pose_estimate.position + rotate(odometry_frame.delta_position, -localiser->pose_estimate.rotation);

    if (localiser->pose_estimate.rotation < 0.0) {localiser->pose_estimate.rotation += M_PI * 2;}
    localiser->pose_estimate.rotation = fmod(localiser->pose_estimate.rotation, M_PI * 2);
  }

  //Comment if it doesnt work
  // if (localiser->pose_estimate.rotation < 0.0) {localiser->pose_estimate.rotation += M_PI * 2;}
  // localiser->pose_estimate.rotation = fmod(localiser->pose_estimate.rotation, M_PI * 2);

  robot->photon.global_tags_prev = robot->photon.global_tags;

}