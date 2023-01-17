/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Popa Ioan Laurentiu*/

// ROS
#include <ros/ros.h>

// MoveIt dependencies
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2 dependencies
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  // Create the grasps to be attempted
  // Might need to create more grasps for testing
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // This is the pose of panda_link8, last link of manipulator robot
  // Create transformation fron last link to end effector
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(-tau / 4, -tau / 8, -tau / 4);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.415;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 0.5;

  // Setting pre-grasp approach with respect to frame_id
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0; // positive movement on X axis
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Setting post-grasp retreat with respect to frame_id 
  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0; // positive movement on Z axis
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // Setting posture of end effector before grasp
  openGripper(grasps[0].pre_grasp_posture);

  // Setting posture of end effector  during grasp
  closedGripper(grasps[0].grasp_posture);

  // Set support surface as table1.
  move_group.setSupportSurfaceName("table1");

  // Call pick method from MoveGroupInterface
  // to pick up the tool using the grasps given
  ROS_INFO_NAMED("project", "Picking tool");
  move_group.pick("tool", grasps);

  // print out succesfull picking
  ROS_INFO_NAMED("project", "Picked tool");
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
  // create place_locaton, an array might be needed for testing
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, tau / 4);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  // For place location, we set the value to the center of the table2 object
  place_location[0].place_pose.pose.position.x = -0.6;
  place_location[0].place_pose.pose.position.y = 0;
  place_location[0].place_pose.pose.position.z = 0.5;

  // Setting pre-place approach with respect to frame_id
  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  place_location[0].pre_place_approach.direction.vector.z = -1.0; // move negative on Z axis
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat with respect to frame_id
  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  place_location[0].post_place_retreat.direction.vector.x = 1.0; // move negative on Y axis
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of end effector after placing tool
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2
  group.setSupportSurfaceName("table2");

  // start placing, printout
  ROS_INFO_NAMED("project", "Placing tool");
  // Call place method from MoveGroupInterface
  // to place the tool using the place locations given.
  group.place("tool", place_location);

  // succesfull job printout
  ROS_INFO_NAMED("project", "Placed tool");
}

void addObjectstoPlanningScene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // Creating Environment
  // Create vector to hold 4 collision objects
  // Tables 1 and 2, the tool and the machine to do work on
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);

  // Add the first table support where the tool will initially be placed
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  // Define the primitive and its dimensions table support BOX
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  // Define the pose of the table support
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;

  // add table to array of object
  collision_objects[0].operation = collision_objects[0].ADD;


  // Add the 2nd table where to place the tool after work
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "panda_link0";

  // Define the primitive and its dimensions table support BOX
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.2;
  collision_objects[1].primitives[0].dimensions[1] = 0.4;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;

  // Define the pose of the table support
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = -0.6;
  collision_objects[1].primitive_poses[0].position.y = 0;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  collision_objects[1].primitive_poses[0].orientation.w = 1.0;

  // add table to array of object
  collision_objects[1].operation = collision_objects[1].ADD;

  // Define the object tool that will be manipulated
  collision_objects[2].header.frame_id = "panda_link0";
  collision_objects[2].id = "tool";

  // Define the primitive and its dimensions for tool (smaller box)
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.2;

  // Define the pose of the object tool
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.5;
  collision_objects[2].primitive_poses[0].orientation.w = 1.0;

  // add tool to array of object
  collision_objects[2].operation = collision_objects[2].ADD;


  // Define the machine object to be repaired
  // Cylindrical machine
  collision_objects[3].header.frame_id = "panda_link0";
  collision_objects[3].id = "machine";

  // Define the primitive and its dimensions for machine CYLINDER
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[3].primitives[0].CYLINDER;
  collision_objects[3].primitives[0].dimensions.resize(2);
  collision_objects[3].primitives[0].dimensions[0] = 0.6; // height
  collision_objects[3].primitives[0].dimensions[1] = 0.1; // radius

  // Define the pose of the object machine
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = -0.07;
  collision_objects[3].primitive_poses[0].position.y = 0.81;
  collision_objects[3].primitive_poses[0].position.z = 0.5;
  collision_objects[3].primitive_poses[0].orientation.w = 1.0;
 
  // add tool to array of object
  collision_objects[3].operation = collision_objects[3].ADD;

  // add object array to the planning scene
  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void do_work(moveit::planning_interface::MoveGroupInterface& group) {

  // Planning of the goal pose for work to be done
  // Plan a motion the group to a desired pose for the end-effector

  // create desired posed
  geometry_msgs::Pose targetPoseForWork;
  targetPoseForWork.orientation.w = 1.0;
  targetPoseForWork.position.x = 0.1;
  targetPoseForWork.position.y = 0.7;
  targetPoseForWork.position.z = 0.5;

  // seting the target pose 
  group.setPoseTarget(targetPoseForWork);

  // create a plan of movement
  moveit::planning_interface::MoveGroupInterface::Plan movementPlanToWork;

  // checking plan
  bool success = (group.plan(movementPlanToWork) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


  // if plan doesn't fail printout and start executing
  ROS_INFO_NAMED("project", "Moving to desired pose for work at cylindrical machine %s", success ? "" : "FAILED");

  // execute plan -- robot moving
  group.execute(movementPlanToWork);

  // planned movement completed
  ROS_INFO_NAMED("project", "Moved to desired posed");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_arm_pick_dowork_place");
  ros::NodeHandle nh;

  const ros::Time init_time = ros::Time::now();

  // ROS spinning must be running for the MoveGroupInterface to get information about the robot's state. 
  // One way to do this is to start an AsyncSpinner beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // wait
  ros::WallDuration(1.0).sleep();

  // PlanningSceneInterface class 
  // used add and remove collision objects in the "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // the sets of joints called "planning groups" for robot arm
  static const std::string PLANNING_GROUP = "panda_arm";

  // MoveGroupInterface class setup 
  // using the planning group to be controlled and planned for.
  moveit::planning_interface::MoveGroupInterface group(PLANNING_GROUP);

  // add objects to planning scene: supports, tool, machine
  addObjectstoPlanningScene(planning_scene_interface);

  // Wait for ROS components to initialize
  ros::WallDuration(1.0).sleep();

  // start picking operation
  pick(group);

  // wait before starting to move in work pose
  ros::WallDuration(1.0).sleep();

  // move in work pose
  do_work(group);

  // simulate working be wait
  ros::WallDuration(4.0).sleep();

  // start placing the tool after worked finished
  place(group);

  const ros::Time final_time = ros::Time::now();

  float delta_t = final_time.toSec()-init_time.toSec();;

  ROS_INFO_NAMED("project", "Time spent on task: %f sec. \n \
                              From which: \n \
                              2 sec for initialization \n \
                              1 sec wait after picking \n \
                              4 sec wait for working", delta_t);
  
  // close
  ros::shutdown();
  return 0;
}

// DESIRED OUTPUT
//
// ADD OBJECTS TO SCENE:
//    ADD TABLE ONE AND TWO, TOOL, MACHINE
//
// PICK TOOL:
//    MOVE INTO POSITION
//    OPEN GRIPPER
//    CLOSE GRIIPER
//    MOVE OUT OF POSITION
//
// MOVE TO PLANNED POSE FOR WORK
// STAYED IN POSE WITH TOOL
//
// PLACE TOOL:
//    MOVE INTO POSITION
//    OPEN GRIPPER
//    MOVE OUT OF POSITION
// 
// END
