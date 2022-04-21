
// Created by: Kris Gunnarsson
// Date: April 17, 2022

// Purpose: This file was created to test ability to interface with the move_base
//          package. The robot will move from its initial position to a sequence
//          of positions.
//          This node is exercised with src/script/pick_object.sh
//

#include <ros/ros.h>
#include <vector>

// Defines an interrace for clients of the MoveBase package
#include <move_base_msgs/MoveBaseAction.h>

// The package accepts goals and attempts to command a robot to them
#include <actionlib/client/simple_action_client.h>

// This is needed to specify orientations (quaternions)
#include <tf/tf.h>

// For convenience we define a new type
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char ** argv) {
  ros::init(argc, argv, "pick_objects_node_test");

  // Spin a thread
  MoveBaseClient mbClient("move_base", true);

  while(!mbClient.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Starting up the move_base action server...");
  }

  // Define a goal object;
  move_base_msgs::MoveBaseGoal mbGoal;

  // Set the goal's frame parameters
  mbGoal.target_pose.header.frame_id = "map";
  mbGoal.target_pose.header.stamp = ros::Time::now();
  
  typedef struct {
    float x,y,rad;
  } stGoalPose;

  std::vector<stGoalPose> vTargets= {
    {0,-3.5,0},
    {0, 3.5,0},
    {-4.5,0,0},
  };

  for (const auto& target : vTargets)  {

    // Set the target pose
    mbGoal.target_pose.pose.position.x = target.x;
    mbGoal.target_pose.pose.position.y = target.y;
    mbGoal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(target.rad);

    // Send the goal to the package
    ROS_INFO("Sending goal target...");
    mbClient.sendGoal(mbGoal);
    
    // Wait infinitely for the robot to get there (I should use a time out)
    ROS_INFO("Waiting for robot to reach its goal...");
    mbClient.waitForResult();
    
    if (mbClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Robot made it to its goal!");
    } else {
      ROS_INFO("Caramba! Robot failed to read its goal!");
    }
  }
  
  return (0);
}
