
// Created by: Kris Gunnarsson
// Date: April 17, 2022

// Purpose: This file was created to implement the Home Service Robot project.
//          The robot will move to a sequence of locations.
//          The robot will take its queues from parameters posted on rosmaster.
//          The robot will also post its state to the master.
//          In brief,
//            After start up robot will post robot_ready=true;
//            Robot will wait until target_set==true in master
//            Robot will obtain target location from pikk_x and pikk_y values
//              posted on master
//            Robot will post robot_ready=false and move to the target
//            Robot will post robot_atTarget=true once at target
//            Robot will post robot_ready=true and wait for another target
//              to be set

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

  ros::init(argc, argv, "pick_objects_node");
  ros::NodeHandle rHandle;

  // Spin a thread
  MoveBaseClient mbClient("move_base", true);

  while(!mbClient.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Starting up the move_base action server...");
  }


  // INITIALIZE
  //           robot is ready to accept a location
  ROS_INFO("Robot posting ready on rosmaster");
  rHandle.setParam("/robot_ready", true);
  ros::spinOnce();


  // SYNCH UP with MARKER
  //
  ROS_INFO("Waiting for Marker to be ready...");
  int throttle = 5;
  while(!rHandle.hasParam("/target_set")) {

    if (--throttle < 0) {
      ROS_INFO("...still waiting for target to be available");
      throttle = 5;
    }
    
    if (!ros::ok()) { return (1); }
    sleep(2);
  }
  

  // We'll cycle indefinitely
  while(ros::ok()) {


    // READY
    //      for the next target location
    ROS_INFO("***Robot is ready to accept a target location...");
    rHandle.setParam("/robot_ready", true);
    ros::spinOnce();

    // DEBUG state of marker
    bool isTargetSet = false;
    bool ok =!rHandle.getParam("/target_set", isTargetSet);
    ROS_INFO_STREAM(" is target set ? " << isTargetSet );

    int throttle = 5;
    bool target_set = false;
    while(!target_set) {
      if (!rHandle.getParam("/target_set", target_set)) {
	ROS_INFO(" Error! failed to get '/robot_ready'. Bye...");
	return (1);
      }

      if (--throttle < 0) {
	ROS_INFO("...still waiting for target to be set");
	throttle = 5;
      }

      if (!ros::ok()) { return (1); }
      sleep(2);
    }
    ROS_INFO_STREAM("..ok target is set (" << target_set);
    

    // READ
    //     the pickup location
    double pikk_x;
    rHandle.getParam("/pikk_x", pikk_x);
    double pikk_y;
    rHandle.getParam("/pikk_y", pikk_y);
    ROS_INFO_STREAM(" new target =" << pikk_x << ", " << pikk_y);
    
    // We're no longer ready (for a target)
    rHandle.setParam("/robot_ready", false);
    rHandle.setParam("/target_set", false);
    ros::spinOnce();

    
    // GOAL OBJECT
    move_base_msgs::MoveBaseGoal mbGoal;

    // Set the goal's frame parameters
    mbGoal.target_pose.header.frame_id = "map";
    mbGoal.target_pose.header.stamp = ros::Time::now();
    
    // Set the target pose
    mbGoal.target_pose.pose.position.x = pikk_x;
    mbGoal.target_pose.pose.position.y = pikk_y;
    float rad = 0;
    mbGoal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(rad);
    
    // Send the goal to the package
    ROS_INFO("Robot sending goal target...");
    mbClient.sendGoal(mbGoal);

    // Wait infinitely for the robot to get there (I should use a time out)
    ROS_INFO("Waiting for robot to reach its goal...");
    mbClient.waitForResult();
    
    if (mbClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Robot made it to its goal!");
      rHandle.setParam("/robot_atTarget", true);
      ros::spinOnce();

    } else {
      ROS_INFO("Caramba! Robot failed to read its goal!");
      break;
    }

  }
  

  return (0);
}
