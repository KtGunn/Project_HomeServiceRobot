
// Created by: Kris Gunnarsson
// Date: April 17, 2022

// Purpose: This file was created to implement the Home Service Robot project.
//          The idea is to imitate a robot picking an object from one location and
//          transfering it to another. Visible markers in the map simulate the object.
//          In brief,
//            The marker node will place a marker in a location on the map.
//            It will signal the robot to move to that location.
//            When robot signals it has arrived, the marker will delete the object.
//            There will be a short pause imitating the object being picked up.
//            The robot will be instructed to move to another map location.
//            Once the robot signals arrival at that location the object reappears.
//            The robot will move away.
//
//            Signaling between the nodes will occur through parameters in the master
//            The marker node will use target_set=true to signal location is set
//            target_x and target_y will indicate the location coordinates.


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers_node");
  ros::NodeHandle rHandle;
  ROS_INFO("STARTING Add_marke!");


  // POST
  //     Target is not set
  ROS_INFO("Add_marker posting target is NOT set.");
  rHandle.setParam("/target_set", false);
  ros::spinOnce();


  // WAIT
  //     until there's at least one listener (rviz is expected)
  ROS_INFO("Add_marker waiting for a subscriber.");

  // Create a  publishers for 'mark'
  ros::Publisher pubMarker = rHandle.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  int throttle = 5;
  while (pubMarker.getNumSubscribers() < 1) {
    if (--throttle < 0) {
      ROS_INFO("..Please create a subscriber to the marker");
      throttle = 5;
    }
    // Bail if ROS is down
    if (!ros::ok()) { return (1); }
    sleep(5);
  }


  // WAIT
  //     until robot signals it is ready
  ROS_INFO("Add_marker waiting for robot to be ready.");
  throttle = 5;
  while(!rHandle.hasParam("/robot_ready")) {
    if (--throttle < 0) {
      ROS_INFO("...still waiting for robot to be ready");
      throttle = 5;
    }

    // Bail if ROS is down
    if (!ros::ok()) { return (1); }
    sleep(2);
  }


  // VECTOR
  //       Target locations to place the object
  typedef struct {
    double x,y,rad;
  } tdsTarget;

  std::vector<tdsTarget> vTargets= {
    {0,0,0},
    {-4.0, -3.5, 1.5}
  };
  

  // VECTOR 
  //       We 'place' and 'remove' the object
  std::vector<std::string> vAction = {
    std::string("place"),
    std::string("remove"),
    std::string("pause")
  };
  

  // MARKER
  //        Define a marker message
  visualization_msgs::Marker marker;

  // I'll use a cube
  uint32_t object = visualization_msgs::Marker::CUBE;
  marker.type = object;

  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "marker_test";
  
  // Set the scale of the marker to 1x1x1
  marker.scale.x = 0.5;
  marker.scale.y = 0.3;
  marker.scale.z = 1.0;
  
  // Set the color to green
  marker.color.r = 0.6f;
  marker.color.g = 1.0f;
  marker.color.b = 0.2f;
  marker.color.a = 1.0;
  
  // This means 'never auto-delete'
  marker.lifetime = ros::Duration();


  // We'll move the marker back and forth
  int viAction = 0;
  int viTarget = 0;
  
  while (ros::ok()) {

    // GET next action
    std::string actionNow = vAction[viAction++];
    viAction = viAction >= 3 ? 0 : viAction;
    ROS_INFO_STREAM( "*** Next action = " << actionNow.c_str());

    
    // DEBUG state of robot
    bool isRobReady = false;
    bool okR = rHandle.getParam("/robot_ready", isRobReady);
    bool isRobAtTarget = false;
    bool okT = rHandle.getParam("/robot_atTarget", isRobAtTarget);
    ROS_INFO_STREAM(" rReady ? "<< isRobReady << " atTarg ? " << isRobAtTarget);
    

    /// PLACE THE MARKER
    //
    if (actionNow.compare("place") == 0) {

      // WAIT for robot ready
      ROS_INFO("Waiting for robot ready prior to placing marker");
      bool ready = false;
      while (!ready) {
	if (!rHandle.getParam("/robot_ready", ready)) {
	  ROS_INFO(" Error! failed to get '/robot_ready'. Bye...");
	  return (1);
	}
	sleep (2);
      }
      ROS_INFO("..Placing marker for PICK UP");

      tdsTarget target = vTargets[viTarget++];
      marker.pose.position.x = target.x;
      marker.pose.position.y = target.y;
      marker.pose.position.z = 1.25;
      
      // Publish the marker
      marker.action = visualization_msgs::Marker::ADD;
      pubMarker.publish(marker);
      
      ROS_INFO_STREAM("..Marker set at (" << target.x << ", " << target.y << ")");

      // POST target
      rHandle.setParam("/pikk_x", target.x);
      rHandle.setParam("/pikk_y", target.y);
      rHandle.setParam("/target_set", true);
      ros::spinOnce();
      
      // Move the target pointer
      viTarget = viTarget >=2 ? 0 : viTarget;
    }


    if (actionNow.compare("pause") == 0) {

      // PAUSING
      //        robot is picking up the object
      ROS_INFO("Pausing :: robot is picking up object...");
      sleep (5);
      ROS_INFO("..done Pausing :: robot has picked up object");
    }


    if (actionNow.compare("remove") == 0) {

      // ANNOUNCE
      //         aiting for robot and dropping '/target_set'
      ROS_INFO("Waiting for robot to arrive at pickup point" );

      rHandle.setParam("/target_set", false);
      ros::spinOnce();
      
      // WAIT
      //     for robot at target
      bool attarget = false;
      while (!attarget) {
	if (!rHandle.getParam("/robot_atTarget", attarget)) {
	  ROS_INFO(" Error! failed to get '/robot_atTarget'. Bye...");
	  return (1);
	}
	sleep (2);
      }
      ROS_INFO_STREAM( "..robot has reached pickup point (" << attarget << ")" );


      // WAIT
      //     for robot ready (after reaching target)
      bool ready = false;
      while (!ready) {
	if (!rHandle.getParam("/robot_ready", ready)) {
	  ROS_INFO(" Error! failed to get '/robot_ready'. Bye...");
	  return (1);
	}
	sleep (2);
      }
      ROS_INFO_STREAM( "..robot is ready at pickup point (" << attarget << ")" );
      ROS_INFO(" Removing marker");

      // REMOVE marker
      marker.action = visualization_msgs::Marker::DELETE;
      pubMarker.publish(marker);
    }

    // Wait a little
    sleep (2);

  }
  

  
  return (0);
}
