
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers_node_test");
  ros::NodeHandle rHandle;

  // Create a  publishers for 'mark'
  ros::Publisher pubMarker = rHandle.advertise<visualization_msgs::Marker>("visualization_marker", 1);


  // WAIT
  //     until there's at least one listener (rviz is expected)
  while (pubMarker.getNumSubscribers() < 1) {
    // We'll bail if ROS is down
    if (!ros::ok()) {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(5);
  }


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
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  
  // Set the color to green
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  
  // This means 'never auto-delete'
  marker.lifetime = ros::Duration();


  // VECTOR :: Target locations
  //           Target locations to place the object
  typedef struct {
    float x,y,rad;
  } tdsTarget;

  std::vector<tdsTarget> vTargets= {
    {0,0,0},
    {-4.0, -3.5, 1.5}
  };
  

  // VECTOR :: Action
  //           We 'place' and 'remove' the object
  std::vector<std::string> vAction = {
    std::string("place"),
    std::string("remove")
  };
  

  // We'll move the marker back and forth
  int viAction = 0;
  int viTarget = 0;
  
  while (ros::ok()) {
    
    ROS_INFO(" New action ");

    std::string actionNow = vAction[viAction++];
    viAction = viAction >= 2 ? 0 : viAction;
    ROS_INFO_STREAM( " New action= " << actionNow.c_str());
    
    if (actionNow.compare("place") == 0) {
      tdsTarget target = vTargets[viTarget++];
      ROS_INFO("Place 1a ");
      marker.pose.position.x = target.x;
      marker.pose.position.y = target.y;
      marker.pose.position.z = 0.25;
      
      // Publish the marker
      marker.action = visualization_msgs::Marker::ADD;
      pubMarker.publish(marker);
      
      // Move the target pointer
      viTarget = viTarget >=2 ? 0 : viTarget;

    }
    if (actionNow.compare("remove") == 0) {
      
      // Publish the marker
      marker.action = visualization_msgs::Marker::DELETE;
      pubMarker.publish(marker);
    }

    ROS_INFO(" Sleeping ");
    sleep (5);
  }

  return (0);
}
