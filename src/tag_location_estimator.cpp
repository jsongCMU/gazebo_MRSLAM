#include <string>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

struct tag_coord
{
  float x = 0;
  float y = 0;
  float z = 0;
  bool is_new = false;
};

const uint num_robots = 0;
const std::string robot_prefix = "agent";
const uint num_tags = 14;
const std::string tag_prefix = "tag_";

int main(int argc, char** argv){
  ros::init(argc, argv, "tag_location_estimator");
  ros::NodeHandle nh;
  ros::Publisher pub_mk = nh.advertise<visualization_msgs::Marker>("tag_location", 10);
  tf::TransformListener listener;
  ros::Duration(3.0).sleep();
  ros::Rate rate(10.0);
  // Keeping track of 
  while (nh.ok()){
    // Get transform
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/tag_0/agent1",  
                               ros::Time(0), transform);
      // Generate and publish message
      visualization_msgs::Marker m;
      m.header.frame_id = "map";
      // label
      m.ns = "tag";
      m.id = 0;
      // shape
      m.type = 2; // Sphere
      // what to do
      m.action = 0; // Add/modify
      // pose
      m.pose.position.x = transform.getOrigin().x();
      m.pose.position.y = transform.getOrigin().y();
      m.pose.position.z = transform.getOrigin().z();
      m.pose.orientation.x = m.pose.orientation.y = m.pose.orientation.z = 0;
      m.pose.orientation.w = 1;
      // Size
      m.scale.x = 1;
      m.scale.y = 1;
      m.scale.z = 1;
      // Color
      m.color.r = 1;
      m.color.g = 1;
      m.color.b = 1;
      m.color.a = 1;
      // Lifetime
      m.lifetime = ros::Duration(0); // forever
      pub_mk.publish(m);

      rate.sleep();
    }
    catch (tf::TransformException ex){
      ROS_ERROR("[tag_location_estimator] %s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }
  return 0;
};
