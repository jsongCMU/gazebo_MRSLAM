#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"

const std::string base_frame = "/agent1/camera";
const std::string tag_frame = "/tag_1/agent1";
const std::string topic_name = "/robosar_agent_bringup_node/agent1/feedback/apriltag";

int main(int argc, char** argv){
  ros::init(argc, argv, "apriltag_publisher");
  ros::NodeHandle nh;
  ros::Publisher pub_april = nh.advertise<apriltag_ros::AprilTagDetectionArray>(topic_name, 10);
  tf::TransformListener tf_listener;
  ros::Duration(3.0).sleep();
  ros::Rate rate(10.0);
  
  while (nh.ok()){
    tf::StampedTransform transform;
    bool transform_found = true;
    // Try all possible transform combinations of tags and agents
    try{
      tf_listener.lookupTransform(base_frame, tag_frame, ros::Time(0), transform);
      transform_found = true;
    }
    catch (tf::TransformException ex){
      transform_found = false;
    }
    if(transform_found){
      // Found transform; generate and publish message
      apriltag_ros::AprilTagDetectionArray msg;
      msg.header.stamp = transform.stamp_;
      msg.header.frame_id = base_frame;

      apriltag_ros::AprilTagDetection detection;
      detection.id.push_back(1);
      detection.size.push_back(0);
      detection.pose.pose.pose.position.x = transform.getOrigin().getX();
      detection.pose.pose.pose.position.y = transform.getOrigin().getY();
      detection.pose.pose.pose.position.z = transform.getOrigin().getZ();

      msg.detections.push_back(detection);
      
      // publish
      pub_april.publish(msg);
      rate.sleep();
    }
  }
  return 0;
};
