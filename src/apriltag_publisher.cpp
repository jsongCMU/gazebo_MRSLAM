#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"

const std::string base_frame = "agent1/base_link";
const std::string tag_frame = "tag_1/agent1";
const std::string topic_name = "/robosar_agent_bringup_node/agent1/feedback/apriltag";

int main(int argc, char** argv){
  ros::init(argc, argv, "apriltag_publisher");
  ros::NodeHandle nh;
  ros::Publisher pub_april = nh.advertise<apriltag_ros::AprilTagDetectionArray>(topic_name, 10);
  tf::TransformListener tf_listener;
  ros::Duration(3.0).sleep();
  ros::Rate rate(10.0);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  while (nh.ok()){
    // Get transform
    geometry_msgs::TransformStamped transformStamped;
    bool transform_found = false;
    try{
      transformStamped = tfBuffer.lookupTransform(base_frame, tag_frame, ros::Time(0));
      double secs = (ros::Time::now()-transformStamped.header.stamp).toSec();
      if(secs < 1.0)
        transform_found = true;
    }
    catch (tf2::TransformException &ex) {
      transform_found = false;
    }
    // If transform found
    if(transform_found){
      // Found transform; generate and publish message
      apriltag_ros::AprilTagDetectionArray msg;
      msg.header = transformStamped.header;

      apriltag_ros::AprilTagDetection detection;
      detection.id.push_back(1);
      detection.size.push_back(0);
      detection.pose.pose.pose.position.x = transformStamped.transform.translation.x;
      detection.pose.pose.pose.position.y = transformStamped.transform.translation.y;
      detection.pose.pose.pose.position.z = transformStamped.transform.translation.z;

      msg.detections.push_back(detection);
      
      // publish
      pub_april.publish(msg);
      rate.sleep();
    }
  }
  return 0;
};
