/*
 * Simple example that shows a trajectory planner using
 *  mav_trajectory_generation.
 *
 *
 * Launch via
 *   roslaunch mav_trajectory_generation_example example.launch
 *
 * Wait for console to run through all gazebo/rviz messages and then
 * you should see the example below
 *  - After Enter, it receives the current uav position
 *  - After second enter, publishes trajectory information
 *  - After third enter, executes trajectory (sends it to the sampler)
 */

#include  "ros/ros.h"
#include <mav_trajectory_generation_example/example_planner.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>

int main(int argc, char** argv) {

  ros::init(argc, argv, "simple_planner");

  ros::NodeHandle n;
  // ros::Publisher posePub = n.advertise<geometry_msgs::PoseStamped>("/hummingbird/command/pose", 0);
  // ros::ServiceClient unpauseGazebo = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
  // geometry_msgs::PoseStamped poseStampedMsg; 
  // //  pose: {position: {x: 25.0, y: 20.0, z: 1.5}, orientation: {z: 0.707, w: 0.707}}}'
  // poseStampedMsg.header.stamp = ros::Time::now();
  // poseStampedMsg.header.frame_id = "world";
  // poseStampedMsg.pose.position.x = 25.0;
  // poseStampedMsg.pose.position.y = 20.0;
  // poseStampedMsg.pose.position.z = 1.5;
  // poseStampedMsg.pose.orientation.x = 0;
  // poseStampedMsg.pose.orientation.y = 0;
  // poseStampedMsg.pose.orientation.z = 0.707;
  // poseStampedMsg.pose.orientation.w = 0.707;
  // posePub.publish(poseStampedMsg);
  // for (int i = 0; i < 10; i++) {
  //   ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
  // }
  // std_srvs::Empty emptySrv;
  // unpauseGazebo.call(emptySrv);
  
  // THIS SHOULD NORMALLY RUN INSIDE ROS::SPIN!!! JUST FOR DEMO PURPOSES LIKE THIS.
  // ROS_WARN_STREAM("PRESS ENTER TO UPDATE CURRENT POSITION AND SEND TRAJECTORY");
  ExamplePlanner planner(n);
  // std::cin.get();
  ros::Duration(0.5).sleep();
  for (int i = 0; i < 10; i++) {
    ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
  }

  mav_trajectory_generation::Trajectory trajectory;
  planner.planTrajectory(&trajectory);
  planner.publishTrajectory(trajectory);

  return 0;
}