#ifndef MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H
#define MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H

#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include "tf/transform_datatypes.h"

class ExamplePlanner {
 public:
  ExamplePlanner(ros::NodeHandle& nh);

  void gazeboLinkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);

  void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);

  void setMaxSpeed(double max_v);

  // Plans a trajectory to take off from the current position and
  // fly to the given altitude (while maintaining x,y, and yaw).
  bool planTrajectory(mav_trajectory_generation::Trajectory* trajectory);
                      
  // bool planTrajectory(const Eigen::VectorXd& goal_pos,
  //                     const Eigen::VectorXd& goal_vel,
  //                     const Eigen::VectorXd& start_pos,
  //                     const Eigen::VectorXd& start_vel,
  //                     double v_max, double a_max,
  //                     mav_trajectory_generation::Trajectory* trajectory);
                      
  bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

 private:
  ros::Publisher pub_markers_;
  ros::Publisher pub_trajectory_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_gazebo_linkstates;

  ros::NodeHandle& nh_;
  Eigen::Affine3d current_pose_;
  Eigen::Vector3d current_velocity_;
  Eigen::Vector3d current_angular_velocity_;
  double current_yaw_;
  double currentX, currentY, currentZ;
  double max_v_; // m/s
  double max_a_; // m/s^2
  double max_ang_v_;
  double max_ang_a_;
  geometry_msgs::Pose gate_pose;

  std::vector<std::vector<double>> Vs;

};

#endif // MAV_TRAJECTORY_GENERATION_EXAMPLE_PLANNER_H
