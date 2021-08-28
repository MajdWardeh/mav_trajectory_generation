/*
 * Copyright (c) 2017, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Helen Oleynikova, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Rik Bähnemann, ASL, ETH Zurich, Switzerland
 * Copyright (c) 2017, Marija Popovic, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <mav_trajectory_generation_ros/trajectory_sampler_node.h>

TrajectorySamplerNode::TrajectorySamplerNode(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      publish_whole_trajectory_(false),
      dt_(0.01),
      current_sample_time_(0.0) {
  nh_private_.param("publish_whole_trajectory", publish_whole_trajectory_,
                    publish_whole_trajectory_);
  nh_private_.param("dt", dt_, dt_);

  command_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
  trajectory_sub_ = nh_.subscribe(
      "path_segments", 10, &TrajectorySamplerNode::pathSegmentsCallback, this);
  trajectory4D_sub_ = nh_.subscribe(
      "path_segments_4D", 10, &TrajectorySamplerNode::pathSegments4DCallback, this);
  
  getTrajectoryChunk_sub_ = nh_.subscribe(
      "getTrajectoryChunk", 10, &TrajectorySamplerNode::samplePartialTrajectoryCallback, this);
  sampledTrajectoryChunk_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
      "sampledTrajectoryChunk", 50);
  stop_srv_ = nh_.advertiseService(
      "stop_sampling", &TrajectorySamplerNode::stopSamplingCallback, this);
  position_hold_client_ =
      nh_.serviceClient<std_srvs::Empty>("back_to_position_hold");

  const bool oneshot = false;
  const bool autostart = false;
  publish_timer_ = nh_.createTimer(ros::Duration(dt_),
                                   &TrajectorySamplerNode::commandTimerCallback,
                                   this, oneshot, autostart);
}

TrajectorySamplerNode::~TrajectorySamplerNode() { publish_timer_.stop(); }

void TrajectorySamplerNode::pathSegmentsCallback(
    const mav_planning_msgs::PolynomialTrajectory& segments_message) {
  if (segments_message.segments.empty()) {
    ROS_WARN("Trajectory sampler: received empty waypoint message");
    return;
  } else {
    ROS_INFO("Trajectory sampler: received %lu waypoints",
             segments_message.segments.size());
  }

    bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(
        segments_message, &trajectory_);
    if (!success) {
      return;
    }
    processTrajectory();
}

void TrajectorySamplerNode::pathSegments4DCallback(
    const mav_planning_msgs::PolynomialTrajectory4D& segments_message) {
  if (segments_message.segments.empty()) {
    ROS_WARN("Trajectory sampler: received empty waypoint message");
    return;
  } else {
    ROS_INFO("Trajectory sampler: received %lu waypoints",
             segments_message.segments.size());
  }

    bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(
        segments_message, &trajectory_);
    if (!success) {
      return;
    }
    processTrajectory();
}

void TrajectorySamplerNode::processTrajectory() {
  // Call the service call to takeover publishing commands.
  if (position_hold_client_.exists()) {
    std_srvs::Empty empty_call;
    position_hold_client_.call(empty_call);
  }

  if (publish_whole_trajectory_) {
    // Publish the entire trajectory at once.
    mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
    mav_trajectory_generation::sampleWholeTrajectory(trajectory_, dt_,
                                                     &trajectory_points);
    trajectory_msgs::MultiDOFJointTrajectory msg_pub;
    msgMultiDofJointTrajectoryFromEigen(trajectory_points, &msg_pub);
    command_pub_.publish(msg_pub);
  } else {
    publish_timer_.start();
    current_sample_time_ = 0.0;
    start_time_ = ros::Time::now();
    start_time_seconds_ = start_time_.toSec();
  }
}

bool TrajectorySamplerNode::stopSamplingCallback(
    std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response) {
  publish_timer_.stop();
  return true;
}

void TrajectorySamplerNode::commandTimerCallback(const ros::TimerEvent&) {
  if (current_sample_time_ <= trajectory_.getMaxTime()) {
    trajectory_msgs::MultiDOFJointTrajectory msg;
    mav_msgs::EigenTrajectoryPoint trajectory_point;
    bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_, current_sample_time_, &trajectory_point);
    if (!success) {
      publish_timer_.stop();
    }
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
    msg.points[0].time_from_start = ros::Duration(current_sample_time_);
    command_pub_.publish(msg);
    current_sample_time_ += dt_;
  } else {
    publish_timer_.stop();
  }
}

void TrajectorySamplerNode::samplePartialTrajectoryCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  // coded by Majd Wardeh
  double ts_rostime = msg->data[0];
  double ts = ts_rostime - start_time_seconds_;
  int numOfSamples = int(msg->data[1]);
  double dt = msg->data[2];
  double tend = ts + dt*numOfSamples;
  if (tend > trajectory_.getMaxTime()) {
    ROS_ERROR("tend is out of trajectory's max time. returning...");
    return;
  } 
  std::vector<double> Vx, Vy, Vz, Vyaw;
  double tk = ts;
  for(int i=0; i<numOfSamples; ++i){
    mav_msgs::EigenTrajectoryPoint trajectory_point;
    bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_, tk, &trajectory_point);
    if (success) {
        Vx.push_back(trajectory_point.position_W[0]);
        Vy.push_back(trajectory_point.position_W[1]);
        Vz.push_back(trajectory_point.position_W[2]);
        Vyaw.push_back(trajectory_point.getYaw());
    }
    else{
      ROS_ERROR("unsuccessful sampling in samplePartialTrajectoryCallback");
      return;
    }
    tk += dt;
  }
  std_msgs::Float64MultiArray msg_to_send;
  std::vector <double> v;
  // v holds all the data, the first double in the array represents ts_rostime; to identify the received message.
  v.push_back(ts_rostime);
  for(int i=0; i<Vx.size(); ++i){
    v.push_back(Vx[i]);
    v.push_back(Vy[i]);
    v.push_back(Vz[i]);
    v.push_back(Vyaw[i]);
  }
  msg_to_send.data = v; 
  sampledTrajectoryChunk_pub_.publish(msg_to_send);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_sampler_node");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  TrajectorySamplerNode trajectory_sampler_node(nh, nh_private);
  ROS_INFO("Initialized trajectory sampler.");
  ros::spin();
}
