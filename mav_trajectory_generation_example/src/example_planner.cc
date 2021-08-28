#include <mav_trajectory_generation_example/example_planner.h>

ExamplePlanner::ExamplePlanner(ros::NodeHandle& nh) :
    nh_(nh),
    max_v_(2.0),
    max_a_(2.0),
    current_velocity_(Eigen::Vector3d::Zero()),
    current_pose_(Eigen::Affine3d::Identity()) {
      
  // Load params
  if (!nh_.getParam(ros::this_node::getName() + "/max_v", max_v_)){
    ROS_WARN("[example_planner] param max_v not found");
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_a", max_a_)){
    ROS_WARN("[example_planner] param max_a not found");
  }


  // create publisher for RVIZ markers
  pub_markers_ =
      nh.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);

  pub_trajectory_ =
      nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory",
                                                              0);

  // subscriber for Odometry
  sub_odom_ =
      nh.subscribe("uav_pose", 1, &ExamplePlanner::uavOdomCallback, this);

  // sub_gazebo_linkstates = nh.subscribe("/gazebo/link_states", 1, &ExamplePlanner::gazeboLinkStatesCallback, this);

  try {
    YAML::Node node = YAML::LoadFile("/home/majd/catkin_ws/src/basic_rl_agent/scripts/environmentsCreation/txtFiles/posesLocations.yaml");
    for(int i=0; i<node.size(); ++i){
      char buff[100];
      snprintf(buff, sizeof(buff), "v%d", i);
      std::string name = buff;
      std::vector<double> vi = node[name].as<std::vector<double>>();
      Vs.push_back(vi);
    }

  } catch(YAML::ParserException& e) {
      std::cout << e.what() << "\n";
  }

  // print Vs
  for(std::vector<std::vector<double>>::const_iterator v_iter=Vs.begin(); v_iter != Vs.end(); ++v_iter){
    std:: vector<double> vi = *v_iter;
    for (std::vector<double>::const_iterator i = vi.begin(); i != vi.end(); ++i)
      std::cout << *i << ' ';
    std:: cout << std::endl;
  }

}

// void ExamplePlanner::gazeboLinkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg){
//   int gate_id = -1;
//   for(int i=0; i<msg->name.size(); ++i){
//     if(msg->name[i] == "gate_14::gate_14_body")
//       gate_id = i;
//   }
//   if(gate_id == -1)
//     ROS_ERROR("gate 14 was not found!");
//   else
//     gate_pose = msg->pose[gate_id];
//   // ROS_INFO("gate position is %f, %f, %f", gate_pose.position.x, gate_pose.position.y, gate_pose.position.z);
// }

// Callback to get current Pose of UAV
void ExamplePlanner::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

  // store current position in our planner
  tf::poseMsgToEigen(odom->pose.pose, current_pose_);
  // std::cout << current_pose_.rotation()<<std::endl;
  
  currentX = odom->pose.pose.position.x;
  currentY = odom->pose.pose.position.y;
  currentZ = odom->pose.pose.position.z;

  geometry_msgs::Quaternion gmQuat = odom->pose.pose.orientation;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(gmQuat, quat);

  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  current_yaw_ = yaw;

  // store current vleocity
  tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
}

// Method to set maximum speed.
void ExamplePlanner::setMaxSpeed(const double max_v) {
  max_v_ = max_v;
}

// Plans a trajectory from the current position to the a goal position and velocity
// we neglect attitude here for simplicity
bool ExamplePlanner::planTrajectory(mav_trajectory_generation::Trajectory* trajectory) {
  ROS_INFO("planning...");
  
  // 3 Dimensional trajectory => through carteisan space, no orientation
  const int dimension = 4;

  // Array for all waypoints and their constrains
  mav_trajectory_generation::Vertex::Vector vertices;

  // Optimze up to 4th order derivative (SNAP)
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;

  // we have 2 vertices:
  // Start = current position
  // end = desired position and velocity
  mav_trajectory_generation::Vertex start(dimension), end(dimension);
   
  Eigen::Vector4d start_pos, start_vel;

  start_pos << currentX, currentY, currentZ, current_yaw_;
  start_vel << 0.0, 0.0, 0.0, 0.0;
  /******* Configure start point *******/
  // set start point constraints to current position and set all derivatives to zero
  start.makeStartOrEnd(start_pos, //current_pose_.translation(),
                       derivative_to_optimize);

  // set start point's velocity to be constrained to current velocity
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      start_vel); //current_velocity_);

  // add waypoint to list
  vertices.push_back(start);

  // for(std::vector<std::vector<double>>::const_iterator v_iter=Vs.begin(); v_iter != Vs.end(); ++v_iter){
  for(int i=0; i<Vs.size()-1; ++i){
    std::vector<double> vi = Vs[i];
    mav_trajectory_generation::Vertex middle(dimension);
    Eigen::Vector4d pose(vi[0], vi[1], vi[2], vi[3]);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, pose);
    vertices.push_back(middle);
  }

  Eigen::Vector4d goal_pos, goal_vel;
  std:: vector<double> vi = Vs[Vs.size()-1];
  goal_pos << vi[0], vi[1], vi[2], vi[3];
  goal_vel << 0.0, 0.0, 0.0, 0.0;

  end.makeStartOrEnd(goal_pos, derivative_to_optimize);
  end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, goal_vel);

  vertices.push_back(end);

  // setimate initial segment times
  std::vector<double> segment_times;
  segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

  // Set up polynomial solver with default params
  mav_trajectory_generation::NonlinearOptimizationParameters parameters;

  // set up optimization problem
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  // constrain velocity and acceleration
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

  // solve trajectory
  opt.optimize();

  // get trajectory as polynomial parameters
  opt.getTrajectory(&(*trajectory));
  return true;
}

bool ExamplePlanner::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
  // send trajectory as markers to display them in RVIZ
  visualization_msgs::MarkerArray markers;
  double distance =
      0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
  std::string frame_id = "world";

  mav_trajectory_generation::drawMavTrajectory(trajectory,
                                               distance,
                                               frame_id,
                                               &markers);
  pub_markers_.publish(markers);

  // send trajectory to be executed on UAV
  mav_planning_msgs::PolynomialTrajectory4D msg;

  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
                                                                 &msg);
  msg.header.frame_id = "world";
  pub_trajectory_.publish(msg);

  return true;
}

