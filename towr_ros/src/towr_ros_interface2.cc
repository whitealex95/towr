#include <towr_ros/towr_ros_interface2.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <vector>
#include <string> 

#include <xpp_states/convert.h>
#include <xpp_msgs/topic_names.h>
#include <xpp_msgs/TerrainInfo.h>

#include <towr/terrain/height_map.h>
#include <towr/variables/euler_converter.h>
#include <towr_ros/topic_names.h>
#include <towr_ros/towr_xpp_ee_map.h>



namespace towr {

TowrRosInterface2::TowrRosInterface2 ()
{
  ::ros::NodeHandle n;
  user_command2_sub_ = n.subscribe(towr_msgs::user_command2, 1,
                                  &TowrRosInterface2::UserCommandCallback, this);
  bag_command_pub_ = n.advertise<std_msgs::String>("/towr/ros_vis_traj", 1);
  
  initial_state_pub_  = n.advertise<xpp_msgs::RobotStateCartesian>
                                          (xpp_msgs::robot_state_desired, 1);
  robot_parameters_pub_  = n.advertise<xpp_msgs::RobotParameters>
                                    (xpp_msgs::robot_parameters, 1);
  solver_ = std::make_shared<ifopt::IpoptSolver>();
  // solver_ = std::make_shared<ifopt::SnoptSolver>();
  visualization_dt_ = 0.01;
}

BaseState
TowrRosInterface2::GetGoalState(const TowrCommand2Msg& msg) const
{
  ::std::cout << "This should be useless!\n";
  BaseState goal;
  goal.lin.at(kPos) = xpp::Convert::ToXpp(msg.goal_lin.pos); // return vector3d
  goal.lin.at(kVel) = xpp::Convert::ToXpp(msg.goal_lin.vel);
  goal.ang.at(kPos) = xpp::Convert::ToXpp(msg.goal_ang.pos);
  goal.ang.at(kVel) = xpp::Convert::ToXpp(msg.goal_ang.vel);
  return goal;
}

::std::vector<BaseState>
TowrRosInterface2::GetGoalStateList(const TowrCommand2Msg& msg) const
{ 
  ::std::vector<BaseState> goal_list(2);
  for(int j=0; j<2; j++){
    if(j==0){
      goal_list[j].lin.at(kPos) = xpp::Convert::ToXpp(msg.goal_lin.pos); // return vector3d
      goal_list[j].lin.at(kVel) = xpp::Convert::ToXpp(msg.goal_lin.vel);
      goal_list[j].ang.at(kPos) = xpp::Convert::ToXpp(msg.goal_ang.pos);
      goal_list[j].ang.at(kVel) = xpp::Convert::ToXpp(msg.goal_ang.vel);
    }
    else if(j==1){
      goal_list[j].lin.at(kPos) = xpp::Convert::ToXpp(msg.goal_lin2.pos); // return vector3d
      goal_list[j].lin.at(kVel) = xpp::Convert::ToXpp(msg.goal_lin2.vel);
      goal_list[j].ang.at(kPos) = xpp::Convert::ToXpp(msg.goal_ang2.pos);
      goal_list[j].ang.at(kVel) = xpp::Convert::ToXpp(msg.goal_ang2.vel);
    }
  }
  return goal_list;
}

void
TowrRosInterface2::UserCommandCallback(const TowrCommand2Msg& msg)
{
  auto goal_state_list = GetGoalStateList(msg);
  ::std::cout << "Successfully obtained Goal State\n";
  int n_goals = goal_state_list.size();

  ::std::vector<NlpFormulation> formulation_list(2);
  ::std::vector<ifopt::Problem> nlp_list(2);
  ::std::vector<SplineHolder> solution_list(2);

  State::VectorXd start_base_linear(3);
  State::VectorXd start_base_angular(3);
  ::std::vector<State::VectorXd> start_ee_W;

  // Todo: Add from Topic. Currently auto-generated
  // start_base_linear << 0, 0, 0.58; // z-position NOT-ignored
  // start_base_angular << 0, 0, 0;
  // set start_ee_W later
  
  double t_final;
  ::std::cout.precision(2);
  for(int j=0; j < n_goals; j++) {
    formulation_ = formulation_list[j];
    solution = solution_list[j];
    nlp_ = nlp_list[j];

    ::std::cout <<"============================\nFormulating nlp for goal : " 
                << j << "\n============================\n";

    formulation_.model_ = RobotModel(static_cast<RobotModel::Robot>(msg.robot));
    auto robot_params_msg = BuildRobotParametersMsg(formulation_.model_);
    robot_parameters_pub_.publish(robot_params_msg);
    auto terrain_id = static_cast<HeightMap::TerrainID>(msg.terrain);
    formulation_.terrain_ = HeightMap::MakeTerrain(terrain_id);
    int n_ee = formulation_.model_.kinematic_model_->GetNumberOfEndeffectors();
    formulation_.params_ = GetTowrParameters(n_ee, msg);
    formulation_.initial_base_.lin.at(towr::kPos) << start_base_linear;
    // formulation_.initial_base_.lin.at(towr::kPos) << 0, 0;
    formulation_.initial_base_.ang.at(towr::kPos) << start_base_angular;
    
    // I don't know how it matters... ???
    formulation_.final_base_ = goal_state_list[j];


    ::std::cout << "[Q1]" << j << "\n";
    // Initialize Correct start pose for first iteration
    if(j==0) {
      SetTowrInitialState();
      // solver parameters
      SetIpoptParameters(msg);
      // visualization
      PublishInitialState();
    }
    else{
      // Start from end of previous trajectory
      for(int ee_id=0; ee_id < n_ee; ee_id++){
        formulation_.initial_ee_W_.push_back(start_ee_W[ee_id]);
      }
      // solver parameters
      SetIpoptParameters(msg);
    }
    // Defaults to /home/user/.ros/
    std::string bag_file = "towr_trajectory-" + ::std::to_string(j) + ".bag";
    if (msg.optimize || msg.play_initialization) {
      ::std::cout << "Setting up nlp for goal : " << j << "\n";
      // nlp_ = ifopt::Problem();
      for (auto c : formulation_.GetVariableSets(solution))
        nlp_.AddVariableSet(c);
      for (auto c : formulation_.GetConstraints(solution))
        nlp_.AddConstraintSet(c);
      for (auto c : formulation_.GetCosts())
        nlp_.AddCostSet(c);

      solver_->Solve(nlp_);
      ::std::cout << "Solve Complete.\nSaving Rosbag for : " << j << "\n";
      SaveOptimizationAsRosbag(bag_file, robot_params_msg, msg, false);
    }

    // playback using terminal commands
    if (msg.replay_trajectory || msg.play_initialization || msg.optimize) {
      // This blocks...
      ::std::string command =  ("rosbag play --topics "
          + xpp_msgs::robot_state_desired + " "
          + xpp_msgs::terrain_info
          + " -r " + std::to_string(msg.replay_speed)
          + " --quiet " + bag_file);
          // + " &");// & can be added for async
      ::std::cout << command << ::std::endl;
      // just publish command instead of running it itself
      std_msgs::String str;
      str.data = command;
      bag_command_pub_.publish(str);
      // int success = system(command.c_str());
    }

    // Prepare for next iteration
    t_final = solution.base_linear_->GetTotalTime();
    ::std::cout << "Set up next base start state\n";
    start_base_linear = solution.base_linear_->GetPoint(t_final).p();
    start_base_angular = solution.base_angular_->GetPoint(t_final).p();
    start_ee_W.clear();
    ::std::cout << "Set up next ee start state (n_ee: " << n_ee << ")\n";
    for(int ee_id=0; ee_id < n_ee; ee_id++){
      start_ee_W.push_back(solution.ee_motion_.at(ee_id)->GetPoint(t_final).p());
    }
    for(int ee_id=0; ee_id < n_ee; ee_id++){
      ::std::cout << start_ee_W[ee_id].transpose() <<"\n";
    }
  }
}

void
TowrRosInterface2::PublishInitialState()
{
  int n_ee = formulation_.initial_ee_W_.size();
  xpp::RobotStateCartesian xpp(n_ee);
  xpp.base_.lin.p_ = formulation_.initial_base_.lin.p();
  xpp.base_.ang.q  = EulerConverter::GetQuaternionBaseToWorld(formulation_.initial_base_.ang.p());

  for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
    int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;
    xpp.ee_contact_.at(ee_xpp)   = true;
    xpp.ee_motion_.at(ee_xpp).p_ = formulation_.initial_ee_W_.at(ee_towr);
    xpp.ee_forces_.at(ee_xpp).setZero(); // zero for visualization
  }

  initial_state_pub_.publish(xpp::Convert::ToRos(xpp));
}

std::vector<TowrRosInterface2::XppVec>
TowrRosInterface2::GetIntermediateSolutions ()
{
  std::vector<XppVec> trajectories;

  for (int iter=0; iter<nlp_.GetIterationCount(); ++iter) {
    nlp_.SetOptVariables(iter);
    trajectories.push_back(GetTrajectory());
  }

  return trajectories;
}

TowrRosInterface2::XppVec
TowrRosInterface2::GetTrajectory () const
{
  XppVec trajectory;
  double t = 0.0;
  double T = solution.base_linear_->GetTotalTime();

  EulerConverter base_angular(solution.base_angular_);

  while (t<=T+1e-5) {
    int n_ee = solution.ee_motion_.size();
    xpp::RobotStateCartesian state(n_ee);

    state.base_.lin = ToXpp(solution.base_linear_->GetPoint(t));

    state.base_.ang.q  = base_angular.GetQuaternionBaseToWorld(t);
    state.base_.ang.w  = base_angular.GetAngularVelocityInWorld(t);
    state.base_.ang.wd = base_angular.GetAngularAccelerationInWorld(t);

    for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
      int ee_xpp = ToXppEndeffector(n_ee, ee_towr).first;

      state.ee_contact_.at(ee_xpp) = solution.phase_durations_.at(ee_towr)->IsContactPhase(t);
      state.ee_motion_.at(ee_xpp)  = ToXpp(solution.ee_motion_.at(ee_towr)->GetPoint(t));
      state.ee_forces_ .at(ee_xpp) = solution.ee_force_.at(ee_towr)->GetPoint(t).p();
    }

    state.t_global_ = t;
    trajectory.push_back(state);
    t += visualization_dt_;
  }

  return trajectory;
}

xpp_msgs::RobotParameters
TowrRosInterface2::BuildRobotParametersMsg(const RobotModel& model) const
{
  xpp_msgs::RobotParameters params_msg;
  auto max_dev_xyz = model.kinematic_model_->GetMaximumDeviationFromNominal();
  params_msg.ee_max_dev = xpp::Convert::ToRos<geometry_msgs::Vector3>(max_dev_xyz);

  auto nominal_B = model.kinematic_model_->GetNominalStanceInBase();
  int n_ee = nominal_B.size();
  for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
    Vector3d pos = nominal_B.at(ee_towr);
    params_msg.nominal_ee_pos.push_back(xpp::Convert::ToRos<geometry_msgs::Point>(pos));
    params_msg.ee_names.push_back(ToXppEndeffector(n_ee, ee_towr).second);
  }

  params_msg.base_mass = model.dynamic_model_->m();

  return params_msg;
}

void
TowrRosInterface2::SaveOptimizationAsRosbag (const std::string& bag_name,
                                   const xpp_msgs::RobotParameters& robot_params,
                                   const TowrCommand2Msg user_command2_msg,
                                   bool include_iterations)
{
  rosbag::Bag bag;
  bag.open(bag_name, rosbag::bagmode::Write);
  ::ros::Time t0(1e-6); // t=0.0 throws ROS exception

  // save the a-priori fixed optimization variables
  bag.write(xpp_msgs::robot_parameters, t0, robot_params);
  bag.write(towr_msgs::user_command2+"_saved", t0, user_command2_msg);

  // save the trajectory of each iteration
  if (include_iterations) {
    auto trajectories = GetIntermediateSolutions();
    int n_iterations = trajectories.size();
    for (int i=0; i<n_iterations; ++i)
      SaveTrajectoryInRosbag(bag, trajectories.at(i), towr_msgs::nlp_iterations_name + std::to_string(i));

    // save number of iterations the optimizer took
    std_msgs::Int32 m;
    m.data = n_iterations;
    bag.write(towr_msgs::nlp_iterations_count, t0, m);
  }

  // save the final trajectory
  auto final_trajectory = GetTrajectory();
  SaveTrajectoryInRosbag(bag, final_trajectory, xpp_msgs::robot_state_desired);

  bag.close();
}

void
TowrRosInterface2::SaveTrajectoryInRosbag (rosbag::Bag& bag,
                                 const XppVec& traj,
                                 const std::string& topic) const
{
  for (const auto state : traj) {
    auto timestamp = ::ros::Time(state.t_global_ + 1e-6); // t=0.0 throws ROS exception

    xpp_msgs::RobotStateCartesian msg;
    msg = xpp::Convert::ToRos(state);
    bag.write(topic, timestamp, msg);

    xpp_msgs::TerrainInfo terrain_msg;
    for (auto ee : state.ee_motion_.ToImpl()) {
      Vector3d n = formulation_.terrain_->GetNormalizedBasis(HeightMap::Normal, ee.p_.x(), ee.p_.y());
      terrain_msg.surface_normals.push_back(xpp::Convert::ToRos<geometry_msgs::Vector3>(n));
      terrain_msg.friction_coeff = formulation_.terrain_->GetFrictionCoeff();
    }

    bag.write(xpp_msgs::terrain_info, timestamp, terrain_msg);
  }
}

} /* namespace towr */

