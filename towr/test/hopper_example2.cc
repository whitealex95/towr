/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <cmath>
#include <iostream>

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>
// #include <ifopt/snopt_solver.h>

// #include <rosbag/bag.h> // for saving

// #include <xpp_states/convert.h>
// #include <xpp_msgs/topic_names.h>
// #include <xpp_msgs/TerrainInfo.h>



using namespace towr;

// A minimal example how to build a trajectory optimization problem using TOWR.
//
// The more advanced example that includes ROS integration, GUI, rviz
// visualization and plotting can be found here:
// towr_ros/src/towr_ros_app.cc


void plan1(ifopt::Problem& nlp, SplineHolder& solution, double init_x, double init_y, double fin_x, double fin_y){
  NlpFormulation formulation;

  formulation.terrain_ = std::make_shared<FlatGround>(0.0);
  formulation.model_ = RobotModel(RobotModel::Monoped);
  formulation.initial_base_.lin.at(towr::kPos) << init_x, init_y, 0.5;
  formulation.initial_ee_W_.push_back(Eigen::Vector3d(init_x, init_y, 0));
  formulation.final_base_.lin.at(towr::kPos) << fin_x, fin_y, 0.5;

  formulation.params_.ee_phase_durations_.push_back({0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.2});
  formulation.params_.ee_in_contact_at_start_.push_back(true);
  for (auto c : formulation.GetVariableSets(solution))
    nlp.AddVariableSet(c);
  for (auto c : formulation.GetConstraints(solution))
    nlp.AddConstraintSet(c);
  for (auto c : formulation.GetCosts())
    nlp.AddCostSet(c);
  
  auto solver = std::make_shared<ifopt::IpoptSolver>();
  solver->SetOption("jacobian_approximation", "exact");
  solver->SetOption("max_cpu_time", 20.0);
  solver->Solve(nlp);
}

void plan2(ifopt::Problem& nlp, SplineHolder& solution, SplineHolder& last_solution, double fin_x, double fin_y){
  double t = last_solution.base_linear_->GetTotalTime();

  NlpFormulation formulation;

  formulation.terrain_ = std::make_shared<FlatGround>(0.0);
  formulation.model_ = RobotModel(RobotModel::Monoped);
  // get last ones
  formulation.initial_base_.lin.at(towr::kPos) << last_solution.base_linear_->GetPoint(t).p();
  formulation.initial_base_.ang.at(towr::kPos) << last_solution.base_angular_->GetPoint(t).p();
  formulation.initial_ee_W_.push_back(last_solution.ee_motion_.at(0)->GetPoint(t).p().transpose());
  formulation.final_base_.lin.at(towr::kPos) << fin_x, fin_y, 0.5;

  formulation.params_.ee_phase_durations_.push_back({0.4, 0.2, 0.4, 0.2, 0.4, 0.2, 0.2});
  formulation.params_.ee_in_contact_at_start_.push_back(true);

  for (auto c : formulation.GetVariableSets(solution))
    nlp.AddVariableSet(c);
  for (auto c : formulation.GetConstraints(solution))
    nlp.AddConstraintSet(c);
  for (auto c : formulation.GetCosts())
    nlp.AddCostSet(c);
  
  auto solver = std::make_shared<ifopt::IpoptSolver>();
  solver->SetOption("jacobian_approximation", "exact");
  solver->SetOption("max_cpu_time", 20.0);
  solver->Solve(nlp);
}


int main()
{
  ::std::vector<ifopt::Problem> nlp_list(2);
  ::std::vector<SplineHolder> solution_list(2);

  plan1(nlp_list[0], solution_list[0], 0, 0, 1, 0);

  // rosbag::Bag bag;



  plan2(nlp_list[1], solution_list[1], solution_list[0], 2, 0);
  // std::string bag_file = "towr_trajectory.bag";
  // SaveOptimizationAsRosbag(bag_file, )
  // plan1(nlp_list[2], solution_list[2], 2, 0, 3, 0);

  // Can directly view the optimization variables through:
  // Eigen::VectorXd x = nlp.GetVariableValues()
  // However, it's more convenient to access the splines constructed from these
  // variables and query their values at specific times:
  using namespace std;
  cout.precision(2);
  // nlp_list[1].PrintCurrent(); // view variable-set, constraint violations, indices,...
  cout << fixed;
  cout << "\n====================\nMonoped trajectory:\n====================\n";

  double t = 0.0;
  double t_start = 0.0;
  SplineHolder solution;
  // for (auto solution : solution_list) {
  for (int j = 0; j<2; j++){
    solution = solution_list[j];
    cout << "\n====================\n solution " << j << " :\n====================\n";
    t = 0.0;
    while (t<=solution.base_linear_->GetTotalTime() + 1e-5) {
      cout << "t=" << t + t_start << "\n";
      cout << "Base linear position x,y,z:   \t";
      cout << solution.base_linear_->GetPoint(t).p().transpose() << "\t[m]" << endl;

      cout << "Base Euler roll, pitch, yaw:  \t";
      Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
      cout << (rad/M_PI*180).transpose() << "\t[deg]" << endl;

      cout << "Foot position x,y,z:          \t";
      cout << solution.ee_motion_.at(0)->GetPoint(t).p().transpose() << "\t[m]" << endl;

      cout << "Contact force x,y,z:          \t";
      cout << solution.ee_force_.at(0)->GetPoint(t).p().transpose() << "\t[N]" << endl;

      bool contact = solution.phase_durations_.at(0)->IsContactPhase(t);
      std::string foot_in_contact = contact? "yes" : "no";
      cout << "Foot in contact:              \t" + foot_in_contact << endl;

      cout << endl;

      t += 0.2;

    }
    cout << "\n###############3? \n" << t << "\n" << solution.base_linear_->GetTotalTime() << "\n################?\n"<< endl;
    t_start += solution.base_linear_->GetTotalTime();
  }
}
