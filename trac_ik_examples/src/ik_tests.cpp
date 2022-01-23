// Copyright (c) 2015, TRACLabs, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <chrono>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trac_ik/trac_ik.hpp"

void test(
  const rclcpp::Node::SharedPtr node, double num_samples, std::string chain_start,
  std::string chain_end, double timeout, std::string urdf_xml)
{
  double eps = 1e-5;

  // This constructor parses the URDF loaded in rosparm urdf_xml into the
  // needed KDL structures.  We then pull these out to compare against the KDL
  // IK solver.
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_xml, timeout, eps);

  KDL::Chain chain;
  KDL::JntArray ll, ul;  // lower joint limits, upper joint limits

  bool valid = tracik_solver.getKDLChain(chain);

  if (!valid) {
    RCLCPP_ERROR(node->get_logger(), "There was no valid KDL chain found");
    return;
  }

  valid = tracik_solver.getKDLLimits(ll, ul);

  if (!valid) {
    RCLCPP_ERROR(node->get_logger(), "There were no valid KDL joint limits found");
    return;
  }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());

  RCLCPP_INFO(node->get_logger(), "Using %d joints", chain.getNrOfJoints());


  // Set up KDL IK
  KDL::ChainFkSolverPos_recursive fk_solver(chain);  // Forward kin. solver
  KDL::ChainIkSolverVel_pinv vik_solver(chain);  // PseudoInverse vel solver
  // Joint Limit Solver
  KDL::ChainIkSolverPos_NR_JL kdl_solver(chain, ll, ul, fk_solver, vik_solver, 1, eps);
  // 1 iteration per solve (will wrap in timed loop to compare with TRAC-IK)


  // Create Nominal chain configuration midway between all joint limits
  KDL::JntArray nominal(chain.getNrOfJoints());

  for (uint j = 0; j < nominal.data.size(); j++) {
    nominal(j) = (ll(j) + ul(j)) / 2.0;
  }

  // Create desired number of valid, random joint configurations
  std::vector<KDL::JntArray> JointList;
  KDL::JntArray q(chain.getNrOfJoints());

  std::random_device rd;
  std::mt19937 gen(rd());
  for (uint i = 0; i < num_samples; i++) {
    for (uint j = 0; j < ll.data.size(); j++) {
      std::uniform_real_distribution<double> dist(ll(j), ul(j));
      q(j) = dist(gen);
    }
    JointList.push_back(q);
  }


  std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> start_time;
  std::chrono::duration<double> diff;

  KDL::JntArray result;
  KDL::Frame end_effector_pose;
  int rc;

  double total_time = 0;
  uint success = 0;

  RCLCPP_INFO_STREAM(
    node->get_logger(),
    "*** Testing KDL with " << num_samples << " random samples");

  for (uint i = 0; i < num_samples; i++) {
    fk_solver.JntToCart(JointList[i], end_effector_pose);
    result = nominal;  // start with nominal
    start_time = std::chrono::system_clock::now();
    do {
      q = result;  // when iterating start with last solution
      rc = kdl_solver.CartToJnt(q, end_effector_pose, result);
      diff = std::chrono::system_clock::now() - start_time;
    } while (rc < 0 && diff.count() < timeout);
    total_time += diff.count();
    if (rc >= 0) {
      success++;
    }

    if (static_cast<int>(static_cast<double>(i) / num_samples * 100) % 10 == 0) {
      RCLCPP_INFO_STREAM_THROTTLE(
        node->get_logger(),
        *(node->get_clock()),
        1.0,
        static_cast<int>((i) / num_samples * 100) << "% done");
    }
  }

  RCLCPP_INFO_STREAM(
    node->get_logger(),
    "KDL found " << success << " solutions (" << 100.0 * success / num_samples <<
      "%) with an average of " << total_time / num_samples <<
      " secs per sample");


  total_time = 0;
  success = 0;

  RCLCPP_INFO_STREAM(
    node->get_logger(), "*** Testing TRAC-IK with " << num_samples << " random samples");

  for (uint i = 0; i < num_samples; i++) {
    fk_solver.JntToCart(JointList[i], end_effector_pose);
    start_time = std::chrono::system_clock::now();
    rc = tracik_solver.CartToJnt(nominal, end_effector_pose, result);
    diff = std::chrono::system_clock::now() - start_time;
    total_time += diff.count();
    if (rc >= 0) {
      success++;
    }

    if (static_cast<int>(static_cast<double>(i) / num_samples * 100) % 10 == 0) {
      RCLCPP_INFO_STREAM_THROTTLE(
        node->get_logger(),
        *(node->get_clock()),
        1.0,
        static_cast<int>((i) / num_samples * 100) << "% done");
    }
  }

  RCLCPP_INFO_STREAM(
    node->get_logger(),
    "TRAC-IK found " << success << " solutions (" << 100.0 * success / num_samples <<
      "%) with an average of " << total_time / num_samples <<
      " secs per sample");
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ik_tests");

  int num_samples;
  std::string chain_start, chain_end, urdf_xml;
  double timeout;

  node->declare_parameter<int>("num_samples", 1000);
  node->declare_parameter<double>("timeout", 0.005);
  node->declare_parameters<std::string>(
    std::string(),       // parameters are not namespaced
    std::map<std::string, std::string>{
    {"chain_start", std::string()},
    {"chain_end", std::string()},
    {"robot_description", std::string()},
  });

  node->get_parameter("num_samples", num_samples);
  node->get_parameter("timeout", timeout);
  node->get_parameter("chain_start", chain_start);
  node->get_parameter("chain_end", chain_end);
  node->get_parameter("robot_description", urdf_xml);

  if (chain_start.empty() || chain_end.empty()) {
    RCLCPP_FATAL(node->get_logger(), "Missing chain info in launch file");
    exit(-1);
  }

  if (num_samples < 1) {
    num_samples = 1;
  }

  test(node, num_samples, chain_start, chain_end, timeout, urdf_xml);

  // Useful when a script loops over multiple launch files testing different robot chains
  // std::vector<char *> commandVector;
  // commandVector.push_back((char*)"killall");
  // commandVector.push_back((char*)"-9");
  // commandVector.push_back((char*)"roslaunch");
  // commandVector.push_back(NULL);

  // char **command = &commandVector[0];
  // execvp(command[0],command);

  return 0;
}
