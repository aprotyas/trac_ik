/********************************************************************************
Copyright (c) 2015, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/


#include <trac_ik/trac_ik.hpp>
#include <boost/date_time.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <limits>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

namespace TRAC_IK {

  TRAC_IK::TRAC_IK(const std::string& base_link, const std::string& tip_link, const std::string& URDF_param, double _maxtime, double _eps, SolveType _type ) :
    initialized(false),
    eps(_eps),
    maxtime(_maxtime),
    solvetype(_type),
    work(io_service)
  {

    ros::NodeHandle node_handle("~");

    urdf::Model robot_model;
    std::string xml_string;

    std::string urdf_xml,full_urdf_xml;
    node_handle.param("urdf_xml",urdf_xml,URDF_param);
    node_handle.searchParam(urdf_xml,full_urdf_xml);
    
    ROS_DEBUG_NAMED("trac_ik","Reading xml file from parameter server");
    if (!node_handle.getParam(full_urdf_xml, xml_string))
      {
        ROS_FATAL_NAMED("trac_ik","Could not load the xml from parameter server: %s", urdf_xml.c_str());
        return;
      }
    
    node_handle.param(full_urdf_xml,xml_string,std::string());
    robot_model.initString(xml_string);
    
    ROS_DEBUG_STREAM_NAMED("trac_ik","Reading joints and links from URDF");

    KDL::Tree tree;
    
    if (!kdl_parser::treeFromUrdfModel(robot_model, tree))
      ROS_FATAL("Failed to extract kdl tree from xml robot description");

    if(!tree.getChain(base_link, tip_link, chain))
      ROS_FATAL("Couldn't find chain %s to %s",base_link.c_str(),tip_link.c_str());

    std::vector<KDL::Segment> chain_segs = chain.segments;

    boost::shared_ptr<const urdf::Joint> joint;

    std::vector<double> l_bounds, u_bounds;

    lb.resize(chain.getNrOfJoints());
    ub.resize(chain.getNrOfJoints());

    uint joint_num=0;
    for(unsigned int i = 0; i < chain_segs.size(); ++i) {
      joint = robot_model.getJoint(chain_segs[i].getJoint().getName());
      if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
        joint_num++;
        float lower, upper;
        int hasLimits;
        if ( joint->type != urdf::Joint::CONTINUOUS ) {
          if(joint->safety) {
            lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
            upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
          } else {
            lower = joint->limits->lower;
            upper = joint->limits->upper;
          }
          hasLimits = 1;
        }
        else {
          hasLimits = 0;
        }
        if(hasLimits) {
          lb(joint_num-1)=lower;
          ub(joint_num-1)=upper;
        }
        else {
          lb(joint_num-1)=std::numeric_limits<float>::lowest();
          ub(joint_num-1)=std::numeric_limits<float>::max();
        }
        ROS_INFO_STREAM("IK Using joint "<<joint->name<<" "<<lb(joint_num-1)<<" "<<ub(joint_num-1));
      }
    }
    
    initialize();
  }


  TRAC_IK::TRAC_IK(const KDL::Chain& _chain, const KDL::JntArray& _q_min, const KDL::JntArray& _q_max, double _maxtime, double _eps, SolveType _type):
    initialized(false),
    chain(_chain),
    lb(_q_min),
    ub(_q_max),
    eps(_eps),
    maxtime(_maxtime),
    solvetype(_type),
    work(io_service)
  {

    initialize();
  }

  void TRAC_IK::initialize() {

    assert(chain.getNrOfJoints()==lb.data.size());
    assert(chain.getNrOfJoints()==ub.data.size());

    jacsolver.reset(new KDL::ChainJntToJacSolver(chain));
    nl_solver.reset(new NLOPT_IK::NLOPT_IK(chain,lb,ub,maxtime,eps,NLOPT_IK::SumSq));
    iksolver.reset(new KDL::ChainIkSolverPos_TL(chain,lb,ub,maxtime,eps,true,true));

    for (uint i=0; i<chain.segments.size(); i++) {
      std::string type = chain.segments[i].getJoint().getTypeName();
      if (type.find("Rot")!=std::string::npos) {
        if (ub(types.size())>=std::numeric_limits<float>::max() && 
            lb(types.size())<=std::numeric_limits<float>::lowest())
          types.push_back(KDL::BasicJointType::Continuous);
        else
          types.push_back(KDL::BasicJointType::RotJoint);
      }
      else if (type.find("Trans")!=std::string::npos)
        types.push_back(KDL::BasicJointType::TransJoint);
    }
    
    assert(types.size()==lb.data.size());


    threads.create_thread(boost::bind(&boost::asio::io_service::run,
                                      &io_service));
    threads.create_thread(boost::bind(&boost::asio::io_service::run,
                                      &io_service));

    initialized = true;
  }

  bool TRAC_IK::unique_solution(const KDL::JntArray& sol) {

    for (uint i=0; i< solutions.size(); i++)
      if (myEqual(sol,solutions[i]))
        return false;
    return true;

  }

  inline void normalizeAngle(double& val, const double& min, const double& max)
  {
      if (val > max) {
        //Find actual angle offset
        double diffangle = fmod(val-max,2*M_PI);
        // Add that to upper bound and go back a full rotation
        val = max + diffangle - 2*M_PI;
      }

      if (val < min) {
        //Find actual angle offset
        double diffangle = fmod(min-val,2*M_PI);
        // Add that to upper bound and go back a full rotation
        val = min - diffangle + 2*M_PI;
      }
  }

  inline void normalizeAngle(double& val, const double& target)
  {
      if (val > target+M_PI) {
        //Find actual angle offset
        double diffangle = fmod(val-target,2*M_PI);
        // Add that to upper bound and go back a full rotation
        val = target + diffangle - 2*M_PI;
      }

      if (val < target-M_PI) {
        //Find actual angle offset
        double diffangle = fmod(target-val,2*M_PI);
        // Add that to upper bound and go back a full rotation
        val = target - diffangle + 2*M_PI;
      }
  }


  bool TRAC_IK::runKDL(const KDL::JntArray &q_init, const KDL::Frame &p_in)
  {
    KDL::JntArray q_out;

    double fulltime = maxtime;
    KDL::JntArray seed = q_init;

    boost::posix_time::time_duration timediff;
    double time_left;

    while (true) {
      timediff=boost::posix_time::microsec_clock::local_time()-start_time;
      time_left = fulltime - timediff.total_nanoseconds()/1000000000.0;

      if (time_left <= 0)
        break;

      iksolver->setMaxtime(time_left);

      int kdlRC = iksolver->CartToJnt(seed,p_in,q_out,bounds);
      if (kdlRC >=0) {
        switch (solvetype) {
        case Manip1:
        case Manip2:
          normalize_limits(q_init, q_out);
          break;
        default:
          normalize_seed(q_init, q_out);
          break;
        }
        mtx_.lock();
        if (unique_solution(q_out)) {
          solutions.push_back(q_out);
          uint curr_size=solutions.size();
          errors.resize(curr_size);
          mtx_.unlock();
          double err, penalty;
          switch (solvetype) {
          case Manip1:
            penalty = manipPenalty(q_out);
            err = penalty*TRAC_IK::ManipValue1(q_out);
            break;
          case Manip2:
            penalty = manipPenalty(q_out);
            err = penalty*TRAC_IK::ManipValue2(q_out);
            break;
          default:
            err = TRAC_IK::JointErr(q_init,q_out);
            break;
          }
          mtx_.lock();
          errors[curr_size-1] = std::make_pair(err,curr_size-1);
        }
        mtx_.unlock();
      }
      
      if (!solutions.empty() && solvetype == Speed)
        break;
      
      for (unsigned int j=0; j<seed.data.size(); j++)
        if (types[j]==KDL::BasicJointType::Continuous)
          seed(j)=fRand(q_init(j)-2*M_PI, q_init(j)+2*M_PI);
        else
          seed(j)=fRand(lb(j), ub(j));
    }
    nl_solver->abort();

    iksolver->setMaxtime(fulltime);

    return true;
  }

  bool TRAC_IK::runNLOPT(const KDL::JntArray &q_init, const KDL::Frame &p_in)
  {
    KDL::JntArray q_out;

    double fulltime = maxtime;
    KDL::JntArray seed = q_init;

    boost::posix_time::time_duration timediff;
    double time_left;

    while (true) {
      timediff=boost::posix_time::microsec_clock::local_time()-start_time;
      time_left = fulltime - timediff.total_nanoseconds()/1000000000.0;

      if (time_left <= 0)
        break;

      nl_solver->setMaxtime(time_left);

      int nloptRC = nl_solver->CartToJnt(seed,p_in,q_out,bounds);
      if (nloptRC >=0) {
        switch (solvetype) {
        case Manip1:
        case Manip2:
          normalize_limits(q_init, q_out);
          break;
        default:
          normalize_seed(q_init, q_out);
          break;
        }
        mtx_.lock();
        if (unique_solution(q_out)) {
          solutions.push_back(q_out);
          uint curr_size=solutions.size();
          errors.resize(curr_size);
          mtx_.unlock();
          double err, penalty;
          switch (solvetype) {
          case Manip1:
            penalty = manipPenalty(q_out);
            err = penalty*TRAC_IK::ManipValue1(q_out);
            break;
          case Manip2:
            penalty = manipPenalty(q_out);
            err = penalty*TRAC_IK::ManipValue2(q_out);
            break;
          default:
            err = TRAC_IK::JointErr(q_init,q_out);
            break;
          }
          mtx_.lock();
          errors[curr_size-1] = std::make_pair(err,curr_size-1);
        }
        mtx_.unlock();
      }
      
      if (!solutions.empty() && solvetype == Speed)
        break;
      
      for (unsigned int j=0; j<seed.data.size(); j++)
        if (types[j]==KDL::BasicJointType::Continuous)
          seed(j)=fRand(q_init(j)-2*M_PI, q_init(j)+2*M_PI);
        else
          seed(j)=fRand(lb(j), ub(j));
    }

    iksolver->abort();

    nl_solver->setMaxtime(fulltime);

    return true;
  }

  void TRAC_IK::normalize_seed(const KDL::JntArray& seed, KDL::JntArray& solution) {
    // Make sure rotational joint values are within 1 revolution of seed; then
    // ensure joint limits are met.

    bool improved = false;

    for (uint i=0; i<lb.data.size(); i++) {

      if (types[i]==KDL::BasicJointType::TransJoint)
        continue;

      double target = seed(i);
      double val = solution(i);

      normalizeAngle( val, target );

      if (types[i]==KDL::BasicJointType::Continuous) {
        solution(i) = val;
        continue;
      }

      normalizeAngle( val, lb(i), ub(i) );

      solution(i) = val;
    }
  }

  void TRAC_IK::normalize_limits(const KDL::JntArray& seed, KDL::JntArray& solution) {
    // Make sure rotational joint values are within 1 revolution of middle of
    // limits; then ensure joint limits are met.

    bool improved = false;

    for (uint i=0; i<lb.data.size(); i++) {

      if (types[i] == KDL::BasicJointType::TransJoint)
        continue;

      double target = seed(i);

      if (types[i] == KDL::BasicJointType::RotJoint && types[i]!=KDL::BasicJointType::Continuous)
        target = (ub(i)+lb(i))/2.0;

      double val = solution(i);

      normalizeAngle( val, target );

      if (types[i]==KDL::BasicJointType::Continuous) {
        solution(i) = val;
        continue;
      }

      normalizeAngle( val, lb(i), ub(i) );

      solution(i) = val;
    }

  }


  double TRAC_IK::manipPenalty(const KDL::JntArray& arr) {
    double penalty = 1.0;
    for (uint i=0; i< arr.data.size(); i++) {
      if (types[i] == KDL::BasicJointType::Continuous)
        continue;
      double range = ub(i)-lb(i);
      penalty *= ((arr(i)-lb(i))*(ub(i)-arr(i))/(range*range));
    }
    return std::max(0.0,1.0 - exp(-1*penalty));
  }


  double TRAC_IK::ManipValue1(const KDL::JntArray& arr) {
    KDL::Jacobian jac(arr.data.size());

    jacsolver->JntToJac(arr,jac);

    Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jac.data);
    Eigen::MatrixXd singular_values = svdsolver.singularValues();

    double error = 1.0;
    for(unsigned int i=0; i < singular_values.rows(); ++i)
      error *= singular_values(i,0);
    return error;
  }

  double TRAC_IK::ManipValue2(const KDL::JntArray& arr) {
    KDL::Jacobian jac(arr.data.size());

    jacsolver->JntToJac(arr,jac);

    Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jac.data);
    Eigen::MatrixXd singular_values = svdsolver.singularValues();

    return singular_values.minCoeff()/singular_values.maxCoeff();
  }


  int TRAC_IK::CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist& _bounds) {

    if (!initialized) {
      ROS_ERROR("TRAC-IK was not properly initialized with a valid chain or limits.  IK cannot proceed");
      return -1;
    }


    start_time = boost::posix_time::microsec_clock::local_time();

    nl_solver->reset();
    iksolver->reset();

    solutions.clear();
    errors.clear();


    bounds=_bounds;

    std::vector<boost::shared_future<bool> > pending_data;

    typedef boost::packaged_task<bool> task_t;
    boost::shared_ptr<task_t> task1 = boost::make_shared<task_t>(boost::bind(&TRAC_IK::runKDL, this, boost::cref(q_init), boost::cref(p_in)));

    boost::shared_ptr<task_t> task2 = boost::make_shared<task_t>(boost::bind(&TRAC_IK::runNLOPT, this, boost::cref(q_init), boost::cref(p_in)));

    boost::shared_future<bool> fut1(task1->get_future());
    boost::shared_future<bool> fut2(task2->get_future());

    /*
    // this was for pre-c++11
    pending_data.push_back(boost::move(fut1));
    pending_data.push_back(boost::move(fut2));
    */
    pending_data.push_back(fut1);
    pending_data.push_back(fut2);

    io_service.post(boost::bind(&task_t::operator(), task1));
    io_service.post(boost::bind(&task_t::operator(), task2));

    boost::wait_for_all(pending_data.begin(), pending_data.end());

    if (solutions.empty()) {
      q_out=q_init;
      return -3;
    }

    switch (solvetype) {
    case Manip1:
    case Manip2:
      std::sort(errors.rbegin(),errors.rend()); // rbegin/rend to sort by max
      break;
    default:
      std::sort(errors.begin(),errors.end());
      break;
    }

    q_out = solutions[errors[0].second];

    return solutions.size();
  }


  TRAC_IK::~TRAC_IK(){
    // Force all threads to return from io_service::run().
    io_service.stop();

    // Suppress all exceptions.
    try
      {
        threads.join_all();
      }
    catch ( ... ) {}

  }

}
