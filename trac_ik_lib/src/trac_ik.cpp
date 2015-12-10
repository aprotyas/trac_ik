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

namespace TRAC_IK {



  TRAC_IK::TRAC_IK(const KDL::Chain& _chain, const KDL::JntArray& _q_min, const KDL::JntArray& _q_max, double _maxtime, double _eps, SolveType _type):
    chain(_chain),
    jacsolver(_chain),
    eps(_eps),
    maxtime(_maxtime),
    solvetype(_type),
    nl_solver(chain,_q_min,_q_max,maxtime,eps, NLOPT_IK::SumSq),
    iksolver(chain,_q_min,_q_max,maxtime,eps,true,true),
    work(io_service)
  {

  
    assert(chain.getNrOfJoints()==_q_min.data.size());
    assert(chain.getNrOfJoints()==_q_max.data.size());

    for (uint i=0; i<chain.getNrOfJoints(); i++) {
      lb.push_back(_q_min(i));
      ub.push_back(_q_max(i));
    }

    for (uint i=0; i<chain.segments.size(); i++) {
      std::string type = chain.segments[i].getJoint().getTypeName();
      if (type.find("Rot")!=std::string::npos)
        types.push_back(KDL::BasicJointType::RotJoint);
      if (type.find("Trans")!=std::string::npos)
        types.push_back(KDL::BasicJointType::TransJoint);
    }
     
    assert(types.size()==lb.size());
 

    threads.create_thread(boost::bind(&boost::asio::io_service::run,
                                      &io_service));
    threads.create_thread(boost::bind(&boost::asio::io_service::run,
                                      &io_service));

  }

  void TRAC_IK::remove_duplicate_solutions() {
    std::vector<KDL::JntArray> subset;

    for (uint i=0; i < solutions.size(); i++)
      if (unique_vector(solutions[i],subset))
        subset.push_back(solutions[i]);

    solutions=subset;
  }

  bool TRAC_IK::unique_vector(const KDL::JntArray& v1, const std::vector<KDL::JntArray>& list) {
    // Assumes solutions is completed

    for (uint i=0; i < list.size(); i++) {
      bool found_match = true;
      for (uint j=0; j<v1.data.size(); j++) {
        double jntErr = std::abs(v1(j)-list[i](j));
        if (jntErr > 1e-4) {
          found_match = false;
          break;
        }
      }
      if (found_match)
        return false;
    }
    return true;
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
      
      iksolver.setMaxtime(time_left);

      int kdlRC = iksolver.CartToJnt(q_init,p_in,q_out,bounds);
      if (kdlRC >=0) {
        mtx_.lock();
        solutions.push_back(q_out);
        mtx_.unlock();
      }

      if (!solutions.empty() && solvetype == Speed)
        break;
      
      for (unsigned int j=0; j<seed.data.size(); j++) 
        seed(j)=fRand(lb[j], ub[j]);     
    }     
    nl_solver.abort();

    iksolver.setMaxtime(fulltime);

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
     
      nl_solver.setMaxtime(time_left);

      int nloptRC = nl_solver.CartToJnt(q_init,p_in,q_out,bounds);
      if (nloptRC >=0) {
        mtx_.lock();
        solutions.push_back(q_out);
        mtx_.unlock();
      }

      if (!solutions.empty() && solvetype == Speed)
        break;
      
      for (unsigned int j=0; j<seed.data.size(); j++) 
        seed(j)=fRand(lb[j], ub[j]);     
    }     

    iksolver.abort();

    nl_solver.setMaxtime(fulltime);

    return true;
  }

  bool TRAC_IK::reeval(const KDL::JntArray& seed, KDL::JntArray& solution) {

   
    bool improved = false;

    for (uint i=0; i<lb.size(); i++) {
      
      if (types[i]==KDL::BasicJointType::TransJoint)
        continue;

      double target = seed(i);
      double val = solution(i);
      
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

      if (val > ub[i]) {
        //Find actual angle offset
        double diffangle = fmod(val-ub[i],2*M_PI);
        // Add that to upper bound and go back a full rotation
        val = ub[i] + diffangle - 2*M_PI;
      }

      if (val < lb[i]) {
        //Find actual angle offset
        double diffangle = fmod(lb[i]-val,2*M_PI);
        // Add that to upper bound and go back a full rotation
        val = lb[i] - diffangle + 2*M_PI;
      }

      //      if (std::abs(val-solution(i)) > 0.1) {
      //        improved = true;
        solution(i) = val;
        //      }
    }
    
    return true;
    //    return improved;
    
  }

  double TRAC_IK::manipPenalty(const KDL::JntArray& arr) {
    double penalty = 1.0;
    for (uint i=0; i< arr.data.size(); i++) {
      double range = ub[i]-lb[i];
      if (range <= boost::math::tools::epsilon<double>())
        continue;
      penalty *= ((arr(i)-lb[i])*(ub[i]-arr(i))/(range*range));
    }
    return (1.0 - exp(-1*penalty));
  }


  double TRAC_IK::ManipValue1(const KDL::JntArray& arr) {
    KDL::Jacobian jac(arr.data.size());

    jacsolver.JntToJac(arr,jac);

    Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jac.data);
    Eigen::MatrixXd singular_values = svdsolver.singularValues();
    
    double error = 1.0;
    for(unsigned int i=0; i < singular_values.rows(); ++i)
      error *= singular_values(i,0);
    return error;
  }
  
  double TRAC_IK::ManipValue2(const KDL::JntArray& arr) {
    KDL::Jacobian jac(arr.data.size());

    jacsolver.JntToJac(arr,jac);
    
    Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jac.data);
    Eigen::MatrixXd singular_values = svdsolver.singularValues();
    
    return singular_values.minCoeff()/singular_values.maxCoeff();
  }


  int TRAC_IK::CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist& _bounds) {

    static uint calls =0;
    static uint inconsistent =0;

    start_time = boost::posix_time::microsec_clock::local_time();

    nl_solver.reset();
    iksolver.reset();

    solutions.clear();

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

    std::vector<std::pair<double,uint> >  errors;


    for (uint i=0; i<solutions.size(); i++)  {

      double err;
      double penalty;
      
      switch (solvetype) {
      case Manip1:
        remove_duplicate_solutions();
        penalty = manipPenalty(solutions[i]);
        err = penalty*TRAC_IK::ManipValue1(solutions[i]);
        break;
      case Manip2:
        remove_duplicate_solutions();
        penalty = manipPenalty(solutions[i]);
        err = penalty*TRAC_IK::ManipValue2(solutions[i]);
        break;
      case Speed: // Distance and Speed just minimize distance
      case Distance:
        for (uint i=0; i<solutions.size(); i++)
          reeval(q_init,solutions[i]);
        
        remove_duplicate_solutions();
        err = TRAC_IK::JointErr(q_init,solutions[i]);
      }
      
      errors.push_back(std::make_pair(err,i));
    }
    
    switch (solvetype) {
    case Manip1:
      std::sort(errors.rbegin(),errors.rend()); // rbegin/rend to sort by max
      break;
    case Manip2:
      std::sort(errors.rbegin(),errors.rend()); // rbegin/rend to sort by max
      break;
    case Speed: // Distance and Speed just minimize distance
    case Distance:
      std::sort(errors.begin(),errors.end());
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
