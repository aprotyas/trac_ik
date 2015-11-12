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

#include <ros/ros.h>

namespace TRAC_IK {



  TRAC_IK::TRAC_IK(const KDL::Chain& _chain, const KDL::JntArray& _q_min, const KDL::JntArray& _q_max, double _maxtime, double _eps):
    chain(_chain),
    eps(_eps),
    maxtime(_maxtime),
    nl_solver(chain,_q_min,_q_max,maxtime,eps, NLOPT_IK::SumSq),
    iksolver(chain,_q_min,_q_max,maxtime,eps),
    work(io_service)
  {
    assert(chain.getNrOfJoints()==_q_min.data.size());
    assert(chain.getNrOfJoints()==_q_max.data.size());

    for (uint i=0; i<chain.getNrOfJoints(); i++) {
      lb.push_back(_q_min(i));
      ub.push_back(_q_max(i));
    }


    threads.create_thread(boost::bind(&boost::asio::io_service::run,
                                      &io_service));
    threads.create_thread(boost::bind(&boost::asio::io_service::run,
                                      &io_service));
    kdl_count=0;
    nlopt_count=0;

  }

  bool TRAC_IK::runKDL(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray& q_out, const KDL::JntArray& q_desired)
  {
    kdlRC = iksolver.CartToJnt(q_init,p_in,q_out,bounds);
    if (kdlRC >= 0)
      nl_solver.abort();
    return true;
  }


  bool TRAC_IK::runNLOPT(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray& q_out, const KDL::JntArray& q_desired)
  {
    nloptRC = nl_solver.CartToJnt(q_init,p_in,q_out,bounds,q_desired);
    if (nloptRC >=0)
      iksolver.abort();
    return true;
  }

  bool TRAC_IK::reeval(const KDL::JntArray& seed, KDL::JntArray& solution) {

    KDL::JntArray new_solution(solution);
    
    bool improved = false;

    for (uint i=0; i<lb.size(); i++) {
      
      double val = std::abs(seed(i) - solution(i));
      
      double  z = 2*M_PI;

      while (solution(i)+z <= ub[i]) {
        double newval = std::abs(seed(i) - (solution(i)+z));
        if (newval < val) {
          val = newval;
          new_solution(i) = solution(i)+z;
          improved = true;
        }
        else break;
        z += 2*M_PI;
      }

      z = -2*M_PI;

      while (solution(i)+z >= lb[i]) {
        double newval = std::abs(seed(i) - (solution(i)+z));
        if (newval < val) {
          val = newval;
          new_solution(i) = solution(i)+z;
          improved = true;
        }
        else break;
        z -= 2*M_PI;
      }
      
    }

    if (improved) {
      solution = new_solution;
    }

    return improved;

  }


  int TRAC_IK::CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist& _bounds, const KDL::JntArray& q_desired) {

    KDL::JntArray kdl_out=q_init;
    KDL::JntArray nlopt_out=q_init;

    bounds=_bounds;

    kdlRC = -3;
    nloptRC = -3;

    KDL::JntArray des;
    if (q_desired.data.size()!=q_init.data.size())
      des = q_init;
    else 
      des = q_desired;

    std::vector<boost::shared_future<bool> > pending_data;

    typedef boost::packaged_task<bool> task_t;
    boost::shared_ptr<task_t> task1 = boost::make_shared<task_t>(boost::bind(&TRAC_IK::runNLOPT, this, boost::cref(q_init), boost::cref(p_in), boost::ref(nlopt_out), boost::cref(q_desired)));

    boost::shared_ptr<task_t> task2 = boost::make_shared<task_t>(boost::bind(&TRAC_IK::runKDL, this, boost::cref(q_init), boost::cref(p_in), boost::ref(kdl_out), boost::cref(q_desired)));
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

    if (nloptRC == 0)
      reeval(des,nlopt_out);

    if (kdlRC == 0)
      reeval(des,kdl_out);

    int result;
    if (nloptRC > kdlRC) {
      q_out = nlopt_out;
      result = nloptRC;
      nlopt_count++;
    }
    else if (kdlRC > nloptRC) {
      q_out = kdl_out;
      result = kdlRC;
      kdl_count++;
    }
    else { //they both failed or both succeeded
      result = kdlRC;
      double err1 = TRAC_IK::JointErr(des,nlopt_out);
      double err2 = TRAC_IK::JointErr(des,kdl_out);       
      if (err1 < err2) {
        q_out=nlopt_out;
        nlopt_count++;
      }
      else {
        q_out=kdl_out;
        kdl_count++;
      }
    }
   
    return result;    
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
