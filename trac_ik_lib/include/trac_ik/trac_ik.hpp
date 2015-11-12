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


#ifndef TRAC_IK_HPP
#define TRAC_IK_HPP

#include <trac_ik/kdl_tl.hpp>
#include <trac_ik/nlopt_ik.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>

namespace TRAC_IK {


  class TRAC_IK 
  {
  public:
    TRAC_IK(const KDL::Chain& _chain, const KDL::JntArray& _q_min, const KDL::JntArray& _q_max, double _maxtime=0.005, double _eps=1e-3);

    ~TRAC_IK();

    static double JointErr(const KDL::JntArray& arr1, const KDL::JntArray& arr2) {
      double err = 0;
      for (uint i=0; i<arr1.data.size(); i++) {
        err += pow(arr1(i) - arr2(i),2);
        //        err = std::max(err, std::abs(arr1(i) - arr2(i)));
      }
      
      return err;
    }




    int CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist& bounds=KDL::Twist::Zero(), const KDL::JntArray& q_desired=KDL::JntArray());

    std::pair<int, int> getSolveCounts(){
      return std::make_pair(kdl_count,nlopt_count);
    }

    void resetSolveCounts(){
      kdl_count = 0;
      nlopt_count = 0;
    }

  private:
    KDL::Chain chain;
    double eps;
    double maxtime;
    NLOPT_IK::NLOPT_IK nl_solver;
    KDL::ChainIkSolverPos_TL iksolver;


    bool runKDL(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray& q_out, const KDL::JntArray& q_desired);


    bool runNLOPT(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray& q_out, const KDL::JntArray& q_desired);

    bool reeval(const KDL::JntArray& seed, KDL::JntArray& solution);

    std::vector<double> lb, ub;
  
    std::vector<KDL::BasicJointType> types;

    int kdlRC, nloptRC;

    int kdl_count, nlopt_count;
   

    boost::asio::io_service io_service;
    boost::thread_group threads;
    boost::asio::io_service::work work;
    KDL::Twist bounds;

  };

}

#endif
