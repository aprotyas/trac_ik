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

#include <trac_ik/nlopt_ik.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <ros/ros.h>
#include <limits>
#include <boost/date_time.hpp>
#include <trac_ik/dual_quaternion.h>



namespace NLOPT_IK {


  dual_quaternion targetDQ;

  double minfunc(const std::vector<double>& x, std::vector<double>& grad, void* data) {
    // Auxilory function to minimize (Sum of Squared joint angle error
    // from the requested configuration).  Because we wanted a Class
    // without static members, but NLOpt library does not support
    // passing methods of Classes, we use these auxilary functions.

    NLOPT_IK *c = (NLOPT_IK *) data;

    return c->minJoints(x,grad);
  }

  double minfuncDQ(const std::vector<double>& x, std::vector<double>& grad, void* data) {
    // Auxilory function to minimize (Sum of Squared joint angle error
    // from the requested configuration).  Because we wanted a Class
    // without static members, but NLOpt library does not support
    // passing methods of Classes, we use these auxilary functions.
    NLOPT_IK *c = (NLOPT_IK *) data;

    std::vector<double> vals(x);

    double jump=1e-4;
    double result[1]; 
    c->cartDQError(vals, result);

    if (!grad.empty()) {
      for (uint i=0; i<x.size(); i++) {
        double original=vals[i];

        vals[i]=original+jump;
        double v1[1];
        c->cartDQError(vals, v1);

        vals[i]=original-jump;
        double v2[1];	
        c->cartDQError(vals, v2);

        vals[i]=original;
        grad[i]=(v1[0]-v2[0])/(2*jump);
      }
    }

    return result[0]; 
  }


  double minfuncSumSquared(const std::vector<double>& x, std::vector<double>& grad, void* data) {
    // Auxilory function to minimize (Sum of Squared joint angle error
    // from the requested configuration).  Because we wanted a Class
    // without static members, but NLOpt library does not support
    // passing methods of Classes, we use these auxilary functions.

    NLOPT_IK *c = (NLOPT_IK *) data;

    std::vector<double> vals(x);

    double jump=1e-8;
    double result[1]; 
    c->cartSumSquaredError(vals, result);

    if (!grad.empty()) {
      for (uint i=0; i<x.size(); i++) {
        double original=vals[i];

        vals[i]=original+jump;
        double v1[1];
        c->cartSumSquaredError(vals, v1);

        vals[i]=original-jump;
        double v2[1]; 
        c->cartSumSquaredError(vals, v2);

        vals[i]=original;
        grad[i]=(v1[0]-v2[0])/(2.0*jump);
      }
    }

    return result[0]; 
  }


  double minfuncL2(const std::vector<double>& x, std::vector<double>& grad, void* data) {
    // Auxilory function to minimize (Sum of Squared joint angle error
    // from the requested configuration).  Because we wanted a Class
    // without static members, but NLOpt library does not support
    // passing methods of Classes, we use these auxilary functions.

    NLOPT_IK *c = (NLOPT_IK *) data;

    std::vector<double> vals(x);

    double jump=1e-8;
    double result[1]; 
    c->cartL2NormError(vals, result);

    if (!grad.empty()) {
      for (uint i=0; i<x.size(); i++) {
        double original=vals[i];

        vals[i]=original+jump;
        double v1[1];
        c->cartL2NormError(vals, v1);

        vals[i]=original-jump;
        double v2[1]; 
        c->cartL2NormError(vals, v2);

        vals[i]=original;
        grad[i]=(v1[0]-v2[0])/(2.0*jump);
      }
    }

    return result[0]; 
  }




  void constrainfuncm(uint m, double* result, uint n, const double* x, double* grad, void* data) {
    //Equality constraint auxilary function for Euclidean distance .
    //This also uses a small walk to approximate the gradient of the
    //constraint function at the current joint angles.

    NLOPT_IK *c = (NLOPT_IK *) data;

    std::vector<double> vals(n);

    for (uint i=0; i<n; i++) {
      vals[i]=x[i];
    }

    double jump=1e-8;
 
    //    assert(m==2);

    c->cartSumSquaredError(vals, result);

    if (grad!=NULL) {
      for (uint i=0; i<n; i++) {
        double o=vals[i];
        vals[i]=o+jump;
        double v1[m];
        c->cartSumSquaredError(vals, v1);
        vals[i]=o-jump;
        double v2[m];
        c->cartSumSquaredError(vals, v2);
        vals[i]=o;
        for (uint j=0; j<m; j++)  {
          grad[j*n+i]=(v1[j]-v2[j])/(2*jump);
        }
      }
    }
  }


  NLOPT_IK::NLOPT_IK(const KDL::Chain& _chain, const KDL::JntArray& _q_min, const KDL::JntArray& _q_max, double _maxtime, double _eps, OptType _type):
    chain(_chain), fksolver(_chain), maxtime(_maxtime), eps(std::abs(_eps)), TYPE(_type)
  {
    //Constructor for an IK Class.  Takes in a Chain to operate on,
    //the min and max joint limits, an (optional) maximum number of
    //iterations, and an (optional) desired error.

    if (chain.getNrOfJoints() < 2) {
      ROS_WARN_THROTTLE(1.0,"NLOpt_IK can only be run for chains of length 2 or more");
      return;
    }
    opt = nlopt::opt(nlopt::LD_SLSQP, _chain.getNrOfJoints());

    for (uint i=0; i<chain.getNrOfJoints(); i++) {
      lb.push_back(_q_min(i));
      ub.push_back(_q_max(i));
    }
       
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
    
    double tol = 1e-8;
    opt.set_xtol_abs(tol);

    aborted=false;

    std::vector<double> tolerance(1,1e-8);
    
    switch (TYPE) {
    case Joint: 
      opt.set_min_objective(minfunc, this);
      opt.add_equality_mconstraint(constrainfuncm, this, tolerance);
      break;
    case DualQuat: 
      opt.set_min_objective(minfuncDQ, this);
      break; 
    case SumSq: 
      opt.set_min_objective(minfuncSumSquared, this);
      break; 
    case L2: 
      opt.set_min_objective(minfuncL2, this);
      break; 
    }
  }


  double NLOPT_IK::minJoints(const std::vector<double>& x, std::vector<double>& grad)
  {
    // Actual function to compute the error between the current joint
    // configuration and the desired.  The SSE is easy to provide a
    // closed form gradient for.

    //    assert(des.size() == x.size());

    bool gradient = !grad.empty();

    double err = 0;
    for (uint i=0; i<x.size(); i++) {
      err += pow(x[i] - des[i],2);
      if (gradient) 
        grad[i]=2.0*(x[i]-des[i]);
    }

    return err;

  }


  void NLOPT_IK::cartSumSquaredError(const std::vector<double>& x, double error[])
  {
    // Actual function to compute Euclidean distance error.  This uses
    // the KDL Forward Kinematics solver to compute the Cartesian pose
    // of the current joint configuration and compares that to the
    // desired Cartesian pose for the IK solve.

    if (aborted) {
      opt.force_stop();
      return;     
    }

   
    KDL::JntArray q(x.size());

    for (uint i=0; i<x.size(); i++)
      q(i)=x[i];

    int rc = fksolver.JntToCart(q,currentPose);

    if (rc < 0)
      ROS_FATAL_STREAM("KDL FKSolver is failing: "<<q.data);

    if (isnan(currentPose.p.x())) {
      ROS_ERROR("NaNs from NLOpt!!");
      error[0] = std::numeric_limits<float>::max();
      aborted = true;
      return;
    }

    KDL::Twist delta_twist = KDL::diff(targetPose,currentPose);


    if (std::abs(delta_twist.vel.x()) <= std::abs(bounds.vel.x()))
      delta_twist.vel.x(0);
    
    if (std::abs(delta_twist.vel.y()) <= std::abs(bounds.vel.y()))
      delta_twist.vel.y(0);
    
    if (std::abs(delta_twist.vel.z()) <= std::abs(bounds.vel.z()))
      delta_twist.vel.z(0);
    
    if (std::abs(delta_twist.rot.x()) <= std::abs(bounds.rot.x()))
      delta_twist.rot.x(0);
    
    if (std::abs(delta_twist.rot.y()) <= std::abs(bounds.rot.y()))
      delta_twist.rot.y(0);
    
    if (std::abs(delta_twist.rot.z()) <= std::abs(bounds.rot.z()))
      delta_twist.rot.z(0);
    
    error[0] = KDL::dot(delta_twist.vel,delta_twist.vel) + KDL::dot(delta_twist.rot,delta_twist.rot);

    if (KDL::Equal(delta_twist,KDL::Twist::Zero(),eps)) {
      progress=0;
      double err1 = NLOPT_IK::JointErr(des,x);
      if (err1 < best_err) {
        best_x=x;
        best_err=err1;
      }
      if (!find_multiples) {
        aborted=true;
        return;
      }
    }
  }
  


  void NLOPT_IK::cartL2NormError(const std::vector<double>& x, double error[])
  {
    // Actual function to compute Euclidean distance error.  This uses
    // the KDL Forward Kinematics solver to compute the Cartesian pose
    // of the current joint configuration and compares that to the
    // desired Cartesian pose for the IK solve.

    if (aborted) {
      opt.force_stop();
      return;     
    }
  
    KDL::JntArray q(x.size());

    for (uint i=0; i<x.size(); i++)
      q(i)=x[i];

    int rc = fksolver.JntToCart(q,currentPose);

    if (rc < 0)
      ROS_FATAL_STREAM("KDL FKSolver is failing: "<<q.data);


    if (isnan(currentPose.p.x())) {
      ROS_ERROR("NaNs from NLOpt!!");
      error[0] = std::numeric_limits<float>::max();
      aborted = true;
      return;
    }

    KDL::Twist delta_twist = KDL::diff(targetPose,currentPose);


    if (std::abs(delta_twist.vel.x()) <= std::abs(bounds.vel.x()))
      delta_twist.vel.x(0);
    
    if (std::abs(delta_twist.vel.y()) <= std::abs(bounds.vel.y()))
      delta_twist.vel.y(0);
      
    if (std::abs(delta_twist.vel.z()) <= std::abs(bounds.vel.z()))
      delta_twist.vel.z(0);
      
    if (std::abs(delta_twist.rot.x()) <= std::abs(bounds.rot.x()))
      delta_twist.rot.x(0);
      
    if (std::abs(delta_twist.rot.y()) <= std::abs(bounds.rot.y()))
      delta_twist.rot.y(0);
      
    if (std::abs(delta_twist.rot.z()) <= std::abs(bounds.rot.z()))
      delta_twist.rot.z(0);

    error[0] = std::sqrt(KDL::dot(delta_twist.vel,delta_twist.vel) + KDL::dot(delta_twist.rot,delta_twist.rot));

    if (KDL::Equal(delta_twist,KDL::Twist::Zero(),eps)) {
      progress=0;
      double err1 = NLOPT_IK::JointErr(des,x);
      if (err1 < best_err) {
        best_x=x;
        best_err=err1;
      }
      if (!find_multiples) {
        aborted=true;
        return;
      }
    }
  }
  



  void NLOPT_IK::cartDQError(const std::vector<double>& x, double error[])
  {
    // Actual function to compute Euclidean distance error.  This uses
    // the KDL Forward Kinematics solver to compute the Cartesian pose
    // of the current joint configuration and compares that to the
    // desired Cartesian pose for the IK solve.

    if (aborted) {
      opt.force_stop();
      return;     
    }
    
    KDL::JntArray q(x.size());
    
    for (uint i=0; i<x.size(); i++)
      q(i)=x[i];
    
    int rc = fksolver.JntToCart(q,currentPose);

    if (rc < 0)
      ROS_FATAL_STREAM("KDL FKSolver is failing: "<<q.data);


    if (isnan(currentPose.p.x())) {
      ROS_ERROR("NaNs from NLOpt!!");
      error[0] = std::numeric_limits<float>::max();
      aborted = true;
      return;
    }



    KDL::Twist delta_twist = KDL::diff(targetPose,currentPose);

    if (std::abs(delta_twist.vel.x()) <= std::abs(bounds.vel.x()))
      delta_twist.vel.x(0);
      
    if (std::abs(delta_twist.vel.y()) <= std::abs(bounds.vel.y()))
      delta_twist.vel.y(0);
      
    if (std::abs(delta_twist.vel.z()) <= std::abs(bounds.vel.z()))
      delta_twist.vel.z(0);
      
    if (std::abs(delta_twist.rot.x()) <= std::abs(bounds.rot.x()))
      delta_twist.rot.x(0);
      
    if (std::abs(delta_twist.rot.y()) <= std::abs(bounds.rot.y()))
      delta_twist.rot.y(0);
      
    if (std::abs(delta_twist.rot.z()) <= std::abs(bounds.rot.z()))
      delta_twist.rot.z(0);

    currentPose = addDelta(targetPose,delta_twist,1);


    math3d::matrix3x3<double> currentRotationMatrix(currentPose.M.data); 
    math3d::quaternion<double> currentQuaternion = math3d::rot_matrix_to_quaternion<double>(currentRotationMatrix);
    math3d::point3d currentTranslation (currentPose.p.data);
    dual_quaternion currentDQ = dual_quaternion::rigid_transformation(currentQuaternion, currentTranslation); 

    dual_quaternion errorDQ = (currentDQ*!targetDQ).normalize(); 
    errorDQ.log(); 
    error[0] = 4.0f*dot(errorDQ,errorDQ);


    if (KDL::Equal(delta_twist,KDL::Twist::Zero(),eps)) {
      progress=0;
      double err1 = NLOPT_IK::JointErr(des,x);
      if (err1 < best_err) {
        best_x=x;
        best_err=err1;
      }
      if (!find_multiples) {
        aborted=true;
        return;
      }
    }
  }



  int NLOPT_IK::CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist _bounds, const KDL::JntArray& q_desired, bool find_multiples_) {
    // User command to start an IK solve.  Takes in a seed
    // configuration, a Cartesian pose, and (optional) a desired
    // configuration.  If the desired is not provided, the seed is
    // used.  Outputs the joint configuration found that solves the
    // IK.

    // Returns -3 if a configuration could not be found within the eps
    // set up in the constructor.

    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration diff;

    find_multiples=find_multiples_;

    aborted = false;    
    bounds = _bounds;
    q_out=q_init;

    if (chain.getNrOfJoints() < 2) {
      ROS_WARN_THROTTLE(1.0,"NLOpt_IK can only be run for chains of length 2 or more");
      return -3;//std::numeric_limits<float>::max();
    }

    opt.set_maxtime(maxtime);


    double minf; /* the minimum objective value, upon return */

    targetPose = p_in;

    if (TYPE == 1) { // DQ
      math3d::matrix3x3<double> targetRotationMatrix(targetPose.M.data); 
      math3d::quaternion<double> targetQuaternion = math3d::rot_matrix_to_quaternion<double>(targetRotationMatrix);
      math3d::point3d targetTranslation (targetPose.p.data);
      targetDQ = dual_quaternion::rigid_transformation(targetQuaternion, targetTranslation); 
    }
    // else if (TYPE == 1)
    // {
    //   z_target = targetPose*z_up;
    //   x_target = targetPose*x_out;
    //   y_target = targetPose*y_out;     
    // }

    
    //    fksolver.JntToCart(q_init,currentPose);

    std::vector<double> x(chain.getNrOfJoints());

    for (uint i=0; i < x.size(); i++)
      x[i] = q_init(i);
    
    best_x=x;
    progress = -3;
    best_err = DBL_MAX;

    if (q_desired.data.size()==0) {
      des=x;
    }
    else 
      {
        //      assert (q_desired.data.size()==(int)x.size());
        des.resize(x.size());
        for (uint i=0; i< des.size(); i++)
          des[i]=q_desired(i);
      }
    
    try {
      opt.optimize(x, minf);
    } catch (...) {
    }
    
    if (!aborted) {

      double time_left;
      diff=boost::posix_time::microsec_clock::local_time()-start_time;
      time_left = maxtime - diff.total_milliseconds()/1000.0;

      while (time_left > 0 && !aborted) {

        for (uint i=0; i< x.size(); i++)
          x[i]=fRand(lb[i], ub[i]);

        opt.set_maxtime(time_left);
	
        try 
          {
            opt.optimize(x, minf);
          } 
        catch (...) {}
	
        diff=boost::posix_time::microsec_clock::local_time()-start_time;
      	time_left = maxtime - diff.total_milliseconds()/1000.0;
      }
    }
           
    q_out.resize(chain.getNrOfJoints()); 
    
    for (uint i=0; i < x.size(); i++) {
      q_out(i) = best_x[i];
    }
        
    return progress;
    
  }
  

  void NLOPT_IK::abort() {
    aborted = true;
  }

}
