#include "kdl_planner.h"


KDLPlanner::KDLPlanner(){}

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

trajectory_point KDLPlanner::compute_trajectory(double time)
{
  /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  trajectory_point traj;

  Eigen::Vector3d ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);

  if(time <= accDuration_)
  {
    traj.pos = trajInit_ + 0.5*ddot_traj_c*std::pow(time,2);
    traj.vel = ddot_traj_c*time;
    traj.acc = ddot_traj_c;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    traj.pos = trajInit_ + ddot_traj_c*accDuration_*(time-accDuration_/2);
    traj.vel = ddot_traj_c*accDuration_;
    traj.acc = Eigen::Vector3d::Zero();
  }
  else
  {
    traj.pos = trajEnd_ - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
    traj.vel = ddot_traj_c*(trajDuration_-time);
    traj.acc = -ddot_traj_c;
  }

  return traj;
  
}

////////////////////////////////////////////////////////////////////////////////////

void KDLPlanner::trapezoidal_vel(double time, double time_c, double& s, double& s_dot, double& s_ddot){
    double time_f=trajDuration_;
    double ddot_sc = -1.0/(std::pow(time_c,2)-time_f*time_c);
    
    if(time<=time_c){
      s=0.5*ddot_sc*time*time;
      s_dot=ddot_sc*time;
      s_ddot=ddot_sc;
    }
    else if(time<=(time_f-time_c)){
      s=time_c*ddot_sc*(time - 0.5*time_c);
      s_dot=ddot_sc*time_c;
      s_ddot=0.0;
    }
    else if(time<=time_f){
      s=1.0-0.5*ddot_sc*std::pow(time_f-time,2);
      s_dot=ddot_sc*(time_f-time);
      s_ddot=-ddot_sc;
    }
    else{
      s=1.0;
      s_dot=0.0;
      s_ddot=0.0;
    }
}

void KDLPlanner::cubic_polinomial(double time, double& s, double& s_dot, double& s_ddot){
    double time_f=trajDuration_;
    // We supposed s0=0; sf=1; s0_dot=0; sf_dot=0;
    double a0=0.0;
    double a1=0.0;
    double a2=3.0/(time_f*time_f);
    double a3=-2.0/(time_f*time_f*time_f);
    
    s=a3*(time*time*time)+a2*(time*time)+a1*time+a0;
    s_dot=3.0*a3*(time*time)+2.0*a2*time+a1;
    s_ddot=6.0*a3*time+2.0*a2;
}

KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius){
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
}

trajectory_point KDLPlanner::compute_trajectory_circle(double time, double time_c){
    double s, s_dot, s_ddot;
    
    if(time_c==0){
        cubic_polinomial(time, s, s_dot, s_ddot);
    } else {
        trapezoidal_vel(time, time_c, s, s_dot, s_ddot);
    }
    trajectory_point traj;
    
    traj.pos.x() = trajInit_[0];
    traj.pos.y() = trajInit_[1]+trajRadius_ - trajRadius_ * std::cos(2*M_PI*s);
    traj.pos.z() = trajInit_[2] - trajRadius_ * std::sin(2*M_PI*s);

    traj.vel.x() = 0.0;
    traj.vel.y() = 2*M_PI*trajRadius_ * std::sin(2*M_PI*s) * s_dot;
    traj.vel.z() = -2*M_PI*trajRadius_ * std::cos(2*M_PI*s) * s_dot;

    traj.acc.x() = 0.0;
    traj.acc.y() = 2*M_PI*trajRadius_ * (s_ddot*std::sin(2*M_PI*s) + 2*M_PI*s_dot*s_dot * std::cos(2*M_PI*s));
    traj.acc.z() = 2*M_PI*trajRadius_ * (-s_ddot*std::cos(2*M_PI*s) + 2*M_PI*s_dot*s_dot * std::sin(2*M_PI*s));
    
    return traj;
}

KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd){
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

trajectory_point KDLPlanner::compute_trajectory_linear(double time, double time_c){
    double s, s_dot, s_ddot;
    if(time_c==0){
        cubic_polinomial(time, s, s_dot, s_ddot);
    } else {
        trapezoidal_vel(time, time_c, s, s_dot, s_ddot);
    }
    Eigen::Vector3d diff_EI = trajEnd_ - trajInit_;
    
    trajectory_point traj;
    
    traj.pos = trajInit_ + s * (diff_EI);
    traj.vel = s_dot * (diff_EI);
    traj.acc = s_ddot * (diff_EI);
    
    return traj;
}
