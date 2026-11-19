#include "auto_align/vision_align.hpp"
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <sstream>

namespace auto_align {
namespace {
void require(bool ok,const std::string& text) { if(!ok) throw std::runtime_error(text); }
}
Eigen::Vector3d visual_solve(const Eigen::MatrixXd& j,const Eigen::VectorXd& error,
                            double gain,double damping,double maximum) {
  require(j.cols()==3 && j.rows()==error.size() && j.rows()>=2 && j.rows()<=3 &&
    j.allFinite() && error.allFinite() && gain>0 && damping>0 && maximum>0,"유효한 UVS 선형계가 필요합니다.");
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(j);
  require(svd.singularValues().minCoeff()>.1 && svd.singularValues().maxCoeff()/svd.singularValues().minCoeff()<30,
    "UVS 대응 관계가 불충분하거나 특이합니다. 시험 이동·비전 품질을 확인하세요.");
  Eigen::MatrixXd normal=j*j.transpose()+damping*damping*Eigen::MatrixXd::Identity(j.rows(),j.rows());
  Eigen::Vector3d delta=-gain*j.transpose()*normal.ldlt().solve(error);
  require(delta.allFinite(),"UVS 보정량이 유효하지 않습니다.");
  if(delta.norm()>maximum) delta*=maximum/delta.norm();
  return delta;
}
void VisionAlign::learn(size_t side,bool rotation) {
  io_.report(std::string(rotation?"STEP5":"STEP6")+": "+(side==0?"left":"right")+" UVS 시험 이동·대응 관계 추정");
  const double probe=rotation?settings_.probe_rotation:settings_.probe_translation;
  Eigen::Matrix3d commands; VisualJacobian responses;
  for(int axis=0;axis<3;++axis) {
    const auto baseline=current_;
    const Eigen::Vector3d request=probe*Eigen::Vector3d::Unit(axis);
    Eigen::Vector3d actual;
    auto after=move(side,request,rotation,&actual);
    commands.col(axis)=actual;
    responses.col(axis)=features(after,side)-features(baseline,side);
    // Return using measured base-frame displacement; never infer a camera/base transform.
    current_=move(side,-actual,rotation);
  }
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(commands);
  require(svd.singularValues().minCoeff()>.4*probe,"초기 시험 이동의 실제 변화가 부족합니다.");
  auto& j=jacobians_[side][rotation?1:0]; j=responses*commands.inverse();
  // Require observable task directions, not the unobservable pipe-axis roll.
  if(rotation) visual_solve(j.bottomRows<2>(),Eigen::Vector2d::Zero(),1,settings_.damping,settings_.max_rotation);
  else visual_solve(j.topRows<3>(),Eigen::Vector3d::Zero(),1,settings_.damping,settings_.max_translation);
  learned_[side][rotation?1:0]=true;
}
bool VisionAlign::angles_ok(const VisionPair& pair) const {
  for(size_t i=0;i<2;++i)
    if(std::acos(std::clamp(std::abs(pair.pipes[i].axis.x()),0.,1.))>settings_.angle_tolerance) return false;
  return std::acos(std::clamp(std::abs(pair.pipes[0].axis.dot(pair.pipes[1].axis)),0.,1.))<=settings_.relative_angle_tolerance;
}
void VisionAlign::correct(size_t side,bool rotation) {
  if(!learned_[side][rotation?1:0]) learn(side,rotation);
  auto& j=jacobians_[side][rotation?1:0];
  const auto before=current_; const Features f=features(before,side);
  Eigen::Vector3d delta;
  double old_error;
  if(rotation) {
    old_error=f.tail<2>().norm();
    delta=visual_solve(j.bottomRows<2>(),f.tail<2>(),settings_.gain,settings_.damping,settings_.max_rotation);
  } else {
    old_error=(f.head<3>()-targets_[side]).norm();
    delta=visual_solve(j.topRows<3>(),f.head<3>()-targets_[side],settings_.gain,settings_.damping,settings_.max_translation);
  }
  if(delta.norm()<1e-7) return;
  Eigen::Vector3d actual;
  current_=move(side,delta,rotation,&actual);
  const Features change=features(current_,side)-f;
  const double next_error=rotation?features(current_,side).tail<2>().norm():
    (features(current_,side).head<3>()-targets_[side]).norm();
  const double noise=rotation?settings_.max_angle_noise:settings_.max_position_noise;
  auto& worsening=worsening_[side][rotation?1:0];
  if(next_error>old_error+noise) ++worsening;
  else if(next_error<old_error) worsening=0;
  require(worsening<settings_.max_worsening,"보정 오차가 반복해서 악화됐습니다.");
  // Broyden update with a small blend; both endpoint and axis coupling are retained.
  if(actual.norm()>(rotation?settings_.max_rotation:settings_.max_translation)*.2) {
    const Features residual=change-j*actual;
    j+=.2*residual*actual.transpose()/actual.squaredNorm();
  }
  std::ostringstream out;
  out<<(rotation?"STEP5":"STEP6")<<": "<<(side==0?"left":"right")
     <<" error "<<old_error<<" -> "<<next_error<<(rotation?" rad":" m")<<" / moves="<<moves_;
  io_.report(out.str());
}
void VisionAlign::rotation_stage() {
  io_.report("STEP5: 카메라 X축 평행 회전 보정·검증");
  for(int iteration=0;iteration<settings_.max_iterations;++iteration) {
    if(angles_ok(current_)) return;
    for(size_t side=0;side<2;++side) {
      // Also reduce residuals below the individual tolerance when the relative
      // angle is the failing condition (opposite residuals can add up).
      if(features(current_,side).tail<2>().norm()>settings_.angle_tolerance*.4)
        correct(side,true);
    }
    current_=measure();
  }
  throw std::runtime_error("step5 회전 반복 상한을 넘었습니다.");
}
} // namespace auto_align
