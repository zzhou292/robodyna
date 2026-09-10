#include "WallStateMetric.h"
#include <algorithm>
#include <cmath>
#include <utility>

namespace tl::qualification::qeph::wall_recurrence {
bool ValidWallStateDiagonal(const Eigen::VectorXd& d) noexcept {
  return d.size()>0&&d.size()<=194&&d.allFinite()&&d.minCoeff()>0&&
    std::isfinite(d.maxCoeff()/d.minCoeff())&&d.maxCoeff()/d.minCoeff()<=MaximumMetricCondition;
}
bool BuildWallStateMetric(const WallRecurrenceModel& model,WallStateMetric& out,std::string& error) {
  if(!model.prepared()) { error="Unprepared wall metric model"; return false; }
  WallStateMetric result;
  result.sound_speed=std::sqrt(recurrence::Young/recurrence::Density);
  result.eta=ScreenHorizon*result.sound_speed/recurrence::Length;
  result.tangential_weight=1/(1+result.eta);
  result.normal_weight=std::max(result.tangential_weight,
    model.maximum_frequency()*recurrence::Length/result.sound_speed);
  const auto& dictionary=model.native().dictionary;
  result.diagonal=Eigen::VectorXd::Ones(dictionary.size());
  for(unsigned i=0;i<dictionary.size();++i) {
    const auto& c=dictionary[i];
    if(c.group==recurrence::Group::Position)
      result.diagonal[i]=c.component==0?result.normal_weight:result.tangential_weight;
    else if(c.group==recurrence::Group::OrientationTangent)
      result.diagonal[i]=result.tangential_weight;
  }
  if(!ValidWallStateDiagonal(result.diagonal)) { error="Invalid full-state metric weights/condition"; return false; }
  result.condition=result.diagonal.maxCoeff()/result.diagonal.minCoeff();
  out=std::move(result); error.clear(); return true;
}
bool ApplyWallStateMetric(const Eigen::MatrixXd& a,const Eigen::VectorXd& d,
                          Eigen::MatrixXd& out,std::string& error) {
  if(!ValidWallStateDiagonal(d)||a.rows()!=d.size()||a.cols()!=d.size()||!a.allFinite()) {
    error="Invalid full-state similarity operands"; return false;
  }
  Eigen::MatrixXd result(a.rows(),a.cols());
  for(Eigen::Index row=0;row<a.rows();++row) for(Eigen::Index col=0;col<a.cols();++col)
    result(row,col)=d[row]*a(row,col)/d[col];
  if(!result.allFinite()) { error="Nonfinite full-state similarity result"; return false; }
  out=std::move(result); error.clear(); return true;
}
} // namespace tl::qualification::qeph::wall_recurrence
