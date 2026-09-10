#include "MovingNativeProbe.h"
#include <cmath>

namespace tl::qualification::qeph::wall_recurrence {
MovingMatrixProbe DifferentiateMoving(const recurrence::Model& m,double h,
                                     double epsilon,const Vec3& velocity) {
  MovingMatrixProbe result; result.velocity=velocity;
  auto& p=result.derivative; p.amplitude=epsilon;
  const auto n=static_cast<Eigen::Index>(m.dictionary.size());
  if(n!=109&&n!=194) { p.diagnostic="Unsupported complete native dictionary size"; return result; }
  p.full=Eigen::MatrixXd::Zero(n,n);
  if(!std::isfinite(epsilon)||epsilon<=0) { p.diagnostic="Invalid probe amplitude"; return result; }
  const Eigen::VectorXd zero=Eigen::VectorXd::Zero(n);
  if(!recurrence::NativeMapWithUniformVelocity(m,h,velocity,zero,result.baseline,p.diagnostic)) return result;
  result.baseline_complete=true;
  for(Eigen::Index column=0;column<n;++column) {
    auto operand=zero; operand[column]=epsilon; Eigen::VectorXd plus,minus;
    if(!recurrence::NativeMapWithUniformVelocity(m,h,velocity,operand,plus,p.diagnostic)) return result;
    operand[column]=-epsilon;
    if(!recurrence::NativeMapWithUniformVelocity(m,h,velocity,operand,minus,p.diagnostic)) return result;
    const Eigen::VectorXd derivative=(plus-minus)/(2*epsilon);
    if(!derivative.allFinite()) { p.diagnostic="Nonfinite centered difference"; return result; }
    p.full.col(column)=derivative;
    ++p.completed_columns;
  }
  p.complete=true; return result;
}
} // namespace tl::qualification::qeph::wall_recurrence
