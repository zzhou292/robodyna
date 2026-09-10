// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalRigidInertiaFinalize.h"
#include <Eigen/Eigenvalues>
#include <cmath>
namespace tl::fea::rigid {
namespace {
using Status=NodalRigidGroupStatus;
bool Positive(double x) {return std::isfinite(x)&&x>0;}
NodalRigidGroupReport Fail(Status code,const char* message,std::size_t group) {return {code,message,group,SIZE_MAX};}
Eigen::Vector3d EigenVector(Vec3 v) {return {v.x,v.y,v.z};}
Vec3 Value(const Eigen::Vector3d& v) {return {v[0],v[1],v[2]};}
tl::math::Matrix3 Value(const Eigen::Matrix3d& m) {
  tl::math::Matrix3 out;
  for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)out.v[3*i+j]=m(i,j);
  return out;
}
}
NodalRigidGroupReport FinalizeInertia(const tl::math::Matrix3& input,
    NodalRigidGroupProperties& g,std::size_t group) noexcept {
  Eigen::Matrix3d tensor;
  for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)tensor(i,j)=input.v[3*i+j];
  const double scale=tensor.cwiseAbs().maxCoeff();
  if(!Positive(scale)) return Fail(Status::EigenFailure,"Invalid inertia tensor scale",group);
  // Fixed-size Eigen storage; axes/sign/order are an equivalent tensor
  // factorization, not a claim of native VALPR bitwise eigenvector identity.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(tensor/scale);
  if(eigen.info()!=Eigen::Success) return Fail(Status::EigenFailure,"Principal inertia decomposition failed",group);
  Eigen::Vector3d raw=eigen.eigenvalues()*scale;
  Eigen::Matrix3d axes=eigen.eigenvectors();
  if(axes.determinant()<0) axes.col(2)*=-1;
  rigid::PrincipalCorrection correction;
  if(rigid::CorrectPrincipalInertia(Value(raw),correction)!=rigid::MathStatus::Success)
    return Fail(Status::EigenFailure,"Principal inertia cannot be admitted after source correction",group);
  g.principal={Value(axes),correction.effective};
  if(!rigid::detail::Orthonormal(g.principal.axes))
    return Fail(Status::EigenFailure,"Principal frame is not a proper orthonormal frame",group);
  const Eigen::Matrix3d added=axes*EigenVector(correction.added).asDiagonal()*axes.transpose();
  const Eigen::Matrix3d effective=tensor+added;
  if(!added.allFinite()||!effective.allFinite()) return Fail(Status::NonfiniteResult,"Effective group tensor overflow",group);
  g.raw_tensor=Value(tensor); g.effective_tensor=Value(effective); g.raw_principal_inertia=Value(raw);
  g.regularization.principal_inertia_added=correction.added;
  g.regularization.tensor_added=Value(added);
  g.regularization.principal_threshold_reached=correction.threshold_reached;
  g.regularization.principal_inertia_changed=correction.changed;
  return {};
}
} // namespace tl::fea::rigid
