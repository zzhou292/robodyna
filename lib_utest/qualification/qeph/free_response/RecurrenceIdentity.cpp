#include "RecurrenceIdentity.h"
#include <Eigen/SVD>
#include <cmath>

namespace tl::qualification::qeph::recurrence {
bool CheckStructuralIdentities(const Model& m,double h,const Eigen::MatrixXd& a,MapAnalysis& out) {
  const auto feedback=FeedbackIndices(m); const unsigned nd=3*m.nodes;
  for(unsigned row:feedback) for(unsigned col=0;col<m.dictionary.size();++col)
    if(!m.dictionary[col].feedback) out.zero_feedback_error=std::max(out.zero_feedback_error,std::abs(a(row,col)));
  const double beta=h*std::sqrt(Young/Density)/Length;
  for(unsigned i=0;i<nd;++i) for(unsigned col=0;col<m.dictionary.size();++col) {
    out.observer_error=std::max(out.observer_error,std::abs(a(i,col)-(col==i?1.:0.)-beta*a(2*nd+i,col)));
    out.observer_error=std::max(out.observer_error,std::abs(a(nd+i,col)-(col==nd+i?1.:0.)-beta*a(3*nd+i,col)));
  }
  Vec3 center{}; double mass=0;
  for(unsigned n=0;n<m.nodes;++n) { mass+=m.mass[n]; center.x+=m.mass[n]*m.position[n].x;
    center.y+=m.mass[n]*m.position[n].y; center.z+=m.mass[n]*m.position[n].z; }
  center={center.x/mass,center.y/mass,center.z/mass};
  Eigen::MatrixXd basis=Eigen::MatrixXd::Zero(2*nd,6);
  for(unsigned column=0;column<6+m.nodes;++column) {
    Eigen::VectorXd rigid=Eigen::VectorXd::Zero(a.cols());
    for(unsigned n=0;n<m.nodes;++n) {
      Vec3 v{},w{}; const auto x=m.position[n];
      if(column<3) { if(column==0) v.x=1; if(column==1) v.y=1; if(column==2) v.z=1; }
      else if(column<6) {
        const Vec3 arm{(x.x-center.x)/Length,(x.y-center.y)/Length,(x.z-center.z)/Length};
        if(column==3) { w.x=1; v={0,-arm.z,arm.y}; }
        if(column==4) { w.y=1; v={arm.z,0,-arm.x}; }
        if(column==5) { w.z=1; v={-arm.y,arm.x,0}; }
      } else if(n==column-6) w.z=1; // Selected local-Z drilling is inactive at the flat reference.
      const double vv[]{v.x,v.y,v.z},ww[]{w.x,w.y,w.z};
      for(unsigned axis=0;axis<3;++axis) {
        rigid[2*nd+3*n+axis]=vv[axis]; rigid[3*nd+3*n+axis]=ww[axis];
        if(column<6) {
          basis(3*n+axis,column)=std::sqrt(m.mass[n])*vv[axis];
          basis(nd+3*n+axis,column)=std::sqrt(m.inertia[n])/Length*ww[axis];
        }
      }
    }
    const Eigen::VectorXd residual=a*rigid-rigid;
    for(unsigned row:feedback) out.rigid_error=std::max(out.rigid_error,std::abs(residual[row]));
  }
  for(unsigned column=0;column<6;++column) {
    const double norm=basis.col(column).norm();
    if(!std::isfinite(norm)||norm<=0) { out.diagnostic="Degenerate common-mass rigid basis"; return false; }
    basis.col(column)/=norm;
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(basis,Eigen::ComputeThinU|Eigen::ComputeThinV);
  if(svd.info()!=Eigen::Success||!svd.singularValues().allFinite()||svd.singularValues().tail(1)[0]<=0) {
    out.diagnostic="Unresolved common-mass rigid basis"; return false;
  }
  out.rigid_basis_condition=svd.singularValues()[0]/svd.singularValues().tail(1)[0];
  const auto& u=svd.matrixU();
  out.rigid_projection_residual=(basis-u*(u.transpose()*basis)).norm()/basis.norm();
  return std::isfinite(out.rigid_basis_condition)&&std::isfinite(out.rigid_projection_residual);
}
} // namespace tl::qualification::qeph::recurrence
