#include "RecurrenceAudit.h"
#include "lib_src/math/Quaternion.h"
#include <cmath>

namespace tl::qualification::qeph::recurrence {
namespace {
struct State {
  std::array<Vec3,MaxNodes> x{},v{},omega{},tangent{};
  std::array<HistoryValues,MaxElements> history;
  std::array<std::array<Vec3,4>,MaxElements> force{},couple{};
};
double& Component(Vec3& v,unsigned c) { return c==0?v.x:(c==1?v.y:v.z); }
double& HistoryComponent(HistoryValues& v,unsigned c) {
  if(c<5) return v.stress[c]; if(c<10) return v.material_stress[c-5];
  if(c<13) return v.bending_stress[c-10]; if(c<25) return v.stabilization[c-13];
  if(c<33) return v.strain_curvature[c-25]; if(c==33) return v.thickness;
  return c<36?v.internal_work[c-34]:v.hourglass_viscous_work;
}
double& Value(State& s,const Coordinate& c) {
  switch(c.group) {
    case Group::Position:return Component(s.x[c.entity],c.component);
    case Group::OrientationTangent:return Component(s.tangent[c.entity],c.component);
    case Group::Velocity:return Component(s.v[c.entity],c.component);
    case Group::Spin:return Component(s.omega[c.entity],c.component);
    case Group::History:return HistoryComponent(s.history[c.entity],c.component);
    case Group::ForceCache:return Component(c.component<12?s.force[c.entity][c.component/3]:s.couple[c.entity][(c.component-12)/3],c.component%3);
  }
  return s.x[0].x; // Exhaustive internal enum, validated by immutable dictionary.
}
State Rest(const Model& m) {
  State s; s.x=m.position; for(unsigned e=0;e<m.elements;++e) s.history[e].thickness=Thickness; return s;
}
bool SaneModel(const Model& m) {
  if(m.elements<1||m.elements>2||m.nodes!=2*(m.elements+1)||m.dictionary.size()!=12*m.nodes+61*m.elements) return false;
  for(unsigned e=0;e<m.elements;++e) {
    if(!m.reference[e].prepared()) return false;
    for(unsigned i:m.connectivity[e]) if(i>=m.nodes) return false;
  }
  for(const auto& c:m.dictionary) {
    const auto g=static_cast<unsigned>(c.group);
    if(g>5||c.entity>=(g<4?m.nodes:m.elements)||c.component>=(g<4?3u:(g==4?37u:24u))||
       !std::isfinite(c.scale)||c.scale<=0) return false;
  }
  return true;
}
}
bool NativeMap(const Model& m,double h,const Eigen::VectorXd& input,Eigen::VectorXd& output,std::string& error) {
  if(!SaneModel(m)||input.size()!=static_cast<Eigen::Index>(m.dictionary.size())||
     !input.allFinite()||!std::isfinite(h)||h<=0||!std::isfinite(2*h)||h==2*h) {
    error="Malformed native recurrence operands"; return false;
  }
  State baseline=Rest(m),base=baseline;
  for(unsigned i=0;i<m.dictionary.size();++i) Value(base,m.dictionary[i])+=input[i]*m.dictionary[i].scale;
  State next=base;
  std::array<Vec3,MaxNodes> force{},couple{};
  for(unsigned e=0;e<m.elements;++e) for(unsigned i=0;i<4;++i) for(unsigned a=0;a<3;++a) {
    const auto n=m.connectivity[e][i]; Component(force[n],a)-=Component(base.force[e][i],a);
    Component(couple[n],a)-=Component(base.couple[e][i],a);
  }
  for(unsigned n=0;n<m.nodes;++n) {
    if(!std::isfinite(m.mass[n])||m.mass[n]<=0||!std::isfinite(m.inertia[n])||m.inertia[n]<=0) {
      error="Invalid native nodal mass/inertia"; return false;
    }
    double rotation[3];
    for(unsigned a=0;a<3;++a) {
      const double v=static_cast<double>(Component(base.v[n],a)+static_cast<long double>(h)*Component(force[n],a)/m.mass[n]);
      const double w=static_cast<double>(Component(base.omega[n],a)+static_cast<long double>(h)*Component(couple[n],a)/m.inertia[n]);
      Component(next.v[n],a)=v; Component(next.omega[n],a)=w;
      Component(next.x[n],a)=static_cast<double>(Component(base.x[n],a)+static_cast<long double>(h)*v); rotation[a]=h*w;
    }
    const auto t=base.tangent[n]; const double square=.25*(t.x*t.x+t.y*t.y+t.z*t.z);
    if(!std::isfinite(square)||square>=1) { error="Orientation tangent outside its local chart"; return false; }
    tl::math::Quaternion q;
    if(!tl::math::IncrementWorldRotation({std::sqrt(1-square),.5*t.x,.5*t.y,.5*t.z},rotation,q)) {
      error="Native probe's shared orientation update rejected"; return false;
    }
    next.tangent[n]={2*q.x,2*q.y,2*q.z};
  }
  for(unsigned e=0;e<m.elements;++e) {
    History history;
    if(PreparePrescribedHistory(m.reference[e],base.history[e],{h,1},history)!=Status::kSuccess) {
      error="Native prescribed history rejected"; return false;
    }
    PrescribedInterval interval; interval.base_time=h; interval.dt=h; interval.sample_index=2;
    for(unsigned i=0;i<4;++i) { const auto n=m.connectivity[e][i];
      interval.position_endpoint[i]=next.x[n]; interval.velocity_midpoint[i]=next.v[n]; interval.omega_midpoint[i]=next.omega[n]; }
    ForceTrial trial;
    if(EvaluateForce(m.reference[e],history,interval,trial)!=Status::kSuccess) {
      error="Native full force/history recurrence rejected"; return false;
    }
    next.history[e]=trial.proposed_history.data(); next.force[e]=trial.internal_force; next.couple[e]=trial.internal_couple;
  }
  Eigen::VectorXd result(input.size());
  for(unsigned i=0;i<m.dictionary.size();++i)
    result[i]=(Value(next,m.dictionary[i])-Value(baseline,m.dictionary[i]))/m.dictionary[i].scale;
  if(!result.allFinite()) { error="Nonfinite normalized native map output"; return false; }
  output=std::move(result); error.clear(); return true;
}
MatrixProbe Differentiate(const Model& m,double h,double epsilon) {
  MatrixProbe p; p.amplitude=epsilon;
  const auto n=static_cast<Eigen::Index>(m.dictionary.size()); p.full=Eigen::MatrixXd::Zero(n,n);
  if(!std::isfinite(epsilon)||epsilon<=0) { p.diagnostic="Invalid probe amplitude"; return p; }
  Eigen::VectorXd zero=Eigen::VectorXd::Zero(n),value;
  if(!NativeMap(m,h,zero,value,p.diagnostic)) return p;
  if(value.cwiseAbs().maxCoeff()!=0) { p.diagnostic="Reference/rest is not a fixed point"; return p; }
  for(Eigen::Index column=0;column<n;++column) {
    auto operand=zero; operand[column]=epsilon; Eigen::VectorXd plus,minus;
    if(!NativeMap(m,h,operand,plus,p.diagnostic)) return p;
    operand[column]=-epsilon;
    if(!NativeMap(m,h,operand,minus,p.diagnostic)) return p;
    const Eigen::VectorXd derivative=(plus-minus)/(2*epsilon);
    if(!derivative.allFinite()) { p.diagnostic="Nonfinite centered difference"; return p; }
    p.full.col(column)=derivative;
    ++p.completed_columns;
  }
  p.complete=true; return p;
}
} // namespace tl::qualification::qeph::recurrence
