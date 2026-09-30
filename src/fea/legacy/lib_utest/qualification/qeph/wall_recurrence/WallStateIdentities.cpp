#include "WallStateIdentities.h"
#include <algorithm>
#include <cmath>

namespace tl::qualification::qeph::wall_recurrence {
namespace {
using G=recurrence::Group;
// Diagnostic classification from the old free-shell policy. It has no bearing
// on the full wall-state matrix, norm, spectrum, or admission dimension.
bool FormerlyRetained(const recurrence::Coordinate& c) {
  return c.group==G::Velocity||c.group==G::Spin||c.group==G::ForceCache||
    (c.group==G::History&&c.component>=5&&c.component<=24);
}
}
WallIdentityAnalysis CheckWallStateIdentities(const WallRecurrenceModel& model,double h,
                                            const Eigen::MatrixXd& a) {
  WallIdentityAnalysis out;
  if(!model.prepared()||!FrozenStep(h)||a.rows()!=static_cast<Eigen::Index>(model.native().dictionary.size())||
     a.rows()!=a.cols()||!a.allFinite()) {
    out.diagnostic="Invalid full-state identity operands"; return out;
  }
  const auto& m=model.native(); const auto& d=m.dictionary;
  for(unsigned row=0;row<d.size();++row) for(unsigned col=0;col<d.size();++col) {
    if(FormerlyRetained(d[row])&&!FormerlyRetained(d[col])&&std::abs(a(row,col))>out.passive_feedback_max) {
      out.passive_feedback_max=std::abs(a(row,col)); out.passive_feedback_row=row; out.passive_feedback_column=col;
    }
    if(d[row].group==G::Position||d[row].group==G::OrientationTangent) {
      const auto rate=model.coordinate(d[row].group==G::Position?G::Velocity:G::Spin,d[row].entity,d[row].component);
      const double expected=(row==col?1.:0.)+h*d[rate].scale/d[row].scale*a(rate,col);
      out.observer_error=std::max(out.observer_error,std::abs(a(row,col)-expected));
    }
    if(d[row].group==G::Velocity||d[row].group==G::Spin) {
      if(d[col].group!=G::ForceCache) continue;
      const bool force=d[col].component<12;
      const unsigned local=(d[col].component%12)/3,axis=d[col].component%3;
      double expected=0;
      if((d[row].group==G::Velocity)==force&&d[row].entity==m.connectivity[d[col].entity][local]&&d[row].component==axis)
        expected=-h*d[col].scale/((force?m.mass[d[row].entity]:m.inertia[d[row].entity])*d[row].scale);
      out.cached_kick_error=std::max(out.cached_kick_error,std::abs(a(row,col)-expected));
    }
  }
  // A geometric center is sufficient: translating the rotation origin adds an
  // already tested uniform tangential velocity, without changing the span.
  Vec3 center{};
  for(unsigned n=0;n<m.nodes;++n) { center.y+=m.position[n].y/m.nodes; center.z+=m.position[n].z/m.nodes; }
  for(unsigned kind=0;kind<3+m.nodes;++kind) {
    auto direction=Eigen::VectorXd::Zero(a.rows()).eval();
    for(unsigned n=0;n<m.nodes;++n) {
      if(kind<2) direction[model.coordinate(G::Velocity,n,kind+1)]=1;
      else if(kind==2) {
        direction[model.coordinate(G::Velocity,n,1)]=-(m.position[n].z-center.z)/recurrence::Length;
        direction[model.coordinate(G::Velocity,n,2)]=(m.position[n].y-center.y)/recurrence::Length;
        direction[model.coordinate(G::Spin,n,0)]=1;
      } else if(n==kind-3) direction[model.coordinate(G::Spin,n,0)]=1;
    }
    auto expected=direction;
    for(unsigned n=0;n<m.nodes;++n) for(unsigned axis=0;axis<3;++axis) {
      const auto p=model.coordinate(G::Position,n,axis),v=model.coordinate(G::Velocity,n,axis);
      const auto q=model.coordinate(G::OrientationTangent,n,axis),w=model.coordinate(G::Spin,n,axis);
      expected[p]+=h*d[v].scale/d[p].scale*direction[v];
      expected[q]+=h*d[w].scale/d[q].scale*direction[w];
    }
    const Eigen::VectorXd residual=a*direction-expected;
    if(!residual.allFinite()) { out.diagnostic="Nonfinite full wall-neutral identity"; return out; }
    const double magnitude=residual.cwiseAbs().maxCoeff();
    out.neutral_errors.push_back(magnitude); out.neutral_error=std::max(out.neutral_error,magnitude);
  }
  out.complete=std::isfinite(out.observer_error)&&std::isfinite(out.cached_kick_error)&&
    std::isfinite(out.neutral_error)&&std::isfinite(out.passive_feedback_max);
  out.passed=out.complete&&out.observer_error<=recurrence::MatrixTolerance&&
    out.cached_kick_error<=recurrence::MatrixTolerance&&out.neutral_error<=recurrence::MatrixTolerance;
  if(!out.passed) out.diagnostic="Full observer/native cache-sign/wall-neutral identity budget failed";
  return out;
}
WallBaselineAnalysis CheckWallMovingBaseline(const WallRecurrenceModel& model,double h,const MovingMatrixProbe& probe) {
  WallBaselineAnalysis out;
  if(!model.prepared()||!FrozenStep(h)||!FrozenVelocity(probe.velocity.x)||probe.velocity.y!=0||probe.velocity.z!=0||
     !probe.baseline_complete||probe.baseline.size()!=static_cast<Eigen::Index>(model.native().dictionary.size())||
     !probe.baseline.allFinite()) { out.diagnostic="Invalid/incomplete moving baseline"; return out; }
  const auto& d=model.native().dictionary; out.expected=Eigen::VectorXd::Zero(d.size());
  for(unsigned i=0;i<d.size();++i) {
    if(d[i].group==G::Position&&d[i].component==0) out.expected[i]=h*probe.velocity.x/d[i].scale;
    if(d[i].group==G::Velocity&&d[i].component==0) out.expected[i]=probe.velocity.x/d[i].scale;
  }
  out.residual=probe.baseline-out.expected;
  Eigen::Index index=0; out.maximum_error=out.residual.cwiseAbs().maxCoeff(&index);
  out.controlling_coordinate=static_cast<unsigned>(index); out.complete=std::isfinite(out.maximum_error);
  out.passed=out.complete&&out.maximum_error<=recurrence::MatrixTolerance;
  if(!out.passed) out.diagnostic="Actual uniform-translation baseline failed before centering";
  return out;
}
} // namespace tl::qualification::qeph::wall_recurrence
