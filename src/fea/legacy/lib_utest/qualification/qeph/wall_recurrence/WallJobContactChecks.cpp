#include "WallJobAnalysis.h"
#include "ContactDerivativeChecks.h"
#include <cmath>
#include <utility>

namespace tl::qualification::qeph::wall_recurrence {
namespace {
bool Sample(const ContactMapSample& s,const WallRecurrenceModel& model,bool active) {
  const auto& m=model.native();
  if(s.state.size()!=static_cast<Eigen::Index>(m.dictionary.size())||!s.state.allFinite()||
     s.nodes.size()!=m.nodes||s.parents.size()!=m.elements||s.base_epoch!=1||s.attempt!=1||
     !contact::nodal_wall_detail::Certificate(s.resultant)||!contact::nodal_wall_detail::Certificate(s.potential)||
     !contact::IsFinite(s.wall_reaction)||!contact::IsFinite(s.wall_moment)||!std::isfinite(s.surface_power)) return false;
  for(unsigned n=0;n<m.nodes;++n) {
    const auto& p=s.nodes[n];
    if(!p.valid||p.fixed||p.node!=n||p.base_epoch!=1||p.attempt!=1||p.touching_or_penetrating!=active||
       !contact::nodal_wall_detail::Certificate(p.force)||!contact::nodal_wall_detail::Certificate(p.potential)||
       !contact::nodal_wall_detail::Certificate(p.stiffness,true)||!contact::IsFinite(p.force_world)||
       !contact::IsFinite(p.wall_point)||!contact::IsFinite(p.wall_reaction)||!contact::IsFinite(p.wall_moment)||
       !std::isfinite(p.surface_power)||!std::isfinite(p.local_velocity_first_timestep)||p.local_velocity_first_timestep<0||
       !p.row.valid||p.row.count!=1||p.row.nodes[0]!=n||p.row.base_epoch!=1||p.row.attempt!=1||
       !std::isfinite(p.row.stiffness[0])||p.row.stiffness[0]<0||!std::isfinite(p.row.damping[0])||p.row.damping[0]<0) return false;
  }
  for(unsigned e=0;e<m.elements;++e) {
    const auto& p=s.parents[e]; const auto& expected=model.weights().parent(e);
    if(!p.valid||p.family!=expected.family||p.arity!=4||p.parent_element_id!=expected.parent_element_id||
       p.parent_face_id!=expected.parent_face_id||p.feature_id!=expected.feature_id||
       !contact::nodal_wall_detail::Certificate(p.resultant)||!contact::nodal_wall_detail::Certificate(p.potential)) return false;
    for(const auto& force:p.force) if(!contact::nodal_wall_detail::Certificate(force)) return false;
  }
  return true;
}
}
WallContactRecheck RecheckWallContact(const WallRecurrenceModel& model,double h,double velocity,
                                    ContactBranch branch,const Eigen::MatrixXd& shell,const ContactBranchProbe& raw) {
  WallContactRecheck out; out.branch=branch;
  if(!model.prepared()||!FrozenStep(h)||!FrozenVelocity(velocity)||
     (branch!=ContactBranch::Inactive&&branch!=ContactBranch::Active)||raw.h!=h||raw.normal_velocity!=velocity||
     raw.branch!=branch||!raw.baseline_complete) {
    out.diagnostic="Invalid stored contact probe identity/baseline"; return out;
  }
  Eigen::MatrixXd rebuilt; std::vector<ContactDirection> directions;
  if(!BuildContactBranch(model,h,branch,shell,rebuilt,out.diagnostic)||
     !SignConeDirections(model,branch,directions,out.diagnostic)) return out;
  out.operator_difference=CompareWallMatrices(raw.full,rebuilt);
  if(raw.directions.size()!=directions.size()||!Sample(raw.baseline,model,true)) {
    out.diagnostic="Missing/malformed physical baseline or frozen direction inventory"; return out;
  }
  MovingMatrixProbe baseline; baseline.velocity={velocity,0,0}; baseline.baseline_complete=true;
  baseline.baseline=raw.baseline.state; out.baseline=CheckWallMovingBaseline(model,h,baseline);
  out.input_valid=true; bool complete=out.operator_difference.complete&&out.baseline.complete;
  bool passed=out.operator_difference.passed&&out.baseline.passed;
  for(unsigned i=0;i<directions.size();++i) {
    const auto& expected=directions[i]; const auto& p=raw.directions[i];
    WallDirectionRecheck d; d.name=expected.name;
    if(p.direction.name!=expected.name||p.direction.value.size()!=expected.value.size()||
       !p.direction.value.allFinite()||p.direction.value!=expected.value||p.completed_samples!=3) {
      d.diagnostic="Changed direction or incomplete physical sample count";
    } else {
      d.samples_valid=true;
      for(const auto& sample:p.samples) d.samples_valid=d.samples_valid&&Sample(sample,model,branch==ContactBranch::Active);
      if(!d.samples_valid) d.diagnostic="Malformed sample identity/certificate/sign cone";
      else {
        d.complete=true; d.passed=true;
        for(unsigned level=0;level<2;++level) {
          d.quotients[level]=derivative_detail::OneSidedQuotient(raw.baseline.state,p.samples[level].state,
                                                               p.samples[level+1].state,recurrence::Amplitudes[level]);
          const bool valid=d.quotients[level].allFinite()&&
            derivative_detail::CompareDirection(rebuilt,expected.value,d.quotients[level],d.residual_upper[level],d.budget_lower[level]);
          d.complete=d.complete&&valid;
          d.passed=d.passed&&valid&&d.residual_upper[level]<=d.budget_lower[level];
        }
        if(!d.passed) d.diagnostic="Recomputed physical/native one-sided derivative failed";
      }
    }
    if(d.complete) ++out.completed_directions;
    complete=complete&&d.complete; passed=passed&&d.passed; out.directions.push_back(std::move(d));
  }
  out.complete=complete; out.passed=complete&&passed;
  if(!out.passed) out.diagnostic="Stored contact samples failed recomputed checks; saved verdicts were not used";
  return out;
}
} // namespace tl::qualification::qeph::wall_recurrence
