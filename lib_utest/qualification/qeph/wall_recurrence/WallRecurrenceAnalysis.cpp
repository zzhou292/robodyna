#include "WallRecurrenceAnalysis.h"

namespace tl::qualification::qeph::wall_recurrence {
WallBranchAnalysis AnalyzeWallBranches(const WallRecurrenceModel& model,double h,const Eigen::MatrixXd& shell) {
  WallBranchAnalysis out; out.h=h;
  if(!model.prepared()||!FrozenStep(h)||shell.rows()!=static_cast<Eigen::Index>(model.native().dictionary.size())||
     shell.rows()!=shell.cols()||!shell.allFinite()) {
    out.diagnostic="Invalid named full-state analysis operands"; return out;
  }
  if(!BuildWallStateMetric(model,out.metric,out.diagnostic)) return out;
  out.schedule=AnalyzeWallSwitchingSchedule(model,h);
  if(!out.schedule.passed) { out.diagnostic=out.schedule.diagnostic; return out; }
  bool complete=true,passed=true;
  for(unsigned branch=0;branch<2;++branch) {
    auto& b=out.branches[branch]; b.branch=branch==0?ContactBranch::Inactive:ContactBranch::Active;
    if(!BuildContactBranch(model,h,b.branch,shell,b.full,out.diagnostic)) return out;
    if(!ApplyWallStateMetric(b.full,out.metric.diagonal,b.weighted,out.diagnostic)) return out;
    b.identities=CheckWallStateIdentities(model,h,b.full);
    b.spectrum=AnalyzeConstantWallSpectrum(b.full);
    b.weighted_spectrum=AnalyzeConstantWallSpectrum(b.weighted);
    b.continuous=AnalyzeWallSequence({{&b.full,out.schedule.ordinary_steps}},out.metric.diagonal);
    const bool done=b.identities.complete&&b.spectrum.complete&&b.weighted_spectrum.complete&&b.continuous.complete;
    if(done) ++out.completed_branches;
    complete=complete&&done;
    passed=passed&&b.identities.passed&&b.spectrum.passed&&b.weighted_spectrum.passed&&b.continuous.passed;
  }
  for(unsigned i=0;i<out.events.size();++i) {
    auto& e=out.events[i]; e.window=out.schedule.windows[i];
    e.sequence=AnalyzeWallSequence({{&out.branches[0].full,e.window.inactive_before},
                                   {&out.branches[1].full,e.window.active},
                                   {&out.branches[0].full,e.window.inactive_after}},out.metric.diagonal);
    if(e.sequence.complete) ++out.completed_events;
    complete=complete&&e.sequence.complete; passed=passed&&e.sequence.passed;
  }
  out.complete=complete; out.passed=complete&&passed;
  if(!out.passed) out.diagnostic="Full-state constant/switching screen failed; component evidence is retained";
  return out;
}
} // namespace tl::qualification::qeph::wall_recurrence
