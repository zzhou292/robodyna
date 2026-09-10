#include "WallScreenSelection.h"
#include <cmath>

namespace tl::qualification::qeph::wall_recurrence {
namespace {
int JobSlot(unsigned cells,double velocity) {
  if(cells!=1&&cells!=2) return -1;
  const int boost=velocity==0&&!std::signbit(velocity)?0:velocity==-8?1:velocity==8?2:-1;
  return boost<0?-1:static_cast<int>(3*(cells-1))+boost;
}
int BoostSlot(unsigned cells,double velocity) {
  if(cells!=1&&cells!=2) return -1;
  return velocity==-8?static_cast<int>(2*(cells-1)):velocity==8?static_cast<int>(2*(cells-1)+1):-1;
}
bool Difference(const WallDifference& d) {
  if(!d.complete) return !d.passed;
  return std::isfinite(d.difference)&&d.difference>=0&&std::isfinite(d.budget)&&d.budget>0&&
    d.passed==(d.difference<=d.budget);
}
bool Boost(const WallBoostComparison& b) {
  if(!b.input_valid||BoostSlot(b.cells,b.normal_velocity)<0||b.dimension!=12*2*(b.cells+1)+61*b.cells) return false;
  for(unsigned s=0;s<6;++s) {
    const auto& step=b.steps[s];
    if(step.h!=recurrence::Steps[s]) return false;
    bool all_complete=true,all_passed=true;
    for(const auto& a:step.amplitudes) {
      bool measured=true,passed=true;
      for(const auto& baseline:a.baselines) {
        if(baseline.complete&&(!std::isfinite(baseline.maximum_error)||baseline.maximum_error<0||
           baseline.passed!=(baseline.maximum_error<=recurrence::MatrixTolerance))) return false;
        if(!baseline.complete&&baseline.passed) return false;
        measured=measured&&baseline.complete; passed=passed&&baseline.passed;
      }
      if(!Difference(a.native_matrix)) return false;
      measured=measured&&a.native_matrix.complete; passed=passed&&a.native_matrix.passed;
      for(const auto& matrix:a.branch_matrices) {
        if(!Difference(matrix)) return false;
        measured=measured&&matrix.complete; passed=passed&&matrix.passed;
      }
      for(unsigned i=0;i<11;++i) {
        if(!Difference(a.raw_gains[i])||!Difference(a.weighted_gains[i])) return false;
        measured=measured&&a.raw_gains[i].complete&&a.weighted_gains[i].complete;
        passed=passed&&a.weighted_gains[i].passed;
      }
      if((a.complete&&(!measured||!std::isfinite(a.lift_residual_max)||a.lift_residual_max<0))||
         a.passed!=(a.complete&&passed)) return false;
      all_complete=all_complete&&a.complete; all_passed=all_passed&&a.passed;
    }
    if(step.complete!=all_complete||step.passed!=(all_complete&&all_passed)) return false;
  }
  return true;
}
}
WallScreenSelection SelectWallScreen(const std::array<WallJobSummary,6>& jobs,
                                     const std::array<WallBoostComparison,4>& boosts) {
  WallScreenSelection out;
  for(unsigned s=0;s<6;++s) out.steps[s].h=recurrence::Steps[s];
  std::array<const WallJobSummary*,6> ordered_jobs{};
  std::array<const WallBoostComparison*,4> ordered_boosts{};
  for(const auto& job:jobs) {
    const int slot=JobSlot(job.cells,job.normal_velocity);
    if(slot<0||ordered_jobs[slot]||!ValidateWallJobSummary(job,out.diagnostic)) {
      if(out.diagnostic.empty()) out.diagnostic="Duplicate/missing or invalid six-job tuple";
      return out;
    }
    ordered_jobs[slot]=&job;
  }
  for(const auto& boost:boosts) {
    const int slot=BoostSlot(boost.cells,boost.normal_velocity);
    if(slot<0||ordered_boosts[slot]||!Boost(boost)) {
      out.diagnostic="Duplicate/missing, invalid or contradictory four-boost tuple"; return out;
    }
    ordered_boosts[slot]=&boost;
  }
  // Exact bounded cardinality plus unique valid slots implies full coverage.
  out.input_valid=true; std::array<bool,6> passing{};
  for(unsigned s=0;s<6;++s) {
    auto& step=out.steps[s]; bool passed=true;
    for(unsigned j=0;j<6;++j) {
      step.jobs[j]=ordered_jobs[j]->steps[s].complete&&ordered_jobs[j]->steps[s].passed;
      passed=passed&&step.jobs[j];
    }
    for(unsigned b=0;b<4;++b) {
      step.boosts[b]=ordered_boosts[b]->steps[s].complete&&ordered_boosts[b]->steps[s].passed;
      passed=passed&&step.boosts[b];
    }
    step.passed=passed; passing[s]=passed;
  }
  out.selected_h=SelectWallScreenStep(passing); out.passed=out.selected_h>0;
  if(!out.passed) out.diagnostic="No step has the frozen factor-two sampled margin across all six jobs/four boosts";
  return out;
}
} // namespace tl::qualification::qeph::wall_recurrence
