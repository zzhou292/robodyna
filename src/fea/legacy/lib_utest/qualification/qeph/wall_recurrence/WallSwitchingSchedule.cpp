#include "WallSwitchingSchedule.h"
#include <algorithm>
#include <cmath>
#include <utility>

namespace tl::qualification::qeph::wall_recurrence {
bool BuildWallSwitchingWindows(unsigned total,unsigned entry,unsigned exit,
                              std::array<WallSwitchingWindow,9>& out,std::string& error) {
  if(total<2||total>32768||entry<1||entry>=exit||exit>total) {
    error="Invalid ordinary-step event indices"; return false;
  }
  std::array<WallSwitchingWindow,9> result{}; unsigned index=0;
  for(int de:{-1,0,1}) for(int dx:{-1,0,1}) {
    const auto first=static_cast<long long>(entry)+de,last=static_cast<long long>(exit)+dx;
    if(first<1||last>total||first>=last) {
      error="A frozen +/-one event window is invalid or outside the horizon"; return false;
    }
    auto& w=result[index++]; w.entry_shift=de; w.exit_shift=dx;
    w.entry_base_epoch=static_cast<unsigned>(first); w.exit_base_epoch=static_cast<unsigned>(last);
    w.inactive_before=w.entry_base_epoch-1; w.active=w.exit_base_epoch-w.entry_base_epoch;
    w.inactive_after=total-w.exit_base_epoch;
  }
  out=result; error.clear(); return true;
}
WallSwitchingSchedule AnalyzeWallSwitchingSchedule(const WallRecurrenceModel& model,double h) {
  WallSwitchingSchedule result; result.h=h;
  if(!model.prepared()||!FrozenStep(h)||!(InitialGap>4*recurrence::H0*ImpactSpeed)) {
    result.diagnostic="Unprepared/unfrozen scalar schedule or nonzero startup contact"; return result;
  }
  result.frequency=model.maximum_frequency();
  const double total=ScreenHorizon/h;
  if(!std::isfinite(total)||total<2||total>32768||total!=std::floor(total)||
     !std::isfinite(result.frequency)||result.frequency<=0) {
    result.diagnostic="Invalid scalar schedule frequency/count"; return result;
  }
  result.total_steps=static_cast<unsigned>(total); result.ordinary_steps=result.total_steps-1;
  result.scalar.reserve(result.total_steps+1);
  result.scalar.push_back({-InitialGap,ImpactSpeed,false});
  // Explicit startup: h/2 * zero force, followed by a full h drift.
  result.scalar.push_back({-InitialGap+h*ImpactSpeed,ImpactSpeed,false});
  bool entered=false,exited=false;
  for(unsigned epoch=1;epoch<result.total_steps;++epoch) {
    auto& base=result.scalar.back(); base.active=WallScalarContactActive(base.position);
    if(base.active&&!entered) { entered=true; result.entry_base_epoch=epoch; }
    if(!base.active&&entered&&!exited) { exited=true; result.exit_base_epoch=epoch; }
    if(base.active&&exited) { result.diagnostic="Scalar schedule has a second contact window"; return result; }
    const double depth=std::max(base.position,0.);
    result.maximum_depth=std::max(result.maximum_depth,depth);
    const double velocity=base.velocity-h*result.frequency*result.frequency*depth;
    const double position=base.position+h*velocity;
    if(!std::isfinite(velocity)||!std::isfinite(position)) { result.diagnostic="Nonfinite scalar recurrence"; return result; }
    result.scalar.push_back({position,velocity,WallScalarContactActive(position)});
  }
  const auto& endpoint=result.scalar.back();
  result.complete=true;
  result.maximum_depth=std::max(result.maximum_depth,std::max(endpoint.position,0.));
  if(entered&&!exited&&!endpoint.active) { exited=true; result.exit_base_epoch=result.total_steps; }
  if(!entered||!exited||!BuildWallSwitchingWindows(result.total_steps,result.entry_base_epoch,
                                                 result.exit_base_epoch,result.windows,result.diagnostic)) {
    if(result.diagnostic.empty()) result.diagnostic="Scalar schedule lacks a complete entry/exit window";
    return result;
  }
  result.passed=true; return result;
}
bool BuildWallSwitchingSchedule(const WallRecurrenceModel& model,double h,
                               WallSwitchingSchedule& out,std::string& error) {
  auto result=AnalyzeWallSwitchingSchedule(model,h);
  if(!result.passed) { error=result.diagnostic; return false; }
  out=std::move(result); error.clear(); return true;
}
} // namespace tl::qualification::qeph::wall_recurrence
