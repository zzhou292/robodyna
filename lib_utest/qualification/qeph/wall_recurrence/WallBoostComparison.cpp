#include "WallBoostComparison.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace tl::qualification::qeph::wall_recurrence {
namespace {
bool SameWindow(const WallSwitchingWindow& a,const WallSwitchingWindow& b) {
  return a.entry_shift==b.entry_shift&&a.exit_shift==b.exit_shift&&
    a.entry_base_epoch==b.entry_base_epoch&&a.exit_base_epoch==b.exit_base_epoch&&
    a.inactive_before==b.inactive_before&&a.active==b.active&&a.inactive_after==b.inactive_after;
}
bool SameContext(const WallStepSummary& a,const WallStepSummary& b) {
  if(a.h!=b.h||a.total_steps!=b.total_steps||a.ordinary_steps!=b.ordinary_steps||
     a.diagonal.size()!=b.diagonal.size()||a.diagonal!=b.diagonal) return false;
  for(unsigned i=0;i<9;++i) if(!SameWindow(a.windows[i],b.windows[i])) return false;
  return true;
}
bool Context(const WallRecurrenceModel& model,unsigned step,WallStepSummary& out,std::string& error) {
  WallStateMetric metric;
  if(!BuildWallStateMetric(model,metric,error)) return false;
  const auto schedule=AnalyzeWallSwitchingSchedule(model,recurrence::Steps[step]);
  if(!schedule.passed) { error=schedule.diagnostic; return false; }
  out.h=recurrence::Steps[step]; out.diagonal=std::move(metric.diagonal);
  out.total_steps=schedule.total_steps; out.ordinary_steps=schedule.ordinary_steps; out.windows=schedule.windows;
  return true;
}
bool Native(const MovingMatrixProbe& p,unsigned dimension,double velocity,unsigned amplitude) {
  return p.velocity.x==velocity&&p.velocity.y==0&&p.velocity.z==0&&p.baseline_complete&&
    p.baseline.size()==dimension&&p.baseline.allFinite()&&p.derivative.complete&&
    p.derivative.amplitude==recurrence::Amplitudes[amplitude]&&p.derivative.completed_columns==dimension&&
    p.derivative.full.rows()==dimension&&p.derivative.full.cols()==dimension&&p.derivative.full.allFinite();
}
bool Bound(const RawJob& raw,const WallJobSummary& summary,std::string& error) {
  if(!ValidateWallJobSummary(summary,error)) return false;
  if(raw.cells!=summary.cells||raw.normal_velocity!=summary.normal_velocity||!raw.model.prepared()||
     raw.model.native().elements!=raw.cells||raw.model.native().dictionary.size()!=summary.dimension) {
    error="Raw/summary named model or boost differs"; return false;
  }
  for(unsigned s=0;s<6;++s) {
    WallStepSummary expected;
    if(raw.steps[s].h!=recurrence::Steps[s]||!Context(raw.model,s,expected,error)||!SameContext(expected,summary.steps[s])) {
      error="Raw/summary frozen step, metric or windows differ"; return false;
    }
    for(unsigned a=0;a<3;++a) {
      const auto& p=raw.steps[s].native[a];
      const bool empty=p.derivative.full.rows()==0&&p.derivative.full.cols()==0;
      if((!empty&&(p.derivative.full.rows()!=summary.dimension||p.derivative.full.cols()!=summary.dimension))||
         (p.baseline.size()!=0&&p.baseline.size()!=summary.dimension)) {
        error="Partial native operands exceed the exact full-state extent"; return false;
      }
      if(summary.steps[s].amplitudes[a].complete&&
         (!raw.steps[s].native_attempted[a]||!Native(p,summary.dimension,summary.normal_velocity,a))) {
        error="Complete summary lacks its complete native observation"; return false;
      }
    }
  }
  return true;
}
WallBoostAmplitudeComparison Amplitude(const WallRecurrenceModel& model,double h,unsigned dimension,
                                      unsigned amplitude,double velocity,const MovingMatrixProbe& zero,
                                      const MovingMatrixProbe& boost,bool zero_present,bool boost_present,
                                      const WallAmplitudeSummary& a,const WallAmplitudeSummary& b) {
  WallBoostAmplitudeComparison out;
  if(!zero_present||!boost_present||!Native(zero,dimension,0,amplitude)||!Native(boost,dimension,velocity,amplitude)) {
    out.diagnostic="Missing or malformed native boost operands"; return out;
  }
  out.baselines[0]=CheckWallMovingBaseline(model,h,zero);
  out.baselines[1]=CheckWallMovingBaseline(model,h,boost);
  bool complete=out.baselines[0].complete&&out.baselines[1].complete;
  bool passed=out.baselines[0].passed&&out.baselines[1].passed;
  if(complete) {
    const Eigen::VectorXd residual=(boost.baseline-zero.baseline)-
      (out.baselines[1].expected-out.baselines[0].expected);
    if(residual.allFinite()) {
      Eigen::Index index=0; out.lift_residual_max=residual.cwiseAbs().maxCoeff(&index);
      out.lift_controlling_coordinate=static_cast<unsigned>(index);
    } else { out.lift_residual_max=std::numeric_limits<double>::infinity(); complete=false; }
  }
  out.native_matrix=CompareWallMatrices(zero.derivative.full,boost.derivative.full);
  complete=complete&&out.native_matrix.complete; passed=passed&&out.native_matrix.passed;
  for(unsigned branch=0;branch<2;++branch) {
    Eigen::MatrixXd first,second; std::string error;
    const auto selected=branch?ContactBranch::Active:ContactBranch::Inactive;
    if(BuildContactBranch(model,h,selected,zero.derivative.full,first,error)&&
       BuildContactBranch(model,h,selected,boost.derivative.full,second,error))
      out.branch_matrices[branch]=CompareWallMatrices(first,second);
    else out.diagnostic=error;
    complete=complete&&out.branch_matrices[branch].complete; passed=passed&&out.branch_matrices[branch].passed;
  }
  for(unsigned sequence=0;sequence<11;++sequence) {
    const auto& first=a.gains[sequence]; const auto& second=b.gains[sequence];
    out.raw_gains[sequence]=CompareWallGains(first.raw,second.raw);
    out.weighted_gains[sequence]=CompareWallGains(first.weighted,second.weighted);
    complete=complete&&a.complete&&b.complete&&first.complete&&second.complete&&
      out.raw_gains[sequence].complete&&out.weighted_gains[sequence].complete;
    passed=passed&&out.weighted_gains[sequence].passed;
  }
  out.complete=complete; out.passed=complete&&passed;
  if(!out.passed&&out.diagnostic.empty()) out.diagnostic="Fresh baseline/matrix/weighted-gain boost comparison failed";
  return out;
}
}
bool ValidateWallJobSummary(const WallJobSummary& job,std::string& error) {
  if((job.cells!=1&&job.cells!=2)||job.dimension!=12*2*(job.cells+1)+61*job.cells||
     !FrozenVelocity(job.normal_velocity)||(job.normal_velocity==0&&std::signbit(job.normal_velocity))) {
    error="Invalid compact job tuple/dimension"; return false;
  }
  for(unsigned s=0;s<6;++s) {
    const auto& step=job.steps[s];
    if(step.h!=recurrence::Steps[s]||step.diagonal.size()!=job.dimension||!ValidWallStateDiagonal(step.diagonal)||
       step.total_steps!=static_cast<unsigned>(ScreenHorizon/step.h)||step.ordinary_steps+1!=step.total_steps||
       (step.passed&&!step.complete)) { error="Invalid compact step/context/completion"; return false; }
    for(unsigned i=0;i<9;++i) {
      const auto& w=step.windows[i];
      if(w.entry_shift!=static_cast<int>(i/3)-1||w.exit_shift!=static_cast<int>(i%3)-1||
         w.entry_base_epoch<1||w.entry_base_epoch>=w.exit_base_epoch||w.exit_base_epoch>step.total_steps||
         w.inactive_before!=w.entry_base_epoch-1||w.active!=w.exit_base_epoch-w.entry_base_epoch||
         w.inactive_after!=step.total_steps-w.exit_base_epoch) {
        error="Invalid compact chronological window"; return false;
      }
    }
    for(const auto& amplitude:step.amplitudes) {
      if((amplitude.passed&&!amplitude.complete)||(step.complete&&!amplitude.complete)||(step.passed&&!amplitude.passed)) {
        error="Contradictory compact amplitude/step completion"; return false;
      }
      for(const auto& gain:amplitude.gains) if((amplitude.complete&&!gain.complete)||
        (gain.complete&&(!std::isfinite(gain.raw)||!std::isfinite(gain.weighted)||gain.raw<0||gain.weighted<0))||
        (amplitude.passed&&gain.weighted>recurrence::MaximumGramGain)) {
        error="Missing or nonfinite complete gain measurement"; return false;
      }
    }
  }
  error.clear(); return true;
}
bool CompactWallJobAnalysis(const RawJob& raw,const WallJobAnalysis& analysis,WallJobSummary& out,std::string& error) {
  if(!analysis.input_valid||!raw.model.prepared()||analysis.cells!=raw.cells||
     analysis.normal_velocity!=raw.normal_velocity||analysis.dimension!=raw.model.native().dictionary.size()) {
    error="Fresh/validated analysis identity differs from raw model"; return false;
  }
  WallJobSummary result; result.cells=analysis.cells; result.dimension=analysis.dimension; result.normal_velocity=analysis.normal_velocity;
  for(unsigned s=0;s<6;++s) {
    auto& step=result.steps[s]; const auto& source=analysis.steps[s];
    if(source.h!=recurrence::Steps[s]) { error="Changed validated analysis step"; return false; }
    if(!Context(raw.model,s,step,error)) return false;
    step.complete=source.complete; step.passed=source.passed;
    for(unsigned a=0;a<3;++a) {
      const auto& input=source.amplitudes[a]; auto& amplitude=step.amplitudes[a];
      if(input.amplitude!=recurrence::Amplitudes[a]) { error="Changed compact amplitude identity"; return false; }
      amplitude.complete=input.complete; amplitude.passed=input.passed;
      if(input.complete) {
        WallStepSummary actual; actual.h=input.branches.h; actual.diagonal=input.branches.metric.diagonal;
        actual.total_steps=input.branches.schedule.total_steps; actual.ordinary_steps=input.branches.schedule.ordinary_steps;
        for(unsigned i=0;i<9;++i) actual.windows[i]=input.branches.events[i].window;
        if(!input.branches.complete||!SameContext(step,actual)) { error="Complete analysis changed metric/windows"; return false; }
      }
      for(unsigned i=0;i<11;++i) {
        const auto& sequence=i<2?input.branches.branches[i].continuous:input.branches.events[i-2].sequence;
        amplitude.gains[i]={sequence.raw.mean_gain,sequence.weighted.mean_gain,sequence.complete};
      }
    }
  }
  if(!Bound(raw,result,error)) return false;
  out=std::move(result); error.clear(); return true;
}
bool PrepareWallZeroBoostReference(const RawJob& raw,const WallJobSummary& summary,WallZeroBoostReference& out,std::string& error) {
  if(out.prepared()) { error="Zero-boost reference is already prepared"; return false; }
  if(summary.normal_velocity!=0) { error="Reference must be the named zero-boost job"; return false; }
  if(!Bound(raw,summary,error)) return false;
  WallZeroBoostReference result; result.model_=raw.model; result.summary_=summary;
  for(unsigned s=0;s<6;++s) { result.native_[s]=raw.steps[s].native; result.attempted_[s]=raw.steps[s].native_attempted; }
  result.prepared_=true; out=std::move(result); error.clear(); return true;
}
struct WallBoostAccess {
  static WallBoostComparison Compare(const WallZeroBoostReference& zero,const RawJob& raw,const WallJobSummary& summary) {
    WallBoostComparison out; out.cells=summary.cells; out.dimension=summary.dimension; out.normal_velocity=summary.normal_velocity;
    for(unsigned s=0;s<6;++s) out.steps[s].h=recurrence::Steps[s];
    if(!zero.prepared_||(summary.normal_velocity!=-8&&summary.normal_velocity!=8)||
       zero.summary_.cells!=summary.cells||zero.summary_.dimension!=summary.dimension||!Bound(raw,summary,out.diagnostic)) {
      if(out.diagnostic.empty()) out.diagnostic="Invalid zero/boost tuple or unprepared reference";
      return out;
    }
    for(unsigned s=0;s<6;++s) if(!SameContext(zero.summary_.steps[s],summary.steps[s])) {
      out.diagnostic="Zero/boost metric or chronological context differs"; return out;
    }
    out.input_valid=true;
    for(unsigned s=0;s<6;++s) {
      auto& step=out.steps[s]; bool complete=true,passed=true;
      for(unsigned a=0;a<3;++a) {
        step.amplitudes[a]=Amplitude(zero.model_,step.h,summary.dimension,a,summary.normal_velocity,
          zero.native_[s][a],raw.steps[s].native[a],zero.attempted_[s][a],raw.steps[s].native_attempted[a],
          zero.summary_.steps[s].amplitudes[a],summary.steps[s].amplitudes[a]);
        complete=complete&&step.amplitudes[a].complete; passed=passed&&step.amplitudes[a].passed;
      }
      step.complete=complete; step.passed=complete&&passed;
    }
    return out;
  }
};
WallBoostComparison CompareWallBoost(const WallZeroBoostReference& zero,const RawJob& raw,const WallJobSummary& summary) {
  return WallBoostAccess::Compare(zero,raw,summary);
}
} // namespace tl::qualification::qeph::wall_recurrence
