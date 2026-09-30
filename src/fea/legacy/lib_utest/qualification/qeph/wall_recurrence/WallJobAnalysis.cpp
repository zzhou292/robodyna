#include "WallJobAnalysis.h"
#include <cmath>

namespace tl::qualification::qeph::wall_recurrence {
namespace {
bool NativeComplete(const MovingMatrixProbe& p,unsigned dimension,double velocity,unsigned amplitude) {
  return p.velocity.x==velocity&&p.velocity.y==0&&p.velocity.z==0&&
    p.derivative.amplitude==recurrence::Amplitudes[amplitude]&&p.baseline_complete&&
    p.baseline.size()==dimension&&p.baseline.allFinite()&&p.derivative.complete&&
    p.derivative.completed_columns==dimension&&p.derivative.full.rows()==dimension&&
    p.derivative.full.cols()==dimension&&p.derivative.full.allFinite();
}
}
WallJobAnalysis AnalyzeWallRawJob(const RawJob& raw,const WallJobProgressCallback& callback) {
  WallJobAnalysis out; out.cells=raw.cells; out.normal_velocity=raw.normal_velocity;
  for(unsigned step=0;step<6;++step) {
    out.steps[step].h=recurrence::Steps[step];
    for(unsigned amplitude=0;amplitude<3;++amplitude)
      out.steps[step].amplitudes[amplitude].amplitude=recurrence::Amplitudes[amplitude];
    out.steps[step].contact[1].branch=ContactBranch::Active;
  }
  auto notify=[&](WallJobProgressKind kind,unsigned step=0,unsigned index=0) {
    if(callback) callback(out,{kind,step,index});
  };
  if((raw.cells!=1&&raw.cells!=2)||!FrozenVelocity(raw.normal_velocity)||
     (raw.normal_velocity==0&&std::signbit(raw.normal_velocity))||!raw.model.prepared()||
     raw.model.native().elements!=raw.cells||raw.model.native().nodes!=2*(raw.cells+1)||
     raw.model.native().dictionary.size()!=12*raw.model.native().nodes+61*raw.cells) {
    out.diagnostic="Invalid one-job model/fixture/boost identity"; notify(WallJobProgressKind::Finished); return out;
  }
  for(unsigned step=0;step<6;++step) if(raw.steps[step].h!=recurrence::Steps[step]) {
    out.diagnostic="Changed frozen one-job step grid"; notify(WallJobProgressKind::Finished); return out;
  }
  out.input_valid=true; out.dimension=static_cast<unsigned>(raw.model.native().dictionary.size());
  bool all_complete=true,all_passed=true;
  for(unsigned step=0;step<6;++step) {
    const auto& source=raw.steps[step]; auto& result=out.steps[step];
    bool complete=true,passed=true;
    for(unsigned amplitude=0;amplitude<3;++amplitude) {
      const auto& p=source.native[amplitude]; auto& a=result.amplitudes[amplitude];
      a.attempted=source.native_attempted[amplitude];
      a.input_complete=a.attempted&&NativeComplete(p,out.dimension,raw.normal_velocity,amplitude);
      if(a.input_complete) {
        // Retain the full analysis even when the baseline fails; it is useful
        // failed evidence, but cannot turn that sample into a passing result.
        a.baseline=CheckWallMovingBaseline(raw.model,source.h,p);
        a.branches=AnalyzeWallBranches(raw.model,source.h,p.derivative.full);
        a.complete=a.baseline.complete&&a.branches.complete; a.passed=a.complete&&a.baseline.passed&&a.branches.passed;
        if(!a.passed) a.diagnostic="Full amplitude baseline/branch analysis failed";
      } else a.diagnostic="Native baseline/matrix collection or identity is incomplete";
      if(a.complete) ++out.completed_amplitudes;
      complete=complete&&a.complete; passed=passed&&a.passed;
      notify(WallJobProgressKind::Amplitude,step,amplitude);
    }
    for(unsigned pair=0;pair<2;++pair) {
      if(result.amplitudes[pair].input_complete&&result.amplitudes[pair+1].input_complete)
        result.native_amplitude_comparisons[pair]=CompareWallMatrices(source.native[pair].derivative.full,
                                                                     source.native[pair+1].derivative.full);
      result.derived_amplitude_comparisons[pair]=CompareWallAnalyses(result.amplitudes[pair].branches,
                                                                    result.amplitudes[pair+1].branches);
      complete=complete&&result.native_amplitude_comparisons[pair].complete&&result.derived_amplitude_comparisons[pair].complete;
      passed=passed&&result.native_amplitude_comparisons[pair].passed&&result.derived_amplitude_comparisons[pair].passed;
    }
    for(unsigned branch=0;branch<2;++branch) {
      auto& checked=result.contact[branch]; checked.branch=branch?ContactBranch::Active:ContactBranch::Inactive;
      if(source.contact_attempted[branch]&&result.amplitudes[2].input_complete)
        checked=RecheckWallContact(raw.model,source.h,raw.normal_velocity,checked.branch,
                                  source.native[2].derivative.full,source.contact[branch]);
      else checked.diagnostic="Contact samples or their finest native matrix are missing";
      if(checked.complete) ++out.completed_contacts;
      complete=complete&&checked.complete; passed=passed&&checked.passed;
      notify(WallJobProgressKind::Contact,step,branch);
    }
    result.complete=complete; result.passed=complete&&passed;
    if(!result.passed) result.diagnostic="One-job step failed a full-state, amplitude or recomputed contact gate";
    if(result.complete) ++out.completed_steps;
    all_complete=all_complete&&result.complete; all_passed=all_passed&&result.passed;
    notify(WallJobProgressKind::Step,step);
  }
  // The saved collection flag cannot upgrade missing data, nor can saved
  // numerical flags downgrade recomputed passing checks. No boost decision.
  out.complete=all_complete; out.passed=all_complete&&all_passed;
  if(!out.passed) out.diagnostic="One-job analysis failed; every available component result is retained";
  notify(WallJobProgressKind::Finished); return out;
}
} // namespace tl::qualification::qeph::wall_recurrence
