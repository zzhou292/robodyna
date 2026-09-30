#pragma once
#include "WallScreenSelection.h"
#include "WallJobAnalysisTestFixture.h"

namespace tl::qualification::qeph::wall_recurrence::boost_test {
// Synthetic operator/measurement fixtures for pure comparison and selector
// arithmetic. These identity matrices are not native shell tangents, and the
// declared summary verdicts do not constitute authenticated CW1 evidence.
inline RawJob Raw(unsigned cells,double velocity) {
  auto result=job_test::Job(cells,velocity);
  for(unsigned s=0;s<6;++s) for(unsigned a=0;a<3;++a) job_test::Native(result,s,a);
  return result;
}
inline WallJobSummary Summary(const RawJob& raw) {
  WallJobSummary result; result.cells=raw.cells; result.normal_velocity=raw.normal_velocity;
  result.dimension=static_cast<unsigned>(raw.model.native().dictionary.size());
  WallStateMetric metric; std::string error;
  if(!BuildWallStateMetric(raw.model,metric,error)) throw std::runtime_error(error);
  for(unsigned s=0;s<6;++s) {
    auto& step=result.steps[s]; const auto schedule=AnalyzeWallSwitchingSchedule(raw.model,recurrence::Steps[s]);
    if(!schedule.passed) throw std::runtime_error(schedule.diagnostic);
    step.h=recurrence::Steps[s]; step.diagonal=metric.diagonal;
    step.total_steps=schedule.total_steps; step.ordinary_steps=schedule.ordinary_steps; step.windows=schedule.windows;
    step.complete=true; step.passed=true;
    for(auto& a:step.amplitudes) {
      a.complete=true; a.passed=true;
      for(auto& gain:a.gains) gain={1,1,true};
    }
  }
  return result;
}
struct Set {
  std::array<WallJobSummary,6> jobs;
  std::array<WallBoostComparison,4> boosts;
};
inline Set CompleteSet() {
  Set result; std::string error;
  for(unsigned cells=1;cells<=2;++cells) {
    const auto zero=Raw(cells,0); auto summary=Summary(zero); WallZeroBoostReference reference;
    if(!PrepareWallZeroBoostReference(zero,summary,reference,error)) throw std::runtime_error(error);
    result.jobs[3*(cells-1)]=summary;
    for(unsigned sign=0;sign<2;++sign) {
      const auto boosted=Raw(cells,sign?8:-8); summary=Summary(boosted);
      result.jobs[3*(cells-1)+sign+1]=summary;
      result.boosts[2*(cells-1)+sign]=CompareWallBoost(reference,boosted,summary);
      if(!result.boosts[2*(cells-1)+sign].input_valid) throw std::runtime_error("Synthetic comparison tuple failed");
    }
  }
  return result;
}
} // namespace tl::qualification::qeph::wall_recurrence::boost_test
