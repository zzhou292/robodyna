#include "RecurrenceAudit.h"
#include <algorithm>
#include <cmath>

namespace tl::qualification::qeph::recurrence {
Audit CollectProbes() {
  Audit result; std::array<Model,2> models;
  // Finish ALL raw finite-difference work before checking any decision budget.
  for(unsigned c=0;c<2;++c) {
    auto& fixture=result.cases[c]; fixture.elements=c+1;
    if(!BuildModel(c+1,models[c],fixture.diagnostic)) continue;
    fixture.nodes=models[c].nodes; fixture.dictionary=models[c].dictionary; fixture.complete=true;
    for(unsigned s=0;s<Steps.size();++s) {
      auto& step=fixture.steps[s]; step.h=Steps[s];
      for(unsigned e=0;e<Amplitudes.size();++e) step.probes[e]=Differentiate(models[c],step.h,Amplitudes[e]);
    }
  }
  return result;
}
void Decide(Audit& result) {
  std::array<Model,2> models;
  result.audit_passed=false; result.selected_h=0;
  for(unsigned c=0;c<2;++c) {
    auto& fixture=result.cases[c]; if(!fixture.complete) continue;
    if(!BuildModel(fixture.elements,models[c],fixture.diagnostic)) { fixture.complete=false; continue; }
    for(auto& step:fixture.steps) {
      step.passed=true;
      for(unsigned e=0;e<Amplitudes.size();++e) {
        step.analysis[e]=Analyze(models[c],step.h,step.probes[e]);
        step.passed&=step.analysis[e].complete&&step.analysis[e].passed;
      }
      for(unsigned e=0;e<2;++e) {
        if(!step.probes[e].complete||!step.probes[e+1].complete) { step.passed=false; continue; }
        const auto& fine=step.probes[e+1].full;
        step.matrix_differences[e]=(step.probes[e].full-fine).cwiseAbs().maxCoeff();
        const auto a=step.analysis[e].gram_gain,b=step.analysis[e+1].gram_gain;
        step.gram_relative_differences[e]=std::abs(a-b)/std::max({std::abs(a),std::abs(b),1e-10});
        step.passed&=step.matrix_differences[e]<=MatrixTolerance*std::max(1.,fine.cwiseAbs().maxCoeff());
        step.passed&=std::abs(a-b)<=.005*std::max(std::abs(a),std::abs(b))+1e-10;
      }
      if(!step.passed) step.diagnostic="Frozen native matrix admission rejected or unresolved; raw probes retained";
    }
  }
  auto through=[&](unsigned last) {
    for(const auto& c:result.cases) {
      if(!c.complete) return false;
      for(unsigned s=0;s<=last;++s) if(!c.steps[s].passed) return false;
    }
    return true;
  };
  if(through(4)) result.selected_h=H0;
  else if(through(3)) result.selected_h=H0/2;
  result.audit_passed=result.selected_h>0;
}
Audit RunAudit() { auto result=CollectProbes(); Decide(result); return result; }
} // namespace tl::qualification::qeph::recurrence
