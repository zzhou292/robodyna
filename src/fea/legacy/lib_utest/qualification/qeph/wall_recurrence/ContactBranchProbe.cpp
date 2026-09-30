#include "ContactBranchProbe.h"
#include "ContactDerivativeChecks.h"
#include <algorithm>
#include <cmath>
#include <utility>

namespace tl::qualification::qeph::wall_recurrence {
namespace r=recurrence;
namespace {
bool CorrectCone(const ContactMapSample& sample,ContactBranch branch,unsigned count) {
  if(sample.nodes.size()!=count) return false;
  for(const auto& node:sample.nodes)
    if(!node.valid||node.touching_or_penetrating!=(branch==ContactBranch::Active)) return false;
  return true;
}
}
ContactBranchProbe ProbeContactBranch(const WallRecurrenceModel& model,double h,double velocity,
                                      ContactBranch branch,const Eigen::MatrixXd& shell) {
  ContactBranchProbe result; result.h=h; result.normal_velocity=velocity; result.branch=branch;
  if(!BuildContactBranch(model,h,branch,shell,result.full,result.diagnostic)) return result;
  std::vector<ContactDirection> directions;
  if(!SignConeDirections(model,branch,directions,result.diagnostic)) return result;
  const auto n=model.native().dictionary.size();
  if(!EvaluateContactNativeMap(model,h,velocity,Eigen::VectorXd::Zero(n),result.baseline,result.diagnostic)) return result;
  result.baseline_complete=true;
  // Collect every actual sample before any numerical comparison. Individual
  // native/contact failures retain the earlier complete fields and directions.
  for(auto& direction:directions) {
    ContactDirectionProbe probe; probe.direction=std::move(direction);
    for(unsigned amplitude=0;amplitude<r::Amplitudes.size();++amplitude) {
      const Eigen::VectorXd input=r::Amplitudes[amplitude]*probe.direction.value;
      if(!EvaluateContactNativeMap(model,h,velocity,input,probe.samples[amplitude],probe.diagnostic)||
         !CorrectCone(probe.samples[amplitude],branch,model.native().nodes)) {
        if(probe.diagnostic.empty()) probe.diagnostic="Physical point law disagrees with declared sign cone";
        result.diagnostic=probe.diagnostic; result.directions.push_back(std::move(probe)); return result;
      }
      ++probe.completed_samples;
    }
    result.directions.push_back(std::move(probe));
  }
  result.passed=true;
  for(auto& probe:result.directions) {
    for(unsigned level=0;level<2;++level) {
      auto& quotient=probe.quotients[level];
      quotient=derivative_detail::OneSidedQuotient(result.baseline.state,probe.samples[level].state,
                                                 probe.samples[level+1].state,r::Amplitudes[level]);
      if(!quotient.allFinite()||!derivative_detail::CompareDirection(result.full,probe.direction.value,quotient,
                       probe.residual_upper[level],probe.budget[level])) {
        probe.diagnostic="Nonfinite one-sided quotient/comparison";
        result.diagnostic=probe.diagnostic; result.passed=false; return result;
      }
    }
    probe.complete=true;
    probe.passed=probe.residual_upper[0]<=probe.budget[0]&&probe.residual_upper[1]<=probe.budget[1];
    if(!probe.passed) { probe.diagnostic="Frozen sign-cone derivative budget failed"; result.passed=false; }
  }
  result.complete=true;
  if(!result.passed) result.diagnostic="One or more physical contact/native directions failed";
  return result;
}
} // namespace tl::qualification::qeph::wall_recurrence
