#include "WallResponseComparison.h"
#include "WallResponseComparisonBounds.h"
#include <algorithm>

namespace tl::qualification::qeph::wall_response {
namespace {
bool DifferenceBetween(const Run& a,const Run& b,ResponseDifference& result) {
  const auto& fields=a.model.dictionary();
  for(unsigned sample=0;sample<SampleCount;++sample) for(unsigned field=0;field<fields.size();++field) {
    if(!fields[field].compare) continue;
    Interval difference;
    if(!comparison_detail::Difference(a.samples[sample].values[field],a.samples[sample].errors[field],
        b.samples[sample].values[field],b.samples[sample].errors[field],fields[field].scale,difference))
      return false;
    result.maximum.lower=std::max(result.maximum.lower,difference.lower);
    if(difference.upper>result.maximum.upper) {
      result.maximum.upper=difference.upper; result.time=a.samples[sample].time; result.field=field;
    }
  }
  return true;
}
bool SameDictionary(const Model& a,const Model& b) {
  if(a.dictionary().size()!=b.dictionary().size()) return false;
  for(unsigned i=0;i<a.dictionary().size();++i) {
    const auto& x=a.dictionary()[i]; const auto& y=b.dictionary()[i];
    if(x.name!=y.name||x.unit!=y.unit||x.scale!=y.scale||x.compare!=y.compare) return false;
  }
  return a.energy()==b.energy()&&a.momentum()==b.momentum()&&a.force_scale()==b.force_scale()&&
      a.fields().initial_position==b.fields().initial_position&&a.fields().connectivity==b.fields().connectivity&&
      a.fields().mass==b.fields().mass&&a.fields().inertia==b.fields().inertia&&
      a.fields().physical==b.fields().physical&&a.fields().added==b.fields().added;
}
}
Comparison Compare(const std::array<Run,3>& runs) {
  Comparison out;
  for(unsigned i=0;i<3;++i) {
    if(!ValidateRun(runs[i],true,out.diagnostic)) return out;
    const auto& config=runs[i].config;
    if(config.refinement!=(1u<<i)||config.cells!=runs[0].config.cells||
       config.selected_h!=runs[0].config.selected_h||config.screen_index_sha!=runs[0].config.screen_index_sha||
       !SameDictionary(runs[0].model,runs[i].model)) {
      out.diagnostic="Comparison needs the same screened fixture at refinements 1,2,4"; return out;
    }
  }
  out.input_valid=true; out.cells=runs[0].config.cells; out.selected_h=runs[0].config.selected_h;
  out.screen_index_sha=runs[0].config.screen_index_sha; out.energy_normalization=runs[0].model.energy();
  if(!DifferenceBetween(runs[0],runs[1],out.coarse_medium)||
     !DifferenceBetween(runs[1],runs[2],out.medium_fine)) {
    out.diagnostic="Unresolved directed response difference"; return out;
  }
  out.response_passed=out.coarse_medium.maximum.upper<=CoarseResponseLimit&&
      out.medium_fine.maximum.upper<=FineResponseLimit&&
      comparison_detail::Refines(out.medium_fine.maximum,out.coarse_medium.maximum,ResponseContractionFloor);
  double energy_floor=0;
  if(!comparison_detail::owning::MultiplyScalar(EnergyContractionFloor,out.energy_normalization,false,&energy_floor)) {
    out.diagnostic="Unresolved fixed energy refinement floor"; return out;
  }
  out.energy_passed=true; out.analytic_passed=true;
  for(unsigned i=0;i<3;++i) {
    const auto& summary=runs[i].summary;
    if(!comparison_detail::Normalize(summary.maximum_absolute_residual,out.energy_normalization,out.residual_ratios[i])) {
      out.energy_passed=false; out.diagnostic="Unresolved directed energy ratio"; return out;
    }
    out.analytic_differences[i]=summary.maximum_analytic_difference;
    Interval depth,force;
    if(!PeakTruth(runs[i].model,summary,depth,force,out.diagnostic)) {
      out.analytic_passed=false; return out;
    }
    out.analytic_differences[i].lower=std::max({out.analytic_differences[i].lower,depth.lower,force.lower});
    out.analytic_differences[i].upper=std::max({out.analytic_differences[i].upper,depth.upper,force.upper});
    out.energy_passed&=out.residual_ratios[i].upper<=MaximumEnergyRatio;
    out.analytic_passed&=out.analytic_differences[i].upper<=MaximumAnalyticError;
    if(i) out.energy_passed&=comparison_detail::Refines(summary.maximum_absolute_residual,
        runs[i-1].summary.maximum_absolute_residual,energy_floor);
  }
  out.passed=out.response_passed&&out.energy_passed&&out.analytic_passed;
  out.diagnostic=out.passed?"Frozen broadside response, analytic and energy checks passed":
      "Frozen broadside response, analytic or energy refinement gate rejected";
  return out;
}
} // namespace tl::qualification::qeph::wall_response
