#include "ResponseSamples.h"
#include "ResponseSampleValidation.h"
#include "ResponseBounds.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace tl::qualification::qeph::response {
namespace {
bool Fail(std::string& error,const char* message) { error=message; return false; }
bool Near(long double a,long double b,long double terms) {
  return std::isfinite(a)&&std::isfinite(b)&&std::abs(a-b)<=256*std::numeric_limits<double>::epsilon()*terms;
}
bool ValidSample(const Run& r,const Sample& s,std::string& error) {
  const double h=H0/r.config.refinement;
  if(s.epoch>4096*r.config.refinement||s.time!=s.epoch*h||s.interval_available!=(s.epoch!=0)||
     s.carried_velocity_time!=(s.epoch?s.time-h/2:0)||s.kick_dt!=(s.epoch?(s.epoch==1?h/2:h):0))
    return Fail(error,"Invalid endpoint/carried-velocity phase association");
  for(unsigned i=0;i<r.fields.size();++i) if(!std::isfinite(s.values[i])) return Fail(error,"Nonfinite source field");
  for(double x:s.carried_kinetic) if(!std::isfinite(x)||x<0) return Fail(error,"Invalid carried kinetic partition");
  for(double x:s.synchronous_kinetic) if(!std::isfinite(x)||x<0) return Fail(error,"Invalid synchronous kinetic partition");
  for(double x:s.source_work) if(!std::isfinite(x)) return Fail(error,"Invalid source work");
  if(!std::isfinite(s.external_work)||!std::isfinite(s.residual)) return Fail(error,"Invalid external-work/residual diagnostic");
  const long double total=static_cast<long double>(s.synchronous_kinetic[0])+s.synchronous_kinetic[1]+
      s.source_work[0]+s.source_work[1]+s.source_work[2]-s.external_work;
  long double terms=std::abs(static_cast<long double>(s.external_work));
  for(double x:s.synchronous_kinetic) terms+=std::abs(static_cast<long double>(x));
  for(double x:s.source_work) terms+=std::abs(static_cast<long double>(x));
  if(!Near(s.residual,total,terms)) return Fail(error,"Energy residual does not use the declared distinct partitions");
  if(std::abs(s.residual)>r.maximum_abs_residual) return Fail(error,"Summary omits a retained energy residual");
  return ValidateSampleFields(r,s,error);
}
Difference DifferenceBetween(const Run& a,const Run& b) {
  Difference d;
  for(unsigned j=0;j<SampleCount;++j) for(unsigned i=0;i<a.fields.size();++i) if(a.fields[i].compare) {
    bounds::Interval value;
    if(!bounds::NormalizedDifference(a.samples[j].values[i],b.samples[j].values[i],a.fields[i].scale,value))
      return {std::numeric_limits<double>::max(),a.samples[j].time,i,0};
    d.lower=std::max(d.lower,value.lower);
    if(value.upper>d.maximum) { d.maximum=value.upper; d.time=a.samples[j].time; d.field=i; }
  }
  return d;
}
}
bool ValidateRun(const Run& r,bool require_complete,std::string& error) {
  if(!ValidConfig(r.config)||r.model.cells!=r.config.cells||r.model.nodes!=2*(r.config.cells+1)) return Fail(error,"Invalid frozen fixture identity");
  Model model;
  if(!BuildModel(r.config.cells,model,error)) return false;
  if(r.model.initial_position!=model.initial_position||r.model.connectivity!=model.connectivity||r.model.mass!=model.mass||
      r.model.inertia!=model.inertia||r.model.physical!=model.physical||r.model.added!=model.added)
    return Fail(error,"Changed immutable source-scale geometry or native mass/inertia");
  const auto expected=Dictionary(r.model);
  if(r.fields.size()!=expected.size()||r.fields.size()>MaxFields) return Fail(error,"Invalid dictionary capacity");
  for(unsigned i=0;i<expected.size();++i) if(r.fields[i].name!=expected[i].name||r.fields[i].unit!=expected[i].unit||
      r.fields[i].scale!=expected[i].scale||r.fields[i].compare!=expected[i].compare) return Fail(error,"Changed frozen response dictionary");
  if(r.samples.size()>SampleCount||r.accepted_steps>4096*r.config.refinement||r.attempted_steps<r.accepted_steps||
     r.attempted_steps>r.accepted_steps+1) return Fail(error,"Invalid interval/sample count");
  if(!std::isfinite(r.elapsed_seconds)||r.elapsed_seconds<0||!std::isfinite(r.maximum_abs_residual)||r.maximum_abs_residual<0||
     !std::isfinite(r.external_work_at_pulse)||!std::isfinite(r.residual_time)||r.residual_time<0||r.residual_time>Horizon)
    return Fail(error,"Invalid finite run summary");
  if((require_complete||r.completed)&&(!r.completed||r.accepted_steps!=4096*r.config.refinement||r.samples.size()!=SampleCount||
      !r.failure.empty()||r.owner_id==0)) return Fail(error,"Run is incomplete");
  if(r.completed&&!InsideLimits(r.observed)) return Fail(error,"Completed run exceeded frozen small-response domain");
  for(unsigned i=0;i<r.ledger_maxima.size();++i) if(!std::isfinite(r.ledger_maxima[i])||(i<6&&r.ledger_maxima[i]<0)) return Fail(error,"Invalid ledger summary");
  const double bounds[]{r.observed.displacement_over_side,r.observed.rotation_angle,r.observed.strain,r.observed.thickness_curvature,
    r.observed.minimum_area_ratio,r.observed.maximum_area_ratio,r.observed.minimum_thickness_ratio,r.observed.maximum_thickness_ratio};
  for(double value:bounds) if(!std::isfinite(value)||value<0) return Fail(error,"Invalid small-response summary");
  for(double value:r.external_linear_impulse) if(!std::isfinite(value)) return Fail(error,"Invalid external impulse");
  for(double value:r.external_angular_impulse) if(!std::isfinite(value)) return Fail(error,"Invalid external angular impulse");
  if(r.completed) for(unsigned i=0;i<5;++i) if(r.ledger_maxima[i]>1) return Fail(error,"Completed run failed a per-step ledger");
  for(unsigned j=0;j<r.samples.size();++j) {
    if(r.samples[j].epoch!=16*j*r.config.refinement) return Fail(error,"Skipped, duplicate or noncommon endpoint sample");
    if(!ValidSample(r,r.samples[j],error)) return false;
  }
  if(!r.samples.empty()) {
    if(r.last_accepted.epoch!=r.accepted_steps||!ValidSample(r,r.last_accepted,error)) return Fail(error,"Last accepted sample mismatch");
    if(r.completed&&r.external_work_at_pulse!=r.samples[64].external_work) return Fail(error,"Pulse work not bound to the common endpoint");
    if(r.completed&&!SameSample(r.last_accepted,r.samples.back())) return Fail(error,"Completed final sample differs from last accepted endpoint");
    Extremum maximum;
    for(const auto& sample:r.samples) { const auto current=ResponseMaximum(r.fields,r.samples[0],sample); if(current.value>maximum.value) maximum=current; }
    if(maximum.value!=r.maximum_response.value||maximum.field!=r.maximum_response.field||maximum.time!=r.maximum_response.time||maximum.lower!=r.maximum_response.lower)
      return Fail(error,"Response extremum does not match retained samples");
  }
  error.clear(); return true;
}
Comparison Compare(const std::array<Run,3>& runs) {
  Comparison result;
  for(unsigned i=0;i<3;++i) {
    if(!ValidateRun(runs[i],true,result.diagnostic)) return result;
    if(runs[i].config.refinement!=(1u<<i)||runs[i].config.cells!=runs[0].config.cells) {
      result.diagnostic="Comparison requires one fixture at h, h/2, h/4 in that order"; return result;
    }
    result.energy_normalization=std::max(result.energy_normalization,std::abs(runs[i].external_work_at_pulse));
  }
  result.coarse_medium=DifferenceBetween(runs[0],runs[1]); result.medium_fine=DifferenceBetween(runs[1],runs[2]);
  const double energy=ExperimentEnergy(runs[0].model);
  bool pass=result.energy_normalization>=bounds::Product(1e-6,energy,true)&&result.coarse_medium.maximum<=.02&&result.medium_fine.maximum<=.015&&
      bounds::Refines(result.medium_fine.maximum,result.coarse_medium.lower,1e-8);
  for(unsigned i=0;i<3;++i) {
    result.residual_ratios[i]=result.energy_normalization?bounds::RatioUpper(runs[i].maximum_abs_residual,result.energy_normalization):0;
    pass&=runs[i].maximum_response.lower>=1e-3&&result.residual_ratios[i]<=.02;
    if(i) pass&=bounds::Refines(runs[i].maximum_abs_residual,runs[i-1].maximum_abs_residual,bounds::Product(1e-10,energy,false));
  }
  result.passed=pass; result.diagnostic=pass?"Frozen free-response/refinement checks passed":"Frozen response/energy/refinement gate rejected";
  return result;
}
} // namespace tl::qualification::qeph::response
