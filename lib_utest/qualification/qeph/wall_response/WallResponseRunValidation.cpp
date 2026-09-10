#include "WallResponseComparison.h"
#include "WallResponseComparisonBounds.h"
#include <algorithm>
#include <cmath>
#include <iterator>
#include <tuple>

namespace tl::qualification::qeph::wall_response {
namespace {
bool Fail(std::string& error,const char* text) { error=text; return false; }
bool Same(Interval a,Interval b) { return a.lower==b.lower&&a.upper==b.upper; }
bool Same(contact::Q4CertifiedIntegral a,contact::Q4CertifiedIntegral b) {
  return a.value==b.value&&a.lower==b.lower&&a.upper==b.upper&&a.error==b.error;
}
bool Same(contact::Vec3 a,contact::Vec3 b) { return a.x==b.x&&a.y==b.y&&a.z==b.z; }
bool Same(const contact::NodalWallPointResult& a,const contact::NodalWallPointResult& b) {
  return Same(a.force,b.force)&&Same(a.potential,b.potential)&&Same(a.stiffness,b.stiffness)&&
      Same(a.force_world,b.force_world)&&Same(a.wall_point,b.wall_point)&&Same(a.wall_reaction,b.wall_reaction)&&
      Same(a.wall_moment,b.wall_moment)&&a.surface_power==b.surface_power&&
      a.local_velocity_first_timestep==b.local_velocity_first_timestep&&
      std::tie(a.base_epoch,a.attempt,a.node,a.fixed,a.touching_or_penetrating,a.valid)==
      std::tie(b.base_epoch,b.attempt,b.node,b.fixed,b.touching_or_penetrating,b.valid)&&
      std::tie(a.row.count,a.row.base_epoch,a.row.attempt,a.row.valid)==
      std::tie(b.row.count,b.row.base_epoch,b.row.attempt,b.row.valid)&&
      std::equal(std::begin(a.row.nodes),std::end(a.row.nodes),std::begin(b.row.nodes))&&
      std::equal(std::begin(a.row.stiffness),std::end(a.row.stiffness),std::begin(b.row.stiffness))&&
      std::equal(std::begin(a.row.damping),std::end(a.row.damping),std::begin(b.row.damping));
}
bool SameEndpoint(const Sample& a,const Sample& b) {
  if(std::tie(a.epoch,a.time,a.carried_velocity_time,a.kick_dt,a.wall_impulse,a.wall_impulse_error,
      a.synchronous_wall_impulse,a.synchronous_wall_impulse_error,
      a.minimum_gap,a.maximum_gap,a.minimum_velocity,a.maximum_velocity,a.relative_displacement,a.rotation_angle,
      a.strain,a.thickness_curvature,a.minimum_area_ratio,a.maximum_area_ratio,a.minimum_thickness_ratio,
      a.maximum_thickness_ratio,a.mask)!=
     std::tie(b.epoch,b.time,b.carried_velocity_time,b.kick_dt,b.wall_impulse,b.wall_impulse_error,
      b.synchronous_wall_impulse,b.synchronous_wall_impulse_error,
      b.minimum_gap,b.maximum_gap,b.minimum_velocity,b.maximum_velocity,b.relative_displacement,b.rotation_angle,
      b.strain,b.thickness_curvature,b.minimum_area_ratio,b.maximum_area_ratio,b.minimum_thickness_ratio,
      b.maximum_thickness_ratio,b.mask)) return false;
  if(a.state.x!=b.state.x||a.state.v!=b.state.v||a.state.omega!=b.state.omega||a.state.q!=b.state.q||
      a.synchronous_velocity!=b.synchronous_velocity||a.synchronous_omega!=b.synchronous_omega||
      a.rotation_vector!=b.rotation_vector||a.velocity_error!=b.velocity_error||a.omega_error!=b.omega_error||
      a.endpoint_native_force_error!=b.endpoint_native_force_error||
      a.endpoint_native_couple_error!=b.endpoint_native_couple_error||
      a.endpoint_contact_force_error!=b.endpoint_contact_force_error||
      a.endpoint_rhs!=b.endpoint_rhs||a.endpoint_couple!=b.endpoint_couple||a.carried_kinetic!=b.carried_kinetic||
      a.synchronous_kinetic!=b.synchronous_kinetic||a.kinetic_error!=b.kinetic_error||a.source_work!=b.source_work||
      a.values!=b.values||a.errors!=b.errors||!Same(a.residual,b.residual)||
      !Same(a.absolute_residual,b.absolute_residual)||!Same(a.analytic_difference,b.analytic_difference)) return false;
  const auto& x=a.contact; const auto& y=b.contact;
  if(std::tie(x.base_epoch,x.attempt,x.node_count,x.candidate)!=std::tie(y.base_epoch,y.attempt,y.node_count,y.candidate)||
      !Same(x.resultant,y.resultant)||!Same(x.potential,y.potential)) return false;
  for(unsigned n=0;n<MaxNodes;++n) if(!Same(x.nodes[n],y.nodes[n])) return false;
  return true;
}
bool ContainsMaximum(Interval maximum,Interval sample) {
  return comparison_detail::owning::Nonnegative(maximum)&&comparison_detail::owning::Nonnegative(sample)&&
      maximum.lower>=sample.lower&&maximum.upper>=sample.upper;
}
bool Contains(const Summary& m,const Sample& s) {
  double peak_upper=0,sample_upper=0;
  if(!comparison_detail::owning::AddScalar(m.maximum_resultant,m.maximum_resultant_error,true,&peak_upper)||
      !comparison_detail::owning::AddScalar(s.contact.resultant.value,s.contact.resultant.error,true,&sample_upper))
    return false;
  return ContainsMaximum(m.maximum_absolute_residual,s.absolute_residual)&&
      ContainsMaximum(m.maximum_analytic_difference,s.analytic_difference)&&
      m.maximum_depth>=std::max(0.,s.maximum_gap)&&m.maximum_resultant>=s.contact.resultant.value&&
      m.maximum_resultant_error>=s.contact.resultant.error&&peak_upper>=sample_upper&&
      m.maximum_relative_displacement>=s.relative_displacement&&m.maximum_rotation>=s.rotation_angle&&
      m.maximum_strain>=s.strain&&m.maximum_thickness_curvature>=s.thickness_curvature&&
      m.minimum_area_ratio<=s.minimum_area_ratio&&m.maximum_area_ratio>=s.maximum_area_ratio&&
      m.minimum_thickness_ratio<=s.minimum_thickness_ratio&&m.maximum_thickness_ratio>=s.maximum_thickness_ratio;
}
bool ValidEvent(const EventBracket& e,const Config& c,std::uint64_t last_epoch) {
  if(!e.observed) return e.before_epoch==0&&e.after_epoch==0&&e.before_time==0&&e.after_time==0;
  return e.after_epoch>0&&e.after_epoch<=last_epoch&&e.before_epoch==e.after_epoch-1&&
      e.before_time==e.before_epoch*Step(c)&&e.after_time==e.after_epoch*Step(c);
}
bool ValidSummary(const Summary& s,const Config& c) {
  if(!s.initialized||s.last_epoch>Steps(c)||s.last_time!=s.last_epoch*Step(c)||
      !std::isfinite(s.last_minimum_velocity)||!std::isfinite(s.last_maximum_velocity)||
      s.last_minimum_velocity>s.last_maximum_velocity||
      !comparison_detail::owning::Nonnegative(s.maximum_absolute_residual)||
      !comparison_detail::owning::Nonnegative(s.maximum_analytic_difference)||
      !ValidEvent(s.entry,c,s.last_epoch)||!ValidEvent(s.peak,c,s.last_epoch)||!ValidEvent(s.exit,c,s.last_epoch))
    return false;
  const double maxima[]{s.maximum_depth,s.maximum_resultant,s.maximum_resultant_error,s.maximum_relative_displacement,
      s.maximum_rotation,s.maximum_strain,s.maximum_thickness_curvature,s.minimum_area_ratio,s.maximum_area_ratio,
      s.minimum_thickness_ratio,s.maximum_thickness_ratio};
  for(double x:maxima) if(!std::isfinite(x)||x<0) return false;
  const double times[]{s.residual_time,s.analytic_time,s.depth_time,s.resultant_time};
  for(double t:times) if(!std::isfinite(t)||t<0||t>s.last_time) return false;
  const unsigned full=(1u<<(2*(c.cells+1)))-1;
  if(s.last_mask!=0&&s.last_mask!=full) return false;
  switch(s.phase) {
    case Phase::BeforeEntry: return !s.entry.observed&&!s.peak.observed&&!s.exit.observed&&s.last_mask==0;
    case Phase::Compression: return s.entry.observed&&!s.peak.observed&&!s.exit.observed&&s.last_mask==full;
    case Phase::Unloading: return s.entry.observed&&s.peak.observed&&!s.exit.observed&&
        s.entry.after_epoch<=s.peak.before_epoch&&s.last_mask==full;
    case Phase::Separated: return s.entry.observed&&s.peak.observed&&s.exit.observed&&
        s.entry.after_epoch<=s.peak.before_epoch&&s.peak.after_epoch<=s.exit.before_epoch&&s.last_mask==0;
  }
  return false;
}
}
bool ValidateRun(const Run& r,bool require_complete,std::string& error) {
  if(!ValidConfig(r.config)||!std::isfinite(r.elapsed_seconds)||r.elapsed_seconds<0||
      r.accepted_steps>Steps(r.config)||r.attempted_steps<r.accepted_steps||r.attempted_steps>MaxSteps||
      r.native_cell_intervals<r.accepted_steps*r.config.cells||r.native_cell_intervals>r.attempted_steps*r.config.cells||
      r.samples.size()>SampleCount) return Fail(error,"Invalid frozen run identity, counts or resource observation");
  for(double value:r.ledger_maxima) if(!std::isfinite(value)) return Fail(error,"Nonfinite retained ledger observation");
  if(!r.model.prepared()) {
    if(require_complete||r.completed||r.accepted_steps||r.native_cell_intervals||!r.samples.empty()||r.summary.initialized)
      return Fail(error,"Unprepared run has published model-dependent results");
    error.clear(); return true;
  }
  const auto& m=r.model;
  if(m.fields().cells!=r.config.cells||m.fields().nodes!=2*(r.config.cells+1)||
      m.dictionary().empty()||m.dictionary().size()>MaxFields||
      !(std::isfinite(m.energy())&&m.energy()>0&&std::isfinite(m.force_scale())&&m.force_scale()>0&&
        std::isfinite(m.momentum())&&m.momentum()>0)) return Fail(error,"Invalid immutable response model");
  if(require_complete||r.completed) {
    if(!r.completed||r.accepted_steps!=Steps(r.config)||r.attempted_steps!=r.accepted_steps||
        r.samples.size()!=SampleCount||!r.failure.empty()||
        r.owner_id==0||!r.owner_device_bytes||!r.batch_device_bytes||!r.wall_device_bytes||
        !r.owner_allocations||!r.batch_allocations||!r.wall_allocations)
      return Fail(error,"Run has not completed the frozen horizon");
    if(!CompleteSummary(m,r.config,r.summary,error)) return false;
  }
  if(r.samples.empty()) {
    if(r.accepted_steps||r.summary.initialized) return Fail(error,"Accepted state lacks its initial observation");
    error.clear(); return true;
  }
  if(r.samples.size()!=r.accepted_steps/SampleStride(r.config)+1||!ValidSummary(r.summary,r.config)||
      r.summary.last_epoch!=r.accepted_steps) return Fail(error,"Skipped samples or inconsistent all-endpoint summary");
  for(unsigned j=0;j<r.samples.size();++j) {
    const auto& s=r.samples[j];
    if(s.epoch!=j*SampleStride(r.config)||s.time!=j*(Horizon/256)||
        !ValidateSample(m,r.config,s,error)) return Fail(error,"Invalid common endpoint sample");
    if(!Contains(r.summary,s)) return Fail(error,"All-endpoint summary omits retained sample evidence");
  }
  const auto& last=r.last_accepted;
  if(last.epoch!=r.accepted_steps||!ValidateSample(m,r.config,last,error)||!Contains(r.summary,last)||
      r.summary.last_time!=last.time||r.summary.last_mask!=last.mask||
      r.summary.last_minimum_velocity!=last.minimum_velocity||r.summary.last_maximum_velocity!=last.maximum_velocity)
    return Fail(error,"Last accepted endpoint differs from its summary");
  if(r.accepted_steps%SampleStride(r.config)==0&&!SameEndpoint(last,r.samples.back()))
    return Fail(error,"Last common sample differs from the complete last accepted endpoint");
  error.clear(); return true;
}
} // namespace tl::qualification::qeph::wall_response
