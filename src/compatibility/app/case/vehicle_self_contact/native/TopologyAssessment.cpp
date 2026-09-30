#include "TopologyAssessmentInternal.h"
#include <chrono>
namespace crash::cases::vehicle_self_contact::native {
namespace {
using Clock=std::chrono::steady_clock;
double Elapsed(Clock::time_point start){return std::chrono::duration<double>(Clock::now()-start).count();}
}
Forecast Preflight(const selection::OriginalSelection& original,Config config,Limits limits) {
  return detail::Plan(original.canonical().data(),original.data(),config,limits);
}
Result Assess(const selection::OriginalSelection& original,Config config,Limits limits) {
  return detail::EvaluateValues(original.canonical().data(),original.data(),config,limits);
}
namespace detail {
Result EvaluateValues(const source::CanonicalData& canonical,const selection::Data& original,Config config,Limits limits) {
  Result result;result.config=config;result.forecast=Plan(canonical,original,config,limits);
  output::Require(result.forecast.admitted,"Complete topology assessment forecast exceeds limits");
  result.parts=original.parts;
  result.provenance={canonical.inputs.canonical_manifest,canonical.inputs.scope_report,canonical.inputs.source_member,
      original.auxiliary_sha256,original.combine_sha256,canonical.inputs.units};
  auto start=Clock::now();auto inputs=PrepareInputs(canonical,original,config,limits,result.counts);
  result.input_digest=InputDigest(inputs,config,limits.metadata_bytes);result.input_wall_s=Elapsed(start);
  tl::util::HostArena arena;
  output::Require(arena.Initialize(result.forecast.topology.output_bytes),"Topology output allocation failed");
  s::Snapshot snapshot;start=Clock::now();
  {
    tl::util::HostArena scratch;
    output::Require(scratch.Initialize(result.forecast.topology.scratch_bytes),"Topology scratch allocation failed");
    // Exactly one complete call. No failed-row deletion, reordering, unit change or fallback retry.
    result.topology_report=s::BuildStarter(inputs.View(),limits.topology,arena,scratch,&snapshot);
  }
  result.topology_wall_s=Elapsed(start); // Scratch is gone before output digest buffers exist.
  const auto& report=result.topology_report;result.failure=MapLocation(inputs,report.primary,report.node);
  if(report.neighbor_warnings.count) {
    const auto m=report.neighbor_warnings.first_main,edge=report.neighbor_warnings.first_edge;
    output::Require(m<2*inputs.primary.size()&&edge<4,"TL returned an invalid warning source location");
    const auto parent=m<inputs.primary.size()?m:m-inputs.primary.size();
    result.first_warning=MapLocation(inputs,parent,SIZE_MAX);result.first_warning.expanded_main=m;result.first_warning.edge=edge;
  }
  if(report.status!=s::Status::Ok)return result;
  result.output_complete=true;result.counts.output_mains=snapshot.main_count;
  result.counts.output_references=snapshot.starter.reference_count;result.counts.output_incidences=snapshot.normal_incidence_count;
  for(std::size_t m=0;m<snapshot.main_count;++m)for(unsigned k=0;k<4;++k)
    if(!snapshot.mains[m].neighbors[k]&&!(k==2&&snapshot.mains[m].nodes[2]==snapshot.mains[m].nodes[3]))++result.counts.free_edge_slots;
  start=Clock::now();result.output_digest=TopologyDigest(snapshot,result.input_digest.sha256,limits.metadata_bytes);result.digest_wall_s=Elapsed(start);
  return result;
}
} // namespace detail
} // namespace crash::cases::vehicle_self_contact::native
