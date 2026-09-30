#include "WallResponseJson.h"

namespace tl::qualification::qeph::wall_response {
namespace json {
io::Document ModelReport(const Model& model) {
  io::Require(model.prepared(),"Unprepared wall-response model"); const auto& m=model.fields();
  io::Document d; d.SetObject(); io::Integer(d,"cells",m.cells); io::Integer(d,"nodes",m.nodes);
  Array(d,"initial_position_xyz_m",m.initial_position,3*m.nodes); Array(d,"nodal_mass_kg",m.mass,m.nodes);
  Array(d,"nodal_native_inertia_kg_m2",m.inertia,m.nodes); Array(d,"nodal_physical_inertia_kg_m2",m.physical,m.nodes);
  Array(d,"nodal_area_added_inertia_kg_m2",m.added,m.nodes);
  io::Number(d,"K0_J",model.energy()); io::Number(d,"P0_N_s",model.momentum());
  io::Number(d,"force_scale_N",model.force_scale()); io::Number(d,"time_scale_s",model.time_scale());
  io::Number(d,"target_depth_m",wr::TargetDepth); io::Number(d,"incoming_speed_m_s",wr::ImpactSpeed);
  io::Number(d,"initial_gap_m",wr::InitialGap); io::Number(d,"stiffness_per_area_N_m3",model.screened().law().stiffness_per_area);
  const auto& material=m.reference[0].input;
  io::Number(d,"young_modulus_Pa",material.young_modulus); io::Number(d,"density_kg_m3",material.density);
  io::Number(d,"thickness_m",material.thickness); io::Number(d,"poisson_ratio",material.poisson_ratio);
  io::Number(d,"maximum_depth_m",wr::MaximumDepth); io::Number(d,"motion_extent_m",wr::MotionExtent);
  io::Number(d,"wall_clearance_m",wr::WallClearance); io::Number(d,"parent_force_error_N",wr::ParentForceError);
  io::Number(d,"parent_energy_error_J",wr::ParentEnergyError);
  io::Value connectivity(rapidjson::kArrayType);
  for(unsigned e=0;e<m.cells;++e) { io::Value row(rapidjson::kArrayType);
    for(auto n:m.connectivity[e]) row.PushBack(n,d.GetAllocator()); connectivity.PushBack(row,d.GetAllocator()); }
  d.AddMember("connectivity",connectivity,d.GetAllocator()); io::Value fields(rapidjson::kArrayType);
  for(const auto& field:model.dictionary()) { io::Document f; f.SetObject();
    io::String(f,"name",field.name); io::String(f,"unit",field.unit); io::Number(f,"scale",field.scale);
    io::Boolean(f,"compare",field.compare); p::Push(d,fields,f); }
  d.AddMember("fields",fields,d.GetAllocator()); return d;
}
void Bind(io::Document& d,const ReportBinding& b) {
  io::Require(p::Hash(b.screen_index_sha256)&&p::Hash(b.runtime_sha256)&&p::Hash(b.provenance_sha256),"Invalid wall-response binding");
  io::String(d,"screen_index_sha256",b.screen_index_sha256); io::String(d,"runtime_sha256",b.runtime_sha256);
  io::String(d,"provenance_sha256",b.provenance_sha256);
}
ReportBinding ReadBinding(const io::Value& d) {
  ReportBinding b{p::String(d,"screen_index_sha256"),p::String(d,"runtime_sha256"),p::String(d,"provenance_sha256")};
  io::Require(p::Hash(b.screen_index_sha256)&&p::Hash(b.runtime_sha256)&&p::Hash(b.provenance_sha256),"Malformed wall-response binding"); return b;
}
}
bool SameBinding(const ReportBinding& a,const ReportBinding& b) noexcept {
  return a.screen_index_sha256==b.screen_index_sha256&&a.runtime_sha256==b.runtime_sha256&&a.provenance_sha256==b.provenance_sha256;
}
std::string EncodeReport(const io::Document& d) {
  auto bytes=free_response::Encode(d); io::Require(bytes.size()<=ReportByteCap,"Wall-response report exceeds 16 MiB"); return bytes;
}
io::Document DescribeRun(const Run& r,const ReportBinding& b,const std::string& exact_provenance) {
  std::string error; const bool valid=ValidateRun(r,false,error); io::Require(valid,error.c_str());
  io::Require(exact_provenance.size()<=1024*1024&&io::Sha256(exact_provenance)==b.provenance_sha256,"Exact producer provenance binding mismatch");
  const auto provenance=json::p::Parse(exact_provenance);
  io::Require(provenance.IsObject()&&!provenance.ObjectEmpty(),"Missing reviewed source/runtime provenance");
  io::Require(b.screen_index_sha256==r.config.screen_index_sha,"Run and report name different wall screens");
  io::Document d; d.SetObject(); io::String(d,"schema","robo-dyna-qeph-wall-response-v1");
  io::Boolean(d,"refinement_admitted",false); io::Boolean(d,"simulation_ready",false);
  io::String(d,"scope","Frozen one/two-cell elastic rigid-broadside contact experiment; no deforming impact, mixed feedback, vehicle or renderable archive admission");
  io::String(d,"trust_scope","Externally pinned reviewed producer/source/runtime; retained samples rechecked; unsaved per-step extrema are producer evidence");
  io::String(d,"phase_contract",json::PhaseContract); io::String(d,"energy_contract",json::EnergyContract);
  io::String(d,"ledger_order","kick, linear momentum, angular momentum, internal work, source work, contact work, contact impulse, contact convex defect, regular rate, hourglass rate: each normalized to its owning retained budget");
  io::Integer(d,"qualification_id",Qualification); io::Integer(d,"shell_configuration_id",ShellConfiguration);
  io::Integer(d,"wall_configuration_id",WallConfiguration); io::Integer(d,"wall_binding_id",WallBinding);
  io::Integer(d,"cells",r.config.cells); io::Integer(d,"refinement",r.config.refinement);
  io::Number(d,"selected_h_s",r.config.selected_h); io::Number(d,"fixed_dt_s",Step(r.config)); io::Number(d,"horizon_s",Horizon);
  json::Bind(d,b); io::String(d,"provenance_exact_bytes",exact_provenance); json::RunMetadata(d,r);
  json::p::Object(d,"model",json::ModelReport(r.model)); json::p::Object(d,"all_endpoint_summary",json::SummaryReport(r.summary));
  io::Value samples(rapidjson::kArrayType);
  for(const auto& sample:r.samples) json::p::Push(d,samples,json::SampleReport(sample,r.model));
  d.AddMember("common_endpoint_samples",samples,d.GetAllocator());
  if(r.summary.initialized) json::p::Object(d,"last_accepted_endpoint",json::SampleReport(r.last_accepted,r.model));
  return d;
}
io::Document DescribeComparison(const std::array<Run,3>& runs,const std::array<std::string,3>& hashes,const ReportBinding& binding) {
  for(const auto& run:runs) io::Require(run.config.screen_index_sha==binding.screen_index_sha256,"Comparison and runs name different wall screens");
  const auto c=Compare(runs); io::Document d; d.SetObject(); io::String(d,"schema","robo-dyna-qeph-wall-response-comparison-v1");
  io::Boolean(d,"input_valid",c.input_valid); io::Boolean(d,"broadside_refinement_passed",c.passed);
  io::Boolean(d,"simulation_ready",false); io::String(d,"scope","Frozen rigid broadside experiment only; no wall-tessellation, deforming-impact or vehicle admission");
  io::String(d,"diagnostic",c.diagnostic); json::Bind(d,binding); io::Integer(d,"cells",c.cells);
  io::Number(d,"selected_h_s",c.selected_h); io::Number(d,"energy_normalization_J",c.energy_normalization);
  io::Boolean(d,"response_passed",c.response_passed); io::Boolean(d,"energy_passed",c.energy_passed);
  io::Boolean(d,"analytic_passed",c.analytic_passed); io::Value inputs(rapidjson::kArrayType);
  for(const auto& hash:hashes) { io::Require(json::p::Hash(hash),"Invalid source run hash"); inputs.PushBack(io::Value(hash.c_str(),d.GetAllocator()),d.GetAllocator()); }
  d.AddMember("run_sha256_h_h2_h4",inputs,d.GetAllocator());
  auto difference=[&](const char* key,const ResponseDifference& v) {
    io::Document x; x.SetObject(); json::IntervalField(x,"maximum",v.maximum);
    io::Number(x,"time_s",v.time); io::Integer(x,"field",v.field);
    if(v.field<runs[0].model.dictionary().size()) io::String(x,"field_name",runs[0].model.dictionary()[v.field].name);
    json::p::Object(d,key,x);
  };
  difference("coarse_medium",c.coarse_medium); difference("medium_fine",c.medium_fine);
  io::Value energy(rapidjson::kArrayType),analytic(rapidjson::kArrayType);
  for(unsigned i=0;i<3;++i) { json::p::Push(d,energy,json::raw::Interval(c.residual_ratios[i])); json::p::Push(d,analytic,json::raw::Interval(c.analytic_differences[i])); }
  d.AddMember("maximum_energy_ratios",energy,d.GetAllocator()); d.AddMember("maximum_analytic_differences",analytic,d.GetAllocator());
  io::Number(d,"coarse_difference_limit",CoarseResponseLimit); io::Number(d,"fine_difference_limit",FineResponseLimit);
  io::Number(d,"contraction_factor",.75); io::Number(d,"response_contraction_floor",ResponseContractionFloor);
  io::Number(d,"energy_contraction_floor_J",EnergyContractionFloor*runs[0].model.energy());
  io::Number(d,"maximum_energy_ratio",MaximumEnergyRatio); io::Number(d,"maximum_analytic_error",MaximumAnalyticError); return d;
}
} // namespace tl::qualification::qeph::wall_response
