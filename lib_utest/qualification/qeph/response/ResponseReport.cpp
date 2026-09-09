#include "ResponseProtocol.h"

namespace tl::qualification::qeph::response {
namespace p=protocol;
namespace {
io::Document ExtremumReport(const Extremum& m,const std::vector<Field>& fields) {
  io::Require(m.field<fields.size(),"Invalid response extremum field");
  io::Document d; d.SetObject(); io::Number(d,"value",m.value); io::Number(d,"time_s",m.time);
  io::Number(d,"lower",m.lower);
  io::Integer(d,"field",m.field); io::String(d,"name",fields[m.field].name); return d;
}
void ModelFields(io::Document& d,const Model& m) {
  io::Number(d,"young_modulus_Pa",Young); io::Number(d,"density_kg_per_m3",Density);
  io::Number(d,"thickness_m",Thickness); io::Number(d,"poisson_ratio",Poisson); io::Number(d,"side_m",Side);
  io::Number(d,"theta_amplitude_rad",Theta); io::Number(d,"delta_amplitude_m",Delta);
  io::Number(d,"experiment_energy_scale_J",ExperimentEnergy(m));
  io::FiniteArray(d,"initial_position_xyz_m",m.initial_position.data(),3*m.nodes);
  io::FiniteArray(d,"nodal_mass_kg",m.mass.data(),m.nodes); io::FiniteArray(d,"nodal_native_inertia_kg_m2",m.inertia.data(),m.nodes);
  io::FiniteArray(d,"nodal_physical_isotropic_inertia_kg_m2",m.physical.data(),m.nodes);
  io::FiniteArray(d,"nodal_area_added_isotropic_inertia_kg_m2",m.added.data(),m.nodes);
  io::Value connectivity(rapidjson::kArrayType);
  for(unsigned e=0;e<m.cells;++e) { io::Value row(rapidjson::kArrayType);
    for(auto n:m.connectivity[e]) row.PushBack(n,d.GetAllocator()); connectivity.PushBack(row,d.GetAllocator()); }
  d.AddMember("connectivity",connectivity,d.GetAllocator());
}
}
io::Document DescribeRun(const Run& run,const Binding& binding,const io::Document& provenance) {
  std::string error; const bool valid=ValidateRun(run,false,error); io::Require(valid,error.c_str());
  io::Require(provenance.IsObject()&&!provenance.ObjectEmpty(),"Missing response source provenance");
  io::Document d; d.SetObject(); io::String(d,"schema","robo-dyna-qeph-response-numeric-v1");
  io::String(d,"stage",run.completed?"single-run-complete":"single-run-incomplete");
  io::Boolean(d,"completed",run.completed); io::Boolean(d,"refinement_admitted",false); io::Boolean(d,"simulation_ready",false);
  io::String(d,"scope","Fixed free LAW1 QEPH experiment only; no accepted visualization/archive or contact/vehicle admission");
  io::String(d,"phase_contract","x/q/history/cache at endpoint; carried v/omega at endpoint-h/2 except physical initial rest; synchronous output reconstructed with endpoint total RHS and h/2");
  io::String(d,"energy_contract","Ksync + EINT0 + EINT1 + EVIS - external base-load drift work; source work is not a potential");
  io::Integer(d,"qualification_id",Qualification); io::Integer(d,"configuration_id",Configuration);
  io::Integer(d,"owner_id",run.owner_id); io::Integer(d,"cells",run.config.cells); io::Integer(d,"nodes",run.model.nodes);
  io::Integer(d,"refinement",run.config.refinement); io::Number(d,"fixed_dt_s",H0/run.config.refinement);
  io::Number(d,"pulse_duration_s",Pulse); io::Number(d,"horizon_s",Horizon);
  io::Integer(d,"accepted_steps",run.accepted_steps); io::Integer(d,"attempted_steps",run.attempted_steps);
  io::Number(d,"elapsed_seconds",run.elapsed_seconds); io::String(d,"failure",run.failure);
  p::Bind(d,binding); p::Object(d,"provenance",provenance); ModelFields(d,run.model);
  io::Value dictionary(rapidjson::kArrayType);
  for(const auto& field:run.fields) { io::Document f; f.SetObject(); io::String(f,"name",field.name); io::String(f,"unit",field.unit);
    io::Number(f,"scale",field.scale); io::Boolean(f,"compare",field.compare); p::Push(d,dictionary,f); }
  d.AddMember("fields",dictionary,d.GetAllocator());
  io::Value samples(rapidjson::kArrayType);
  for(const auto& sample:run.samples) p::Push(d,samples,p::SampleReport(sample,run.fields.size()));
  d.AddMember("common_endpoint_samples",samples,d.GetAllocator());
  if(!run.samples.empty()) p::Object(d,"last_accepted_endpoint",p::SampleReport(run.last_accepted,run.fields.size()));
  io::Number(d,"external_work_at_pulse_J",run.external_work_at_pulse); io::Number(d,"maximum_abs_residual_J",run.maximum_abs_residual);
  io::Number(d,"residual_extremum_time_s",run.residual_time);
  p::Object(d,"maximum_sampled_response",ExtremumReport(run.maximum_response,run.fields));
  p::Object(d,"all_endpoint_small_response",p::LimitsReport(run.observed));
  io::FiniteArray(d,"external_linear_impulse_N_s",run.external_linear_impulse.data(),3);
  io::FiniteArray(d,"external_base_arm_angular_impulse_N_m_s",run.external_angular_impulse.data(),3);
  io::FiniteArray(d,"ledger_maxima",run.ledger_maxima.data(),7);
  io::String(d,"ledger_order","kick ratio, linear momentum ratio, angular momentum ratio, internal-work ratio, source-work ratio, maximum drift angular rounding kg*m2/s, latest prospective source-work J");
  io::Integer(d,"owned_owner_device_bytes",run.owner_device_bytes); io::Integer(d,"owned_batch_device_bytes",run.batch_device_bytes);
  io::Integer(d,"owner_device_allocations",run.owner_allocations); io::Integer(d,"batch_device_allocations",run.batch_allocations);
  io::Integer(d,"device_free_bytes_before",run.cuda_free_before); io::Integer(d,"device_free_bytes_after_initialize",run.cuda_free_after_initialize);
  io::Integer(d,"device_free_bytes_after_run",run.cuda_free_after_run);
  io::String(d,"memory_scope","cudaMemGetInfo is device-wide; differences are not owned allocations. Fixed owner/batch allocations and sample capacity; existing GTest/native oracle may allocate transient host diagnostics.");
  return d;
}
io::Document DescribeComparison(const Comparison& result,const std::array<Run,3>& runs,
                               const std::array<std::string,3>& hashes,const Binding& binding) {
  const auto expected=Compare(runs);
  auto same=[](const Difference& a,const Difference& b) { return a.maximum==b.maximum&&a.lower==b.lower&&a.time==b.time&&a.field==b.field; };
  io::Require(result.passed==expected.passed&&same(result.coarse_medium,expected.coarse_medium)&&same(result.medium_fine,expected.medium_fine)&&
    result.energy_normalization==expected.energy_normalization&&result.residual_ratios==expected.residual_ratios&&result.diagnostic==expected.diagnostic,
    "Comparison verdict differs from frozen checks");
  io::Document d; d.SetObject(); io::String(d,"schema","robo-dyna-qeph-response-comparison-v1");
  io::Boolean(d,"free_response_refinement_passed",result.passed); io::Boolean(d,"simulation_ready",false);
  io::String(d,"scope","Frozen one/two-cell free pulse only; no contact, constraints, T3, plasticity or vehicle qualification");
  io::String(d,"diagnostic",result.diagnostic); p::Bind(d,binding); io::Integer(d,"cells",runs[0].config.cells);
  io::Value inputs(rapidjson::kArrayType);
  for(const auto& h:hashes) { io::Require(p::Hash(h),"Invalid source report hash"); inputs.PushBack(io::Value(h.c_str(),d.GetAllocator()),d.GetAllocator()); }
  d.AddMember("run_sha256_h_h2_h4",inputs,d.GetAllocator());
  auto difference=[&](const char* name,const Difference& value) {
    p::Object(d,name,ExtremumReport({value.maximum,value.time,value.field,value.lower},runs[0].fields));
  };
  difference("coarse_medium",result.coarse_medium); difference("medium_fine",result.medium_fine);
  io::Number(d,"common_energy_normalization_J",result.energy_normalization);
  io::FiniteArray(d,"maximum_energy_residual_ratios",result.residual_ratios.data(),3);
  io::Number(d,"maximum_coarse_medium_difference",.02); io::Number(d,"maximum_medium_fine_difference",.015);
  io::Number(d,"refinement_factor",.75); io::Number(d,"response_refinement_floor",1e-8);
  io::Number(d,"maximum_energy_ratio",.02); io::Number(d,"energy_refinement_floor_J",1e-10*ExperimentEnergy(runs[0].model)); return d;
}
} // namespace tl::qualification::qeph::response
