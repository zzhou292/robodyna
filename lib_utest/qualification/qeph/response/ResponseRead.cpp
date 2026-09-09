#include "ResponseProtocol.h"

namespace tl::qualification::qeph::response {
namespace p=protocol;
namespace {
void EqualArray(const io::Value& d,const char* key,const double* expected,unsigned count) {
  double values[18]{}; io::Require(count<=18,"Unexpected model array capacity"); p::ReadArray(d,key,values,count);
  for(unsigned i=0;i<count;++i) io::Require(io::Bits(values[i])==io::Bits(expected[i]),"Changed frozen model/mass input");
}
}
Run ReadRun(const std::string& bytes,Binding& output_binding) {
  const auto d=p::Parse(bytes);
  io::Require(p::String(d,"schema")=="robo-dyna-qeph-response-numeric-v1"&&!p::Boolean(d,"refinement_admitted")&&
    !p::Boolean(d,"simulation_ready"),"Not a scoped numeric response report");
  io::Require(p::Integer(d,"qualification_id")==Qualification&&p::Integer(d,"configuration_id")==Configuration,"Foreign experiment identity");
  Run r; const auto cells=p::Integer(d,"cells"),refinement=p::Integer(d,"refinement");
  io::Require(cells<=2&&refinement<=4,"Invalid fixture/refinement"); r.config={static_cast<unsigned>(cells),static_cast<unsigned>(refinement)};
  io::Require(ValidConfig(r.config),"Unsupported response fixture"); std::string error;
  const bool model_valid=BuildModel(r.config.cells,r.model,error); io::Require(model_valid,error.c_str());
  io::Require(p::Integer(d,"nodes")==r.model.nodes&&p::Number(d,"fixed_dt_s")==H0/r.config.refinement&&
    p::Number(d,"pulse_duration_s")==Pulse&&p::Number(d,"horizon_s")==Horizon,"Changed physical horizon/time level");
  io::Require(p::Number(d,"young_modulus_Pa")==Young&&p::Number(d,"density_kg_per_m3")==Density&&
    p::Number(d,"thickness_m")==Thickness&&p::Number(d,"poisson_ratio")==Poisson&&p::Number(d,"side_m")==Side&&
    p::Number(d,"theta_amplitude_rad")==Theta&&p::Number(d,"delta_amplitude_m")==Delta&&
    p::Number(d,"experiment_energy_scale_J")==ExperimentEnergy(r.model),"Changed frozen material/load scales");
  EqualArray(d,"initial_position_xyz_m",r.model.initial_position.data(),3*r.model.nodes);
  EqualArray(d,"nodal_mass_kg",r.model.mass.data(),r.model.nodes); EqualArray(d,"nodal_native_inertia_kg_m2",r.model.inertia.data(),r.model.nodes);
  EqualArray(d,"nodal_physical_isotropic_inertia_kg_m2",r.model.physical.data(),r.model.nodes);
  EqualArray(d,"nodal_area_added_isotropic_inertia_kg_m2",r.model.added.data(),r.model.nodes);
  const auto& connectivity=p::Array(d,"connectivity",r.model.cells);
  for(unsigned e=0;e<r.model.cells;++e) { io::Require(connectivity[e].IsArray()&&connectivity[e].Size()==4,"Wrong native connectivity");
    for(unsigned i=0;i<4;++i) io::Require(connectivity[e][i].IsUint()&&connectivity[e][i].GetUint()==r.model.connectivity[e][i],"Changed native node order"); }
  r.fields=Dictionary(r.model); const auto& fields=p::Array(d,"fields",r.fields.size());
  for(unsigned i=0;i<r.fields.size();++i) io::Require(p::String(fields[i],"name")==r.fields[i].name&&p::String(fields[i],"unit")==r.fields[i].unit&&
    p::Number(fields[i],"scale")==r.fields[i].scale&&p::Boolean(fields[i],"compare")==r.fields[i].compare,"Changed comparison field/scale");
  r.owner_id=p::Integer(d,"owner_id"); r.completed=p::Boolean(d,"completed");
  io::Require(p::String(d,"stage")==(r.completed?"single-run-complete":"single-run-incomplete"),"Completion label mismatch");
  r.accepted_steps=p::Integer(d,"accepted_steps"); r.attempted_steps=p::Integer(d,"attempted_steps"); r.elapsed_seconds=p::Number(d,"elapsed_seconds");
  r.failure=p::String(d,"failure"); r.external_work_at_pulse=p::Number(d,"external_work_at_pulse_J");
  r.maximum_abs_residual=p::Number(d,"maximum_abs_residual_J"); r.residual_time=p::Number(d,"residual_extremum_time_s");
  const auto& maximum=p::Member(d,"maximum_sampled_response"); const auto field=p::Integer(maximum,"field");
  io::Require(field<r.fields.size()&&p::String(maximum,"name")==r.fields[field].name,"Wrong extremum identity");
  r.maximum_response={p::Number(maximum,"value"),p::Number(maximum,"time_s"),static_cast<unsigned>(field),p::Number(maximum,"lower")};
  r.observed=p::ReadLimits(p::Member(d,"all_endpoint_small_response"));
  p::ReadArray(d,"external_linear_impulse_N_s",r.external_linear_impulse.data(),3);
  p::ReadArray(d,"external_base_arm_angular_impulse_N_m_s",r.external_angular_impulse.data(),3);
  p::ReadArray(d,"ledger_maxima",r.ledger_maxima.data(),7);
  r.owner_device_bytes=p::Integer(d,"owned_owner_device_bytes"); r.batch_device_bytes=p::Integer(d,"owned_batch_device_bytes");
  r.owner_allocations=p::Integer(d,"owner_device_allocations"); r.batch_allocations=p::Integer(d,"batch_device_allocations");
  r.cuda_free_before=p::Integer(d,"device_free_bytes_before"); r.cuda_free_after_initialize=p::Integer(d,"device_free_bytes_after_initialize");
  r.cuda_free_after_run=p::Integer(d,"device_free_bytes_after_run");
  const auto& samples=p::Member(d,"common_endpoint_samples"); io::Require(samples.IsArray()&&samples.Size()<=SampleCount,"Unbounded endpoint samples");
  r.samples.reserve(SampleCount); for(const auto& sample:samples.GetArray()) r.samples.push_back(p::ReadSample(sample,r.fields.size()));
  if(!r.samples.empty()) r.last_accepted=p::ReadSample(p::Member(d,"last_accepted_endpoint"),r.fields.size());
  const auto binding=p::ReadBinding(d); io::Require(p::Member(d,"provenance").IsObject(),"Missing retained source provenance");
  const bool valid=ValidateRun(r,false,error); io::Require(valid,error.c_str()); output_binding=binding; return r;
}
} // namespace tl::qualification::qeph::response
