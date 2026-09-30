#include "WallResponseJson.h"

namespace tl::qualification::qeph::wall_response {
Run ReadRun(const std::string& bytes,const std::string& expected_sha256,ReportBinding& output_binding) {
  io::Require(!bytes.empty()&&bytes.size()<=ReportByteCap,"Unbounded wall-response input");
  io::Require(json::p::Hash(expected_sha256)&&io::Sha256(bytes)==expected_sha256,"External wall-response SHA256 mismatch");
  const auto d=json::p::Parse(bytes); namespace p=json::p;
  io::Require(p::String(d,"schema")=="robo-dyna-qeph-wall-response-v1"&&!p::Boolean(d,"refinement_admitted")&&
    !p::Boolean(d,"simulation_ready"),"Foreign or promoted wall-response report");
  io::Require(p::String(d,"phase_contract")==json::PhaseContract&&p::String(d,"energy_contract")==json::EnergyContract,"Changed wall-response phase or energy contract");
  io::Require(p::Integer(d,"qualification_id")==Qualification&&p::Integer(d,"shell_configuration_id")==ShellConfiguration&&
    p::Integer(d,"wall_configuration_id")==WallConfiguration&&p::Integer(d,"wall_binding_id")==WallBinding,"Foreign wall-response identity");
  const auto binding=json::ReadBinding(d); Run r;
  r.config={json::Count(d,"cells",MaxElements),json::Count(d,"refinement",4),p::Number(d,"selected_h_s"),binding.screen_index_sha256};
  io::Require(ValidConfig(r.config)&&p::Number(d,"fixed_dt_s")==Step(r.config)&&p::Number(d,"horizon_s")==Horizon,"Changed wall-response timestep or horizon");
  std::string error; const bool prepared=BuildModel(r.config.cells,r.model,error); io::Require(prepared,error.c_str());
  const auto expected=json::ModelReport(r.model);
  io::Require(json::read::Same(p::Member(d,"model"),expected),"Changed native model or fixed comparison dictionary");
  const auto provenance=p::String(d,"provenance_exact_bytes");
  io::Require(provenance.size()<=1024*1024&&io::Sha256(provenance)==binding.provenance_sha256,"Producer provenance bytes differ from binding");
  const auto source=p::Parse(provenance); io::Require(!source.ObjectEmpty(),"Empty producer provenance");
  json::ReadRunMetadata(d,r); r.summary=json::ReadSummary(p::Member(d,"all_endpoint_summary"));
  const auto& samples=p::Member(d,"common_endpoint_samples"); io::Require(samples.IsArray()&&samples.Size()<=SampleCount,"Unbounded retained sample count");
  r.samples.reserve(SampleCount); for(const auto& sample:samples.GetArray()) r.samples.push_back(json::ReadSample(sample,r.model));
  if(r.summary.initialized) r.last_accepted=json::ReadSample(p::Member(d,"last_accepted_endpoint"),r.model);
  const bool valid=ValidateRun(r,false,error); io::Require(valid,error.c_str()); output_binding=binding; return r;
}
} // namespace tl::qualification::qeph::wall_response
