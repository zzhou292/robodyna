#include "WallSelectionReport.h"
#include "WallJobJson.h"

namespace tl::qualification::qeph::wall_recurrence::selection_json {
namespace io=crash::output;
namespace {
void Link(io::Document& d,const char* label,const std::vector<RawFileReceipt>& files,const std::string& name) {
  io::Document p; p.SetObject(); const RawFileReceipt* found=nullptr;
  for(const auto& f:files) if(f.name==name) found=&f;
  io::String(p,"file",name); io::Boolean(p,"present",found!=nullptr);
  if(found) { io::String(p,"sha256",found->sha256); io::Integer(p,"bytes",found->bytes); }
  raw_detail::Field(d,label,p);
}
}
io::Document Input(const WallSelectionInput& p) {
  io::Document d; d.SetObject(); io::Integer(d,"cells",p.cells); io::Number(d,"normal_velocity_m_s",p.normal_velocity);
  io::Integer(d,"normal_velocity_binary64_bits",io::Bits(p.normal_velocity));
  io::String(d,"raw_directory",p.raw_directory.string()); io::String(d,"raw_index_sha256",p.raw_index_sha256);
  io::String(d,"raw_provenance_sha256",p.raw_provenance_sha256); io::Integer(d,"raw_bytes",p.raw_bytes);
  io::String(d,"derived_directory",p.derived_directory.string()); io::String(d,"derived_index_sha256",p.derived_index_sha256);
  io::String(d,"derived_provenance_sha256",p.derived_provenance_sha256); io::Integer(d,"derived_bytes",p.derived_bytes); return d;
}
io::Document Boost(const WallBoostComparison& b,const RawJobReceipt& zr,const RawJobReceipt& br,
    const WallDerivedReceipt& zd,const WallDerivedReceipt& bd) {
  io::Require((b.cells==1||b.cells==2)&&(b.normal_velocity==-8||b.normal_velocity==8)&&
    b.dimension==12*2*(b.cells+1)+61*b.cells&&b.diagnostic.size()<=4096,"Invalid boost report identity");
  io::Document d; d.SetObject(); io::Integer(d,"cells",b.cells); io::Integer(d,"dimension",b.dimension);
  io::Number(d,"normal_velocity_m_s",b.normal_velocity); io::Boolean(d,"input_valid",b.input_valid);
  io::String(d,"diagnostic",b.diagnostic);
  io::String(d,"operator_recipe","Both A_branch=BuildContactBranch(model,h,branch,retained_native_shell); weighted gain contexts bind ApplyWallStateMetric with retained D");
  io::String(d,"baseline_scope","Two independent absolute moving-baseline gates; expected hV/V lift difference is diagnostic");
  io::String(d,"raw_gain_scope","Raw gain differences are retained diagnostics; weighted gain differences control consistency");
  io::Value steps(rapidjson::kArrayType);
  for(unsigned s=0;s<6;++s) {
    const auto& step=b.steps[s]; io::Require(step.h==recurrence::Steps[s],"Changed boost report h");
    io::Require(!step.complete||b.input_valid,"Complete boost step lacks valid input");
    io::Document p; p.SetObject(); io::Number(p,"fixed_dt_s",step.h); job_json::Status(p,step.complete,step.passed,"");
    const auto context="context-h"+std::to_string(s)+".json";
    Link(p,"zero_context",zd.files,context); Link(p,"boost_context",bd.files,context);
    io::Value amplitudes(rapidjson::kArrayType);
    for(unsigned a=0;a<3;++a) {
      const auto& value=step.amplitudes[a];
      io::Require((!step.complete||value.complete)&&(!step.passed||value.passed),"Boost step overstates amplitude evidence");
      for(const auto& baseline:value.baselines)
        io::Require((!value.complete||baseline.complete)&&(!value.passed||baseline.passed),"Boost amplitude overstates baseline evidence");
      io::Require((!value.complete||value.native_matrix.complete)&&(!value.passed||value.native_matrix.passed),
        "Boost amplitude overstates native-matrix evidence");
      for(const auto& matrix:value.branch_matrices)
        io::Require((!value.complete||matrix.complete)&&(!value.passed||matrix.passed),"Boost amplitude overstates branch-matrix evidence");
      for(unsigned i=0;i<11;++i)
        io::Require((!value.complete||(value.raw_gains[i].complete&&value.weighted_gains[i].complete))&&
          (!value.passed||value.weighted_gains[i].passed),"Boost amplitude overstates gain evidence");
      io::Require(value.lift_controlling_coordinate<b.dimension,"Invalid boost lift coordinate");
      io::Document v; v.SetObject(); io::Number(v,"amplitude",recurrence::Amplitudes[a]);
      job_json::Status(v,value.complete,value.passed,value.diagnostic);
      const auto native="native-h"+std::to_string(s)+"-a"+std::to_string(a)+".json";
      const auto derived="amplitude-h"+std::to_string(s)+"-a"+std::to_string(a)+".json";
      Link(v,"zero_native",zr.files,native); Link(v,"boost_native",br.files,native);
      Link(v,"zero_derived",zd.files,derived); Link(v,"boost_derived",bd.files,derived);
      raw_detail::Field(v,"zero_baseline",job_json::Baseline(value.baselines[0],b.dimension));
      raw_detail::Field(v,"boost_baseline",job_json::Baseline(value.baselines[1],b.dimension));
      job_json::Scalar(v,"lift_residual_max",value.lift_residual_max,!value.complete);
      io::Integer(v,"lift_controlling_coordinate",value.lift_controlling_coordinate);
      raw_detail::Field(v,"native_matrix",job_json::Difference(value.native_matrix));
      io::Value matrices(rapidjson::kArrayType),raw(rapidjson::kArrayType),weighted(rapidjson::kArrayType);
      for(const auto& x:value.branch_matrices) raw_detail::Append(v,matrices,job_json::Difference(x));
      for(const auto& x:value.raw_gains) raw_detail::Append(v,raw,job_json::Difference(x));
      for(const auto& x:value.weighted_gains) raw_detail::Append(v,weighted,job_json::Difference(x));
      v.AddMember("branch_matrices_inactive_active",matrices,v.GetAllocator());
      v.AddMember("raw_gains_diagnostic",raw,v.GetAllocator()); v.AddMember("weighted_gains_gate",weighted,v.GetAllocator());
      raw_detail::Append(p,amplitudes,v);
    }
    p.AddMember("amplitudes",amplitudes,p.GetAllocator()); raw_detail::Append(d,steps,p);
  }
  d.AddMember("steps",steps,d.GetAllocator()); return d;
}
io::Document Selection(const WallScreenSelection& s) {
  io::Require(s.diagnostic.size()<=4096&&(!s.passed||s.input_valid),"Invalid selection verdict");
  io::Document d; d.SetObject(); io::Boolean(d,"input_valid",s.input_valid); io::Boolean(d,"passed",s.passed);
  io::Number(d,"selected_h",s.selected_h); io::String(d,"diagnostic",s.diagnostic);
  io::String(d,"job_order","(1,0),(1,-8),(1,+8),(2,0),(2,-8),(2,+8)");
  io::String(d,"boost_order","(1,-8),(1,+8),(2,-8),(2,+8)");
  io::Value steps(rapidjson::kArrayType); std::array<bool,6> passing{};
  for(unsigned h=0;h<6;++h) {
    const auto& step=s.steps[h]; io::Require(step.h==recurrence::Steps[h],"Changed selection h");
    bool passed=true; io::Document p; p.SetObject(); io::Number(p,"fixed_dt_s",step.h);
    io::Value jobs(rapidjson::kArrayType),boosts(rapidjson::kArrayType);
    for(bool value:step.jobs) { jobs.PushBack(value,p.GetAllocator()); passed=passed&&value; }
    for(bool value:step.boosts) { boosts.PushBack(value,p.GetAllocator()); passed=passed&&value; }
    io::Require(!s.input_valid||step.passed==passed,"Selection step differs from its contributors");
    io::Boolean(p,"passed",step.passed); io::Boolean(p,"diagnostic_only",h==5);
    p.AddMember("jobs",jobs,p.GetAllocator()); p.AddMember("boosts",boosts,p.GetAllocator());
    passing[h]=step.passed; raw_detail::Append(d,steps,p);
  }
  io::Require(s.input_valid?(s.selected_h==SelectWallScreenStep(passing)&&s.passed==(s.selected_h>0)):
    (!s.passed&&s.selected_h==0),"Selection differs from the qualified margin rule");
  d.AddMember("steps",steps,d.GetAllocator()); return d;
}
} // namespace tl::qualification::qeph::wall_recurrence::selection_json
