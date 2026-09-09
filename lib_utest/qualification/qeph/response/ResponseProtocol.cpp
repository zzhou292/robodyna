#include "ResponseProtocol.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
#include <set>

namespace tl::qualification::qeph::response {
namespace protocol {
namespace {
void Unique(const io::Value& v,unsigned depth) {
  io::Require(depth<=32,"Report JSON nesting exceeds bound");
  if(v.IsObject()) {
    std::set<std::string> names;
    for(auto i=v.MemberBegin();i!=v.MemberEnd();++i) {
      io::Require(names.emplace(i->name.GetString(),i->name.GetStringLength()).second,"Duplicate report member"); Unique(i->value,depth+1);
    }
  } else if(v.IsArray()) for(const auto& x:v.GetArray()) Unique(x,depth+1);
}
}
io::Document Parse(const std::string& bytes) {
  io::Require(!bytes.empty()&&bytes.size()<=FileCap,"Report exceeds byte bound");
  io::Document d; d.Parse<rapidjson::kParseFullPrecisionFlag|rapidjson::kParseIterativeFlag>(bytes.c_str(),bytes.size());
  io::Require(!d.HasParseError()&&d.IsObject(),"Malformed report JSON"); Unique(d,0); return d;
}
void Bind(io::Document& d,const Binding& b) {
  io::Require(Hash(b.decision_sha256)&&Hash(b.raw_sha256)&&Hash(b.runtime_sha256),"Invalid exact input/binary hash binding");
  io::String(d,"matrix_decision_sha256",b.decision_sha256); io::String(d,"matrix_raw_sha256",b.raw_sha256);
  io::String(d,"runtime_binary_sha256",b.runtime_sha256);
}
Binding ReadBinding(const io::Value& d) {
  Binding b{String(d,"matrix_decision_sha256"),String(d,"matrix_raw_sha256"),String(d,"runtime_binary_sha256")};
  io::Require(Hash(b.decision_sha256)&&Hash(b.raw_sha256)&&Hash(b.runtime_sha256),"Invalid exact report hashes"); return b;
}
io::Document LimitsReport(const Limits& l) {
  io::Document d; d.SetObject(); io::Number(d,"displacement_over_side",l.displacement_over_side);
  io::Number(d,"rotation_angle",l.rotation_angle); io::Number(d,"strain",l.strain); io::Number(d,"thickness_curvature",l.thickness_curvature);
  io::Number(d,"minimum_area_ratio",l.minimum_area_ratio); io::Number(d,"maximum_area_ratio",l.maximum_area_ratio);
  io::Number(d,"minimum_thickness_ratio",l.minimum_thickness_ratio); io::Number(d,"maximum_thickness_ratio",l.maximum_thickness_ratio); return d;
}
Limits ReadLimits(const io::Value& d) {
  return {Number(d,"displacement_over_side"),Number(d,"rotation_angle"),Number(d,"strain"),Number(d,"thickness_curvature"),
    Number(d,"minimum_area_ratio"),Number(d,"maximum_area_ratio"),Number(d,"minimum_thickness_ratio"),Number(d,"maximum_thickness_ratio")};
}
io::Document SampleReport(const Sample& s,unsigned count) {
  io::Document d; d.SetObject(); io::Integer(d,"epoch",s.epoch); io::Number(d,"time_s",s.time);
  io::Number(d,"carried_velocity_time_s",s.carried_velocity_time); io::Number(d,"completed_kick_dt_s",s.kick_dt);
  io::Boolean(d,"completed_interval_available",s.interval_available); io::FiniteArray(d,"values",s.values.data(),count);
  io::FiniteArray(d,"carried_kinetic_J",s.carried_kinetic.data(),4); io::FiniteArray(d,"synchronous_kinetic_J",s.synchronous_kinetic.data(),4);
  io::FiniteArray(d,"source_work_EINT0_EINT1_EVIS_J",s.source_work.data(),3);
  io::Number(d,"external_work_J",s.external_work); io::Number(d,"energy_residual_J",s.residual); return d;
}
Sample ReadSample(const io::Value& d,unsigned count) {
  Sample s; s.epoch=Integer(d,"epoch"); s.time=Number(d,"time_s"); s.carried_velocity_time=Number(d,"carried_velocity_time_s");
  s.kick_dt=Number(d,"completed_kick_dt_s"); s.interval_available=Boolean(d,"completed_interval_available");
  ReadArray(d,"values",s.values.data(),count); ReadArray(d,"carried_kinetic_J",s.carried_kinetic.data(),4);
  ReadArray(d,"synchronous_kinetic_J",s.synchronous_kinetic.data(),4); ReadArray(d,"source_work_EINT0_EINT1_EVIS_J",s.source_work.data(),3);
  s.external_work=Number(d,"external_work_J"); s.residual=Number(d,"energy_residual_J"); return s;
}
}
std::string Encode(const io::Document& d) {
  rapidjson::StringBuffer buffer; rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  io::Require(d.Accept(writer),"Report serialization failed"); io::Require(buffer.GetSize()<=FileCap,"Report exceeds 32 MiB");
  return {buffer.GetString(),buffer.GetSize()};
}
} // namespace tl::qualification::qeph::response
