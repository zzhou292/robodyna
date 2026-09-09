#pragma once
#include "ResponseReport.h"
#include <cmath>

namespace tl::qualification::qeph::response::protocol {
inline const io::Value& Member(const io::Value& v,const char* key) {
  io::Require(v.IsObject()&&v.HasMember(key),"Missing report member"); return v[key];
}
inline double Number(const io::Value& v,const char* key) {
  const auto& x=Member(v,key); io::Require(x.IsNumber()&&std::isfinite(x.GetDouble()),"Expected finite report number"); return x.GetDouble();
}
inline std::uint64_t Integer(const io::Value& v,const char* key) {
  const auto& x=Member(v,key); io::Require(x.IsUint64(),"Expected exact report integer"); return x.GetUint64();
}
inline bool Boolean(const io::Value& v,const char* key) {
  const auto& x=Member(v,key); io::Require(x.IsBool(),"Expected report boolean"); return x.GetBool();
}
inline std::string String(const io::Value& v,const char* key) {
  const auto& x=Member(v,key); io::Require(x.IsString(),"Expected report string"); return {x.GetString(),x.GetStringLength()};
}
inline const io::Value& Array(const io::Value& v,const char* key,std::size_t count) {
  const auto& a=Member(v,key); io::Require(a.IsArray()&&a.Size()==count,"Report array cardinality mismatch"); return a;
}
inline bool Hash(const std::string& h) { return h.size()==64&&h.find_first_not_of("0123456789abcdef")==std::string::npos; }
inline void ReadArray(const io::Value& v,const char* key,double* output,std::size_t count) {
  const auto& a=Array(v,key,count);
  for(unsigned i=0;i<count;++i) { io::Require(a[i].IsNumber()&&std::isfinite(a[i].GetDouble()),"Nonfinite array member"); output[i]=a[i].GetDouble(); }
}
inline void Push(io::Document& d,io::Value& a,const io::Document& child) {
  io::Value value; value.CopyFrom(child,d.GetAllocator()); a.PushBack(value,d.GetAllocator());
}
inline void Object(io::Document& d,const char* key,const io::Document& child) {
  io::Value value; value.CopyFrom(child,d.GetAllocator()); d.AddMember(rapidjson::Value(key,d.GetAllocator()),value,d.GetAllocator());
}
io::Document Parse(const std::string& bytes);
void Bind(io::Document&,const Binding&);
Binding ReadBinding(const io::Value&);
io::Document LimitsReport(const Limits&);
Limits ReadLimits(const io::Value&);
io::Document SampleReport(const Sample&,unsigned fields);
Sample ReadSample(const io::Value&,unsigned fields);
} // namespace tl::qualification::qeph::response::protocol
