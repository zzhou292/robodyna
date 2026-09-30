#pragma once
#include "WallResponseReport.h"
#include "../response/ResponseProtocol.h"
#include "../wall_recurrence/WallRawJson.h"
#include "../wall_recurrence/WallRawReadFields.h"
#include <array>
#include <limits>

namespace tl::qualification::qeph::wall_response::json {
namespace p=free_response::protocol;
namespace raw=wr::raw_detail;
namespace read=wr::read_detail;
inline constexpr const char* PhaseContract="x/q/cache/history/contact at endpoint; carried v/omega at endpoint-h/2 except physical initial +8 m/s; sync output uses h/2 and total internal-plus-contact RHS";
inline constexpr const char* EnergyContract="Ksync+EINT0+EINT1+EVIS+Uc-K0; contact impulse and kick/drift ledgers remain separate; no second subtraction of contact work";
template<class T> struct NumberField { const char* name; double T::*member; };
template<class T,std::size_t N>
void WriteNumbers(io::Document& d,const T& x,const NumberField<T>(&fields)[N]) {
  for(const auto& f:fields) io::Number(d,f.name,x.*(f.member));
}
template<class T,std::size_t N>
void ReadNumbers(const io::Value& d,T& x,const NumberField<T>(&fields)[N]) {
  for(const auto& f:fields) x.*(f.member)=p::Number(d,f.name);
}
template<std::size_t N>
void Array(io::Document& d,const char* key,const std::array<double,N>& x,std::size_t count=N) {
  io::Require(count<=N,"Array count exceeds fixed wall-response storage");
  io::FiniteArray(d,key,x.data(),count);
}
template<std::size_t N>
void ReadArray(const io::Value& d,const char* key,std::array<double,N>& x,std::size_t count=N) {
  io::Require(count<=N,"Array count exceeds fixed wall-response storage");
  p::ReadArray(d,key,x.data(),count);
}
inline unsigned Count(const io::Value& d,const char* key,unsigned maximum) {
  const auto value=p::Integer(d,key); io::Require(value<=maximum,"Wall-response count exceeds bound");
  return static_cast<unsigned>(value);
}
inline Interval ReadInterval(const io::Value& d,const char* key) {
  const auto& v=p::Member(d,key); Interval out{p::Number(v,"lower"),p::Number(v,"upper")};
  io::Require(out.lower<=out.upper,"Reversed wall-response interval"); return out;
}
inline void IntervalField(io::Document& d,const char* key,Interval value) {
  p::Object(d,key,raw::Interval(value));
}
io::Document SampleReport(const Sample&,const Model&);
Sample ReadSample(const io::Value&,const Model&);
io::Document SummaryReport(const Summary&);
Summary ReadSummary(const io::Value&);
io::Document ModelReport(const Model&);
void Bind(io::Document&,const ReportBinding&);
ReportBinding ReadBinding(const io::Value&);
void RunMetadata(io::Document&,const Run&);
void ReadRunMetadata(const io::Value&,Run&);
} // namespace tl::qualification::qeph::wall_response::json
