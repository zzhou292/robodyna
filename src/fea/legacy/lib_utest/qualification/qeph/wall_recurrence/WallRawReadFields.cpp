#include "WallRawReadFields.h"
#include <cmath>
#include <cstring>
#include <set>

namespace tl::qualification::qeph::wall_recurrence::read_detail {
const io::Value& Field(const io::Value& v,const char* name) {
  io::Require(v.IsObject()&&v.HasMember(name),(std::string("Missing raw field: ")+name).c_str()); return v[name];
}
std::uint64_t Unsigned(const io::Value& v) { io::Require(v.IsUint64(),"Expected unsigned raw integer"); return v.GetUint64(); }
unsigned Count(const io::Value& v,unsigned cap) { const auto n=Unsigned(v); io::Require(n<=cap,"Raw count exceeds capacity"); return n; }
double Number(const io::Value& v) {
  io::Require(v.IsNumber()&&std::isfinite(v.GetDouble()),"Expected finite raw number"); return v.GetDouble();
}
bool Boolean(const io::Value& v) { io::Require(v.IsBool(),"Expected raw boolean"); return v.GetBool(); }
std::string Text(const io::Value& v,std::size_t cap) {
  io::Require(v.IsString()&&v.GetStringLength()<=cap,"Invalid raw text capacity"); return {v.GetString(),v.GetStringLength()};
}
void Keys(const io::Value& v,std::initializer_list<const char*> names) {
  io::Require(v.IsObject()&&v.MemberCount()==names.size(),"Unexpected raw object fields");
  for(const char* name:names) (void)Field(v,name);
}
namespace {
void Structure(const io::Value& v,unsigned depth,std::size_t& count) {
  io::Require(depth<=32&&++count<=262144,"Unbounded raw JSON structure");
  if(v.IsObject()) {
    std::set<std::string> keys;
    for(auto it=v.MemberBegin();it!=v.MemberEnd();++it) {
      io::Require(keys.emplace(it->name.GetString(),it->name.GetStringLength()).second,"Duplicate raw JSON key");
      Structure(it->value,depth+1,count);
    }
  } else if(v.IsArray()) for(const auto& x:v.GetArray()) Structure(x,depth+1,count);
  else if(v.IsNumber()) (void)Number(v);
}
}
io::Document Parse(const std::string& bytes) {
  io::Require(!bytes.empty()&&bytes.size()<=RawFileByteCap,"Invalid raw JSON byte count");
  io::Document d; d.Parse<rapidjson::kParseFullPrecisionFlag|rapidjson::kParseIterativeFlag>(bytes.data(),bytes.size());
  io::Require(!d.HasParseError()&&d.IsObject(),"Malformed raw JSON object");
  std::size_t count=0; Structure(d,0,count); return d;
}
bool Same(const io::Value& a,const io::Value& b) {
  if(a.IsObject()) {
    if(!b.IsObject()||a.MemberCount()!=b.MemberCount()) return false;
    for(auto it=a.MemberBegin();it!=a.MemberEnd();++it)
      if(!b.HasMember(it->name)||!Same(it->value,b[it->name])) return false;
    return true;
  }
  if(a.IsArray()) {
    if(!b.IsArray()||a.Size()!=b.Size()) return false;
    for(rapidjson::SizeType i=0;i<a.Size();++i) if(!Same(a[i],b[i])) return false;
    return true;
  }
  if(a.IsDouble()) return b.IsNumber()&&io::Bits(a.GetDouble())==io::Bits(b.GetDouble());
  if(a.IsUint64()) return b.IsUint64()&&a.GetUint64()==b.GetUint64();
  if(a.IsInt64()) return b.IsInt64()&&a.GetInt64()==b.GetInt64();
  if(a.IsBool()) return b.IsBool()&&a.GetBool()==b.GetBool();
  if(a.IsString()) return b.IsString()&&a.GetStringLength()==b.GetStringLength()&&
    std::memcmp(a.GetString(),b.GetString(),a.GetStringLength())==0;
  return a.IsNull()&&b.IsNull();
}
bool Hash(const std::string& s) { return s.size()==64&&s.find_first_not_of("0123456789abcdef")==std::string::npos; }
double Scalar(const io::Value& v,bool partial) {
  if(v.IsNumber()) return Number(v);
  io::Require(partial,"Nonfinite marker in completed raw operand");
  Keys(v,{"nonfinite","binary64_bits"}); const auto bits=Unsigned(Field(v,"binary64_bits"));
  double value=0; std::memcpy(&value,&bits,sizeof(value));
  io::Require(!std::isfinite(value),"Finite bits in nonfinite raw marker");
  const auto kind=Text(Field(v,"nonfinite"));
  io::Require(kind==(std::isnan(value)?"nan":(value>0?"positive-infinity":"negative-infinity")),"Nonfinite kind/bits mismatch");
  return value;
}
Eigen::VectorXd Vector(const io::Value& v,bool partial) {
  io::Require(v.IsArray()&&v.Size()<=194,"Invalid raw vector length"); Eigen::VectorXd out(v.Size());
  for(rapidjson::SizeType i=0;i<v.Size();++i) out[i]=Scalar(v[i],partial); return out;
}
Eigen::MatrixXd Matrix(const io::Value& v,bool partial) {
  Keys(v,{"rows","columns","all_finite","values_rows"});
  const unsigned rows=Count(Field(v,"rows"),194),columns=Count(Field(v,"columns"),194);
  const auto& data=Field(v,"values_rows"); io::Require(data.IsArray()&&data.Size()==rows,"Raw matrix row mismatch");
  Eigen::MatrixXd out(rows,columns);
  for(unsigned r=0;r<rows;++r) {
    io::Require(data[r].IsArray()&&data[r].Size()==columns,"Raw matrix column mismatch");
    for(unsigned c=0;c<columns;++c) out(r,c)=Scalar(data[r][c],partial);
  }
  io::Require(out.allFinite()==Boolean(Field(v,"all_finite")),"Changed raw matrix finiteness flag"); return out;
}
contact::Q4CertifiedIntegral Certificate(const io::Value& v) {
  Keys(v,{"value","lower","upper","error"});
  contact::Q4CertifiedIntegral out{Number(Field(v,"value")),Number(Field(v,"lower")),Number(Field(v,"upper")),Number(Field(v,"error"))};
  io::Require(contact::nodal_wall_detail::Certificate(out),"Invalid owning raw certificate"); return out;
}
} // namespace tl::qualification::qeph::wall_recurrence::read_detail
