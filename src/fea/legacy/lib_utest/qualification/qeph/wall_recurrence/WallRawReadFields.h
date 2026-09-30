#pragma once
#include "WallRawRead.h"
#include "WallRawJson.h"
#include <initializer_list>

namespace tl::qualification::qeph::wall_recurrence::read_detail {
namespace io=crash::output;
const io::Value& Field(const io::Value&,const char*);
std::uint64_t Unsigned(const io::Value&);
unsigned Count(const io::Value&,unsigned maximum);
double Number(const io::Value&);
bool Boolean(const io::Value&);
std::string Text(const io::Value&,std::size_t maximum=4096);
void Keys(const io::Value&,std::initializer_list<const char*>);
io::Document Parse(const std::string&);
bool Same(const io::Value&,const io::Value&);
bool Hash(const std::string&);
double Scalar(const io::Value&,bool allow_nonfinite=false);
Eigen::VectorXd Vector(const io::Value&,bool allow_nonfinite=false);
Eigen::MatrixXd Matrix(const io::Value&,bool allow_nonfinite=false);
contact::Q4CertifiedIntegral Certificate(const io::Value&);
template<class V> V Vec(const io::Value& v) {
  io::Require(v.IsArray()&&v.Size()==3,"Invalid retained three-vector");
  return {Number(v[0]),Number(v[1]),Number(v[2])};
}
MovingMatrixProbe Native(const io::Value&,unsigned cells,double h,double boost,unsigned amplitude);
ContactBranchProbe Branch(const io::Value&,unsigned cells,double h,double boost,unsigned branch);
ContactMapSample Sample(const io::Value&,unsigned cells,bool required);
void Header(const io::Document&,const char* kind,const RawReadBinding&,
            const RawFileReceipt& provenance,const std::string& origin,const std::string& model_hash);
void Inventory(const io::Document&,const RawJob*,const RawJobReceipt& prefix,
               const RawReadBinding&,bool final);
} // namespace tl::qualification::qeph::wall_recurrence::read_detail
