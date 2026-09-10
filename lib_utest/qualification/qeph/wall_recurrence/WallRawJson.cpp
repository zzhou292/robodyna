#include "WallRawJson.h"
#include <cmath>

namespace tl::qualification::qeph::wall_recurrence::raw_detail {
void Field(io::Document& d,const char* name,const io::Document& child) {
  io::Value value; value.CopyFrom(child,d.GetAllocator());
  d.AddMember(io::Value(name,d.GetAllocator()),value,d.GetAllocator());
}
void Append(io::Document& d,io::Value& array,const io::Document& child) {
  io::Value value; value.CopyFrom(child,d.GetAllocator()); array.PushBack(value,d.GetAllocator());
}
io::Value Scalar(io::Document& d,double value,bool allow_nonfinite) {
  io::Value result;
  if(std::isfinite(value)) { result.SetDouble(value); return result; }
  io::Require(allow_nonfinite,"Nonfinite value in completed raw operand");
  io::Document marker; marker.SetObject();
  io::String(marker,"nonfinite",std::isnan(value)?"nan":(value>0?"positive-infinity":"negative-infinity"));
  io::Integer(marker,"binary64_bits",io::Bits(value)); result.CopyFrom(marker,d.GetAllocator()); return result;
}
io::Document Matrix(const Eigen::MatrixXd& matrix,bool allow_nonfinite) {
  io::Require(matrix.rows()>=0&&matrix.cols()>=0&&matrix.rows()<=194&&matrix.cols()<=194,"Unbounded raw matrix");
  io::Document d; d.SetObject(); io::Integer(d,"rows",matrix.rows()); io::Integer(d,"columns",matrix.cols());
  io::Boolean(d,"all_finite",matrix.allFinite()); io::Value rows(rapidjson::kArrayType);
  for(Eigen::Index r=0;r<matrix.rows();++r) {
    io::Value row(rapidjson::kArrayType);
    for(Eigen::Index c=0;c<matrix.cols();++c) row.PushBack(Scalar(d,matrix(r,c),allow_nonfinite),d.GetAllocator());
    rows.PushBack(row,d.GetAllocator());
  }
  d.AddMember("values_rows",rows,d.GetAllocator()); return d;
}
io::Value Vector(io::Document& d,const Eigen::VectorXd& vector,bool allow_nonfinite) {
  io::Require(vector.size()<=194,"Unbounded raw state vector");
  if(!allow_nonfinite) return io::FiniteArray(d,vector.data(),vector.size());
  io::Value result(rapidjson::kArrayType);
  for(double v:vector) result.PushBack(Scalar(d,v,true),d.GetAllocator());
  return result;
}
io::Document Certificate(contact::Q4CertifiedIntegral a) {
  io::Require(contact::nodal_wall_detail::Certificate(a),"Invalid retained owning certificate");
  io::Document d; d.SetObject(); io::Number(d,"value",a.value); io::Number(d,"lower",a.lower);
  io::Number(d,"upper",a.upper); io::Number(d,"error",a.error); return d;
}
io::Document Interval(contact::Q4IntegralInterval a) {
  io::Require(contact::q4_bounds::Finite(a),"Invalid retained interval");
  io::Document d; d.SetObject(); io::Number(d,"lower",a.lower); io::Number(d,"upper",a.upper); return d;
}
} // namespace tl::qualification::qeph::wall_recurrence::raw_detail
