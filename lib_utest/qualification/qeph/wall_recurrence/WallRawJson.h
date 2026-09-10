#pragma once
#include "WallRawCapture.h"
#include "output/ArtifactIO.h"

namespace tl::qualification::qeph::wall_recurrence::raw_detail {
namespace io=crash::output;
void Field(io::Document&,const char* name,const io::Document&);
void Append(io::Document&,io::Value&,const io::Document&);
io::Value Scalar(io::Document&,double,bool allow_nonfinite);
io::Document Matrix(const Eigen::MatrixXd&,bool allow_nonfinite=false);
io::Value Vector(io::Document&,const Eigen::VectorXd&,bool allow_nonfinite=false);
io::Document Certificate(contact::Q4CertifiedIntegral);
io::Document Interval(contact::Q4IntegralInterval);
template<class V> void Vec(io::Document& d,const char* name,const V& v) {
  const double values[]{v.x,v.y,v.z}; io::FiniteArray(d,name,values,3);
}
template<class I> void Indices(io::Document& d,const char* name,const I* ids,std::size_t count) {
  io::Require(count<=194,"Unbounded raw index array");
  io::Value array(rapidjson::kArrayType);
  for(std::size_t i=0;i<count;++i) { io::Value v; v.SetUint64(ids[i]); array.PushBack(v,d.GetAllocator()); }
  d.AddMember(io::Value(name,d.GetAllocator()),array,d.GetAllocator());
}
io::Document DescribeModel(const WallRecurrenceModel&);
io::Document DescribeContactSample(const ContactMapSample&,unsigned cells,bool required);
io::Document DescribePoint(const contact::NodalWallPointResult&);
io::Document DescribeParent(const contact::NodalWallParentResult&);
io::Document DescribeNative(const MovingMatrixProbe&,unsigned cells,double h,double velocity,unsigned amplitude);
io::Document DescribeBranch(const ContactBranchProbe&,unsigned cells,double h,double velocity,unsigned branch);
} // namespace tl::qualification::qeph::wall_recurrence::raw_detail
