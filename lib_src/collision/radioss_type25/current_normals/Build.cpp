// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Layout.h"
#include "Admission.h"
#include <cstring>
namespace tlfea::contact::radioss_type25::current_normals {
Report Preflight(const Input& in,Limits limits,Forecast& out) noexcept {
  detail::Layout layout;double length=1;const auto result=detail::Plan(in,limits,layout,length);
  if(result.status!=Status::Ok)return result;
  if(!geometry_detail::RangeValid({&out,sizeof(out),alignof(Forecast)})||
     !detail::InputDisjoint(in,&out,sizeof(out)))return {Status::InvalidInput};
  out=layout.forecast;return result;
}
Report Evaluate(const Input& in,Limits limits,void* scratch,std::size_t bytes,Output output) noexcept {
  detail::Layout layout;double length=1;auto report=detail::Plan(in,limits,layout,length);
  if(report.status!=Status::Ok)return report;
  report=detail::Storage(in,layout,scratch,bytes,output);if(report.status!=Status::Ok)return report;
  const auto work=detail::Construct(scratch,layout);const auto& t=in.topology;
  std::memcpy(work.normal,in.prior_normals,in.prior_count*sizeof(StoredNormal));
  for(std::size_t i=0;i<t.primary_count;++i){report=detail::Primary(in,work,i,length);if(report.status!=Status::Ok)return report;}
  for(std::size_t i=0;i<in.free_count;++i)detail::FreeEligibility(in,work,i);
  for(std::size_t i=0;i<t.references;++i)detail::ReferenceSlots(in,work,i);
  for(std::size_t i=0;i<in.free_count;++i){report=detail::TransformFree(in,work,i,length);if(report.status!=Status::Ok)return report;}
  for(std::size_t i=0;i<t.references;++i)detail::FillReference(work,i);
  for(std::size_t i=0;i<t.main_count;++i)detail::GatherNeighbor(in,work,i);
  for(std::size_t i=0;i<t.main_count;++i){report=detail::Average(in,work,i);if(report.status!=Status::Ok)return report;}
  // One host publication fence after every original stage succeeds. The CUDA
  // owner follows the same stage dependencies on private trial slabs.
  std::memcpy(output.face_normals,work.normal,output.normal_count*sizeof(StoredNormal));
  std::memcpy(output.references,work.references,output.reference_count*sizeof(startup::NormalReference));
  return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::current_normals
