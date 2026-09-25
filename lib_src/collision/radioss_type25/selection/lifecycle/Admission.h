// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Row.h"
namespace tlfea::contact::radioss_type25::selection::lifecycle::detail {
template<class T> TL_MATH_HOST_DEVICE inline bool Span(const T* data,std::size_t count) {
  return count<=SIZE_MAX/sizeof(T)&&g::RangeValid({data,count*sizeof(T),alignof(T)});
}
TL_MATH_HOST_DEVICE inline bool VectorSpan(VectorView view,std::size_t nodes) {
  if(!nodes)return view.node_count==0;
  if(!view.valid()||view.node_count!=nodes)return false;
  const auto last=(view.node_count-1)*view.node_stride+2*view.component_stride;
  return last<SIZE_MAX&&Span(view.data,std::size_t(last)+1);
}
TL_MATH_HOST_DEVICE inline bool CsrValid(Csr csr,std::size_t rows,
    std::size_t upper,bool positive) {
  if(csr.offset_count!=rows+1||!Span(csr.offsets,csr.offset_count)||
     !Span(csr.entries,csr.entry_count)||!csr.offsets||csr.offsets[0]!=0||
     csr.offsets[rows]!=csr.entry_count||csr.entry_count>UINT32_MAX||
     (csr.entry_count&&!csr.entries))return false;
  for(std::size_t row=0;row<rows;++row)
    if(csr.offsets[row]>csr.offsets[row+1]||csr.offsets[row+1]>csr.entry_count)return false;
  for(std::size_t i=0;i<csr.entry_count;++i)
    if(positive?(csr.entries[i]==0||csr.entries[i]>upper):(csr.entries[i]>=upper))return false;
  return true;
}
TL_MATH_HOST_DEVICE inline Status Validate(const Input& input) {
  const auto& source=input.source;const auto& profile=input.profile;
  if(!values::Supported(profile.selection)||!g::Supported(profile.geometry)||
     profile.coefficient.stiffness_formulation!=4||
     profile.coefficient.mass_timestep_augmentation!=0||profile.neighbor_removal!=2||
     profile.optcd_response_precision<0||profile.optcd_response_precision>2)return Status::UnsupportedProfile;
  if(!normal_detail::Nonnegative(profile.minimum_coefficient)||
     !normal_detail::Nonnegative(profile.maximum_coefficient)||
     profile.minimum_coefficient>profile.maximum_coefficient||
     !normal_detail::Nonnegative(input.step.time)||!normal_detail::Nonnegative(input.step.previous_dt))
    return Status::InvalidInput;
  units_detail::Factors units;
  if(!Factors(input.current,units)||source.node_count>UINT32_MAX||
     source.main_count>INT_MAX||source.secondary_count>INT_MAX||source.normal_count>INT_MAX||
     input.spatial_count>UINT32_MAX||
     input.accepted_row_count!=source.secondary_count||
     (source.node_count&&!source.nodes)||(source.main_count&&!source.mains)||
     (source.secondary_count&&(!source.secondary||!input.accepted_rows))||
     (!HasCurrentNormals(input)&&source.normal_count&&!source.normals)||(input.spatial_count&&!input.spatial))return Status::InvalidInput;
  if(!Span(source.nodes,source.node_count)||!Span(source.mains,source.main_count)||
     !Span(source.secondary,source.secondary_count)||
     (!HasCurrentNormals(input)&&!Span(source.normals,source.normal_count))||!CurrentNormalShape(input)||
     !Span(input.accepted_rows,input.accepted_row_count)||!Span(input.spatial,input.spatial_count)||
     !VectorSpan(input.current.positions,source.node_count)||!VectorSpan(input.current.velocities,source.node_count))
    return Status::InvalidInput;
  for(std::size_t i=0;i<source.node_count;++i) {
    const auto& node=source.nodes[i];
    if(!node.source_id||node.constraint<0||node.skew<0||
       !tl::math::fixed3::Finite(Position(input,std::uint32_t(i),units))||
       !tl::math::fixed3::Finite(Velocity(input,std::uint32_t(i),units)))return Status::InvalidInput;
  }
  for(std::size_t i=0;i<source.normal_count;++i)if(ReferenceNormal(input,i).boundary)
    for(const auto& normal:ReferenceNormal(input,i).bisector)
      if(!tl::math::fixed3::Finite(g::Promote(normal)))return Status::InvalidInput;
  for(std::size_t i=0;i<source.main_count;++i) {
    const auto& main=source.mains[i];
    if(main.global_id<=0||main.segment_type==INT_MIN||
       std::int64_t(main.segment_type)<-2*std::int64_t(source.main_count)||
       std::int64_t(main.segment_type)>2*std::int64_t(source.main_count)||
       !normal_detail::Nonnegative(main.coefficient)||!normal_detail::Nonnegative(main.maximum_gap))
      return Status::InvalidInput;
    for(unsigned j=0;j<4;++j)
      if(main.nodes[j]>=source.node_count||main.normal_reference[j]<=0||
         std::size_t(main.normal_reference[j])>source.normal_count||
         !tl::math::fixed3::Finite(g::Promote(FaceNormal(input,i,j)))||
         !normal_detail::Nonnegative(main.gap[j]))return Status::InvalidInput;
  }
  for(std::size_t row=0;row<source.secondary_count;++row) {
    const auto& secondary=source.secondary[row];const auto& accepted=input.accepted_rows[row];
    if(secondary.node>=source.node_count||!normal_detail::Nonnegative(secondary.coefficient)||
       !normal_detail::Nonnegative(secondary.gap)||!g::ValidRow(accepted.row)||
       accepted.row.irtlm[0]==INT_MIN||
       accepted.secondary_source_id!=source.nodes[secondary.node].source_id||
       accepted.generation!=source.generation)return Status::InvalidInput;
  }
  if(!CsrValid(source.normal_to_main,source.normal_count,source.main_count,true)||
     !CsrValid(source.removed_main_by_secondary,source.secondary_count,source.main_count,true)||
     !CsrValid(input.spatial_by_secondary,source.secondary_count,input.spatial_count,false))
    return Status::InvalidInput;
  if(input.spatial_by_secondary.entry_count!=input.spatial_count)return Status::InvalidInput;
  for(std::size_t row=0;row<source.secondary_count;++row) {
    const auto& csr=input.spatial_by_secondary;std::uint32_t previous=0;
    for(std::size_t i=csr.offsets[row];i<csr.offsets[row+1];++i) {
      const auto ordinal=csr.entries[i];const auto& candidate=input.spatial[ordinal];
      if(i>csr.offsets[row]&&ordinal<=previous)return Status::InvalidInput;
      previous=ordinal;
      if(candidate.secondary!=int(row+1)||candidate.local_main<=0||
         std::size_t(candidate.local_main)>source.main_count)return Status::InvalidInput;
    }
  }
  // Strictly ordered per-row entries + matching secondary + exact total prove
  // the CSR is a bijection, without a dense per-main/per-secondary bitmap.
  return Status::Ok;
}
struct RowRequirements {Status status=Status::Ok;std::size_t occurrences=0,sliding=0;};
TL_MATH_HOST_DEVICE inline RowRequirements Requirements(const Input& input,std::size_t row) {
  RowRequirements out;RowResult begun;
  out.status=BeginRow(input,row,begun);if(out.status!=Status::Ok)return out;
  if(begun.retained_count) {
    const auto& main=input.source.mains[begun.history.row.irtlm[2]-1];
    const auto& csr=input.source.normal_to_main;
    for(int reference:main.normal_reference) {
      const auto count=std::size_t(csr.offsets[reference]-csr.offsets[reference-1]);
      if(count>SIZE_MAX-out.sliding){out.status=Status::CapacityExceeded;return out;}
      out.sliding+=count;
    }
  }
  const auto raw=std::size_t(input.spatial_by_secondary.offsets[row+1]-input.spatial_by_secondary.offsets[row]);
  if(raw>SIZE_MAX-begun.retained_count||out.sliding>SIZE_MAX-raw-begun.retained_count)
    out.status=Status::CapacityExceeded;
  else out.occurrences=raw+begun.retained_count+out.sliding;
  return out;
}
} // namespace tlfea::contact::radioss_type25::selection::lifecycle::detail
