// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
namespace tlfea::contact::radioss_type25::surface_interface::detail {
Report Classify(const Input& in,source::Work context,const Vector* points,Data output) noexcept {
  for(std::size_t i=0;i<in.raw_face_count;++i) {
    const auto& face=in.raw_faces[i];
    RawClassification result;result.role=face.raw_role;
    if(face.raw_role==1) {
      output.solid_flags[face.source.reader_row]=1;
      output.classifications[i]=result;
      continue;
    }
    // Exact IN24 first-candidate order from full corner-major raw8 incidence.
    const auto node=face.nodes[0];
    for(auto at=context.solid_offsets[node];at<context.solid_offsets[node+1];++at) {
      const auto row=context.solid_rows[at];
      const auto& solid=in.physical.solids[row];
      bool all=true;
      for(unsigned j=0;j<4;++j) {
        bool found=false;
        for(unsigned k=0;k<8;++k)found=found||solid.nodes[k]==face.nodes[j];
        all=all&&found;
      }
      if(!all)continue;
      startup::NativeCoatingOrientationInput geometry;geometry.node_count=8;
      for(unsigned k=0;k<8;++k)geometry.positions[k]=points[solid.nodes[k]];
      for(unsigned j=0;j<3;++j) {
        unsigned k=0;
        while(k<8&&solid.nodes[k]!=face.nodes[j])++k;
        geometry.segment_slots[j]=k;
      }
      startup::NativeCoatingOrientationResult orientation;
      const auto status=startup::EvaluateNativeCoatingOrientation(geometry,&orientation);
      if(status!=startup::Status::Ok)return {Status::NonfiniteResult,i,row};
      result.role=face.raw_role+1;
      if(orientation.orientation==startup::CoatingOrientation::Reversed)result.role=-result.role;
      result.matched_solid=row;
      break;
    }
    output.classifications[i]=result;
  }
  return {Status::Ok};
}
}
