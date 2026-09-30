// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Origins.h"
#include "../../source_surfaces/Internal.h"
namespace tlfea::contact::radioss_type25::runtime_detail::physical_main {
bool Origin(const Index& index,const startup::PrimaryFaceIdentity& origin,const Face& actual) noexcept {
  if(origin.origin!=startup::PrimaryOrigin::SingleSourceFace||origin.origin_count!=1||!origin.physical_parent_id)
    return false;
  if(origin.kind==startup::PrimaryFaceKind::Shell) {
    Face expected;bool triangle=false;
    return !origin.local_face&&index.Shell(origin.physical_parent_id,expected,triangle)&&SameFace(expected,actual);
  }
  if(origin.kind!=startup::PrimaryFaceKind::Solid||!origin.local_face||origin.local_face>6)return false;
  std::array<std::uint32_t,8> nodes;
  if(!index.Solid(origin.physical_parent_id,nodes))return false;
  source_surfaces::Solid source;
  for(unsigned slot=0;slot<8;++slot)source.nodes[slot]=nodes[slot];
  Face expected{};
  const auto count=source_surfaces::detail::CompactFace(source,origin.local_face-1,expected.data());
  if(count<3)return false;
  if(count==3)expected[3]=expected[2];
  return SameFace(expected,actual);
}
bool SolidSupport(const Index& index,std::uint64_t id,const Face& face) noexcept {
  std::array<std::uint32_t,8> nodes;
  return id&&index.Solid(id,nodes)&&Contains(nodes.data(),nodes.size(),face);
}
bool Support(const Index& index,const startup::PostGapmMainSupport& support,const Face& face) noexcept {
  using Kind=startup::PhysicalSupportKind;
  if(support.first.kind==Kind::EightSlotSolid) {
    if(!SolidSupport(index,support.first.source_element_id,face))return false;
    return !support.second_solid_source_id||
      (support.second_solid_source_id!=support.first.source_element_id&&
       SolidSupport(index,support.second_solid_source_id,face));
  }
  if(support.second_solid_source_id||
      (support.first.kind!=Kind::ShellQuad&&support.first.kind!=Kind::ShellTriangle))return false;
  Face nodes;bool triangle=false;
  if(!index.Shell(support.first.source_element_id,nodes,triangle)||
      triangle!=(support.first.kind==Kind::ShellTriangle))return false;
  // INCOQ3 may select a nonselected Q4 containing a triangular query. This
  // authenticates its real incidence; the source producer owns ranking/ties.
  return Contains(nodes.data(),triangle?3:4,face);
}
} // namespace tlfea::contact::radioss_type25::runtime_detail::physical_main
