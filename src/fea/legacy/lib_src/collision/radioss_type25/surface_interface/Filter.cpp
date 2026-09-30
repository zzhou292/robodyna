// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include <algorithm>
namespace tlfea::contact::radioss_type25::surface_interface::detail {
namespace {
bool Less(const Key& a,const Key& b) {
  for(unsigned k=0;k<6;++k)if(a.words[k]!=b.words[k])return a.words[k]<b.words[k];
  return a.raw<b.raw;
}
bool Same(const Key& a,const Key& b) {
  for(unsigned k=0;k<5;++k)if(a.words[k]!=b.words[k])return false;
  return true;
}
}
Report Filter(const Input& in,Data out,Key* keys,std::size_t& primaries,std::size_t& shells) noexcept {
  for(std::size_t i=0;i<in.raw_face_count;++i) {
    auto& key=keys[i];key.raw=static_cast<std::uint32_t>(i);
    for(unsigned k=0;k<4;++k)key.words[k]=in.raw_faces[i].nodes[k]+1;
    const auto& source=in.raw_faces[i].source;
    out.raw_origins[i]={source.kind==source_surfaces::ParentKind::Solid?startup::PrimaryFaceKind::Solid:
        startup::PrimaryFaceKind::Shell,source.element_id,source.solid_face};
    const auto role=out.classifications[i].role;
    key.words[4]=static_cast<std::uint32_t>(role);key.words[5]=0; // IMBIN0.
    if(role==0||role==1) {
      if(key.words[3]==key.words[2])key.words[3]=0;
      // Original selection-sort excludes the zero T3 filler from each minimum.
      for(unsigned j=0;j<3;++j) {
        unsigned minimum=j;
        for(unsigned k=j+1;k<4;++k)
          if(key.words[k]&&key.words[k]<key.words[minimum])minimum=k;
        if(minimum!=j)std::swap(key.words[minimum],key.words[j]);
      }
    }
  }
  std::sort(keys,keys+in.raw_face_count,Less);
  primaries=0;shells=0;
  for(std::size_t first=0;first<in.raw_face_count;) {
    std::size_t last=first+1;
    while(last<in.raw_face_count&&Same(keys[first],keys[last]))++last;
    const auto winner=keys[first].raw;
    const auto& face=in.raw_faces[winner];
    const int role=out.classifications[winner].role;
    const bool solid=role==0||role==1;
    auto& primary=out.primary[primaries];
    const bool multiple=last-first>1;
    primary.source_id=multiple?0:face.source.element_id;
    primary.layout=face.nodes[2]==face.nodes[3]?ShellLayout::Triangle3:ShellLayout::Quad4;
    std::copy_n(face.nodes,4,primary.nodes); // Saved pre-canonical orientation.
    primary.side_role=role<0?startup::ShellSideRole::CoatingReversed:
        (role==4||role==8)?startup::ShellSideRole::CoatingForward:startup::ShellSideRole::Ordinary;
    out.identities[primaries]={solid?startup::PrimaryFaceKind::Solid:startup::PrimaryFaceKind::Shell,
        multiple?0:face.source.element_id,static_cast<std::uint8_t>(multiple?0:face.source.solid_face),
        multiple?startup::PrimaryOrigin::MultipleOrigins:startup::PrimaryOrigin::SingleSourceFace,
        static_cast<std::uint32_t>(last-first)};
    out.primary_to_raw[primaries]=winner;
    for(auto k=first;k<last;++k)out.raw_to_primary[keys[k].raw]=static_cast<std::uint32_t>(primaries);
    ++primaries;
    if(!solid)++shells;
    first=last;
  }
  return {Status::Ok};
}
}
