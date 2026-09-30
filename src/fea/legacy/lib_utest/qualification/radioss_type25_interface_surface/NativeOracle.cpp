// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <array>
#include <cmath>
#include <mutex>
#include <set>
#include <stdexcept>
namespace type25_interface_surface_test {
namespace {
constexpr std::size_t Nodes=256,Rows=64,Faces=256,Mains=512;
std::mutex native_mutex;
extern "C" void rd_interface_source(const int*,const double*,const int*,const int*,const int*,
    const int*,const int*,const int*,int*,int*,int*,int*,int*,int*,int*,int*,int*,int*);
void Need(bool condition,const char* message) {
  if(!condition)throw std::invalid_argument(message);
}
template<class T> void Span(const T* pointer,std::size_t count,std::size_t cap) {
  Need(count<=cap&&(!count||pointer),"Native interface span/cap");
}
s::PrimaryFaceIdentity Origin(const n::source_surfaces::Face& face) {
  return {face.source.kind==n::source_surfaces::ParentKind::Solid?s::PrimaryFaceKind::Solid:s::PrimaryFaceKind::Shell,
      face.source.element_id,face.source.solid_face};
}
}
NativeResult Oracle(const f::Input& in) {
  namespace physical=n::source_surfaces;
  const auto& p=in.physical;
  Need(in.profile==f::Profile::SingleSurfaceIlev1&&
      p.phase==physical::ReaderPhase::BeforeGroupingAndInitia&&in.source_generation&&
      p.node_count&&p.node_count<=Nodes&&in.raw_face_count&&in.raw_face_count<=Faces,
      "Native interface source profile");
  Span(p.solids,p.solid_count,Rows);
  Span(p.quads,p.quad_count,Rows);
  Span(p.triangles,p.triangle_count,Rows);
  Span(in.raw_faces,in.raw_face_count,Faces);
  Need(in.positions.valid()&&in.positions.node_count==p.node_count,"Native interface coordinate view");
  double length=1;
  if(in.coordinates==s::Coordinates::Si) {
    Need(std::isfinite(in.units.length_m)&&in.units.length_m>0,"Native interface SI length");
    length=in.units.length_m;
  } else Need(in.coordinates==s::Coordinates::Native,"Native interface coordinate profile");
  std::array<double,3*Nodes> x{};
  for(std::size_t i=0;i<p.node_count;++i) {
    const auto value=in.positions.at(static_cast<std::uint32_t>(i));
    x[3*i]=value.x/length;x[3*i+1]=value.y/length;x[3*i+2]=value.z/length;
    for(unsigned k=0;k<3;++k)Need(std::isfinite(x[3*i+k]),"Native interface nonfinite coordinate");
  }
  std::array<int,8*Rows> solids{};
  std::array<int,4*Rows> quads{};
  std::array<int,3*Rows> triangles{};
  for(std::size_t i=0;i<p.solid_count;++i) {
    const auto& row=p.solids[i];std::set<std::uint32_t> unique;
    Need(row.element_id&&row.part_id,"Native interface solid identity");
    for(unsigned k=0;k<8;++k) {
      Need(row.nodes[k]<p.node_count,"Native interface solid node");
      solids[8*i+k]=int(row.nodes[k]+1);unique.insert(row.nodes[k]);
    }
    Need(row.topology==physical::SolidTopology::NativeRaw8 ||
        (row.topology==physical::SolidTopology::Hex8&&unique.size()==8) ||
        (row.topology==physical::SolidTopology::DeclaredPenta6&&unique.size()==6&&
         row.nodes[3]==row.nodes[0]&&row.nodes[7]==row.nodes[4]),"Native interface raw8 shape");
  }
  const auto shells=[&](const physical::Shell* rows,std::size_t count,unsigned width,int* target) {
    for(std::size_t i=0;i<count;++i) {
      std::set<std::uint32_t> unique;
      Need(rows[i].element_id&&rows[i].part_id,"Native interface shell identity");
      for(unsigned k=0;k<width;++k) {
        Need(rows[i].nodes[k]<p.node_count,"Native interface shell node");
        target[width*i+k]=int(rows[i].nodes[k]+1);unique.insert(rows[i].nodes[k]);
      }
      Need(unique.size()==width&&(width!=3||rows[i].nodes[2]==rows[i].nodes[3]),"Native shell shape");
    }
  };
  shells(p.quads,p.quad_count,4,quads.data());
  shells(p.triangles,p.triangle_count,3,triangles.data());
  std::array<int,4*Faces> raw_nodes{};
  std::array<int,Faces> raw_roles{},raw_elements{};
  for(std::size_t i=0;i<in.raw_face_count;++i) {
    const auto& face=in.raw_faces[i];
    const auto row=face.source.reader_row;
    const bool tri=face.nodes[2]==face.nodes[3];
    std::set<std::uint32_t> unique;
    for(unsigned k=0;k<4;++k) {
      Need(face.nodes[k]<p.node_count,"Native interface raw face node");
      raw_nodes[4*i+k]=int(face.nodes[k]+1);unique.insert(face.nodes[k]);
    }
    Need(unique.size()==(tri?3u:4u),"Native interface distinct corners");
    const std::uint32_t* parent_nodes=nullptr;unsigned width=0;
    if(face.source.kind==physical::ParentKind::Solid) {
      Need(row<p.solid_count&&face.raw_role==1&&face.source.solid_face&&face.source.solid_face<=6,
          "Native interface raw solid face");
      Need(face.source.element_id==p.solids[row].element_id&&face.source.part_id==p.solids[row].part_id,
          "Native interface solid source association");
      parent_nodes=p.solids[row].nodes;width=8;
    } else {
      const bool triangle=face.source.kind==physical::ParentKind::ShellTriangle;
      Need((triangle||face.source.kind==physical::ParentKind::ShellQuad)&&triangle==tri&&
          face.raw_role==(triangle?7:3)&&!face.source.solid_face&&row<(triangle?p.triangle_count:p.quad_count),
          "Native interface raw shell face");
      const auto& parent=(triangle?p.triangles:p.quads)[row];
      Need(parent.element_id==face.source.element_id&&parent.part_id==face.source.part_id,
          "Native interface shell source association");
      parent_nodes=parent.nodes;width=triangle?3:4;
    }
    for(const auto node:face.nodes) {
      bool found=false;
      for(unsigned k=0;k<width;++k)found=found||parent_nodes[k]==node;
      Need(found,"Native interface raw parent incidence");
    }
    raw_roles[i]=face.raw_role;raw_elements[i]=int(row+1);
  }
  const int counts[]{int(p.node_count),int(p.solid_count),int(p.quad_count),int(p.triangle_count),int(in.raw_face_count)};
  std::array<int,Faces> classified{},matched{},primary_roles{},winner{},raw_primary{};
  std::array<int,Rows> flags{};
  std::array<int,3> out_counts{};
  std::array<int,4*Faces> primary_nodes{};
  std::array<int,4*Mains> expanded_nodes{};
  std::array<int,Mains> expanded_roles{};
  {
    std::lock_guard<std::mutex> serial(native_mutex);
    rd_interface_source(counts,x.data(),solids.data(),quads.data(),triangles.data(),
        raw_nodes.data(),raw_roles.data(),raw_elements.data(),classified.data(),matched.data(),flags.data(),
        out_counts.data(),primary_nodes.data(),primary_roles.data(),winner.data(),raw_primary.data(),
        expanded_nodes.data(),expanded_roles.data());
  }
  const auto count=std::size_t(out_counts[0]),shell_count=std::size_t(out_counts[1]),g=std::size_t(out_counts[2]);
  Need(count&&count<=in.raw_face_count&&shell_count<=count&&g==count+shell_count&&g<=Mains,
      "Native interface result extent");
  NativeResult out;
  out.shell_primary_count=shell_count;
  out.primary.resize(count);out.identities.resize(count);
  out.raw_origins.resize(in.raw_face_count);out.classifications.resize(in.raw_face_count);
  out.raw_to_primary.resize(in.raw_face_count);out.primary_to_raw.resize(count);
  std::vector<std::uint32_t> origin_counts(count,0);
  for(std::size_t i=0;i<in.raw_face_count;++i) {
    Need(raw_primary[i]>0&&std::size_t(raw_primary[i])<=count,"Native filter raw occurrence map");
    const auto primary=std::uint32_t(raw_primary[i]-1);
    out.raw_to_primary[i]=primary;++origin_counts[primary];out.raw_origins[i]=Origin(in.raw_faces[i]);
    Need(matched[i]>=0&&std::size_t(matched[i])<=p.solid_count,"Native observed first solid");
    out.classifications[i]={classified[i],matched[i]?std::uint32_t(matched[i]-1):UINT32_MAX};
  }
  out.surface_solid_flags.resize(p.solid_count);
  for(std::size_t i=0;i<p.solid_count;++i) {
    Need(flags[i]==0||flags[i]==1,"Native interface source-solid flag");
    out.surface_solid_flags[i]=std::uint8_t(flags[i]);
  }
  for(std::size_t i=0;i<count;++i) {
    Need(winner[i]>0&&std::size_t(winner[i])<=in.raw_face_count,"Native filter winning occurrence");
    const auto raw=std::uint32_t(winner[i]-1);out.primary_to_raw[i]=raw;
    Need(out.raw_to_primary[raw]==i&&origin_counts[i],"Native filter winner/map association");
    const auto role=primary_roles[i];
    const bool solid=role==0||role==1,multiple=origin_counts[i]>1;
    auto& value=out.primary[i];
    for(unsigned k=0;k<4;++k) {
      Need(primary_nodes[4*i+k]>0&&std::size_t(primary_nodes[4*i+k])<=p.node_count,"Native filtered corner");
      value.nodes[k]=std::uint32_t(primary_nodes[4*i+k]-1);
    }
    value.layout=value.nodes[2]==value.nodes[3]?n::ShellLayout::Triangle3:n::ShellLayout::Quad4;
    value.source_id=multiple?0:in.raw_faces[raw].source.element_id;
    value.side_role=role<0?s::ShellSideRole::CoatingReversed:
        (role==4||role==8)?s::ShellSideRole::CoatingForward:s::ShellSideRole::Ordinary;
    out.identities[i]={solid?s::PrimaryFaceKind::Solid:s::PrimaryFaceKind::Shell,
        value.source_id,std::uint8_t(multiple?0:in.raw_faces[raw].source.solid_face),
        multiple?s::PrimaryOrigin::MultipleOrigins:s::PrimaryOrigin::SingleSourceFace,origin_counts[i]};
  }
  out.mains.resize(g);out.expanded_to_primary.assign(g,UINT32_MAX);out.primary_to_partner.assign(count,0);
  for(std::size_t i=0;i<count;++i) {
    out.expanded_to_primary[i]=std::uint32_t(i);
    int partner=expanded_roles[i];
    if(partner>int(g))partner-=int(g);
    Need(partner>=0&&std::size_t(partner)<=g,"Native SH2 partner range");
    if(partner) {
      Need(std::size_t(partner)>count&&out.expanded_to_primary[partner-1]==UINT32_MAX,
          "Native SH2 unique appended partner");
      out.primary_to_partner[i]=std::uint32_t(partner);out.expanded_to_primary[partner-1]=std::uint32_t(i);
    }
  }
  for(std::size_t i=0;i<g;++i) {
    Need(out.expanded_to_primary[i]<count,"Native SH2 complete identity map");
    auto& main=out.mains[i];
    main.source_id=out.primary[out.expanded_to_primary[i]].source_id;
    main.global_id=int(i+1);main.segment_type=expanded_roles[i];
    for(unsigned k=0;k<4;++k) {
      Need(expanded_nodes[4*i+k]>0&&std::size_t(expanded_nodes[4*i+k])<=p.node_count,"Native SH2 corner");
      main.nodes[k]=std::uint32_t(expanded_nodes[4*i+k]-1);
    }
  }
  return out;
}
}
