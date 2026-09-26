#pragma once
#include "lib_src/collision/RadiossType25SurfaceSource.h"
#include <algorithm>
#include <array>
#include <cstring>
#include <gtest/gtest.h>
#include <vector>
namespace type25_surface_source_test {
namespace s=tlfea::contact::radioss_type25::source_surfaces;
struct Case {
  std::size_t nodes=24;
  std::vector<s::Solid> solids;
  std::vector<s::Shell> quads,triangles;
  std::vector<std::uint64_t> parts{10};
  std::vector<std::uint32_t> selected;
  s::ClauseKind kind=s::ClauseKind::Parts;
  s::SurfaceMode mode=s::SurfaceMode::Exterior;
  bool reverse=false;
  s::Input Input() const {
    s::Input in;
    in.phase=s::ReaderPhase::BeforeGroupingAndInitia;in.node_count=nodes;
    in.solids=solids.empty()?nullptr:solids.data();in.solid_count=solids.size();
    in.quads=quads.empty()?nullptr:quads.data();in.quad_count=quads.size();
    in.triangles=triangles.empty()?nullptr:triangles.data();in.triangle_count=triangles.size();
    in.clause.kind=kind;in.clause.mode=mode;in.clause.reverse_shell_normals=reverse;
    in.clause.part_ids=parts.empty()?nullptr:parts.data();in.clause.part_count=parts.size();
    in.clause.solid_rows=selected.empty()?nullptr:selected.data();in.clause.solid_row_count=selected.size();
    return in;
  }
};
inline s::Solid Hex(std::uint64_t id=101,std::uint64_t part=10) {
  return {id,part,s::SolidTopology::Hex8,{0,1,2,3,4,5,6,7}};
}
inline s::Solid Penta(std::uint64_t id=102,std::uint64_t part=10,unsigned shift=0) {
  s::Solid result{id,part,s::SolidTopology::DeclaredPenta6,{0,1,2,0,3,4,5,3}};
  for(auto& node:result.nodes)node+=shift;return result;
}
inline Case SingleHex() {Case c;c.solids={Hex()};return c;}
inline Case Adjacent() {
  auto c=SingleHex();c.solids.push_back({102,20,s::SolidTopology::Hex8,{4,5,6,7,8,9,10,11}});
  c.parts={10,20};return c;
}
struct Built {
  tl::util::HostArena output,scratch;
  s::Forecast forecast;
  s::Snapshot result;
  s::Report report;
  explicit Built(const Case& c,s::Limits cap={}) {
    const auto in=c.Input();report=s::Preflight(in,cap,forecast);
    if(report.status!=s::Status::Ok)return;
    if(!output.Initialize(forecast.output_bytes)||!scratch.Initialize(forecast.scratch_bytes))
      throw std::runtime_error("Surface fixture allocation failed");
    report=s::Build(in,cap,output,scratch,&result);
  }
};
inline void SameFace(const s::Face& a,const s::Face& b) {
  EXPECT_EQ(a.source.kind,b.source.kind);EXPECT_EQ(a.source.element_id,b.source.element_id);
  EXPECT_EQ(a.source.part_id,b.source.part_id);EXPECT_EQ(a.source.reader_row,b.source.reader_row);
  EXPECT_EQ(a.source.solid_face,b.source.solid_face);EXPECT_EQ(a.raw_role,b.raw_role);
  EXPECT_EQ(a.buffer_ordinal,b.buffer_ordinal);
  for(unsigned k=0;k<4;++k)EXPECT_EQ(a.nodes[k],b.nodes[k]);
}
inline std::vector<unsigned char> Bytes(const tl::util::HostArena& arena) {
  const auto* begin=static_cast<const unsigned char*>(arena.data());return {begin,begin+arena.bytes()};
}
inline void SameSnapshot(const s::Snapshot& a,const s::Snapshot& b) {
  EXPECT_EQ(a.faces,b.faces);EXPECT_EQ(a.face_count,b.face_count);
  EXPECT_EQ(a.surface_solid_flags,b.surface_solid_flags);EXPECT_EQ(a.solid_count,b.solid_count);
  const auto& x=a.counts;const auto& y=b.counts;
  EXPECT_EQ(x.selected_solids,y.selected_solids);EXPECT_EQ(x.selected_quads,y.selected_quads);
  EXPECT_EQ(x.selected_triangles,y.selected_triangles);EXPECT_EQ(x.internal_faces,y.internal_faces);
  EXPECT_EQ(x.degenerate_faces,y.degenerate_faces);EXPECT_EQ(x.shell_suppressed_faces,y.shell_suppressed_faces);
  EXPECT_EQ(x.solid_faces,y.solid_faces);EXPECT_EQ(x.shell_faces,y.shell_faces);
}
}
