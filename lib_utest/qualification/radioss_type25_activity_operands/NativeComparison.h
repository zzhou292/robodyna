// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "GpuFixture.h"
#include "lib_src/collision/radioss_type25/activity_source/Internal.h"
namespace activity_operands_test {
inline native::Mesh Mesh(const GpuFixture& f,std::array<std::uint8_t,2> before,
    std::array<std::uint8_t,2> after,std::uint8_t t_before,std::uint8_t t_after) {
  std::vector<n::activity_source::detail::ParentRow> rows;n::activity_source::Counts counts;
  const auto visit=[](void* context,const n::activity_source::detail::ParentRow& row){
    static_cast<std::vector<n::activity_source::detail::ParentRow>*>(context)->push_back(row);return true;};
  EXPECT_TRUE(Good(n::activity_source::detail::Parents({f.physical.physical,nullptr},counts,&rows,visit)));
  native::Mesh mesh;mesh.node_count=int(counts.nodes);
  n::runtime_detail::physical_main::Index index;
  EXPECT_TRUE(Good(index.Initialize(f.physical.physical,1u<<20)));
  for(const auto& row:rows) {
    using F=n::activity_source::Family;native::Element e;
    for(unsigned k=0;k<row.count;++k)e.nodes[k]=int(row.nodes[k]+1);
    if(row.identity.family==F::Qeph){e.kind=native::Kind::QuadShell;e.off=after[row.identity.family_index];e.previously_active=before[row.identity.family_index];}
    else if(row.identity.family==F::T3){e.kind=native::Kind::Triangle;e.off=t_after;e.previously_active=t_before;}
    else if(row.identity.family==F::Qbat)e.kind=native::Kind::QuadShell;
    else if(row.identity.family==F::Beam18)e.kind=native::Kind::Beam;
    else if(row.identity.family==F::Type13||row.identity.family==F::Type25||row.identity.family==F::Type45)e.kind=native::Kind::Spring;
    else {e.kind=native::Kind::Solid8;std::array<std::uint32_t,8> nodes;
      EXPECT_TRUE(index.Solid(row.identity.source_element_id,nodes));
      for(unsigned k=0;k<8;++k)e.nodes[k]=int(nodes[k]+1);}
    mesh.elements.push_back(e);
  }
  return mesh;
}
struct Saved {
  std::vector<n::lifecycle::Main> mains;
  std::vector<n::startup::Main> normal;
  std::vector<double> secondary,main_si,secondary_si,normal_coefficients;
  std::vector<std::int32_t> connected;std::vector<std::uint32_t> free;
};
inline Saved Read(GpuFixture& f,unsigned slot) {
  const auto v=f.operands.view(slot);EXPECT_EQ(v.main_count,f.source.selection.main_count);
  const auto stream=f.resources.stream;Saved result;
  result.mains=Read(v.mains,v.main_count,stream);result.secondary=Read(v.secondary_coefficients,v.secondary_count,stream);
  result.main_si=Read(v.main_stiffness_si,v.primary_count,stream);result.secondary_si=Read(v.secondary_stiffness_si,v.secondary_count,stream);
  result.connected=Read(v.connected_elements,v.main_count,stream);result.free=Read(v.free_mains,v.free_count,stream);
  if(f.normals){result.normal=Read(v.normal_mains,v.main_count,stream);result.normal_coefficients=Read(v.normal_coefficients,v.main_count,stream);}
  return result;
}
inline void Compare(GpuFixture& f,const Saved& old,const Saved& actual,const native::Mesh& mesh,const a::StageReport& report) {
  native::SurfaceCase c;c.mesh=mesh;c.deletion=f.controls.deletion==n::activity_source::Deletion::Disabled?0:1;
  c.solid_erosion=f.controls.solid_erosion==n::startup::SolidErosion::Enabled;
  c.connected_elements.assign(old.connected.begin(),old.connected.end());std::vector<int> neighbors;
  for(const auto& main:old.mains){std::array<int,4> corners;
    for(unsigned k=0;k<4;++k){corners[k]=int(main.nodes[k]+1);neighbors.push_back(main.neighbors[k]);}
    c.corners.push_back(corners);c.coefficients.push_back(main.coefficient);}
  c.affected=native::Discover(mesh,c.corners,c.deletion!=0);
  const auto surfaces=native::Surfaces(c);ASSERT_TRUE(surfaces.exposed.empty());
  for(int removed:surfaces.removed)neighbors=native::RemoveNeighbors(neighbors,removed);
  std::size_t newly_removed=0;
  n::units_detail::Factors factors;ASSERT_TRUE(n::units_detail::Make(f.units,factors));
  for(std::size_t i=0;i<actual.mains.size();++i){
    EXPECT_DOUBLE_EQ(actual.mains[i].coefficient,surfaces.coefficients[i]);
    newly_removed+=old.mains[i].coefficient!=0&&actual.mains[i].coefficient==0;
    if(i<actual.main_si.size())EXPECT_DOUBLE_EQ(actual.main_si[i],surfaces.coefficients[i]*factors.stiffness);
    for(unsigned k=0;k<4;++k){EXPECT_EQ(actual.mains[i].neighbors[k],neighbors[4*i+k]);
      if(f.normals){EXPECT_EQ(actual.normal[i].neighbors[k],neighbors[4*i+k]);
        if(!neighbors[4*i+k])EXPECT_EQ(actual.normal[i].neighbor_edges[k],0);}}
    if(f.normals)EXPECT_DOUBLE_EQ(actual.normal_coefficients[i],surfaces.coefficients[i]);
  }
  if(c.solid_erosion)EXPECT_EQ(actual.connected,(std::vector<std::int32_t>(surfaces.connected_elements.begin(),surfaces.connected_elements.end())));
  else EXPECT_EQ(actual.connected,old.connected);
  std::vector<int> nodes;for(const auto& secondary:f.physical.secondary)nodes.push_back(int(secondary.node+1));
  const auto secondary=native::Secondaries(surfaces.tags,nodes,old.secondary,c.deletion!=0&&!f.controls.keep_disconnected_nodes);
  std::size_t orphans=0;
  for(double marked:secondary.marked_coefficients)orphans+=marked<0;
  EXPECT_EQ(report.orphan_secondaries,orphans);
  for(std::size_t i=0;i<nodes.size();++i){const auto value=secondary.marked_coefficients[i]<0?0:secondary.marked_coefficients[i];
    EXPECT_DOUBLE_EQ(actual.secondary[i],value);EXPECT_DOUBLE_EQ(actual.secondary_si[i],value*factors.stiffness);}
  const auto free=f.normals?native::FreeBoundaries(c.corners,surfaces.coefficients,neighbors):std::vector<int>{};
  EXPECT_EQ(actual.free,(std::vector<std::uint32_t>(free.begin(),free.end())));
  EXPECT_EQ(report.affected_events,c.affected.size());EXPECT_EQ(report.removed_events,surfaces.removed.size());
  EXPECT_EQ(report.removed_mains,newly_removed);EXPECT_EQ(report.free_count,free.size());
}
} // namespace activity_operands_test
