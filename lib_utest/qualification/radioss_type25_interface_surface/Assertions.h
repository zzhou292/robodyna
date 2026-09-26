#pragma once
#include "Fixture.h"
namespace type25_interface_surface_test {
inline void Same(const s::PrimaryFaceIdentity& a,const s::PrimaryFaceIdentity& b) {
  EXPECT_EQ(a.kind,b.kind);EXPECT_EQ(a.physical_parent_id,b.physical_parent_id);
  EXPECT_EQ(a.local_face,b.local_face);EXPECT_EQ(a.origin,b.origin);EXPECT_EQ(a.origin_count,b.origin_count);
}
inline void Same(const f::Snapshot& value,const NativeResult& native) {
  ASSERT_EQ(value.primary_count,native.primary.size());
  ASSERT_EQ(value.shell_primary_count,native.shell_primary_count);
  ASSERT_EQ(value.main_count,native.mains.size());
  ASSERT_EQ(value.raw_face_count,native.classifications.size());
  for(std::size_t i=0;i<value.primary_count;++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(value.primary[i].source_id,native.primary[i].source_id);
    EXPECT_EQ(value.primary[i].layout,native.primary[i].layout);
    EXPECT_EQ(value.primary[i].side_role,native.primary[i].side_role);
    for(unsigned k=0;k<4;++k)EXPECT_EQ(value.primary[i].nodes[k],native.primary[i].nodes[k]);
    Same(value.identities[i],native.identities[i]);
    EXPECT_EQ(value.primary_to_raw[i],native.primary_to_raw[i]);
  }
  for(std::size_t i=0;i<value.raw_face_count;++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(value.classifications[i].role,native.classifications[i].role);
    EXPECT_EQ(value.classifications[i].matched_solid,native.classifications[i].matched_solid);
    EXPECT_EQ(value.raw_to_primary[i],native.raw_to_primary[i]);
    Same(value.raw_origins[i],native.raw_origins[i]);
  }
  ASSERT_EQ(value.physical_solid_count,native.surface_solid_flags.size());
  for(std::size_t i=0;i<value.physical_solid_count;++i)EXPECT_EQ(value.surface_solid_flags[i],native.surface_solid_flags[i]);
}
inline void Same(const s::MixedSidesSnapshot& value,const NativeResult& native) {
  ASSERT_EQ(value.main_count,native.mains.size());ASSERT_EQ(value.primary_count,native.primary.size());
  ASSERT_EQ(value.shell_primary_count,native.shell_primary_count);
  for(std::size_t i=0;i<value.main_count;++i) {
    const auto& a=value.mains[i];const auto& b=native.mains[i];
    EXPECT_EQ(a.source_id,b.source_id);EXPECT_EQ(a.global_id,b.global_id);EXPECT_EQ(a.segment_type,b.segment_type);
    EXPECT_EQ(value.expanded_to_primary[i],native.expanded_to_primary[i]);
    for(unsigned k=0;k<4;++k) {
      EXPECT_EQ(a.nodes[k],b.nodes[k]);
      EXPECT_EQ(a.neighbors[k],0);EXPECT_EQ(a.neighbor_edges[k],0);EXPECT_EQ(a.normal_reference[k],0);
    }
  }
  for(std::size_t i=0;i<value.primary_count;++i) {
    EXPECT_EQ(value.primary_to_partner[i],native.primary_to_partner[i]);
    EXPECT_EQ(value.primary_roles[i],native.primary[i].side_role);
    Same(value.primary_identities[i],native.identities[i]);
  }
  ASSERT_EQ(value.raw_origin_count,native.raw_origins.size());
  for(std::size_t i=0;i<value.raw_origin_count;++i) {
    Same(value.raw_origins[i],native.raw_origins[i]);
    EXPECT_EQ(value.raw_origin_to_primary[i],native.raw_to_primary[i]);
  }
}
inline void Compare(const Case& c) {
  const auto native=Oracle(c.Input());
  Built built(c);ASSERT_EQ(built.report.status,f::Status::Ok);
  Same(built.result,native);
  const auto input=SideInput(c,built.result);
  Sides sides(input);ASSERT_EQ(sides.report.status,s::Status::Ok);
  Same(sides.result,native);
  EXPECT_EQ(sides.forecast.ready_output_bytes,0u);
  EXPECT_EQ(sides.forecast.maximum_references,0u);
  EXPECT_EQ(sides.forecast.maximum_incidence,0u);
}
}
