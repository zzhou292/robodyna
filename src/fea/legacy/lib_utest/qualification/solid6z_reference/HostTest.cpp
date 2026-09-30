// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include <gtest/gtest.h>
#include <set>
#include <vector>
namespace solid6z_test {
TEST(Solid6zHost, ReferenceVolumeAndMassHaveSixDistinctContributions) {
  const auto input=Wedge(); s::Reference result;
  ASSERT_EQ(s::InitializeReference(input,result),s::Status::Success);
  EXPECT_NEAR(result.geometry().reference_volume_m3,.5*.02*.03*.04,1e-20);
  EXPECT_NEAR(result.geometry().volume_m3,result.geometry().reference_volume_m3,1e-20);
  EXPECT_NEAR(result.geometry().axial_volume_gradient_m3,0,1e-20);
  EXPECT_NEAR(result.mass().element_mass_kg,1980*.5*.02*.03*.04,1e-16);
  double sum=0;
  for (double m:result.mass().source_slot_mass_kg) sum+=m;
  EXPECT_NEAR(sum,result.mass().element_mass_kg,1e-16);
  EXPECT_EQ(result.mass().isotropic_inertia_kg_m2(),0);
  for (unsigned n=0;n<6;++n) EXPECT_EQ(result.source_slot(n),n);
}
TEST(Solid6zHost, ReversedOrientationRetainsSourceAssociation) {
  auto input=Distorted(); s::Reference original,reversed;
  ASSERT_EQ(s::InitializeReference(input,original),s::Status::Success);
  for (unsigned n=0;n<3;++n) {
    std::swap(input.position_m[n],input.position_m[n+3]);
    std::swap(input.source_node_id[n],input.source_node_id[n+3]);
  }
  ASSERT_EQ(s::InitializeReference(input,reversed),s::Status::Success);
  EXPECT_EQ(Values(original),Values(reversed));
  for (unsigned n=0;n<6;++n) EXPECT_EQ(reversed.source_slot(n),(n+3)%6);
}
TEST(Solid6zHost, InvalidProfileAndLateOverflowPreservePreviousThenRetry) {
  s::Reference result; const auto good=Distorted();
  ASSERT_EQ(s::InitializeReference(good,result),s::Status::Success);
  const auto saved=Values(result);
  auto bad=good; bad.profile.engine_jhbe=1;
  EXPECT_EQ(s::InitializeReference(bad,result),s::Status::UnsupportedProfile);
  bad=good; bad.profile.mass_distribution=1;
  EXPECT_EQ(s::InitializeReference(bad,result),s::Status::UnsupportedProfile);
  bad=good; bad.source_node_id[5]=bad.source_node_id[0];
  EXPECT_EQ(s::InitializeReference(bad,result),s::Status::InvalidInput);
  bad=good; bad.position_m[5].x=std::numeric_limits<double>::max();
  EXPECT_EQ(s::InitializeReference(bad,result),s::Status::InvalidGeometry);
  bad=good;
  for (auto& x:bad.position_m) { x.x*=1e6; x.y*=1e6; x.z*=1e6; }
  bad.density_kg_m3=std::numeric_limits<double>::max();
  EXPECT_EQ(s::InitializeReference(bad,result),s::Status::NonfiniteResult);
  bad=good; for (auto& x:bad.position_m) x.z=0;
  EXPECT_EQ(s::InitializeReference(bad,result),s::Status::InvalidGeometry);
  EXPECT_EQ(saved,Values(result));
  EXPECT_EQ(result.input().source_node_id[5],good.source_node_id[5]);
  ASSERT_EQ(s::InitializeReference(good,result),s::Status::Success);
  EXPECT_EQ(saved,Values(result));
}
TEST(Solid6zHost, ExplicitCollapsedMappingPreservesAllNondegenerateFacesAndEdges) {
  const std::uint64_t raw[8]{11,22,33,44,55,55,66,66};
  s::CollapsedBrickTopology map;
  ASSERT_EQ(s::MapCollapsedTopEdges(raw,map),s::Status::Success);
  const unsigned hex[6][4]{{0,1,2,3},{4,5,6,7},{0,1,5,4},
                           {1,2,6,5},{2,3,7,6},{3,0,4,7}};
  const unsigned wedge[5][4]{{0,1,2,2},{3,4,5,5},{0,1,4,3},
                             {1,2,5,4},{2,0,3,5}};
  using Face=std::set<std::uint64_t>;
  using Edge=std::pair<std::uint64_t,std::uint64_t>;
  std::set<Face> a,b; std::set<Edge> ea,eb;
  auto insert=[](const std::uint64_t (&ids)[4],auto& faces,auto& edges) {
    Face face(std::begin(ids),std::end(ids));
    if (face.size()<3) return;
    faces.insert(face);
    for (unsigned i=0;i<4;++i) {
      const auto j=(i+1)%4;
      if (ids[i]!=ids[j]) edges.insert(std::minmax(ids[i],ids[j]));
    }
  };
  for (const auto& f:hex) {
    const std::uint64_t ids[4]{raw[f[0]],raw[f[1]],raw[f[2]],raw[f[3]]};
    insert(ids,a,ea);
  }
  for (const auto& f:wedge) {
    const std::uint64_t ids[4]{raw[map.six_to_raw[f[0]]],raw[map.six_to_raw[f[1]]],
                              raw[map.six_to_raw[f[2]]],raw[map.six_to_raw[f[3]]]};
    insert(ids,b,eb);
  }
  EXPECT_EQ(a,b); EXPECT_EQ(ea,eb); EXPECT_EQ(a.size(),5u); EXPECT_EQ(ea.size(),9u);
  for (unsigned n=0;n<8;++n) EXPECT_EQ(map.raw_source_node_id[n],raw[n]);
  auto prior=map; std::uint64_t bad[8]{11,22,33,44,55,55,66,77};
  EXPECT_EQ(s::MapCollapsedTopEdges(bad,map),s::Status::UnsupportedProfile);
  for (unsigned n=0;n<8;++n) EXPECT_EQ(map.raw_source_node_id[n],prior.raw_source_node_id[n]);
}
}  // namespace solid6z_test
