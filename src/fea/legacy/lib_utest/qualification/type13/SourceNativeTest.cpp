// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "source_fixture/YarisType13SourceFixture.h"
#include <gtest/gtest.h>
#include <cstring>
#include <iterator>
#include <limits>

namespace type13_test {
TEST(Type13SourceNative,All4442OriginalFramesAndEndpointCoefficients) {
  namespace source=yaris_type13_fixture;
  ASSERT_EQ(std::size(source::Nodes),7494);ASSERT_EQ(std::size(source::Beams),4442);
  Fixture fixture;t::Property property;
  ASSERT_EQ(t::InitializeProperty(fixture.Input(),property),t::Status::Success);
  double minimum=std::numeric_limits<double>::infinity(),maximum=0,closest=minimum;
  std::uint64_t min_id=0,max_id=0,closest_id=0;std::size_t fallback=0;
  long double mass_sum=0;
  for(const auto& beam:source::Beams) {
    SCOPED_TRACE(beam.id);t::ReferenceInput in;
    ASSERT_EQ(beam.blank_mask,480);ASSERT_EQ(beam.local,2);
    for(unsigned i=0;i<4;++i){ASSERT_EQ(beam.releases[i],0);in.endpoint_release[i]=beam.releases[i];}
    for(unsigned i=0;i<3;++i) {
      ASSERT_LT(beam.nodes[i],std::size(source::Nodes));
      const auto& node=source::Nodes[beam.nodes[i]];
      in.position[i]={node.native[0],node.native[1],node.native[2]};
      for(unsigned k=0;k<3;++k) {
        const double si=node.native[k]*.001;
        EXPECT_EQ(std::memcmp(&si,&node.si[k],sizeof(si)),0);
      }
      if(i==2)ASSERT_EQ(node.id,2000001);
    }
    t::Startup actual;ASSERT_EQ(t::InitializeElement(property,in,actual),t::Status::Success);
    const auto native=NativeReference(in);
    EXPECT_EQ(actual.reference.length_native,native.length);
    EXPECT_EQ(static_cast<int>(actual.reference.branch),native.branch);
    for(unsigned k=0;k<3;++k)EXPECT_EQ(actual.reference.axes.v[3*k+1],native.y[k]);
    const auto native_mass=NativeMass(property.mass_per_length(),property.inertia_per_length(),native.length);
    EXPECT_EQ(actual.endpoint.mass_kg,native_mass[0]*1000);
    EXPECT_EQ(actual.endpoint.isotropic_inertia_kg_m2,native_mass[1]*.001);
    mass_sum+=2*static_cast<long double>(actual.endpoint.mass_kg);
    if(actual.reference.length_m<minimum){minimum=actual.reference.length_m;min_id=beam.id;}
    if(actual.reference.length_m>maximum){maximum=actual.reference.length_m;max_id=beam.id;}
    if(actual.reference.third_node_alignment<closest){closest=actual.reference.third_node_alignment;closest_id=beam.id;}
    fallback+=actual.reference.branch!=t::FrameBranch::ThirdNode;
  }
  EXPECT_EQ(source::Beams[0].id,2102273);EXPECT_EQ(source::Beams[4441].id,2409378);
  EXPECT_EQ(min_id,2405011);EXPECT_EQ(max_id,2407177);EXPECT_EQ(closest_id,2407223);
  EXPECT_EQ(fallback,0);EXPECT_GT(closest,1e-5);EXPECT_LT(closest,1.12e-5);
  EXPECT_GT(mass_sum,1.61L);EXPECT_LT(mass_sum,1.62L);
}
} // namespace type13_test
