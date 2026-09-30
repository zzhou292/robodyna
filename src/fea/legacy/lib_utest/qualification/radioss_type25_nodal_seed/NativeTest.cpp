// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "NativeOracle.h"
#include "../radioss_type25_coefficients/Assertions.h"
#include <algorithm>
#include <limits>
namespace type25_seed_test {
namespace {
void Compare(const Case& input) {
  const auto expected=Oracle(input);const auto rows=PreparePort(input);
  SeedFixture actual;ASSERT_EQ(actual.Build(rows.Input(input.nodes)).status,seed::Status::Ok);
  ASSERT_EQ(actual.nodes.size(),expected.seeds.size());
  ShellFixture finalized(rows,input.nodes,shell::Population::PhysicalShellsWithNodalSeed);
  ASSERT_EQ(finalized.Build({actual.nodes.data(),actual.nodes.size()}).status,shell::Status::Ok);
  for(std::size_t i=0;i<input.nodes;++i) {
    SCOPED_TRACE(i);
    SameSeed(actual.nodes[i],expected.seeds[i]);
    EXPECT_TRUE(tl::math::SameScalarBits(finalized.nodes[i].young_thickness_sum,expected.young_thickness[i]));
    EXPECT_EQ(finalized.nodes[i].shell_incidence_count,expected.shell_incidence[i]);
    // The already-qualified ASSTIFI finite-scalar bound permits native pow
    // rounding differences; all input accumulators/counts above are bit-exact.
    type25_coefficient_test::Number(finalized.nodes[i].stiffness,expected.finalized[i].stiffness);
    type25_coefficient_test::Number(finalized.values[i].stiffness,expected.finalized[i].stiffness);
  }
}
}
TEST(NodalSeedNative, FullMixedNativeOrderRawSlotsAndNodesWithoutShells) {
  const auto input=MixedCase();const auto rows=PreparePort(input);
  EXPECT_EQ(rows.volumes.size(),8*input.solids.size());EXPECT_EQ(rows.stiffness.size(),2*input.direct.size());
  Compare(input);
  const auto native=Oracle(input);
  EXPECT_EQ(native.shell_incidence[12],0);EXPECT_GT(native.seeds[12].existing_stiffness,0);
  EXPECT_EQ(native.shell_incidence[8],0);EXPECT_GT(native.seeds[8].volume,0);
  SameSeed(native.seeds[15],{}); // Beam orientation-only node receives no coefficient.
}
TEST(NodalSeedNative, SourceDefinedJointZerosAndAllPentaOccurrencesStayInCompleteSchedule) {
  auto input=MixedCase();
  input.direct.erase(std::remove_if(input.direct.begin(),input.direct.end(),[](const auto& p){return p.kind==DirectKind::Type45;}),input.direct.end());
  for(unsigned i=0;i<44;++i) {
    Direct joint;joint.kind=DirectKind::Type45;joint.eid=2000+int(i);joint.nodes={0,4+i%6};
    joint.joint_kn=i%2?1e20:0.;input.direct.push_back(joint);
  }
  const auto rows=PreparePort(input);EXPECT_EQ(rows.stiffness.size(),2*input.direct.size());
  std::size_t zeros=0;for(const auto& p:rows.stiffness)zeros+=tl::math::SameScalarBits(p.stiffness,0.);
  EXPECT_EQ(zeros,88u);
  std::size_t zero_volume=0;for(const auto& p:rows.volumes)zero_volume+=tl::math::SameScalarBits(p.volume,0.);
  EXPECT_EQ(zero_volume,2u);Compare(input);
}
TEST(NodalSeedNative, NativeSpringEidSortDiffersFromPropertyFamilyConcatenation) {
  auto input=MixedCase();input.direct.clear();
  for(unsigned i=0;i<3;++i) {
    Direct d;d.nodes={0,1};d.eid=int(10+10*i);
    d.kind=i==1?DirectKind::Type25:DirectKind::Type13;
    d.spring={i==1?n::SpringNodalKind::Type25:n::SpringNodalKind::Type13,1,0,
        {{i==1?1e16:1.,1},{0,1},{0,1}},0};input.direct.push_back(d);
  }
  Compare(input);
  const auto expected=Oracle(input);
  const auto wrong=PreparePort(input,SpringOrder::PropertyFamilyNegativeControl);
  SeedFixture bad;ASSERT_EQ(bad.Build(wrong.Input(input.nodes)).status,seed::Status::Ok);
  EXPECT_FALSE(tl::math::SameScalarBits(bad.nodes[0].existing_stiffness,expected.seeds[0].existing_stiffness));
}
TEST(NodalSeedNative, FloorNeighborsAndScaledNativePacketsKeepIndependentOracleAgreement) {
  for(double volume:{std::nextafter(n::native_constant::em30,0.),n::native_constant::em30,
      std::nextafter(n::native_constant::em30,std::numeric_limits<double>::infinity()),1e-12,1.,1e12}) {
    SCOPED_TRACE(volume);
    auto input=MixedCase();for(auto& s:input.solids){s.input.volume=volume;s.input.bulk=123.25;}
    Compare(input);
  }
}
TEST(NodalSeedNative, SiMillimetreAndOtherNativeUnitsUseExplicitPropertyDimensions) {
  // Beam18's separate reference API admits SI/t-mm-s only. This new synthetic
  // unit coupon contains solids/shells/springs/prepared STT; the complete beam
  // path remains covered by the full mixed test, without claiming a cm frame.
  auto base=MixedCase();
  base.direct.erase(std::remove_if(base.direct.begin(),base.direct.end(),
      [](const auto& row){return row.kind==DirectKind::Beam18;}),base.direct.end());
  const auto original=Oracle(base);
  const n::UnitScale contexts[]{{1.,1.,1.},{.001,1000.,1.},{.01,1.,.1}};
  for(const auto units:contexts) {
    SCOPED_TRACE(units.length_m);
    const double length=.001/units.length_m;
    const double mass=1000./units.mass_kg;
    const double time=1./units.time_s;
    const double pressure=mass/(length*time*time);
    const double stiffness=mass/(time*time);
    const double force=stiffness*length;
    auto input=base;
    for(auto& row:input.solids) {
      row.input.volume*=length*length*length;
      row.input.bulk*=pressure;
    }
    for(auto& row:input.shells) {
      row.young*=pressure;
      row.structural_thickness*=length;
      row.element_thickness*=length;
      row.property_thickness*=length;
      row.part_contact_thickness*=length;
    }
    for(auto& row:input.direct) {
      row.truss_stiffness*=stiffness;
      row.joint_kn*=stiffness;
      if(row.kind==DirectKind::Type13||row.kind==DirectKind::Type25) {
        const double factor=row.spring.length_mode>0?force:stiffness;
        for(auto& channel:row.spring.translation)channel.slope*=factor;
        row.spring.geometric_length*=length;
      }
    }
    Compare(input);
    const auto actual=Oracle(input);
    const double to_si=units.mass_kg/(units.time_s*units.time_s);
    for(std::size_t i=0;i<input.nodes;++i)
      type25_coefficient_test::Number(actual.finalized[i].stiffness*to_si,
          original.finalized[i].stiffness*1000.);
  }
}
} // namespace type25_seed_test
