// SPDX-License-Identifier: AGPL-3.0-or-later
#include "SourceAdmissionFixture.h"
#include "lib_src/collision/radioss_type25/runtime/Layout.h"
#include "lib_src/collision/radioss_type25/runtime/Response.h"
#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <limits>
namespace {
namespace n=tlfea::contact::radioss_type25;
namespace rd=n::runtime_detail;
TEST(NativeType25Mass, ExplicitPolicyOmitsOnlyTheStaticMassStorage) {
  type25_source_test::Fixture fixture;
  const auto source=fixture.Source();auto config=fixture.Config();
  EXPECT_EQ(config.response_mass,n::ResponseMassPolicy::StaticPhysicalLedger);
  rd::SourceStaging legacy,current;
  ASSERT_EQ(rd::PrepareSource(config,source,fixture.physical.physical,{},legacy).status,n::TransactionStatus::Ok);
  config.response_mass=n::ResponseMassPolicy::AcceptedOwnerCoefficients;
  ASSERT_EQ(rd::PrepareSource(config,source,fixture.physical.physical,{},current).status,n::TransactionStatus::Ok);
  EXPECT_EQ(legacy.native_mass.size(),source.selection.node_count);
  EXPECT_TRUE(current.native_mass.empty());
  EXPECT_GE(legacy.bytes-current.bytes,source.selection.node_count*sizeof(double));
  rd::Layout a,b;n::TransactionLimits limits;
  limits.optimized_candidates=128;limits.sliding_entries=128;limits.inventory.max_pairs=128;
  ASSERT_TRUE(rd::MakeLayout(source,limits,0,a));
  ASSERT_TRUE(rd::MakeLayout(source,limits,0,b,{},config.response_mass));
  EXPECT_EQ(a.native_mass.bytes,source.selection.node_count*sizeof(double));
  EXPECT_EQ(b.native_mass.bytes,0u);EXPECT_LE(b.bytes,a.bytes);
  // Omitted regions must bind null, not alias the following reference positions.
  std::vector<std::max_align_t> arena((b.bytes+sizeof(std::max_align_t)-1)/sizeof(std::max_align_t));
  EXPECT_EQ(rd::Bind(arena.data(),b,source,limits).native_mass,nullptr);
  config.response_mass=static_cast<n::ResponseMassPolicy>(99);
  EXPECT_EQ(rd::PrepareSource(config,source,fixture.physical.physical,{},current).status,n::TransactionStatus::UnsupportedProfile);
  EXPECT_TRUE(current.native_mass.empty());
  EXPECT_FALSE(rd::MakeLayout(source,limits,0,b,{},config.response_mass));
}
TEST(NativeType25Mass, NativeAndSiOperandsKeepZeroAndRejectUnrepresentableDivision) {
  n::units_detail::Factors units;ASSERT_TRUE(n::units_detail::Make({.001,1000,1},units));
  for(double value:{0.,-0.,.125,123.75}) {
    double actual=-7;
    ASSERT_TRUE(rd::NativeMass({&value},0,units,actual));
    EXPECT_EQ(std::memcmp(&actual,&value,sizeof(double)),0);
    ASSERT_TRUE(rd::NativeMass({&value,rd::MassOperands::Units::Si},0,units,actual));
    const double expected=value/1000.;EXPECT_EQ(std::memcmp(&actual,&expected,sizeof(double)),0);
  }
  for(double value:{-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::denorm_min()}) {
    double sentinel=19;
    EXPECT_FALSE(rd::NativeMass({&value,rd::MassOperands::Units::Si},0,units,sentinel));
    EXPECT_EQ(sentinel,19);
  }
  double largest=std::numeric_limits<double>::max(),sentinel=19;units.mass=.5;
  EXPECT_FALSE(rd::NativeMass({&largest,rd::MassOperands::Units::Si},0,units,sentinel));EXPECT_EQ(sentinel,19);
}
TEST(NativeType25Mass, CompleteOrderedMassReadIncludesRepeatedAndZeroWeightSlots) {
  type25_source_test::Fixture fixture;auto source=fixture.Source();
  auto& main=fixture.mains[0];main.nodes[3]=main.nodes[2];
  n::lifecycle::Input input;input.source=source.selection;
  std::vector<double> velocity(fixture.physical.positions.size());
  input.current.positions={fixture.physical.positions.data(),7,3,1};
  input.current.velocities={velocity.data(),7,3,1};
  input.current.units=n::lifecycle::KinematicsUnits::Native;
  n::lifecycle::Occurrence occurrence;occurrence.selected.key.history_index=4;occurrence.selected.local_main=1;
  n::NativeGeometryFinalResult geometry;geometry.geometry.weights[0]=.25;geometry.geometry.weights[1]=.25;geometry.geometry.weights[2]=.5;
  geometry.geometry.normal={0,0,1};geometry.geometry.incoming_stiffness=100;geometry.penetration=.01;
  std::array<double,7> mass{1,2,3,4,0,6,7};
  n::units_detail::Factors units;ASSERT_TRUE(n::units_detail::Make({1,2,1},units));
  n::NativeFrictionInput out;
  ASSERT_TRUE(rd::ForceInput(input,occurrence,geometry,{mass.data(),rd::MassOperands::Units::Si},units,.01,out));
  EXPECT_EQ(out.normal.secondary_mass,0.);EXPECT_EQ(out.normal.main_mass[2],1.5);EXPECT_EQ(out.normal.main_mass[3],1.5);
  const auto before=out;main.nodes[3]=3;mass[3]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(rd::ForceInput(input,occurrence,geometry,{mass.data(),rd::MassOperands::Units::Si},units,.01,out));
  EXPECT_EQ(std::memcmp(&before,&out,sizeof(out)),0);
}
} // namespace
