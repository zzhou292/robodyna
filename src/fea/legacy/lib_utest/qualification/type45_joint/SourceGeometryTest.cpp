// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "OriginalJoints.h"
#include <cstring>

namespace type45_test {
TEST(Type45Source, All44OriginalGeometriesWithExplicitSuppliedPropertiesAndContext) {
  unsigned retained=0,kinds[3]{};
  for(const auto& row:type45_source::rows) {
    SCOPED_TRACE(row.eid);
    ASSERT_GE(row.kind,1u);
    ASSERT_LE(row.kind,3u);
    ++kinds[row.kind-1];
    retained+=row.retained;
    // Retain literal RPS/DAMP blanks. This does not interpret reader/export
    // defaults or claim these supplied body coefficients came from the deck.
    EXPECT_LE(std::strlen(row.raw),60u);
    Fixture fixture(static_cast<Kind>(row.kind));
    fixture.property.working_units=WorkingUnits::MillimetreTonneSecond;
    fixture.geometry.source_joint_id=row.eid;
    for(unsigned i=0;i<3;++i) {
      fixture.geometry.source_node_id[i]=row.kind==1 && i==2 ? 0 : row.raw_nodes[i];
      fixture.geometry.position_m[i]=row.positions[i];
    }
    for(unsigned i=0;i<2;++i)
      fixture.context.main[i].position_m=f::Add(row.positions[i],{.013,-.007,.019});
    const auto reference=fixture.Prepare();
    int status=-1;
    NativeOracle native(fixture,status);
    ASSERT_EQ(status,0);
    CompareReference(fixture,reference,native);
    History accepted;
    ASSERT_EQ(History::Initialize(reference,accepted),Status::Success);
    for(unsigned i=0;i<4;++i) {
      SCOPED_TRACE(i);
      auto step=fixture.Step(accepted);
      step.position_m[1].y+=(i+1)*3e-7;
      step.angular_velocity_rad_s[0]={.13,-.07,.03};
      step.angular_velocity_rad_s[1]={.23,.19,-.11};
      Evaluation actual;
      ASSERT_EQ(Evaluate(reference,accepted,step,actual),Status::Success);
      type45_native::Step expected;
      ASSERT_TRUE(native.Step(step,expected));
      CompareStep(actual,native,expected);
      accepted=actual.history;
    }
  }
  EXPECT_EQ(retained,38u);
  EXPECT_EQ(kinds[0],17u);
  EXPECT_EQ(kinds[1],22u);
  EXPECT_EQ(kinds[2],5u);
}
} // namespace type45_test
