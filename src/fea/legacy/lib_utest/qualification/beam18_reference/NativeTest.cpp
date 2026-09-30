// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#ifdef BEAM18_ORIGINAL
#include "OriginalFixture.h"
#endif
namespace beam18_test {
TEST(Beam18Native, CircularSectionAndNativeMassBranchesInBothUnits) {
  for (double radius : {4.5,5.9}) for (double length : {.01,4.,16.,20.})
    for (bool si : {false,true}) {
      auto input=Input(length,radius);
      if (si) {
        input.units=beam::WorkingUnits::SI;
        input.radius*=.001;input.density*=1e12;input.young*=1e6;
        for(auto& x:input.position)x=tl::math::fixed3::Scale(x,.001);
      }
      beam::Reference actual;
      ASSERT_EQ(beam::InitializeReference(input,actual),beam::Status::Success);
      Compare(actual,Native(input));
      ASSERT_FALSE(HasFatalFailure());
    }
}
TEST(Beam18Native, OriginalN3AndReaderFallbackDoNotCreateAnEndpoint) {
  for(unsigned mode=0;mode<4;++mode) {
    auto input=Input();
    if(mode==1)input.position[2]={101,200,300};
    if(mode>=2) {input.source_node_id[2]=0;input.position[2]={};}
    if(mode==3)input.position[1]={100,216,300};
    beam::Reference actual;
    ASSERT_EQ(beam::InitializeReference(input,actual),beam::Status::Success);
    Compare(actual,Native(input));
  }
}
#ifdef BEAM18_ORIGINAL
TEST(Beam18NativeSource, All142OriginalSourceMassInertiaStiffnessAndOrientation) {
  ASSERT_EQ(std::size(original::Cells),142u);
  unsigned bar=0,headrest=0;
  for(unsigned i=0;i<std::size(original::Cells);++i) {
    const auto input=original::Input(i);SCOPED_TRACE(input.source_element_id);
    beam::Reference actual;
    ASSERT_EQ(beam::InitializeReference(input,actual),beam::Status::Success);
    Compare(actual,Native(input));
    ASSERT_FALSE(HasFatalFailure());
    input.radius==4.5?++bar:++headrest;
    EXPECT_EQ(actual.geometry().orientation_branch,beam::OrientationBranch::ThirdNode);
  }
  EXPECT_EQ(bar,72u);EXPECT_EQ(headrest,70u);
}
#endif
} // namespace beam18_test
