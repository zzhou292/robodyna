// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include "ObservedMesh.h"
#include "ObservedSearch.h"
namespace type25_search_startup_test {
TEST(Type25SearchStartup, ActualCycleZeroWallMarginAndEmptyRemovalAreComputedFromSourceInputs) {
  namespace mesh=type25_startup_test::observed;
  old::Case source;source.ids.assign(std::begin(mesh::Ids),std::end(mesh::Ids));
  source.positions.assign(std::begin(mesh::Positions),std::end(mesh::Positions));
  for(unsigned i=0;i<8;++i)source.Add(n::ShellLayout::Triangle3,mesh::PrimaryNodes[4*i],
      mesh::PrimaryNodes[4*i+1],mesh::PrimaryNodes[4*i+2],mesh::PrimaryNodes[4*i+3]);
  Fixture f(std::move(source));f.physical_shells=12;f.secondary.clear();
  for(unsigned i=0;i<18;++i)f.secondary.push_back({observed::SecondaryNodes[i],observed::Stiffness[i],observed::SecondaryGaps[i]});
  for(unsigned i=0;i<8;++i)f.main_gaps[i]=f.main_gaps[i+8]=observed::PrimaryGaps[i];
  f.profile.initialization=s::Initialization::InvariantNoExpansion;
  const Built computed(f);Same(computed.view,Oracle(f.Input()));
  EXPECT_EQ(Bits(computed.view.margin),Bits(observed::Margin));
  EXPECT_EQ(computed.view.removal_count,0u);
  for(unsigned i=0;i<8;++i)EXPECT_EQ(Bits(computed.view.primary_extent[i]),Bits(observed::PrimaryExtent[i]));
  for(unsigned i=0;i<18;++i)EXPECT_EQ(computed.view.initial_contact[i],0);
}
} // namespace type25_search_startup_test
