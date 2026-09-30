#include "NativeOracle.h"
#include "Trajectory.h"
#include "OriginalFixture.h"
namespace rear_force_test {
TEST(Rear18ForceSource, EveryOriginalCellKeepsEightHistoriesAndSourceSlots) {
  ASSERT_EQ(std::size(rear18_test::original::Cells),306u);
  unsigned collapsed = 0;
  for (unsigned i = 0; i < std::size(rear18_test::original::Cells); ++i) {
    const auto input = rear18_test::original::Input(i);
    SCOPED_TRACE(input.source_element_id);
    law::Reference reference;
    ASSERT_EQ(law::InitializeReference(input,reference),s::Status::Success);
    const auto material = OriginalParameters(input.source_part_id);
    law::History accepted;
    ASSERT_EQ(law::InitializeHistory(reference,material,accepted),s::Status::Success);
    auto native = NativeInitial(input);
    collapsed += reference.topology() == law::SourceTopology::RepeatedPairs56And78;
    for (unsigned step = 0; step < 3; ++step) {
      auto interval = Path(reference,step);
      interval.base_time_s = accepted.stamp().time_s;
      const auto expected = Native(material,native,interval);
      ASSERT_EQ(expected.status,0);
      law::ForceTrial actual;
      ASSERT_EQ(law::EvaluateForce(reference,accepted,interval,material,actual),s::Status::Success);
      ASSERT_TRUE(Agree(actual,expected));
      accepted = actual.proposed_history;
      native = expected.next;
    }
  }
  EXPECT_EQ(collapsed,109u);
}
}
