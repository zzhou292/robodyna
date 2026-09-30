#include "lib_src/solvers/NodalCinRuntime.h"
#include <gtest/gtest.h>
namespace accepted_mass_test {
TEST(AcceptedRawMassHost, UninitializedOwnerRejectsWithoutPublishing) {
  const tl::fea::FENodalState owner;
  const tl::fea::NodalTrialToken token;
  const double host_sentinel=3.5;
  tl::fea::NodalAcceptedRawMassView view{&host_sentinel,13,17,19,23,29,nullptr};
  EXPECT_EQ(owner.BorrowAcceptedRawMass(token,&view).status,tl::fea::NodalStatus::NotInitialized);
  EXPECT_EQ(view.mass_kg,&host_sentinel);
  EXPECT_EQ(view.node_count,13u);
  EXPECT_EQ(view.owner_id,17u);
  EXPECT_EQ(view.base_epoch,19u);
  EXPECT_EQ(view.attempt,23u);
  EXPECT_EQ(view.qualification_id,29u);
  EXPECT_EQ(view.stream,nullptr);
  EXPECT_EQ(owner.AuthenticateAcceptedRawMass(token,view).status,tl::fea::NodalStatus::NotInitialized);
  EXPECT_EQ(owner.BorrowAcceptedRawMass(token,nullptr).status,tl::fea::NodalStatus::NotInitialized);
}
} // namespace accepted_mass_test
