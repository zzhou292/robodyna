// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/RadiossType25Transaction.h"
#include <gtest/gtest.h>
#include <type_traits>
namespace n=tlfea::contact::radioss_type25;
TEST(NativeType25RuntimeConsumer,StableLifetimeAndUninitializedBoundary) {
  static_assert(!std::is_copy_constructible_v<n::Transaction>);
  static_assert(!std::is_move_constructible_v<n::Transaction>);
  n::Transaction transaction;EXPECT_FALSE(transaction.accepted().available);
  EXPECT_FALSE(transaction.source_info().available);EXPECT_EQ(transaction.source_info().secondaries,0u);
  EXPECT_EQ(transaction.allocations().device_bytes,0u);
  EXPECT_EQ(transaction.roster_entry().issuer,nullptr);
  EXPECT_EQ(transaction.CopyAccepted({},nullptr).status,n::TransactionStatus::NotInitialized);
}
