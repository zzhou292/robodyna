// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_src/collision/radioss_type25/startup/Types.h"
#include <gtest/gtest.h>
#include <type_traits>
namespace type25_lifecycle_test {
static_assert(std::is_same<n::startup::NormalReference,l::NormalReference>::value);
TEST(Type25NormalViewConsumer, CompleteSharedTypedViewNeedsNoOracleOrOwner) {
  Fixture f;std::vector<n::StoredNormal> faces;
  for(const auto& main:f.mains)for(const auto& slot:main.normal_slot)faces.push_back(slot);
  auto in=f.Input();in.current_normals={faces.data(),faces.size(),f.normals.data(),f.normals.size()};
  in.source.normals=nullptr;
  l::HostResult out;
  ASSERT_EQ(l::EvaluateNativeLifecycleHost(in,Fixture::Limits(),&out).status,n::selection::Status::Ok);
  ASSERT_EQ(out.rows.size(),1u);EXPECT_GT(out.rows[0].new_impact_count,0u);
  EXPECT_EQ(f.accepted[0].row.irtlm[0],0);
}
}
