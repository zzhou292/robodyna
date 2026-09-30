#include "Source.h"
#include <limits>
namespace resident_tab1_test {
using Status = fe::ShellPlasticityBindingStatus;
TEST(ResidentTab1Binding,CompletePolicyAndOwnedParametersRetainExactSourceAndPlacementIdentity) {
  for (auto plane : placed::Planes) {
    Source source(plane);
    fe::ShellBatchBinding binding;
    fe::ShellBatchPlasticityBinding catalog;
    ASSERT_TRUE(source.Prepare(binding, catalog));
    fe::ShellBatchFailureBinding failure;
    ASSERT_EQ(failure.Initialize(catalog, source.failures.data(), source.failures.size()).status, Status::Success);
    EXPECT_TRUE(failure.Matches(catalog));
    EXPECT_EQ(failure.parent(0)->source.family, fe::ShellBindingFamily::T3);
    const auto* last = failure.parent(fe::ShellBindingFamily::Qeph, Parents - 1);
    ASSERT_NE(last, nullptr);
    EXPECT_EQ(last->tab1.table.failure_strain, .015);
    auto copy = failure;
    EXPECT_TRUE(copy.SameScope(failure));
    source.failures.back().tab1.table.triaxiality[2] = .4;
    fe::ShellBatchFailureBinding changed;
    ASSERT_EQ(changed.Initialize(catalog, source.failures.data(), source.failures.size()).status, Status::Success);
    EXPECT_FALSE(changed.SameScope(failure));
    EXPECT_EQ(last->tab1.table.triaxiality[2], .3);
    Source opposite(plane == Placement::TopReferencePlane ? Placement::BottomReferencePlane : Placement::TopReferencePlane);
    fe::ShellBatchBinding other_binding;
    fe::ShellBatchPlasticityBinding other_catalog;
    ASSERT_TRUE(opposite.Prepare(other_binding, other_catalog));
    EXPECT_FALSE(failure.Matches(other_catalog));
  }
}
TEST(ResidentTab1Binding,LateInvalidParametersTagsAndCapsRejectAtomicallyBeforeBorrowedRows) {
  Source source;
  fe::ShellBatchBinding binding;
  fe::ShellBatchPlasticityBinding catalog;
  ASSERT_TRUE(source.Prepare(binding, catalog));
  for (unsigned fault = 0; fault < 8; ++fault) {
    auto rows = source.failures;
    switch (fault) {
      case 0: rows.back().tab1.table.failure_strain = 0; break;
      case 1: rows.back().tab1.table.triaxiality[2] = 0; break;
      case 2: rows.back().tab1.table.triaxiality[2] = std::numeric_limits<double>::quiet_NaN(); break;
      case 3: rows.back().tab1.parent_policy = static_cast<fe::sections::ShellTab1ParentPolicy>(99); break;
      case 4: rows.back().constant.failure_strain = .015; break;
      case 5: rows[0].tab1 = {tab1_test::Table()}; break;
      case 6: rows.back().policy = static_cast<fe::ShellFailurePolicy>(99); break;
      case 7: rows.back().source.source_part_id++; break;
    }
    fe::ShellBatchFailureBinding rejected;
    EXPECT_NE(rejected.Initialize(catalog, rows.data(), rows.size()).status, Status::Success) << fault;
    EXPECT_FALSE(rejected.prepared());
    EXPECT_EQ(rejected.Initialize(catalog, source.failures.data(), source.failures.size()).status, Status::Success);
  }
  fe::ShellBatchFailureBinding rejected;
  auto limits = fe::ShellBatchFailureLimits{};
  limits.max_host_bytes = 1;
  EXPECT_EQ(rejected.Initialize(catalog, reinterpret_cast<const fe::ShellFailureParentInput*>(1),
      source.failures.size(), limits).status, Status::ResourceLimit);
  Source invalid;
  invalid.materials[1].rate = {true, 8000, 8, 10000};
  fe::ShellBatchBinding other_binding;
  fe::ShellBatchPlasticityBinding other_catalog;
  ASSERT_TRUE(invalid.Prepare(other_binding, other_catalog));
  EXPECT_EQ(rejected.Initialize(other_catalog, invalid.failures.data(), invalid.failures.size()).status, Status::InvalidMaterial);
}
} // namespace resident_tab1_test
