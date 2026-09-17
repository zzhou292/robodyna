// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/collision/self_contact_physical_activity/Selection.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace self_contact_physical_activity_selection_test {
namespace c = tlfea::contact;
namespace a = c::self_contact_physical_activity;
namespace fe = tl::fea;
using Family = fe::ShellBindingFamily;
using Parent = fe::ShellPlasticityParentInput;
using Report = c::SelfContactPhysicalActivityReport;
using Status = c::SelfContactPhysicalActivityStatus;

// Deliberately independent inventory: exact identity is authenticated before
// either implementation may use a source ID to prove row uniqueness.
struct Inventory {
  std::array<std::array<std::uint64_t, 4>, 3> ids{{
      {{10, 40, 70, 100}}, {{20, 50, 80, 110}}, {{30, 60, 90, 120}}}};

  static std::size_t Slot(Family family) noexcept {
    switch (family) {
      case Family::Qeph: return 0;
      case Family::T3: return 1;
      case Family::Qbat: return 2;
      default: return 3;
    }
  }

  Parent Row(Family family, std::size_t index) const noexcept {
    Parent row;
    row.family = family;
    row.family_index = index;
    const auto slot = Slot(family);
    if (slot < ids.size() && index < ids[slot].size())
      row.source_parent_id = ids[slot][index];
    return row;
  }

  bool Exact(const Parent& row) const noexcept {
    const auto slot = Slot(row.family);
    return slot < ids.size() && row.family_index < ids[slot].size() &&
        row.source_parent_id != 0 &&
        row.source_parent_id == ids[slot][row.family_index];
  }
};

Report RowFailure(const Parent& row, std::size_t parent,
                  const char* message) noexcept {
  Report report;
  report.status = Status::IdentityMismatch;
  report.parent = parent;
  report.family = row.family;
  report.family_index = row.family_index;
  report.message = message;
  return report;
}

// The pre-optimization algorithm is the differential oracle. Preserve its
// ordered row validation and prior-row duplicate scan without a prefix proof.
Report Original(const std::vector<Parent>& rows,
                const Inventory& inventory) noexcept {
  if (rows.empty()) {
    Report report;
    report.status = Status::InvalidInput;
    report.message = "Self-contact activity selection is empty";
    return report;
  }
  for (std::size_t parent = 0; parent < rows.size(); ++parent) {
    const auto& row = rows[parent];
    if (!inventory.Exact(row))
      return RowFailure(row, parent,
          "Selected parent is not an exact QEPH/T3/QBAT inventory row");
    for (std::size_t prior = 0; prior < parent; ++prior) {
      if (rows[prior].family == row.family &&
          rows[prior].family_index == row.family_index)
        return RowFailure(row, parent,
            "Selected activity inventory repeats a physical family row");
    }
  }
  return {};
}

Report Validate(const std::vector<Parent>& rows,
                const Inventory& inventory,
                std::size_t* accesses = nullptr) noexcept {
  return a::ValidateSelectionRows(rows.size(),
      [&](std::size_t parent) noexcept -> const Parent& {
        if (accesses) ++*accesses;
        return rows[parent];
      },
      [&](const Parent& row) noexcept { return inventory.Exact(row); });
}

void ExpectSame(const Report& actual, const Report& expected) {
  EXPECT_EQ(actual.status, expected.status);
  EXPECT_EQ(actual.family, expected.family);
  EXPECT_EQ(actual.parent, expected.parent);
  EXPECT_EQ(actual.family_index, expected.family_index);
  EXPECT_EQ(actual.publication_status, expected.publication_status);
  EXPECT_EQ(actual.owner_status, expected.owner_status);
  EXPECT_STREQ(actual.message, expected.message);
}

TEST(SelfContactPhysicalActivitySelection, EmptyNeverReadsSource) {
  std::size_t accesses = 0;
  const Inventory inventory;
  ExpectSame(Validate({}, inventory, &accesses), Original({}, inventory));
  EXPECT_EQ(accesses, 0u);
}

TEST(SelfContactPhysicalActivitySelection,
     SortedVehicleSizedRosterUsesExactlyOneAccessPerParent) {
  constexpr std::size_t count = 337092;
  constexpr std::array<Family, 3> families{{
      Family::Qeph, Family::T3, Family::Qbat}};
  std::size_t accesses = 0;
  std::size_t exact_checks = 0;
  // Generate rows by value: the coupon itself needs no large inventory buffer.
  const auto report = a::ValidateSelectionRows(count,
      [&](std::size_t parent) noexcept {
        ++accesses;
        Parent row;
        row.family = families[parent % 3];
        row.family_index = parent / 3;
        row.source_parent_id = parent + 1;
        return row;
      },
      [&](const Parent& row) noexcept {
        ++exact_checks;
        const auto slot = Inventory::Slot(row.family);
        return slot < 3 && row.family_index < count / 3 &&
            row.source_parent_id == row.family_index * 3 + slot + 1;
      });
  EXPECT_EQ(report.status, Status::Ok);
  EXPECT_EQ(accesses, count);
  EXPECT_EQ(exact_checks, count);
}

TEST(SelfContactPhysicalActivitySelection,
     SameIndexDifferentFamiliesAndFullUnsignedIdRangeRemainValid) {
  Inventory inventory;
  inventory.ids[0][0] = 1;
  inventory.ids[1][0] = std::numeric_limits<std::uint64_t>::max() - 1;
  inventory.ids[2][0] = std::numeric_limits<std::uint64_t>::max();
  const std::vector<Parent> rows{
      inventory.Row(Family::Qeph, 0), inventory.Row(Family::T3, 0),
      inventory.Row(Family::Qbat, 0)};
  std::size_t accesses = 0;
  ExpectSame(Validate(rows, inventory, &accesses), Original(rows, inventory));
  EXPECT_EQ(accesses, rows.size());
}

TEST(SelfContactPhysicalActivitySelection,
     EqualSourceIdsForDistinctPhysicalRowsUseFallbackAndRemainValid) {
  Inventory inventory;
  inventory.ids[1][0] = inventory.ids[0][0];
  inventory.ids[2][0] = inventory.ids[0][0];
  const std::vector<Parent> rows{
      inventory.Row(Family::Qeph, 0), inventory.Row(Family::T3, 0),
      inventory.Row(Family::Qbat, 0)};
  std::size_t accesses = 0;
  const auto report = Validate(rows, inventory, &accesses);
  ExpectSame(report, Original(rows, inventory));
  EXPECT_EQ(report.status, Status::Ok);
  EXPECT_EQ(accesses, 6u);
}

TEST(SelfContactPhysicalActivitySelection,
     DuplicateAtFirstDisorderAndAfterRenewedIncreaseAreRejected) {
  const Inventory inventory;
  const auto q = inventory.Row(Family::Qeph, 0);
  const auto t = inventory.Row(Family::T3, 0);
  const auto b = inventory.Row(Family::Qbat, 0);
  const std::vector<Parent> first_disorder{q, t, q};
  const auto first = Validate(first_disorder, inventory);
  ExpectSame(first, Original(first_disorder, inventory));
  EXPECT_EQ(first.parent, 2u);

  // 30, 10, 20, 30: restarting the prefix proof after descent would miss this.
  const std::vector<Parent> renewed_increase{b, q, t, b};
  const auto later = Validate(renewed_increase, inventory);
  ExpectSame(later, Original(renewed_increase, inventory));
  EXPECT_EQ(later.parent, 3u);
}

TEST(SelfContactPhysicalActivitySelection,
     FirstOffendingRowAndExactIdentityPrecedeDuplicateDiagnostics) {
  const Inventory inventory;
  const auto low = inventory.Row(Family::Qeph, 0);
  const auto high = inventory.Row(Family::Qeph, 2);
  const std::vector<Parent> repeats{high, low, high, low};
  const auto duplicate = Validate(repeats, inventory);
  ExpectSame(duplicate, Original(repeats, inventory));
  EXPECT_EQ(duplicate.parent, 2u);
  EXPECT_EQ(duplicate.family_index, 2u);

  auto forged = low;
  forged.source_parent_id = 1000;  // Increasing ID cannot certify a forged row.
  const std::vector<Parent> same_row_error{low, forged};
  const auto invalid = Validate(same_row_error, inventory);
  ExpectSame(invalid, Original(same_row_error, inventory));
  EXPECT_STREQ(invalid.message,
      "Selected parent is not an exact QEPH/T3/QBAT inventory row");
  for (const auto& rows : {std::vector<Parent>{low, low, forged},
                           std::vector<Parent>{low, forged, low}})
    ExpectSame(Validate(rows, inventory), Original(rows, inventory));
}

TEST(SelfContactPhysicalActivitySelection,
     AllValidPermutationsPreserveOriginalResult) {
  const Inventory inventory;
  std::array<std::size_t, 4> order{{0, 1, 2, 3}};
  const std::array<Parent, 4> pool{{
      inventory.Row(Family::Qeph, 0), inventory.Row(Family::T3, 0),
      inventory.Row(Family::Qbat, 0), inventory.Row(Family::Qeph, 1)}};
  do {
    const std::vector<Parent> rows{
        pool[order[0]], pool[order[1]], pool[order[2]], pool[order[3]]};
    ExpectSame(Validate(rows, inventory), Original(rows, inventory));
  } while (std::next_permutation(order.begin(), order.end()));
}

TEST(SelfContactPhysicalActivitySelection,
     ExhaustiveSmallRostersMatchEveryOriginalDiagnosticField) {
  const Inventory inventory;
  auto zero = inventory.Row(Family::Qeph, 0);
  zero.source_parent_id = 0;
  auto mismatch = inventory.Row(Family::T3, 0);
  ++mismatch.source_parent_id;
  const std::array<Parent, 8> pool{{
      inventory.Row(Family::Qeph, 0), inventory.Row(Family::Qeph, 1),
      inventory.Row(Family::T3, 0), inventory.Row(Family::Qbat, 0),
      inventory.Row(Family::None, 0),
      inventory.Row(Family::Qbat, std::numeric_limits<std::size_t>::max()),
      zero, mismatch}};
  std::size_t combinations = 1;
  for (std::size_t length = 0; length <= 4; ++length) {
    for (std::size_t code = 0; code < combinations; ++code) {
      SCOPED_TRACE(::testing::Message() << "length=" << length << " code=" << code);
      std::vector<Parent> rows;
      auto digits = code;
      for (std::size_t index = 0; index < length; ++index) {
        rows.push_back(pool[digits % pool.size()]);
        digits /= pool.size();
      }
      ExpectSame(Validate(rows, inventory), Original(rows, inventory));
    }
    combinations *= pool.size();
  }
}

}  // namespace self_contact_physical_activity_selection_test
