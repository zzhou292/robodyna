// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CoatedCases.h"
#include "Assertions.h"
#include <algorithm>
#include <limits>
namespace type25_startup_test {
namespace {
std::vector<unsigned char> ArenaBytes(const tl::util::HostArena& arena) {
  const auto* begin = static_cast<const unsigned char*>(arena.data());
  return {begin, begin + arena.bytes()};
}
void CompareCoated(const Case& source) {
  const CoatedBuilt built(source);
  const auto native = Oracle(source.Input(), source.coefficients.data(), source.coefficients.size());
  SameStarter(built.startup, native);
  EXPECT_EQ(built.report.neighbor_warnings.count, std::size_t(native.warning_count));
  EXPECT_EQ(built.startup.profile, source.profile);
  EXPECT_EQ(built.startup.topology, source.topology);
  const auto p = source.primary.size(), g = 2 * p;
  for (std::size_t i = 0; i < p; ++i) {
    SCOPED_TRACE(i);
    const auto role = source.primary[i].side_role;
    const auto& a = built.startup.mains[i];
    const auto& b = built.startup.mains[p + i];
    const int offset = role == s::ShellSideRole::Ordinary ? 0 : int(g);
    EXPECT_EQ(a.segment_type, int(p + i + 1) + offset);
    EXPECT_EQ(b.segment_type, -(int(i + 1) + offset));
    EXPECT_EQ(built.startup.primary_to_partner[i], p + i + 1);
    constexpr unsigned reverse[]{1, 0, 3, 2};
    for (unsigned k = 0; k < 4; ++k) {
      const auto input_slot = role == s::ShellSideRole::CoatingReversed ? reverse[k] : k;
      EXPECT_EQ(a.nodes[k], source.primary[i].nodes[input_slot]);
      EXPECT_EQ(b.nodes[k], a.nodes[reverse[k]]);
    }
  }
  // The original neighbor selection separates ordinary and coating roles.
  for (std::size_t i = 0; i < g; ++i) {
    const auto& main = built.startup.mains[i];
    for (unsigned k = 0; k < 4; ++k) if (main.neighbors[k]) {
      const auto& adjacent = built.startup.mains[main.neighbors[k] - 1];
      EXPECT_EQ(std::abs(main.segment_type) > int(g), std::abs(adjacent.segment_type) > int(g));
      ASSERT_GE(main.neighbor_edges[k], 1);
      ASSERT_LE(main.neighbor_edges[k], 4);
      EXPECT_EQ(adjacent.neighbors[main.neighbor_edges[k] - 1], int(i + 1));
    }
  }
}
}
TEST(Type25CoatedStartup, CompleteMixedNativeExpansionNeighborsReferencesAndNormals) {
  for (unsigned mode = 0; mode < 3; ++mode) for (unsigned shift = 0; shift < 3; ++shift) {
    SCOPED_TRACE(mode);
    SCOPED_TRACE(shift);
    auto source = Coated(Grid(3, 2, mode), shift);
    CompareCoated(source);
    std::reverse(source.primary.begin(), source.primary.end());
    CompareCoated(source);
  }
  for (unsigned valence : {3u, 4u}) for (unsigned mode = 0; mode < 3; ++mode) {
    SCOPED_TRACE(valence);
    SCOPED_TRACE(mode);
    CompareCoated(Coated(EdgeStar(valence, mode, true)));
  }
  CompareCoated(Coated(DisconnectedFan(), 1));
}
TEST(Type25CoatedStartup, RotatedWarpedSmallSiAndSignedZeroInputsMatchOriginalSource) {
  for (unsigned change = 0; change < 5; ++change) {
    SCOPED_TRACE(change);
    auto source = Coated(Grid(3, 2, 2));
    for (std::size_t i = 0; i < source.ids.size(); ++i) {
      auto& x = source.positions[3 * i];
      auto& y = source.positions[3 * i + 1];
      auto& z = source.positions[3 * i + 2];
      if (change == 0) { const auto old = x; x = .8 * x - .6 * z; z = .6 * old + .8 * z; }
      if (change == 1) z += .125 * x * y;
      if (change == 2) { x *= 1e-12; y *= 1e-12; z *= 1e-12; }
      if (change == 3) { x *= .001; y *= .001; z *= .001; }
      if (change == 4) { if (x == 0) x = -0.; if (y == 0) y = -0.; if (z == 0) z = -0.; }
    }
    if (change == 3) source.units = s::Coordinates::Si;
    CompareCoated(source);
  }
}
TEST(Type25CoatedStartup, OriginalRoleProvenanceSurvivesIdenticalNormalizedGeometry) {
  auto forward = Coated(Grid(1, 1), 1);
  auto reversed = forward;
  reversed.primary[0].side_role = s::ShellSideRole::CoatingReversed;
  std::swap(reversed.primary[0].nodes[0], reversed.primary[0].nodes[1]);
  std::swap(reversed.primary[0].nodes[2], reversed.primary[0].nodes[3]);
  const CoatedBuilt a(forward), b(reversed);
  SameStarter(a.startup, Oracle(forward.Input(), forward.coefficients.data(), forward.coefficients.size()));
  SameStarter(b.startup, Oracle(reversed.Input(), reversed.coefficients.data(), reversed.coefficients.size()));
  EXPECT_NE(a.startup.primary_roles[0], b.startup.primary_roles[0]);
  for (std::size_t i = 0; i < a.startup.main_count; ++i) for (unsigned k = 0; k < 4; ++k) {
    EXPECT_EQ(a.startup.mains[i].nodes[k], b.startup.mains[i].nodes[k]);
    EXPECT_EQ(a.startup.mains[i].neighbors[k], b.startup.mains[i].neighbors[k]);
    Same(a.startup.starter.face_normals[4 * i + k], b.startup.starter.face_normals[4 * i + k]);
  }
  // Snapshot owns its role copy; no borrow of the mutable source table remains.
  forward.primary[0].side_role = s::ShellSideRole::Ordinary;
  EXPECT_EQ(a.startup.primary_roles[0], s::ShellSideRole::CoatingForward);
}
TEST(Type25CoatedStartup, LegacyScopeAndExactSizeForecastRemainSeparate) {
  auto source = Grid(3, 1, 2);
  Built old(source);
  const auto legacy = s::Preflight(source.ids.size(), source.primary.size());
  const auto legacy_input = s::Preflight(source.Input());
  EXPECT_EQ(legacy.output_bytes, legacy_input.output_bytes);
  EXPECT_EQ(legacy.scratch_bytes, legacy_input.scratch_bytes);
  EXPECT_EQ(old.startup.primary_roles, nullptr);
  EXPECT_EQ(old.startup.primary_role_count, 0u);
  auto resolved = Coated(source);
  const CoatedBuilt built(resolved);
  const auto ordinary = s::Preflight(GeneralInput(source));
  EXPECT_EQ(built.forecast.output_bytes, ordinary.output_bytes + source.primary.size() * sizeof(s::ShellSideRole));
  const auto align_points = [](std::size_t bytes) {
    constexpr auto alignment = alignof(n::Vector);
    return (bytes + alignment - 1) / alignment * alignment;
  };
  // The copied role table is the last output region. Private points follow the
  // identical staging prefix with their own alignment; existing padding counts.
  const auto staged_delta = align_points(built.forecast.output_bytes) - align_points(ordinary.output_bytes);
  EXPECT_EQ(built.forecast.scratch_bytes, ordinary.scratch_bytes + staged_delta);
  EXPECT_EQ(built.forecast.ready_output_bytes, 0u);
  EXPECT_EQ(built.forecast.ready_scratch_bytes, 0u);
  RecordProperty("primary_face_size", std::to_string(sizeof(s::PrimaryFace)));
  RecordProperty("role_size", std::to_string(sizeof(s::ShellSideRole)));
  RecordProperty("snapshot_size", std::to_string(sizeof(s::Snapshot)));
  s::FixedMainView view = old.ready;
  const auto bytes = ArenaBytes(old.ready_output);
  // BuildFixedMain rejects before constructing or writing any ready output.
  EXPECT_EQ(s::BuildFixedMain(resolved.Input(), built.startup,
      {resolved.coefficients.data(), resolved.coefficients.size()}, {},
      old.ready_output, old.ready_scratch, &view).status,
      s::Status::UnsupportedProfile);
  EXPECT_EQ(std::memcmp(bytes.data(), old.ready_output.data(), bytes.size()), 0);
}
TEST(Type25CoatedStartup, InvalidRolesOriginsCapsAndAliasingPreserveOutputThenRetry) {
  auto source = Coated(Grid(3, 1, 2));
  CoatedBuilt built(source);
  const auto before = ArenaBytes(built.output);
  const auto old = built.startup;
  auto input = source.Input();
  const auto role = source.primary.back().side_role;
  source.primary.back().side_role = static_cast<s::ShellSideRole>(99);
  EXPECT_EQ(s::BuildStarter(input, {}, built.output, built.scratch, &built.startup).status, s::Status::UnsupportedProfile);
  source.primary.back().side_role = role;
  for (auto profile : {s::Profile::OrdinaryExteriorFixedMain, s::Profile::OrdinaryExteriorMovingMain,
       static_cast<s::Profile>(99)}) {
    input.profile = profile;
    EXPECT_EQ(s::Preflight(input).status, s::Status::UnsupportedProfile);
    EXPECT_EQ(s::BuildStarter(input, {}, built.output, built.scratch, &built.startup).status, s::Status::UnsupportedProfile);
  }
  input = source.Input(); input.topology = static_cast<s::TopologyPolicy>(99);
  EXPECT_EQ(s::Preflight(input).status, s::Status::UnsupportedProfile);
  input = source.Input();
  auto cap = s::Limits{}; cap.max_output_bytes = built.forecast.output_bytes - 1;
  EXPECT_EQ(s::BuildStarter(input, cap, built.output, built.scratch, &built.startup).status, s::Status::ResourceLimit);
  cap = {}; cap.max_scratch_bytes = built.forecast.scratch_bytes - 1;
  EXPECT_EQ(s::BuildStarter(input, cap, built.output, built.scratch, &built.startup).status, s::Status::ResourceLimit);
  EXPECT_EQ(s::BuildStarter(input, {}, built.scratch, built.scratch, &built.startup).status, s::Status::InvalidInput);
  // A matched ordinary profile still cannot silently consume a coating role.
  input.profile = s::Profile::OrdinaryExteriorMovingMain;
  input.topology = s::TopologyPolicy::NativeOrdinaryShell;
  EXPECT_EQ(s::BuildStarter(input, {}, built.output, built.scratch, &built.startup).status, s::Status::UnsupportedProfile);
  EXPECT_EQ(std::memcmp(before.data(), built.output.data(), before.size()), 0);
  EXPECT_EQ(built.startup.mains, old.mains);
  EXPECT_EQ(built.startup.primary_roles, old.primary_roles);
  cap = {}; cap.max_output_bytes = built.forecast.output_bytes; cap.max_scratch_bytes = built.forecast.scratch_bytes;
  EXPECT_EQ(s::BuildStarter(source.Input(), cap, built.output, built.scratch, &built.startup).status, s::Status::Ok);
  EXPECT_EQ(std::memcmp(before.data(), built.output.data(), before.size()), 0);
}
} // namespace type25_startup_test
