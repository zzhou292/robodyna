// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include "../radioss_type25_fixed_main_startup/CoatedCases.h"
#include <climits>
#include <limits>
#include <new>
namespace type25_current_normals_test {
namespace {
using CoatedFixture = FixtureT<type25_startup_test::CoatedBuilt>;
using type25_startup_test::Coated;
using type25_startup_test::EdgeStar;
struct Publication {
  std::vector<n::StoredNormal> normals;
  std::vector<s::NormalReference> references;
  explicit Publication(const c::Input& in) : normals(in.prior_count, {3, -0.f, 5}), references(in.topology.references) {
    for (auto& ref : references) {
      ref.boundary = 17;
      ref.bisector[0] = {9, 8, 7};
      ref.bisector[1] = {6, 5, 4};
    }
  }
  c::Output Output() { return {normals.data(), normals.size(), references.data(), references.size()}; }
};
void Preserved(const Publication& actual, const Publication& before) {
  SameNormals(actual.normals, before.normals);
  SameReferences(actual.references, before.references);
}
}
TEST(CoatedCurrentNormals, GenuineMixedStartupAndNativeCacheEvolveIndependently) {
  const std::vector<Case> sources{Coated(Grid(4, 3, 2)), Coated(EdgeStar(3, 1), 1),
      Coated(EdgeStar(4, 2, true), 2)};
  for (std::size_t shape = 0; shape < sources.size(); ++shape) {
    SCOPED_TRACE(shape);
    CoatedFixture f(sources[shape]);
    for (unsigned step = 0; step < 5; ++step) {
      SCOPED_TRACE(step);
      for (std::size_t i = 0; i < f.mesh.ids.size(); ++i) {
        const double x = f.mesh.positions[3*i], y = f.mesh.positions[3*i+1], z = f.mesh.positions[3*i+2];
        f.positions[3*i] = x + .025 * step * y;
        f.positions[3*i+1] = y + .125 * step * z;
        f.positions[3*i+2] = z + .03125 * step * x * y;
      }
      if (step == 0) f.AllActive();
      else if (step == 1) f.GeneratedMasks({2});
      else if (step == 2) f.GeneratedMasks({}, 2, {1});
      else if (step == 3) f.GeneratedMasks();
      else {
        f.coefficients[1] = 0;
        f.coefficients[f.mesh.primary.size()+1] = 0;
        f.RefreshFree();
        f.GeneratedMasks();
      }
      const auto native = Oracle(f.Input(true));
      const auto actual = EvaluateHostNormals(f.Input());
      Same(actual, native);
      f.prior = actual.normals;
      f.native_prior = native.normals;
    }
  }
}
TEST(CoatedCurrentNormals, DeclaredCoatingCacheRetainsInactiveAndTriangleUnusedBits) {
  auto closed = Coated(Cube(), 1);
  for (auto& face : closed.primary) face.side_role = s::ShellSideRole::CoatingForward;
  CoatedFixture f(closed);
  f.GeneratedMasks();
  ASSERT_TRUE(f.free_ids.empty());
  ASSERT_TRUE(std::all_of(f.main_active.begin(), f.main_active.end(), [](auto v) { return v == 0; }));
  const auto prior = f.prior;
  for (std::size_t i = 0; i < f.mesh.ids.size(); ++i) f.positions[3*i+2] += .07 * double(i);
  const auto native = Oracle(f.Input(true));
  const auto actual = EvaluateHostNormals(f.Input());
  Same(actual, native);
  SameNormals(actual.normals, prior);

  CoatedFixture tri(Coated(Grid(1, 1, 1), 1));
  for (std::size_t main = 0; main < tri.built.startup.main_count; ++main) {
    const n::StoredNormal seed{float(main+1), -0.f, .125f};
    tri.prior[4*main+2] = seed;
    tri.native_prior[4*main+2] = seed;
  }
  Same(EvaluateHostNormals(tri.Input()), Oracle(tri.Input(true)));
  const auto first = EvaluateHostNormals(tri.Input());
  for (std::size_t main = 0; main < tri.built.startup.main_count; ++main)
    SameNormal(first.normals[4*main+2], tri.prior[4*main+2]);
  std::fill(tri.coefficients.begin(), tri.coefficients.end(), 0);
  tri.RefreshFree();
  const auto zero = EvaluateHostNormals(tri.Input());
  Same(zero, Oracle(tri.Input(true)));
  for (const auto normal : zero.normals) SameNormal(normal, {});
}
TEST(CoatedCurrentNormals, Engine129BoundarySiAndLargeEncodedPartnerRemainNative) {
  CoatedFixture f(Coated(Grid(11, 13)));
  ASSERT_EQ(f.mesh.primary.size(), 143u);
  ASSERT_GT(f.built.startup.mains.back().segment_type < 0 ? -f.built.startup.mains.back().segment_type : 0,
      int(f.built.startup.main_count));
  Same(EvaluateHostNormals(f.Input()), Oracle(f.Input(true)));
  f.mesh.units = s::Coordinates::Si;
  for (auto& position : f.positions) position *= f.mesh.scale.length_m;
  Same(EvaluateHostNormals(f.Input()), Oracle(f.Input(true)));
}
TEST(CoatedCurrentNormals, MalformedRoleTableOriginAndPartnerRejectBeforePublication) {
  for (unsigned fault = 0; fault < 12; ++fault) {
    SCOPED_TRACE(fault);
    CoatedFixture f(Coated(Grid(3, 1, 2)));
    auto input = f.Input();
    c::Forecast plan;
    ASSERT_EQ(c::Preflight(input, CoatedFixture::Limits(), plan).status, c::Status::Ok);
    tl::util::HostArena scratch;
    ASSERT_TRUE(scratch.Initialize(plan.scratch_bytes));
    std::vector<s::Main> mains(input.topology.mains, input.topology.mains + input.topology.main_count);
    std::vector<s::ShellSideRole> roles(input.topology.primary_roles, input.topology.primary_roles + input.topology.primary_role_count);
    input.topology.mains = mains.data();
    input.topology.primary_roles = roles.data();
    Publication output(input);
    const auto before = output;
    if (fault == 0) input.profile = c::Profile::OrdinaryShellLocal;
    if (fault == 1) input.topology.source_profile = s::Profile::OrdinaryExteriorMovingMain;
    if (fault == 2) input.topology.source_topology = s::TopologyPolicy::NativeOrdinaryShell;
    if (fault == 3) --input.topology.primary_role_count;
    if (fault == 4) ++input.topology.primary_role_count;
    if (fault == 5) input.topology.primary_roles = nullptr;
    if (fault == 6) roles.back() = static_cast<s::ShellSideRole>(99);
    if (fault == 7) roles[1] = s::ShellSideRole::Ordinary;
    if (fault == 8) mains[1].segment_type = INT_MIN;
    if (fault == 9) mains[1].segment_type = int(2 * input.topology.main_count + 1);
    if (fault == 10) mains[input.topology.primary_count+1].segment_type = -1;
    if (fault == 11) input.topology.source_profile = static_cast<s::Profile>(99);
    EXPECT_NE(c::Evaluate(input, CoatedFixture::Limits(), scratch.data(), scratch.bytes(), output.Output()).status, c::Status::Ok);
    Preserved(output, before);
    ASSERT_EQ(c::Evaluate(f.Input(), CoatedFixture::Limits(), scratch.data(), scratch.bytes(), output.Output()).status, c::Status::Ok);
    const auto native = Oracle(f.Input(true));
    SameNormals(output.normals, native.normals);
    SameReferences(output.references, native.references);
  }
}
TEST(CoatedCurrentNormals, LiveRoleScratchAliasAndCapacityFailurePreserveThenRetry) {
  CoatedFixture f(Coated(Grid(3, 1, 2)));
  auto input = f.Input();
  c::Forecast plan;
  ASSERT_EQ(c::Preflight(input, CoatedFixture::Limits(), plan).status, c::Status::Ok);
  tl::util::HostArena scratch;
  ASSERT_TRUE(scratch.Initialize(plan.scratch_bytes));
  // These are live role objects in raw arena storage, not a cast of another
  // object type. Admission must reject overlap before constructing work arrays.
  const tl::util::ArenaRegion region{0, input.topology.primary_count,
      input.topology.primary_count * sizeof(s::ShellSideRole)};
  auto* roles = scratch.Construct<s::ShellSideRole>(region);
  std::copy_n(input.topology.primary_roles, input.topology.primary_count, roles);
  input.topology.primary_roles = roles;
  Publication output(input);
  const auto before = output;
  EXPECT_EQ(c::Evaluate(input, CoatedFixture::Limits(), scratch.data(), scratch.bytes(), output.Output()).status, c::Status::InvalidInput);
  Preserved(output, before);
  for (std::size_t i = 0; i < input.topology.primary_count; ++i) EXPECT_EQ(roles[i], f.mesh.primary[i].side_role);
  EXPECT_EQ(c::Evaluate(f.Input(), CoatedFixture::Limits(), scratch.data(), scratch.bytes()-1, output.Output()).status, c::Status::ResourceLimit);
  Preserved(output, before);
  ASSERT_EQ(c::Evaluate(f.Input(), CoatedFixture::Limits(), scratch.data(), scratch.bytes(), output.Output()).status, c::Status::Ok);
  const auto native = Oracle(f.Input(true));
  SameNormals(output.normals, native.normals);
  SameReferences(output.references, native.references);
}
TEST(CoatedCurrentNormals, OrdinaryOriginUnknownEnumsRejectButLegacyUnspecifiedRemainsValid) {
  Fixture f(Grid(1, 1));
  auto input = f.Input();
  ASSERT_EQ(input.topology.source_profile, s::Profile::Unspecified);
  Same(EvaluateHostNormals(input), Oracle(f.Input(true)));
  input.topology.source_profile = static_cast<s::Profile>(99);
  EXPECT_EQ(EvaluateHostNormals(input).report.status, c::Status::UnsupportedProfile);
  input = f.Input();
  input.topology.source_topology = static_cast<s::TopologyPolicy>(99);
  EXPECT_EQ(EvaluateHostNormals(input).report.status, c::Status::UnsupportedProfile);
}
} // namespace type25_current_normals_test
