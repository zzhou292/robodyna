// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include "CudaFixture.h"
#include "../radioss_type25_fixed_main_startup/CoatedCases.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace type25_current_normals_test {
namespace {
using CoatedFixture = FixtureT<type25_startup_test::CoatedBuilt>;
using CoatedNormalsCuda = device::CurrentNormalsCuda;

void SameCoatedStages(const device::Observation& actual, const NativeResult& native,
                      const Result& host) {
    ASSERT_EQ(actual.report.status, c::Status::Ok);
    ASSERT_TRUE(native.finite);
    ASSERT_TRUE(actual.values.finite);
    Same(host, native);
    SameNormals(actual.values.flag1_normals, native.flag1_normals);
    SameNormals(actual.values.normals, native.normals);
    SameReferences(actual.values.references, native.references);
    EXPECT_EQ(actual.values.primary_skip, native.primary_skip);
}

void DeformCoated(CoatedFixture& fixture, unsigned step) {
    const double angle = .13 * step;
    const double cosine = std::cos(angle), sine = std::sin(angle);
    for (std::size_t node = 0; node < fixture.mesh.ids.size(); ++node) {
        const double x = fixture.mesh.positions[3 * node];
        const double y = fixture.mesh.positions[3 * node + 1];
        const double z = fixture.mesh.positions[3 * node + 2] + .025 * step * x * y;
        fixture.positions[3 * node] = cosine * x - sine * z + .2 * step;
        fixture.positions[3 * node + 1] = y + .03 * step * x;
        fixture.positions[3 * node + 2] = sine * x + cosine * z;
    }
}

void AdvanceCoated(CoatedFixture& fixture, const device::Observation& actual,
                   const NativeResult& native) {
    // Independent initial caches and independent update recurrences.
    fixture.prior = actual.values.normals;
    fixture.native_prior = native.normals;
}
} // namespace

TEST_F(CoatedNormalsCuda, DeclaredSideRolesFeedCompleteNativeStagesAcrossCurrentUpdates) {
    using namespace type25_startup_test;
    const std::vector<Case> sources{
        Coated(Grid(3, 2)), Coated(Grid(3, 2, 1)), Coated(Grid(3, 2, 2)),
        Coated(Cube()), Coated(EdgeStar(4, 2, true))};
    for (std::size_t shape = 0; shape < sources.size(); ++shape) {
        for (unsigned threads : {1u, 7u, 32u, 64u}) {
            for (bool reverse : {false, true}) {
                SCOPED_TRACE(shape);
                SCOPED_TRACE(threads);
                SCOPED_TRACE(reverse);
                CoatedFixture fixture(sources[shape]);
                ASSERT_EQ(fixture.built.startup.profile, s::Profile::ResolvedShellSides);
                ASSERT_EQ(fixture.built.startup.topology, s::TopologyPolicy::NativeResolvedShellSides);
                ASSERT_EQ(fixture.built.startup.primary_role_count, fixture.mesh.primary.size());
                // Explicit fixture roles go separately through the genuine
                // production builder and original native startup. These tests
                // do not infer shell/solid membership or alter a user model.
                for (unsigned step = 0; step < 4; ++step) {
                    SCOPED_TRACE(step);
                    DeformCoated(fixture, step);
                    if (step == 0) fixture.AllActive();
                    else if (step == 1) fixture.GeneratedMasks({2});
                    else if (step == 2) fixture.GeneratedMasks({}, 3, {2});
                    else {
                        fixture.coefficients[1] = 0;
                        fixture.coefficients[fixture.mesh.primary.size() + 1] = 0;
                        fixture.RefreshFree();
                        fixture.GeneratedMasks();
                    }
                    const auto native = Oracle(fixture.Input(true));
                    const auto host = EvaluateHostNormals(fixture.Input());
                    const auto actual = EvaluateDevice(fixture.Input(), threads, reverse, CoatedFixture::Limits());
                    ASSERT_NO_FATAL_FAILURE(SameCoatedStages(actual, native, host));
                    AdvanceCoated(fixture, actual, native);
                }
            }
        }
    }
}

TEST_F(CoatedNormalsCuda, TriangleUnusedChannelsKeepTheirPriorBitsUntilNativeZeroing) {
    using namespace type25_startup_test;
    for (unsigned shift : {0u, 1u, 2u}) {
        for (unsigned threads : {1u, 32u, 64u}) {
            for (bool reverse : {false, true}) {
                SCOPED_TRACE(shift);
                SCOPED_TRACE(threads);
                SCOPED_TRACE(reverse);
                CoatedFixture fixture(Coated(Grid(2, 2, 1), shift));
                for (std::size_t main = 0; main < fixture.built.startup.main_count; ++main) {
                    const n::StoredNormal seed{float(main + 1), -0.f, .125f};
                    fixture.prior[4 * main + 2] = seed;
                    fixture.native_prior[4 * main + 2] = seed;
                }
                const auto initial = fixture.prior;
                for (unsigned step = 0; step < 3; ++step) {
                    SCOPED_TRACE(step);
                    DeformCoated(fixture, step);
                    fixture.AllActive();
                    if (step == 2) {
                        std::fill(fixture.coefficients.begin(), fixture.coefficients.end(), 0.);
                        fixture.RefreshFree();
                    }
                    const auto native = Oracle(fixture.Input(true));
                    const auto host = EvaluateHostNormals(fixture.Input());
                    const auto actual = EvaluateDevice(fixture.Input(), threads, reverse, CoatedFixture::Limits());
                    ASSERT_NO_FATAL_FAILURE(SameCoatedStages(actual, native, host));
                    for (std::size_t main = 0; main < fixture.built.startup.main_count; ++main)
                        SameNormal(actual.values.normals[4 * main + 2],
                            step == 2 ? n::StoredNormal{} : initial[4 * main + 2]);
                    AdvanceCoated(fixture, actual, native);
                }
            }
        }
    }
}

TEST_F(CoatedNormalsCuda, EncodedPartnersAndNativeSiBoundaryCrossOriginalNormalCohort) {
    using namespace type25_startup_test;
    for (bool si : {false, true}) {
        for (unsigned threads : {1u, 7u, 32u, 64u}) {
            for (bool reverse : {false, true}) {
                SCOPED_TRACE(si);
                SCOPED_TRACE(threads);
                SCOPED_TRACE(reverse);
                CoatedFixture fixture(Coated(Grid(11, 13)));
                ASSERT_EQ(fixture.mesh.primary.size(), 143u);
                ASSERT_TRUE(std::any_of(fixture.built.startup.mains,
                    fixture.built.startup.mains + fixture.built.startup.primary_count,
                    [&](const auto& main) { return main.segment_type > int(fixture.built.startup.main_count); }));
                for (unsigned step = 0; step < 3; ++step) {
                    SCOPED_TRACE(step);
                    DeformCoated(fixture, step);
                    fixture.mesh.units = si ? s::Coordinates::Si : s::Coordinates::Native;
                    if (si)
                        for (auto& value : fixture.positions) value *= fixture.mesh.scale.length_m;
                    if (step == 0) fixture.AllActive();
                    else if (step == 1) fixture.GeneratedMasks({2, 142});
                    else fixture.GeneratedMasks({}, 3, {2, 142});
                    const auto native = Oracle(fixture.Input(true));
                    const auto host = EvaluateHostNormals(fixture.Input());
                    const auto actual = EvaluateDevice(fixture.Input(), threads, reverse, CoatedFixture::Limits());
                    ASSERT_NO_FATAL_FAILURE(SameCoatedStages(actual, native, host));
                    AdvanceCoated(fixture, actual, native);
                }
            }
        }
    }
}

TEST_F(CoatedNormalsCuda, RoleCountOriginAndPartnerAdmissionPreservePublicationThenRetry) {
    using namespace type25_startup_test;
    CoatedFixture fixture(Coated(Grid(3, 1)));
    const auto input = fixture.Input();
    const auto native = Oracle(fixture.Input(true));
    const auto host = EvaluateHostNormals(input);
    const auto valid = EvaluateDevice(input, 32, false, CoatedFixture::Limits());
    ASSERT_NO_FATAL_FAILURE(SameCoatedStages(valid, native, host));
    ASSERT_EQ(input.topology.primary_roles[1], s::ShellSideRole::CoatingForward);
    for (unsigned fault = 0; fault < 9; ++fault) {
        SCOPED_TRACE(fault);
        auto bad = input;
        auto cap = CoatedFixture::Limits();
        std::vector<s::Main> mains(input.topology.mains,
            input.topology.mains + input.topology.main_count);
        std::vector<s::ShellSideRole> roles(input.topology.primary_roles,
            input.topology.primary_roles + input.topology.primary_count);
        switch (fault) {
        case 0: bad.topology.primary_roles = nullptr; break;
        case 1: --bad.topology.primary_role_count; break;
        case 2:
            roles[1] = static_cast<s::ShellSideRole>(99);
            bad.topology.primary_roles = roles.data();
            break;
        case 3: bad.topology.source_profile = s::Profile::OrdinaryExteriorMovingMain; break;
        case 4: bad.topology.source_topology = s::TopologyPolicy::NativeOrdinaryShell; break;
        case 5: bad.profile = c::Profile::OrdinaryShellLocal; break;
        case 6:
            mains[1].segment_type -= int(input.topology.main_count);
            bad.topology.mains = mains.data();
            break;
        case 7:
            mains[input.topology.primary_count + 1].segment_type += int(input.topology.main_count);
            bad.topology.mains = mains.data();
            break;
        case 8: cap.references = input.topology.references - 1; break;
        }
        // Every supplied host span has genuine complete backing. Malformed
        // counts/origins are carried unchanged into device admission; no invalid
        // host address is manufactured by the packing helper.
        const auto expected = EvaluateHostNormals(bad, cap);
        ASSERT_NE(expected.report.status, c::Status::Ok);
        const auto rejected = EvaluateDevice(bad, 64, true, cap);
        SameReport(rejected.report, expected.report);
        EXPECT_TRUE(rejected.publication_unchanged);
        SameNormals(rejected.values.flag1_normals, valid.values.flag1_normals);
        SameNormals(rejected.values.normals, valid.values.normals);
        SameReferences(rejected.values.references, valid.values.references);
        EXPECT_EQ(rejected.values.primary_skip, valid.values.primary_skip);
        ASSERT_NO_FATAL_FAILURE(SameCoatedStages(
            EvaluateDevice(input, 7, false, CoatedFixture::Limits()), native, host));
    }
}
} // namespace type25_current_normals_test
