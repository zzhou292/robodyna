// SPDX-License-Identifier: AGPL-3.0-or-later
#include "InventoryFixture.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
namespace candidate_test {
namespace {
c::Limits CompactLimits(std::size_t pairs = 16384, std::size_t tasks = 512,
                        std::size_t encounters = 65536) {
    auto limits = Limits(pairs, tasks);
    limits.strategy = c::EnumerationStrategy::CompactGrid;
    limits.max_encounters = encounters;
    return limits;
}
void CheckCompact(c::Inventory& inventory, const std::vector<c::Pair>& expected, std::size_t rows) {
    const auto view = inventory.view();
    ASSERT_TRUE(inventory.IsCurrent(view));
    ASSERT_EQ(view.pair_count(), expected.size());
    ASSERT_EQ(view.secondary_count(), rows);
    std::vector<c::Pair> pairs(view.pair_count());
    std::vector<std::uint64_t> offsets(rows + 1);
    if (!pairs.empty()) {
        ASSERT_EQ(cudaMemcpy(pairs.data(), view.pairs(), pairs.size() * sizeof(c::Pair), cudaMemcpyDeviceToHost), cudaSuccess);
    }
    ASSERT_EQ(cudaMemcpy(offsets.data(), view.secondary_offsets(), offsets.size() * sizeof(std::uint64_t),
                         cudaMemcpyDeviceToHost), cudaSuccess);
    for (std::size_t i = 0; i < expected.size(); ++i) {
        SCOPED_TRACE(i);
        EXPECT_EQ(pairs[i].secondary_row, expected[i].secondary_row);
        EXPECT_EQ(pairs[i].main_occurrence, expected[i].main_occurrence);
    }
    for (std::size_t row = 0; row <= rows; ++row) {
        const auto first = std::lower_bound(expected.begin(), expected.end(), row,
            [](c::Pair pair, std::size_t value) { return pair.secondary_row < value; });
        EXPECT_EQ(offsets[row], std::size_t(first - expected.begin()));
    }
    EXPECT_EQ(inventory.last_report().strategy, c::EnumerationStrategy::CompactGrid);
    EXPECT_TRUE(inventory.last_report().encounters_counted);
    EXPECT_TRUE(inventory.last_report().pairs_counted);
}
void Rotate(Scene& scene, unsigned axis) {
    for (std::size_t node = 0; node < scene.ids.size(); ++node) {
        double position[3], velocity[3];
        for (unsigned k = 0; k < 3; ++k) {
            position[k] = scene.positions[3 * node + k];
            velocity[k] = scene.velocities[3 * node + k];
        }
        for (unsigned k = 0; k < 3; ++k) {
            scene.positions[3 * node + (k + axis) % 3] = position[k];
            scene.velocities[3 * node + (k + axis) % 3] = velocity[k];
        }
    }
}
void PatchLayout(Scene& scene, double spacing) {
    for (std::size_t main = 0; main < scene.mains.size(); ++main) {
        const double y = spacing * (main % 4), z = spacing * (main / 4);
        const double xyz[12]{-.75, y - .75, z, .75, y - .75, z,
                            .75, y + .75, z + .0625 * (main % 2), -.75, y + .75, z};
        for (unsigned corner = 0; corner < 4; ++corner)
            for (unsigned axis = 0; axis < 3; ++axis)
                scene.positions[3 * (4 * main + corner) + axis] = xyz[3 * corner + axis];
    }
    // Set physical nodes once: repeated native role occurrences retain the
    // same node backing, and the existing own-node/removal exclusions remain.
    for (std::size_t node = 4 * scene.mains.size(); node < scene.ids.size(); ++node) {
        const auto row = node - 4 * scene.mains.size(), patch = row % scene.mains.size();
        scene.positions[3 * node] = .125 * double(int(row % 5) - 2);
        scene.positions[3 * node + 1] = spacing * (patch % 4) + .0625 * (row % 2);
        scene.positions[3 * node + 2] = spacing * (patch / 4) + .0625 * (1 + row % 3);
    }
}
}
TEST(NativeCandidateCompactGrid, CompleteNativePairsAndOffsetsAcrossAxesDensityAndBorrowedUpdates) {
    for (unsigned axis = 0; axis < 3; ++axis) for (double spacing : {4., .125}) {
        SCOPED_TRACE(axis);
        SCOPED_TRACE(spacing);
        Scene scene(12, 512);
        PatchLayout(scene, spacing);
        Rotate(scene, axis);
        c::Inventory compact, legacy;
        const auto limits = CompactLimits();
        c::Forecast plan, old;
        ASSERT_EQ(c::Inventory::Preflight(scene.Source(), limits, plan), c::Status::Ok);
        ASSERT_EQ(c::Inventory::Preflight(scene.Source(), Limits(16384, 512), old), c::Status::Ok);
        EXPECT_GT(plan.index_device_bytes, 0u);
        EXPECT_LE(plan.index_device_bytes, plan.device_bytes);
        EXPECT_EQ(old.index_device_bytes, 0u);
        ASSERT_EQ(compact.Initialize(scene.Source(), limits, scene.stream), c::Status::Ok);
        ASSERT_EQ(legacy.Initialize(scene.Source(), Limits(16384, 512), scene.stream), c::Status::Ok);
        auto current = scene.Current();
        for (unsigned step = 0; step < 3; ++step) {
            SCOPED_TRACE(step);
            current.stamp.attempt = current.stamp.geometry = current.stamp.gaps = step + 1;
            const auto expected = Reference(scene, current); // Original native Screen, COR3T and PEN3.
            ASSERT_FALSE(expected.empty());
            ASSERT_EQ(compact.Stage(current), c::Status::Ok);
            ASSERT_NO_FATAL_FAILURE(CheckCompact(compact, expected, scene.secondaries.size()));
            ASSERT_EQ(legacy.Stage(current), c::Status::Ok);
            EXPECT_EQ(legacy.view().pair_count(), expected.size());
            EXPECT_LE(compact.last_report().envelope_encounters, legacy.last_report().envelope_encounters);
            if (axis == 0 && spacing == 4.)
                EXPECT_LT(compact.last_report().envelope_encounters, legacy.last_report().envelope_encounters / 2);
            for (std::size_t node = 0; node < scene.ids.size(); ++node)
                scene.positions[3 * node + (axis + 2) % 3] += .0078125 * double(node % 2);
            for (std::size_t row = 0; row < scene.secondaries.size(); ++row) scene.secondary_gaps[row] += .0078125;
        }
    }
}
TEST(NativeCandidateCompactGrid, DegenerateActiveAxesTranslatedBinBoundariesAndEmptyRolesMatchNative) {
    for (unsigned axis = 0; axis < 3; ++axis) for (double shift : {0., 0x1p20}) for (bool coincident : {false, true}) {
        SCOPED_TRACE(axis);
        SCOPED_TRACE(shift);
        SCOPED_TRACE(coincident);
        Scene scene(1, 18);
        scene.secondaries[2] = scene.secondaries[0]; // Active cloud has no borrowed main corner.
        for (std::size_t i = 0; i < scene.positions.size; ++i) {
            scene.positions[i] += shift;
            scene.velocities[i] = 0;
        }
        // An active cloud with two degenerate axes, or all three. The finite
        // endpoints fix a span3, and interior dyadic/ULP points exercise cell
        // boundaries independently of final native membership expectations.
        for (std::size_t row = 0; row < 18; ++row) {
            double x = shift;
            if (!coincident) {
                if (row == 0) x = shift - 1.5;
                else if (row == 1) x = shift + 1.5;
                else {
                    const double center = shift + .75 * double(int((row - 2) / 3) % 3 - 1);
                    x = row % 3 == 0 ? std::nextafter(center, -INFINITY) :
                        row % 3 == 1 ? center : std::nextafter(center, INFINITY);
                }
            }
            const auto node = 4 + row;
            scene.positions[3 * node] = x;
            scene.positions[3 * node + 1] = shift;
            scene.positions[3 * node + 2] = shift + .125;
        }
        scene.curvature[0] = 0;
        Rotate(scene, axis);
        auto expected_input = scene.Current();
        expected_input.stored_motion = 0;
        expected_input.domain = {{shift - 100, shift - 100, shift - 100}, {shift + 100, shift + 100, shift + 100}};
        const auto expected = Reference(scene, expected_input);
        ASSERT_FALSE(expected.empty());
        auto current = expected_input;
        current.domain_policy = c::DomainPolicy::AllFinite;
        current.domain = {{NAN, NAN, NAN}, {NAN, NAN, NAN}}; // Unconsumed in this real profile.
        c::Inventory inventory;
        ASSERT_EQ(inventory.Initialize(scene.Source(), CompactLimits(), scene.stream), c::Status::Ok);
        ASSERT_EQ(inventory.Stage(current), c::Status::Ok);
        ASSERT_NO_FATAL_FAILURE(CheckCompact(inventory, expected, scene.secondaries.size()));
    }
    {
        Scene scene(2, 20);
        const double far = std::numeric_limits<double>::max();
        scene.positions[3 * scene.secondaries[16]] = -far;
        scene.positions[3 * scene.secondaries[17]] = far;
        auto reference = scene.Current();
        reference.domain = {{-far, -far, -far}, {far, far, far}};
        const auto expected = Reference(scene, reference);
        ASSERT_FALSE(expected.empty());
        EXPECT_TRUE(std::none_of(expected.begin(), expected.end(), [](c::Pair pair) {
            return pair.secondary_row == 16 || pair.secondary_row == 17;
        }));
        // The active coordinates are finite but their x span overflows. Native
        // strict screens reject far rows before distance arithmetic. A compact
        // index must conservatively use one bin, not reject those inputs.
        auto current = reference;
        current.domain_policy = c::DomainPolicy::AllFinite;
        current.domain = {{NAN, NAN, NAN}, {NAN, NAN, NAN}};
        c::Inventory inventory;
        ASSERT_EQ(inventory.Initialize(scene.Source(), CompactLimits(), scene.stream), c::Status::Ok);
        ASSERT_EQ(inventory.Stage(current), c::Status::Ok);
        ASSERT_NO_FATAL_FAILURE(CheckCompact(inventory, expected, scene.secondaries.size()));
    }
    for (unsigned secondaries : {0u, 7u}) {
        Scene scene(0, secondaries);
        c::Inventory inventory;
        ASSERT_EQ(inventory.Initialize(scene.Source(), CompactLimits(), scene.stream), c::Status::Ok);
        ASSERT_EQ(inventory.Stage(scene.Current()), c::Status::Ok);
        ASSERT_NO_FATAL_FAILURE(CheckCompact(inventory, {}, secondaries));
    }
}
TEST(NativeCandidateCompactGrid, CompleteEncounterTaskAndPairCapsRejectWithoutTruncationAndPreserveAcceptedInventory) {
    Scene scene(3, 900);
    const auto current = scene.Current();
    const auto expected = Reference(scene, current);
    ASSERT_GT(expected.size(), 1u);
    c::Inventory accepted;
    ASSERT_EQ(accepted.Initialize(scene.Source(), CompactLimits(), scene.stream), c::Status::Ok);
    ASSERT_EQ(accepted.Stage(current), c::Status::Ok);
    const auto old = accepted.view();
    const auto report = accepted.last_report();
    ASSERT_GT(report.envelope_encounters, 1u);
    ASSERT_GT(report.tasks, 1u);
    const auto exact_limits = CompactLimits(expected.size(), report.tasks, report.envelope_encounters);
    c::Inventory exact;
    ASSERT_EQ(exact.Initialize(scene.Source(), exact_limits, scene.stream), c::Status::Ok);
    ASSERT_EQ(exact.Stage(current), c::Status::Ok);
    ASSERT_NO_FATAL_FAILURE(CheckCompact(exact, expected, scene.secondaries.size()));
    for (unsigned field = 0; field < 3; ++field) {
        SCOPED_TRACE(field);
        auto short_limit = exact_limits;
        if (field == 0) --short_limit.max_encounters;
        else if (field == 1) --short_limit.max_tasks;
        else --short_limit.max_pairs;
        c::Inventory trial;
        ASSERT_EQ(trial.Initialize(scene.Source(), short_limit, scene.stream), c::Status::Ok);
        EXPECT_EQ(trial.Stage(current), c::Status::ResourceLimit);
        const auto failure = trial.last_report();
        EXPECT_TRUE(failure.encounters_counted);
        EXPECT_EQ(failure.envelope_encounters, report.envelope_encounters);
        EXPECT_EQ(failure.pairs_counted, field == 2);
        if (field == 2) EXPECT_EQ(failure.pairs, expected.size());
        EXPECT_EQ(trial.view().pairs(), nullptr);
        EXPECT_TRUE(accepted.IsCurrent(old));
        trial.Discard();
        // Corrected capacity on the independent trial workspace, with the
        // exact same current/source and an untouched accepted inventory.
        ASSERT_EQ(exact.Stage(current), c::Status::Ok);
        ASSERT_NO_FATAL_FAILURE(CheckCompact(exact, expected, scene.secondaries.size()));
    }
    ASSERT_NO_FATAL_FAILURE(CheckCompact(accepted, expected, scene.secondaries.size()));
}
TEST(NativeCandidateCompactGrid, NativeAndSiInputsAndInactiveNonreadsRetainFailureDiscardRetryContract) {
    {
        Scene scene(6, 42);
        auto source = scene.Source();
        source.main_coefficient_domain = tlfea::contact::radioss_type25::MainCoefficientDomain::NativeSigned;
        c::Inventory accepted, trial;
        ASSERT_EQ(accepted.Initialize(source, CompactLimits(), scene.stream), c::Status::Ok);
        ASSERT_EQ(trial.Initialize(source, CompactLimits(), scene.stream), c::Status::Ok);
        auto current = scene.Current();
        ASSERT_EQ(accepted.Stage(current), c::Status::Ok);
        const auto old = accepted.view();
        scene.secondary_stiffness[1] = 0;
        scene.secondary_gaps[1] = NAN;
        const auto inactive = scene.secondaries[1];
        scene.positions[3 * inactive] = NAN;
        scene.velocities[3 * inactive] = NAN;
        scene.main_stiffness[1] = -3;
        scene.main_gaps[1] = scene.curvature[1] = NAN;
        const auto clipped = scene.secondaries[6];
        scene.positions[3 * clipped] = 1000;
        scene.secondary_gaps[6] = scene.velocities[3 * clipped] = NAN;
        const auto screened = scene.secondaries[8];
        scene.positions[3 * screened] = 0;
        scene.positions[3 * screened + 1] = 0;
        scene.positions[3 * screened + 2] = .55;
        scene.secondary_gaps[8] = 0;
        scene.velocities[3 * screened] = NAN;
        // This active finite in-domain row lies inside the shared max-gap box
        // but outside every actual x0 face screen. Its velocity is not consumed.
        c::ScreenRow common;
        for (unsigned corner = 0; corner < 4; ++corner)
            common.vertices[corner] = Read(current, scene.mains[0].nodes[corner]);
        common.secondary = Read(current, screened);
        common.margin = current.margin;
        common.curvature = scene.curvature[0];
        common.secondary_gap = .125;
        common.main_gap = scene.main_gaps[0];
        common.stored_motion = current.stored_motion;
        EXPECT_TRUE(NativeScreen(common));
        common.secondary_gap = 0;
        EXPECT_FALSE(NativeScreen(common));
        ++current.stamp.attempt;
        ++current.stamp.activity;
        const auto expected = Reference(scene, current);
        ASSERT_FALSE(expected.empty());
        EXPECT_TRUE(std::none_of(expected.begin(), expected.end(), [](c::Pair pair) {
            return pair.secondary_row == 8;
        }));
        ASSERT_EQ(trial.Stage(current), c::Status::Ok);
        ASSERT_NO_FATAL_FAILURE(CheckCompact(trial, expected, scene.secondaries.size()));
        const auto prior = trial.view();
        const auto node = scene.secondaries[7];
        const double original = scene.positions[3 * node + 2];
        scene.positions[3 * node + 2] = INFINITY;
        EXPECT_EQ(trial.Stage(current), c::Status::InvalidInput);
        EXPECT_FALSE(trial.IsCurrent(prior));
        EXPECT_EQ(trial.view().pairs(), nullptr);
        EXPECT_TRUE(accepted.IsCurrent(old));
        scene.positions[3 * node + 2] = original;
        ASSERT_EQ(trial.Stage(current), c::Status::Ok);
        ASSERT_NO_FATAL_FAILURE(CheckCompact(trial, expected, scene.secondaries.size()));
        const auto retried = trial.view();
        trial.Discard();
        EXPECT_FALSE(trial.IsCurrent(retried));
        EXPECT_TRUE(accepted.IsCurrent(old));
    }
    {
        Scene scene;
        const auto expected = Reference(scene, scene.Current());
        c::Inventory inventory;
        ASSERT_EQ(inventory.Initialize(scene.Source(c::InputUnits::Si), CompactLimits(), scene.stream), c::Status::Ok);
        for (std::size_t i = 0; i < scene.positions.size; ++i) {
            scene.positions[i] *= .001;
            scene.velocities[i] *= .001;
        }
        for (std::size_t i = 0; i < scene.secondary_gaps.size; ++i) scene.secondary_gaps[i] *= .001;
        for (std::size_t i = 0; i < scene.main_gaps.size; ++i) {
            scene.main_gaps[i] *= .001;
            scene.curvature[i] *= .001;
        }
        auto current = scene.Current();
        current.margin *= .001;
        current.stored_motion *= .001;
        current.domain = {{-.1, -.1, -.1}, {.1, .1, .1}};
        ASSERT_EQ(inventory.Stage(current), c::Status::Ok);
        ASSERT_NO_FATAL_FAILURE(CheckCompact(inventory, expected, scene.secondaries.size()));
    }
}
} // namespace candidate_test
