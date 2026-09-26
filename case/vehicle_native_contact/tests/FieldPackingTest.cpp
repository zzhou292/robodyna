#include "../FieldPacking.h"
#include "lib_utest/qualification/radioss_type25_fixed_main_startup/Fixture.h"
#include "lib_src/math/ScalarBits.h"
#include <gtest/gtest.h>
namespace crash::cases::vehicle_native_contact::detail::test {
namespace old = type25_startup_test;
struct Fixture {
    old::Case source = old::Grid(2, 1, 2);
    old::Built built{source};
    std::vector<native::lifecycle::Node> nodes;
    std::vector<std::uint32_t> nsv;
    std::vector<double> coefficients, gaps;
    std::vector<native::source_gaps::MainGapFields> main_gaps;
    Fixture() {
        for (auto id : source.ids) {
            nodes.push_back({id, 0, 0});
            nsv.push_back(static_cast<std::uint32_t>(nsv.size()));
            coefficients.push_back(200. + nsv.size());
            gaps.push_back(.5);
        }
        main_gaps.resize(built.startup.main_count);
        for (auto& gap : main_gaps) {
            gap.corner[0] = .25;
            gap.corner[1] = .5;
            gap.corner[2] = .75;
            gap.corner[3] = 1.;
            gap.maximum = 1.;
        }
    }
    FieldInputs Input() const {
        return {built.startup, {nodes.data(), nodes.size()},
            {source.coefficients.data(), source.coefficients.size()}, {main_gaps.data(), main_gaps.size()},
            {nsv.data(), nsv.size()}, {coefficients.data(), coefficients.size()}, {gaps.data(), gaps.size()}};
    }
};
TEST(NativeVehicleFieldPacking, PreservesGenuineStarterAndReadyPhasesAndSourceRoster) {
    Fixture f;
    const auto starter = FieldPacking::Starter(f.Input());
    const auto ready = FieldPacking::FixedReady(f.Input(), f.built.ready);
    EXPECT_EQ(starter.phase(), NormalPhase::StarterBeforeInitialContact);
    EXPECT_EQ(ready.phase(), NormalPhase::FixedReady);
    EXPECT_EQ(starter.view().normals, f.built.startup.starter.references);
    EXPECT_EQ(ready.view().normals, f.built.ready.normals.references);
    EXPECT_EQ(starter.view().nodes, f.nodes.data());
    EXPECT_EQ(starter.view().normal_to_main.entries, f.built.startup.normal_mains);
    EXPECT_EQ(starter.view().removed_main_by_secondary.offsets, nullptr);
    EXPECT_EQ(starter.view().removed_main_by_secondary.offset_count, 0u);
    for (std::size_t i = 0; i < f.built.startup.main_count; ++i) {
        for (unsigned k = 0; k < 4; ++k) {
            const auto& a = starter.view().mains[i].normal_slot[k];
            const auto& b = f.built.startup.starter.face_normals[4 * i + k];
            const auto& x = ready.view().mains[i].normal_slot[k];
            const auto& y = f.built.ready.normals.face_normals[4 * i + k];
            EXPECT_TRUE(tl::math::SameScalarBits(a.x, b.x));
            EXPECT_TRUE(tl::math::SameScalarBits(a.y, b.y));
            EXPECT_TRUE(tl::math::SameScalarBits(a.z, b.z));
            EXPECT_TRUE(tl::math::SameScalarBits(x.x, y.x));
            EXPECT_TRUE(tl::math::SameScalarBits(x.y, y.y));
            EXPECT_TRUE(tl::math::SameScalarBits(x.z, y.z));
        }
    }
    for (std::size_t i = 0; i < f.nsv.size(); ++i) {
        EXPECT_EQ(starter.view().secondary[i].node, f.nsv[i]);
        EXPECT_EQ(starter.view().secondary[i].initial_contact_flag, 0);
    }
}
TEST(NativeVehicleFieldPacking, MoveRebindsOwnedArraysWhileBorrowedAuthorityStaysExplicit) {
    Fixture f;
    auto first = FieldPacking::Starter(f.Input());
    const auto before = first.view();
    auto moved = std::move(first);
    EXPECT_EQ(moved.view().mains, before.mains);
    EXPECT_EQ(moved.view().secondary, before.secondary);
    EXPECT_EQ(moved.view().nodes, f.nodes.data());
    EXPECT_EQ(moved.view().generation, f.built.startup.source_generation);
    auto foreign = f.built.ready;
    ++foreign.source_generation;
    EXPECT_THROW(FieldPacking::FixedReady(f.Input(), foreign), std::exception);
    EXPECT_EQ(moved.view().mains, before.mains);
    EXPECT_NO_THROW(FieldPacking::FixedReady(f.Input(), f.built.ready));
}
TEST(NativeVehicleFieldPacking, CountAndCapRejectBeforePayloadAndExactCapacityRetries) {
    Fixture f;
    auto input = f.Input();
    const auto exact = FieldPacking::Preflight(input).retained_bytes;
    FieldPackingLimits limits;
    limits.host_bytes = exact;
    EXPECT_NO_THROW(FieldPacking::Starter(input, limits));
    --limits.host_bytes;
    EXPECT_THROW(FieldPacking::Starter(input, limits), std::exception);
    auto bad_topology = f.built.startup;
    bad_topology.main_count = FieldPackingLimits{}.mains + 1;
    bad_topology.mains = reinterpret_cast<const native::startup::Main*>(std::uintptr_t{1});
    FieldInputs bad{bad_topology, input.nodes, input.main_coefficients, input.main_gaps,
        input.secondary_nodes, input.secondary_coefficients, input.secondary_gaps};
    EXPECT_THROW(FieldPacking::Preflight(bad), std::exception);
    auto malformed = input;
    malformed.secondary_gaps = {};
    EXPECT_THROW(FieldPacking::Starter(malformed), std::exception);
    EXPECT_NO_THROW(FieldPacking::Starter(input));
}
} // namespace crash::cases::vehicle_native_contact::detail::test
