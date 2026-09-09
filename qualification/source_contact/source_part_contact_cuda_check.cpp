#include "SourceContactCudaFixture.h"
#include "case/CanonicalWallArtifacts.h"
#include "case/WallTessellation.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <sstream>

namespace {
namespace sf = crash::qualification::source_contact;
namespace cf = sf::force;
namespace gpu = sf::cuda_test;
namespace sc = tlfea::contact;
namespace cw = crash::case_data;
std::filesystem::path readiness_path, wall_path;
enum class Profile { Separated, IndependentParents, WholePart };

void Overlap(sc::Q4CertifiedIntegral host, sc::Q4CertifiedIntegral device, double budget) {
    EXPECT_TRUE(std::isfinite(device.value)); EXPECT_TRUE(std::isfinite(device.error));
    EXPECT_TRUE(std::isfinite(device.lower)); EXPECT_TRUE(std::isfinite(device.upper));
    EXPECT_GE(device.lower, 0); EXPECT_LE(device.lower, device.upper);
    // Certify permits a nominal estimate outside the truth interval. Its
    // reported absolute error must cover both endpoint distances instead.
    EXPECT_GE(static_cast<long double>(device.error),
              std::max(std::abs(static_cast<long double>(device.value) - device.lower),
                       std::abs(static_cast<long double>(device.value) - device.upper)));
    EXPECT_LE(device.error, budget);
    EXPECT_LE(device.lower, host.upper);
    EXPECT_GE(device.upper, host.lower);
    EXPECT_LE(std::abs(static_cast<long double>(device.value) - host.value),
              static_cast<long double>(device.error) + host.error);
}
void CheckParent(const sf::SourceParentBinding& binding, const cf::ParentForce& host,
                 const gpu::ParentOutput& device) {
    const bool quad = binding.arity == 4;
    const auto& q = device.quad.integration;
    const auto& t = device.triangle;
    ASSERT_TRUE(quad ? q.valid : t.valid);
    EXPECT_EQ(quad ? q.parent_element_id : t.parent_element_id, binding.source_id);
    EXPECT_EQ(quad ? q.feature_id : t.feature_id, binding.source_id);
    EXPECT_EQ(quad ? q.base_epoch : t.base_epoch, 17u);
    EXPECT_EQ(quad ? q.attempt : t.attempt, 31u);
    const auto resultant = quad ? q.resultant : t.resultant;
    Overlap(host.resultant, resultant, cf::ForceBudget);
    Overlap(host.potential, quad ? q.potential : t.potential, cf::EnergyBudget);
    const auto area = quad ? q.active_area : t.active_area;
    EXPECT_TRUE(std::isfinite(area.lower)); EXPECT_TRUE(std::isfinite(area.upper));
    EXPECT_GE(area.lower, 0); EXPECT_LE(area.lower, area.upper);
    EXPECT_LE(area.lower, host.active_area.upper); EXPECT_GE(area.upper, host.active_area.lower);
    for (unsigned n = 0; n < binding.arity; ++n) {
        const auto magnitude = quad ? q.force[n] : t.force[n];
        const auto force = quad ? q.nodal.forces[n] : t.nodal.forces[n];
        EXPECT_EQ(quad ? q.nodal.nodes[n] : t.nodal.nodes[n], binding.local_node_indices[n]);
        Overlap(host.magnitude[n], magnitude, cf::ForceBudget);
        EXPECT_EQ(force.x, -magnitude.value);
        EXPECT_EQ(force.y, 0); EXPECT_EQ(force.z, 0);
        if (quad) {
            const auto couple = q.nodal.couples[n];
            EXPECT_EQ(couple.x, 0); EXPECT_EQ(couple.y, 0); EXPECT_EQ(couple.z, 0);
        } // Native zero-offset T3 exports translation forces only.
    }
    if (quad) {
        EXPECT_LE(q.leaf_count, sc::MaxQ4IntegrationLeaves);
        EXPECT_LE(q.visited, sc::MaxQ4IntegrationVisits);
        EXPECT_LE(device.quad.deepest_u, sc::MaxQ4IntegrationDepth);
        EXPECT_LE(device.quad.deepest_v, sc::MaxQ4IntegrationDepth);
    } else {
        EXPECT_EQ(t.parent_face_id, 0u);
        EXPECT_LE(t.subtriangle_count, 2u); EXPECT_LE(t.sample_count, 6u);
    }
}

void Same(sc::Q4CertifiedIntegral a, sc::Q4CertifiedIntegral b) {
    EXPECT_EQ(a.value, b.value); EXPECT_EQ(a.lower, b.lower);
    EXPECT_EQ(a.upper, b.upper); EXPECT_EQ(a.error, b.error);
}
void Same(sc::Vec3 a, sc::Vec3 b) {
    EXPECT_EQ(a.x, b.x); EXPECT_EQ(a.y, b.y); EXPECT_EQ(a.z, b.z);
}
template<class T> void SameIntegral(const T& a, const T& b, unsigned count) {
    EXPECT_EQ(a.valid, b.valid); EXPECT_EQ(a.feature_id, b.feature_id);
    EXPECT_EQ(a.parent_element_id, b.parent_element_id);
    EXPECT_EQ(a.base_epoch, b.base_epoch); EXPECT_EQ(a.attempt, b.attempt);
    Same(a.resultant, b.resultant); Same(a.potential, b.potential);
    EXPECT_EQ(a.active_area.lower, b.active_area.lower); EXPECT_EQ(a.active_area.upper, b.active_area.upper);
    for (unsigned n = 0; n < count; ++n) {
        EXPECT_EQ(a.nodal.nodes[n], b.nodal.nodes[n]);
        Same(a.nodal.forces[n], b.nodal.forces[n]); Same(a.force[n], b.force[n]);
    }
}
void SameRetry(const gpu::Result& clean, const gpu::Result& retry, const sf::SourcePartContactFixture& source) {
    for (unsigned p = 0; p < sf::ParentCount; ++p) {
        SCOPED_TRACE(source.parents()[p].source_id);
        EXPECT_EQ(clean.accepted[p], retry.accepted[p]);
        if (source.parents()[p].arity == 4) {
            const auto& a = clean.parents[p].quad; const auto& b = retry.parents[p].quad;
            SameIntegral(a.integration, b.integration, 4);
            for (unsigned n = 0; n < 4; ++n) Same(a.integration.nodal.couples[n], b.integration.nodal.couples[n]);
            EXPECT_EQ(a.integration.leaf_count, b.integration.leaf_count);
            EXPECT_EQ(a.integration.visited, b.integration.visited);
            EXPECT_EQ(a.integration.deepest_leaf, b.integration.deepest_leaf);
            EXPECT_EQ(a.deepest_u, b.deepest_u); EXPECT_EQ(a.deepest_v, b.deepest_v);
            const auto& x = clean.quad_reports[p]; const auto& y = retry.quad_reports[p];
            EXPECT_EQ(x.status, y.status); EXPECT_EQ(x.cause, y.cause); EXPECT_EQ(x.cell, y.cell);
            EXPECT_EQ(x.depth, y.depth); EXPECT_EQ(x.visited, y.visited); EXPECT_EQ(x.leaves, y.leaves);
        } else {
            const auto& a = clean.parents[p].triangle; const auto& b = retry.parents[p].triangle;
            SameIntegral(a, b, 3);
            EXPECT_EQ(a.parent_face_id, b.parent_face_id); EXPECT_EQ(a.sample_count, b.sample_count);
            EXPECT_EQ(a.subtriangle_count, b.subtriangle_count);
            const auto& x = clean.triangle_reports[p]; const auto& y = retry.triangle_reports[p];
            EXPECT_EQ(x.status, y.status); EXPECT_EQ(x.cause, y.cause);
            EXPECT_EQ(x.node, y.node); EXPECT_EQ(x.sample, y.sample);
        }
    }
}

class SourcePartContactCuda : public ::testing::Test {
  protected:
    sf::SourcePartContactFixture source;
    cw::CanonicalWall wall;
    std::string wall_bytes;
    std::unique_ptr<gpu::Input> input = std::make_unique<gpu::Input>();
    std::array<cf::ParentForce, sf::ParentCount> expected;
    void SetUp() override {
        const auto loaded = sf::LoadPinnedSourcePartContact(readiness_path, &source);
        ASSERT_EQ(loaded.status, sf::FixtureStatus::Ok) << loaded.diagnostic;
        wall_bytes = cw::ReadPinnedWallManifest(wall_path.string());
        std::istringstream stream(wall_bytes);
        ASSERT_EQ(wall.Load(stream).status, cw::WallStatus::Ok);
    }
    void Prepare(Profile profile, cw::WallTessellationKind kind) {
        cf::Harness host(source);
        ASSERT_TRUE(host.mass.Initialize(source));
        cw::WallTessellation prepared;
        ASSERT_EQ(prepared.Initialize(wall, wall_bytes, kind).status, cw::WallTessellationStatus::Ok);
        const auto& geometry = *prepared.geometry();
        input->wall_x = geometry.wall_x();
        std::copy(host.mass.inverse.begin(), host.mass.inverse.end(), input->inverse_mass);
        std::copy(host.mass.fixed.begin(), host.mass.fixed.end(), input->fixed);
        for (unsigned p = 0; p < sf::ParentCount; ++p) {
            SCOPED_TRACE(source.parents()[p].source_id);
            const double shift = profile == Profile::Separated ? 0 :
                profile == Profile::IndependentParents ? cf::ParentShift(source, p, geometry.wall_x()) :
                                                        cf::WholeShift(source, geometry.wall_x());
            const auto position = cf::Shift(source, shift), velocity = cf::Velocity(shift);
            // The qualified host operation checks actual finite-wall coverage,
            // every mass/fixed-node stencil and both endpoint caps first.
            ASSERT_TRUE(host.EvaluateParent(p, source.coordinates(), position, velocity, geometry, &expected[p]))
                << host.diagnostic;
            auto& packet = input->parents[p];
            packet.arity = source.parents()[p].arity;
            packet.area = expected[p].area;
            packet.attempt = 31;
            std::copy(position.begin(), position.end(), packet.position);
            std::copy(velocity.begin(), velocity.end(), packet.velocity);
            if (packet.arity == 4) {
                ASSERT_TRUE(source.q4_parent(p, packet.quad));
            } else {
                ASSERT_TRUE(source.t3_parent(p, packet.triangle));
                ASSERT_EQ(sc::PrepareT3MaterialMeasure(source.positions(), packet.triangle,
                                                       &packet.triangle_reference), sc::SurfaceMeasureStatus::Ok);
            }
        }
    }
    void Compare(const gpu::Result& result, bool positive) {
        for (unsigned p = 0; p < sf::ParentCount; ++p) {
            SCOPED_TRACE(source.parents()[p].source_id);
            ASSERT_TRUE(result.accepted[p]);
            CheckParent(source.parents()[p], expected[p], result.parents[p]);
            const auto r = source.parents()[p].arity == 4 ? result.parents[p].quad.integration.resultant :
                                                          result.parents[p].triangle.resultant;
            if (positive) EXPECT_GT(r.lower, 0);
        }
    }
};

TEST_F(SourcePartContactCuda, EveryNativeParentMatchesHostAcrossProfilesAndActualWallTessellations) {
    gpu::Device device; ASSERT_EQ(device.initialization_status(), cudaSuccess);
    const cw::WallTessellationKind kinds[]{cw::WallTessellationKind::Original,
        cw::WallTessellationKind::FlipConvexPairs, cw::WallTessellationKind::UniformFour};
    float max_ms = 0;
    for (auto kind : kinds) for (auto profile : {Profile::Separated, Profile::IndependentParents}) {
        ASSERT_NO_FATAL_FAILURE(Prepare(profile, kind));
        const auto unchanged = *input;
        gpu::Result result; float elapsed = 0;
        ASSERT_EQ(device.Evaluate(*input, result, elapsed), cudaSuccess);
        ASSERT_NO_FATAL_FAILURE(Compare(result, profile == Profile::IndependentParents));
        EXPECT_EQ(std::memcmp(input.get(), &unchanged, sizeof(unchanged)), 0);
        max_ms = std::max(max_ms, elapsed);
    }
    RecordProperty("source_parents", 94); RecordProperty("physical_nodes", 117);
    RecordProperty("worker_blocks", static_cast<int>(gpu::Workers));
    RecordProperty("threads_per_block", 1);
    RecordProperty("explicit_device_bytes", static_cast<int>(sizeof(gpu::Storage)));
    RecordProperty("maximum_kernel_event_ms", std::to_string(max_ms));
    RecordProperty("connected_device_assembly_qualified", "false");
}

TEST_F(SourcePartContactCuda, CoherentWholepartProfileRetainsSharedNodeContributions) {
    ASSERT_NO_FATAL_FAILURE(Prepare(Profile::WholePart, cw::WallTessellationKind::Original));
    gpu::Device device; ASSERT_EQ(device.initialization_status(), cudaSuccess);
    gpu::Result result; float elapsed = 0;
    ASSERT_EQ(device.Evaluate(*input, result, elapsed), cudaSuccess);
    ASSERT_NO_FATAL_FAILURE(Compare(result, false));
    long double total = 0;
    for (unsigned node = 0; node < sf::NodeCount; ++node) {
        long double host_force = 0, device_force = 0, allowance = 0;
        for (unsigned p = 0; p < sf::ParentCount; ++p) for (unsigned n = 0; n < expected[p].arity; ++n) {
            if (expected[p].nodes[n] != node) continue;
            const auto r = expected[p].arity == 4 ? result.parents[p].quad.integration.force[n] :
                                                  result.parents[p].triangle.force[n];
            host_force -= expected[p].magnitude[n].value;
            device_force -= r.value;
            allowance += static_cast<long double>(expected[p].magnitude[n].error) + r.error;
        }
        EXPECT_LE(std::abs(host_force - device_force), allowance);
        total += device_force;
    }
    EXPECT_LT(total, 0);
    RecordProperty("reduction_location", "host; no connected GPU assembly claim");
    RecordProperty("kernel_event_ms", std::to_string(elapsed));
}

TEST_F(SourcePartContactCuda, LateParentRejectionPreservesItsPriorFieldsAndCleanRetryMatches) {
    ASSERT_NO_FATAL_FAILURE(Prepare(Profile::IndependentParents, cw::WallTessellationKind::Original));
    gpu::Device device; ASSERT_EQ(device.initialization_status(), cudaSuccess);
    gpu::Result clean; float elapsed = 0;
    ASSERT_EQ(device.Evaluate(*input, clean, elapsed), cudaSuccess);
    ASSERT_NO_FATAL_FAILURE(Compare(clean, true));
    // Last source parent fails only after its worker processes earlier parents.
    const unsigned bad = sf::ParentCount - 1;
    input->parents[bad].attempt = 0;
    gpu::Result failed;
    ASSERT_EQ(device.Evaluate(*input, failed, elapsed), cudaSuccess);
    EXPECT_FALSE(failed.accepted[bad]);
    EXPECT_EQ(std::memcmp(&failed.parents[bad], &clean.parents[bad], sizeof(gpu::ParentOutput)), 0);
    for (unsigned p = 0; p < bad; ++p) EXPECT_TRUE(failed.accepted[p]);
    input->parents[bad].attempt = 31;
    gpu::Result retry;
    ASSERT_EQ(device.Evaluate(*input, retry, elapsed), cudaSuccess);
    ASSERT_NO_FATAL_FAILURE(Compare(retry, true));
    SameRetry(clean, retry, source);
    RecordProperty("batch_transaction_or_owner_qualified", "false");
}
} // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    if (argc != 3) {
        std::cerr << "Usage: source_part_contact_cuda_check readiness.json wall.manifest.json [gtest options]\n";
        return 2;
    }
    readiness_path = argv[1]; wall_path = argv[2];
    return RUN_ALL_TESTS();
}
