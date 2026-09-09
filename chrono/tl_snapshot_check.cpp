// Opt-in cross-repository qualification. This executable uses a prescribed
// CUDA shell fixture to test the output boundary; it is not a crash runner.
#include "AcceptedSurfaceMesh.h"
#include "PersistentShell.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <gtest/gtest.h>

namespace {
namespace shell = tl::qualification::shell;
namespace visual = crash::visual;

struct Motion {
    std::array<double, 12> x{{0,0,0, 2,0,0, 2,1,0, 0,1,0}};
    std::array<double, 12> v{}, omega{};
    tl::fea::HostNodalKinematicsView view() const {
        return {x.data(), v.data(), omega.data(), 4};
    }
    void StretchFrom(const shell::Snapshot& old, double dt) {
        for (std::size_t i=0; i<4; ++i) {
            x[3*i] = old.position[3*i] * (1 + 1e-5);
            x[3*i+1] = old.position[3*i+1] * (1 - 1e-5);
            for (std::size_t d=0; d<3; ++d)
                v[3*i+d] = (x[3*i+d] - old.position[3*i+d]) / dt;
        }
    }
};

visual::Binding Binding() {
    visual::Binding b;
    b.identity = {41, 7, 3};
    b.tl_node_count = 4;
    // Display order differs from physical-node order. Source IDs cannot be
    // represented exactly by binary64 and must stay integer metadata.
    for (auto node : {2u, 0u, 3u, 1u})
        b.vertices.push_back({node, {9, 2, (std::uint64_t{1} << 54) + node + 1}});
    b.triangles = {{{1,3,0}, 9,2,501,6,0,0}, {{1,0,2}, 9,2,501,6,0,1}};
    b.max_vertices = 4;
    b.max_triangles = 2;
    return b;
}

// The application coordinator reads only accepted state. A valid candidate is
// deliberately never an output source. Epoch/time come from the TL owner.
struct SnapshotSource {
    const shell::PersistentShell* owner;  // Must not outlive this backend.
    visual::Identity identity;           // Assigned by the application at setup.
};
visual::Report PublishAccepted(const shell::PersistentShell& backend,
                              visual::AcceptedSurfaceMesh& mesh,
                              const SnapshotSource& source) {
    if (source.owner != &backend)
        return {visual::Status::WrongOwner, "Snapshot source belongs to another backend"};
    const auto& accepted = backend.accepted();
    const tl::fea::HostNodalKinematicsView view{
        accepted.position.data(), accepted.velocity.data(),
        accepted.angular_velocity.data(), accepted.node_count};
    return mesh.Publish(view, {source.identity, accepted.epoch, accepted.time});
}

void ExpectAccepted(const shell::Snapshot& accepted,
                    const visual::AcceptedSurfaceMesh& mesh) {
    ASSERT_NE(mesh.frame(), nullptr);
    ASSERT_NE(mesh.mesh(), nullptr);
    EXPECT_EQ(mesh.frame()->epoch, accepted.epoch);
    EXPECT_EQ(mesh.frame()->time, accepted.time);
    const auto& positions = mesh.mesh()->GetCoordsVertices();
    ASSERT_EQ(positions.size(), mesh.binding()->vertices.size());
    for (std::size_t i=0; i<positions.size(); ++i) {
        const auto node = mesh.binding()->vertices[i].tl_node;
        EXPECT_EQ(positions[i].x(), accepted.position[3*node]);
        EXPECT_EQ(positions[i].y(), accepted.position[3*node+1]);
        EXPECT_EQ(positions[i].z(), accepted.position[3*node+2]);
    }
}

class TLAcceptedMesh : public ::testing::Test {
  protected:
    Motion motion;
    shell::PersistentShell backend;
    visual::AcceptedSurfaceMesh mesh;
    const SnapshotSource source{&backend, {41,7,3}};
    void SetUp() override {
        shell::Configuration config;
        config.elements[0].nodes = {0,1,2,3};
        config.max_device_bytes = 8192;
        const auto initialized = backend.Initialize(config, motion.view());
        ASSERT_EQ(initialized.status, shell::Status::Ok) << initialized.message;
        ASSERT_EQ(mesh.Initialize(Binding()).status, visual::Status::Ok);
        ASSERT_EQ(PublishAccepted(backend, mesh, source).status, visual::Status::Ok);
    }
};

TEST_F(TLAcceptedMesh, OnlyCommittedGpuStatesAdvanceTheVisibleFrame) {
    ExpectAccepted(backend.accepted(), mesh);
    const auto initial = backend.accepted();
    motion.StretchFrom(initial, 1e-3);
    shell::TrialToken token;
    ASSERT_EQ(backend.Evaluate({motion.view(), 0, 1e-3}, &token).status, shell::Status::Ok);
    ASSERT_NE(backend.trial(), nullptr);
    EXPECT_NE(backend.trial()->position, initial.position);
    EXPECT_EQ(PublishAccepted(backend, mesh, source).status, visual::Status::StaleFrame);
    ExpectAccepted(initial, mesh);
    ASSERT_EQ(backend.Commit(token), shell::Status::Ok);
    ASSERT_EQ(PublishAccepted(backend, mesh, source).status, visual::Status::Ok);
    ExpectAccepted(backend.accepted(), mesh);
    EXPECT_EQ(mesh.binding()->vertices[0].source.node, (std::uint64_t{1} << 54) + 3);
}

TEST_F(TLAcceptedMesh, FailedGpuPhasesPreserveTheFrameAndRetryPublishesOnce) {
    const auto initial = backend.accepted();
    motion.StretchFrom(initial, 1e-3);
    shell::TrialToken token;
    for (auto phase : {shell::RejectAfter::Geometry, shell::RejectAfter::Material,
                       shell::RejectAfter::Assembly}) {
        ASSERT_EQ(backend.Evaluate({motion.view(), 0, 1e-3, phase}, &token).status,
                  shell::Status::Rejected);
        EXPECT_EQ(backend.trial(), nullptr);
        EXPECT_EQ(PublishAccepted(backend, mesh, source).status, visual::Status::StaleFrame);
        ExpectAccepted(initial, mesh);
    }
    ASSERT_EQ(backend.Evaluate({motion.view(), 0, 1e-3}, &token).status, shell::Status::Ok);
    ASSERT_EQ(backend.Commit(token), shell::Status::Ok);
    ASSERT_EQ(PublishAccepted(backend, mesh, source).status, visual::Status::Ok);
    ExpectAccepted(backend.accepted(), mesh);
    EXPECT_EQ(PublishAccepted(backend, mesh, source).status, visual::Status::StaleFrame);
    motion.StretchFrom(backend.accepted(), 1e-3);
    ASSERT_EQ(backend.Evaluate({motion.view(), 1e-3, 2e-3}, &token).status, shell::Status::Ok);
    ASSERT_EQ(backend.Commit(token), shell::Status::Ok);
    ASSERT_EQ(PublishAccepted(backend, mesh, source).status, visual::Status::Ok);
    ExpectAccepted(backend.accepted(), mesh);
    EXPECT_EQ(mesh.frame()->epoch, 2u);
}

TEST_F(TLAcceptedMesh, SourceAssociationRejectsAnotherActualGpuOwner) {
    const auto initial = backend.accepted();
    shell::PersistentShell unrelated;
    shell::Configuration config;
    config.elements[0].nodes = {0,1,2,3};
    ASSERT_EQ(unrelated.Initialize(config, motion.view()).status, shell::Status::Ok);
    motion.StretchFrom(unrelated.accepted(), 1e-3);
    shell::TrialToken token;
    ASSERT_EQ(unrelated.Evaluate({motion.view(), 0, 1e-3}, &token).status, shell::Status::Ok);
    ASSERT_EQ(unrelated.Commit(token), shell::Status::Ok);
    // A later epoch would otherwise be admissible. Source ownership must be
    // checked before assigning provenance to the destination mesh.
    EXPECT_EQ(PublishAccepted(unrelated, mesh, source).status, visual::Status::WrongOwner);
    ExpectAccepted(initial, mesh);
    const SnapshotSource unrelated_source{&unrelated, {42,7,3}};
    EXPECT_EQ(PublishAccepted(unrelated, mesh, unrelated_source).status, visual::Status::WrongOwner);
    ExpectAccepted(initial, mesh);
}

TEST(TLAcceptedMeshLifetime, PublishedCoordinatesSurviveBackendDestruction) {
    visual::AcceptedSurfaceMesh mesh;
    ASSERT_EQ(mesh.Initialize(Binding()).status, visual::Status::Ok);
    shell::Snapshot final;
    {
        Motion motion;
        shell::PersistentShell backend;
        const SnapshotSource source{&backend, {41,7,3}};
        shell::Configuration config;
        config.elements[0].nodes = {0,1,2,3};
        ASSERT_EQ(backend.Initialize(config, motion.view()).status, shell::Status::Ok);
        motion.StretchFrom(backend.accepted(), 1e-3);
        shell::TrialToken token;
        ASSERT_EQ(backend.Evaluate({motion.view(), 0, 1e-3}, &token).status, shell::Status::Ok);
        ASSERT_EQ(backend.Commit(token), shell::Status::Ok);
        ASSERT_EQ(PublishAccepted(backend, mesh, source).status, visual::Status::Ok);
        final = backend.accepted();
        motion.x.fill(99);  // Borrowed input must not own either published copy.
        ExpectAccepted(final, mesh);
    }
    ExpectAccepted(final, mesh);
    EXPECT_NE(mesh.visual_shape(), nullptr);
}
}  // namespace
