#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace chrono {
class ChSystem;
class ChTriangleMeshConnected;
class ChVisualShapeTriangleMesh;
}
namespace crash::output { struct ReplayInfo; struct ReplayFrame; }

namespace crash::visual {
enum class ReplaySceneStatus { Ok, NotInitialized, AlreadyInitialized, InvalidFrame, ResourceLimit };
struct ReplaySceneReport { ReplaySceneStatus status; const char* message; };
struct ReplayStamp {
    std::size_t index = 0;
    std::uint64_t owner_id = 0, epoch = 0;
    double time = 0;
};
enum class ReplayVertical { Y, Z };
struct ReplayCamera {
    std::array<double, 3> position{}, target{};
    double vertical_fov_degrees = 40;
    ReplayVertical vertical = ReplayVertical::Z;
};

// Core-only presentation adapter for a previously validated AcceptedReplay.
// Owns fixed identity-frame visual carriers; no collision shapes, solver calls,
// physical state owner, synthetic source IDs or simulation integration.
// Initialize once, then publish successive recorded frames. The mutable shape
// and its mesh keep their identities: VSG binds them once at Initialize and
// recomputes face normals from changed positions at Render. Do not call BindAll
// on publication. Rendering/publication are externally serialized.
// Borrowed visual handles are for inspection/rendering only; do not change their
// mesh, topology, material, scale or carrier transform behind this adapter.
class AcceptedReplayScene {
  public:
    AcceptedReplayScene();
    ~AcceptedReplayScene();
    AcceptedReplayScene(const AcceptedReplayScene&) = delete;
    AcceptedReplayScene& operator=(const AcceptedReplayScene&) = delete;
    ReplaySceneReport Initialize(const output::ReplayInfo&, const output::ReplayFrame&,
                                std::shared_ptr<const chrono::ChTriangleMeshConnected> wall,
                                bool wireframe = false, double deformation_scale = 1);
    // SourcePartElastic alone permits explicit presentation magnification: X0 +
    // scale*(X-X0), scale in [1,1000]. Caller supplies bounds computed over
    // transformed frames; input archives and accepted state remain untouched.
    // Stages all positions before publication; rejection preserves geometry,
    // visual handles, stamp and presentation time. No per-frame allocation.
    ReplaySceneReport Publish(const output::ReplayFrame&);
    chrono::ChSystem& system();  // Borrow for AttachSystem only; throws before Initialize.
    const ReplayStamp* stamp() const noexcept;
    const ReplayCamera* camera() const noexcept;
    double deformation_scale() const noexcept;
    std::shared_ptr<const chrono::ChTriangleMeshConnected> moving_mesh() const noexcept;
    std::shared_ptr<const chrono::ChTriangleMeshConnected> wall_mesh() const noexcept;
    std::shared_ptr<chrono::ChVisualShapeTriangleMesh> moving_shape() const noexcept;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace crash::visual
