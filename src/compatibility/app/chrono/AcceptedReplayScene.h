#pragma once
#include "chrono/physics/ChSystemFwd.h"

#include "ReplayView.h"
#include "ReplayColorMode.h"
#include "ReplayPartColors.h"
#include "ReplayGeometryLimits.h"
#include "ReplayScalarApplicability.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace chrono {
// System type is declared by the explicit forward header.
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
    ReplayView view = ReplayView::IncidentSide;
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
                                bool wireframe = false, double deformation_scale = 1,
                                ReplayView view = ReplayView::IncidentSide,
                                ReplayColorMode colors = ReplayColorMode::Automatic,
                                ReplayGeometryLimits limits = {});
    // Vehicle geometry requires an explicit capacity opt-in. Archive and solver
    // admission remain separate. The fixed wall retains the small default caps.
    // WallSide is supported by SourcePartWall and SourceAssemblyWall, whose
    // placed wall has its incident normal along -X. It reflects the camera's
    // X offset about the trajectory target; all accepted geometry is retained.
    // SourcePartElastic alone permits explicit presentation magnification: X0 +
    // scale*(X-X0), scale in [1,1000]. Caller supplies bounds computed over
    // transformed frames; input archives and accepted state remain untouched.
    // Automatic retains plastic wall replay's fixed scalar scale; PartId uses
    // complete original PIDs and a fixed palette across frames/subsets. It keeps
    // native plastic-field validation active while displaying categorical colors.
    // Plastic mode stages positions and colors together;
    // rejection preserves both, visual handles, stamp and presentation time.
    // No per-frame allocation or rebinding.
    ReplaySceneReport Publish(const output::ReplayFrame&);
    chrono::ChSystem& system();  // Borrow for AttachSystem only; throws before Initialize.
    const ReplayStamp* stamp() const noexcept;
    const ReplayCamera* camera() const noexcept;
    double deformation_scale() const noexcept;
    ReplayColorMode color_mode() const noexcept;
    const ReplayScalarLegend* scalar_legend() const noexcept;
    const std::vector<ReplayPartLegendEntry>* part_legend() const noexcept;
    std::shared_ptr<const chrono::ChTriangleMeshConnected> moving_mesh() const noexcept;
    std::shared_ptr<const chrono::ChTriangleMeshConnected> wall_mesh() const noexcept;
    std::shared_ptr<chrono::ChVisualShapeTriangleMesh> moving_shape() const noexcept;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}  // namespace crash::visual
