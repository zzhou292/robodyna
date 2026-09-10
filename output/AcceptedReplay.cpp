#include "AcceptedReplayData.h"
#include "AcceptedReplaySourceAssembly.h"
#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>

namespace crash::output {
namespace rd = replay_detail;
namespace {
void MatchTopology(const rd::Bundle& bundle, const chrono::ChTriangleMeshConnected& mesh) {
    Require(mesh.GetCoordsVertices().size() == bundle.info.node_count &&
            mesh.GetIndicesVertices().size() == bundle.topology.size(), "Replay mesh counts changed");
    for (std::size_t i = 0; i < bundle.topology.size(); ++i)
        for (unsigned axis = 0; axis < 3; ++axis)
            Require(mesh.GetIndicesVertices()[i][axis] == bundle.topology[i][axis], "Replay connectivity changed");
}
ReplayFrame Frame(const rd::Bundle& bundle, std::size_t index,
                  const std::shared_ptr<const chrono::ChTriangleMeshConnected>& mesh) {
    const auto& entry = bundle.entries[index];
    ReplayFrame frame{index, entry.owner, entry.epoch, entry.time, mesh};
    if(bundle.info.kind==ReplayKind::SourceAssemblyWall)frame.parent_plastic_strain=rd::ReadSourceAssemblyDisplay(bundle,entry);
    else if(bundle.info.source_plasticity)frame.parent_plastic_strain=rd::ReadSourcePartPlasticDisplay(bundle,entry);
    return frame;
}
}  // namespace

struct AcceptedReplay::Impl {
    rd::Bundle bundle;
    ReplayFrame current;
    std::shared_ptr<const chrono::ChTriangleMeshConnected> wall;
};
AcceptedReplay::AcceptedReplay() = default;
AcceptedReplay::~AcceptedReplay() = default;

ReplayReport AcceptedReplay::Open(const std::filesystem::path& directory) {
    try {
        auto candidate = std::make_unique<Impl>();
        candidate->bundle = rd::ReadIndex(directory);
        auto& bundle = candidate->bundle;
        for (const auto& triangle : bundle.source_triangles)
            bundle.info.triangle_source_part.push_back(triangle[6]);
        if(bundle.info.source_plasticity) {
            bundle.info.plastic_strain_color_max=std::max(.001,bundle.assembly?bundle.assembly->maximum_plastic:bundle.plastic_final_values[0]);
            const auto& v=bundle.source_initial_velocity;
            bundle.info.source_initial_speed_m_per_s=std::hypot(v[0],v[1],v[2]);
            for(const auto& triangle:bundle.source_triangles)
                bundle.info.triangle_source_parent.push_back(triangle[5]);
        }
        bundle.info.bounds_min.fill(std::numeric_limits<double>::infinity());
        bundle.info.bounds_max.fill(-std::numeric_limits<double>::infinity());
        for (std::size_t i = 0; i < bundle.entries.size(); ++i) {
            const auto mesh = rd::ReadMesh(bundle, bundle.entries[i].mesh);
            if (i == 0) {
                Require(!bundle.info.node_count || bundle.info.node_count == mesh->GetCoordsVertices().size(),
                        "Initial mesh differs from recorded node count");
                bundle.info.node_count = mesh->GetCoordsVertices().size();
                if (bundle.topology.empty())
                    for (const auto& face : mesh->GetIndicesVertices()) bundle.topology.push_back({face[0], face[1], face[2]});
                bundle.info.triangle_count = bundle.topology.size();
            }
            MatchTopology(bundle, *mesh); rd::CheckFrameFields(bundle, bundle.entries[i], *mesh);
            if(i==0)candidate->current=Frame(bundle,0,mesh);
            for (const auto& position : mesh->GetCoordsVertices())
                for (unsigned axis = 0; axis < 3; ++axis) {
                    bundle.info.bounds_min[axis] = std::min(bundle.info.bounds_min[axis], position[axis]);
                    bundle.info.bounds_max[axis] = std::max(bundle.info.bounds_max[axis], position[axis]);
                }
        }
        double diameter = 0;
        for (unsigned axis = 0; axis < 3; ++axis)
            diameter = std::hypot(diameter, bundle.info.bounds_max[axis]-bundle.info.bounds_min[axis]);
        Require(std::isfinite(diameter) && diameter > 0, "Replay trajectory bounds are invalid");
        if(bundle.info.kind==ReplayKind::SourcePartWall||bundle.info.kind==ReplayKind::SourceAssemblyWall) candidate->wall=rd::ReadMesh(bundle,"placed-wall.mesh.json");
        else if (bundle.inventory.count("canonical-wall.mesh.json")) {
            candidate->wall = rd::ReadMesh(bundle, "canonical-wall.mesh.json");
            if (bundle.info.kind == ReplayKind::GuidedPlate) rd::CheckGuidedWall(bundle, *candidate->wall);
        }
        impl_ = std::move(candidate);
        return {ReplayStatus::Ok, "Completed accepted replay verified"};
    } catch (const std::exception& error) { return {ReplayStatus::InvalidBundle, error.what()}; }
}

ReplayReport AcceptedReplay::Load(std::size_t index) {
    if (!impl_) return {ReplayStatus::NotInitialized, "Replay is not initialized"};
    if (index >= impl_->bundle.entries.size()) return {ReplayStatus::InvalidFrame, "Replay frame is outside the index"};
    try {
        const auto& bundle = impl_->bundle; const auto& entry = bundle.entries[index];
        const auto mesh = rd::ReadMesh(bundle, entry.mesh);
        MatchTopology(bundle, *mesh); rd::CheckFrameFields(bundle, entry, *mesh);
        impl_->current = Frame(bundle, index, mesh);
        return {ReplayStatus::Ok, "Accepted replay frame loaded"};
    } catch (const std::exception& error) { return {ReplayStatus::InvalidFrame, error.what()}; }
}
const ReplayInfo* AcceptedReplay::info() const noexcept { return impl_ ? &impl_->bundle.info : nullptr; }
const ReplayFrame* AcceptedReplay::frame() const noexcept { return impl_ ? &impl_->current : nullptr; }
std::shared_ptr<const chrono::ChTriangleMeshConnected> AcceptedReplay::wall() const noexcept { return impl_ ? impl_->wall : nullptr; }
}  // namespace crash::output
