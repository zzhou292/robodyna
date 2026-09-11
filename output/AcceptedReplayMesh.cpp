#include "AcceptedReplayData.h"
#include "GeometryMeshArchive.h"
#include "AcceptedReplaySourceAssembly.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include <cmath>
#include <set>
#include <sstream>

namespace crash::output::replay_detail {
std::shared_ptr<chrono::ChTriangleMeshConnected> ReadMesh(const Bundle& bundle,const std::string& name) {
    return ReadGeometryMesh(VerifiedBytes(bundle,name),kVertexCap,kTriangleCap);
}

void CheckPositionFields(const Value& fields, const chrono::ChTriangleMeshConnected& mesh) {
    const auto& positions = Member(fields, "position_xyz_m"); const auto& vertices = mesh.GetCoordsVertices();
    Require(positions.IsArray() && positions.Size() == 3*vertices.size(), "Coupon field/mesh node counts disagree");
    for (std::size_t i = 0; i < vertices.size(); ++i)
        for (unsigned axis = 0; axis < 3; ++axis) {
            const auto& value = positions[3*i+axis];
            Require(value.IsNumber() && std::isfinite(value.GetDouble()) && Bits(value.GetDouble()) == Bits(vertices[i][axis]),
                    "Coupon mesh is not the recorded accepted field geometry");
        }
}
void CheckFrameFields(const Bundle& bundle, const Entry& entry, const chrono::ChTriangleMeshConnected& mesh) {
    if (bundle.info.kind == ReplayKind::SourceAssemblyWall) { CheckSourceAssemblyFields(bundle, entry, mesh); return; }
    if (bundle.info.kind == ReplayKind::SourcePartWall) { CheckSourcePartWallFields(bundle, entry, mesh); return; }
    if (bundle.info.kind == ReplayKind::SourcePartElastic) { CheckSourcePartFields(bundle, entry, mesh); return; }
    if (bundle.info.kind == ReplayKind::GuidedPlate) { CheckGuidedFields(bundle, entry, mesh); return; }
    if (bundle.info.kind != ReplayKind::ElasticCoupon) return;
    const auto name = entry.mesh.substr(0, entry.mesh.size()-10)+".fields.json";
    const auto fields = Json(VerifiedBytes(bundle, name));
    Require(Text(fields, "schema") == "robo_dyna.elastic_coupon_fields.v1" &&
            Unsigned(fields, "owner_id") == entry.owner && Unsigned(fields, "accepted_epoch") == entry.epoch &&
            Bits(Real(fields, "accepted_time_s")) == Bits(entry.time), "Coupon field frame association mismatch");
    CheckPositionFields(fields, mesh); // Legacy coupon admission remains unchanged.
}

}  // namespace crash::output::replay_detail
