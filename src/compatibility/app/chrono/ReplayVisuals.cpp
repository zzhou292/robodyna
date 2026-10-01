#include "ReplayVisuals.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualModel.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "robodyna/mbd/RbBody.h"
#include <algorithm>
#include <cmath>
#include <new>
#include <stdexcept>
#include <vector>

namespace crash::visual {
std::shared_ptr<chrono::ChTriangleMeshConnected> CopyReplayGeometry(const chrono::ChTriangleMeshConnected& source) {
    auto mesh = std::make_shared<chrono::ChTriangleMeshConnected>();
    mesh->GetCoordsVertices() = source.GetCoordsVertices();
    mesh->GetIndicesVertices() = source.GetIndicesVertices();
    return mesh;  // Archive attributes never replace the declared display policy.
}
std::shared_ptr<chrono::ChVisualShapeTriangleMesh> MakeReplayShape(
    const std::shared_ptr<chrono::ChTriangleMeshConnected>& mesh, bool moving, bool wireframe, bool colors,bool fixed_connectivity) {
    auto shape = std::make_shared<chrono::ChVisualShapeTriangleMesh>();
    shape->SetMesh(mesh, false);
    shape->SetMutable(moving);
    if(fixed_connectivity) shape->SetFixedConnectivity();
    shape->SetDoubleFaced(true);
    shape->SetBackfaceCull(false);
    shape->SetWireframe(wireframe);
    // Chrono/VSG binds dynamic color buffers only for its no-material mesh path.
    // Indexed face colors below use that existing path, including mutable normals.
    if(colors)return shape;
    auto material = std::make_shared<chrono::ChVisualMaterial>();
    material->SetDiffuseColor(moving ? chrono::ChColor(0.12f, 0.64f, 0.94f) : chrono::ChColor(0.42f, 0.46f, 0.51f));
    material->SetMetallic(0.0f);
    material->SetRoughness(0.7f);
    shape->AddMaterial(material);
    return shape;
}
std::shared_ptr<robodyna::mbd::RbBody> MakeReplayCarrier(const char* name,
                                      const std::shared_ptr<chrono::ChVisualShapeTriangleMesh>& shape) {
    auto body = std::make_shared<robodyna::mbd::RbBody>();
    body->SetName(name);
    body->SetFixed(true);
    body->SetPos(chrono::VNULL);
    body->SetRot(chrono::QUNIT);
    body->EnableCollision(false);
    body->AddVisualShape(shape);
    // VSG's fixed-shape path reads the model instance flag; its mutable-mesh
    // path reads the shape flag. Keep both existing representations consistent.
    body->GetVisualModel()->EnableWireframe(0U, shape->IsWireframe());
    return body;
}
} // namespace crash::visual
