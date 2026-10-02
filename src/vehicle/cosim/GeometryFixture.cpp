#include "GeometryFixture.h"

#include <stdexcept>

namespace robodyna::verification {
namespace {
void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}
}  // namespace

chrono::utils::ChBodyGeometry MakeTransportGeometry() {
    using namespace chrono;
    utils::ChBodyGeometry geometry;
    geometry.materials.emplace_back(0.375f, 0.125f, 123456.0f, 0.25f, 2048.0f, 16.0f, 1024.0f, 8.0f);
    geometry.coll_boxes.emplace_back(ChVector3d(1, 2, 3), QUNIT, ChVector3d(4, 5, 6), 0);
    geometry.coll_spheres.emplace_back(ChVector3d(-1, 3, 5), 0.625, 0);
    geometry.coll_cylinders.emplace_back(ChVector3d(2, -3, 4), ChQuaternion<>(0.5, 0.5, 0.5, 0.5), 0.25, 2.5, 0);
    auto mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    mesh->GetCoordsVertices() = {{0, 0, 0}, {1, 0, 0}, {0, 2, 0}, {0, 0, 3}};
    mesh->GetCoordsNormals() = {{0, 0, 1}, {0, -1, 0}};
    mesh->GetIndicesVertices() = {{0, 1, 2}, {0, 3, 1}};
    mesh->GetIndicesNormals() = {{0, 0, 0}, {1, 1, 1}};
    geometry.coll_meshes.emplace_back(VNULL, QUNIT, mesh, 1.0, 0.0, 0);
    return geometry;
}

void CheckTransportGeometry(const chrono::utils::ChBodyGeometry& received) {
    const auto expected = MakeTransportGeometry();
    Require(received.materials.size() == 1 && received.coll_boxes.size() == 1 &&
                received.coll_spheres.size() == 1 && received.coll_cylinders.size() == 1 &&
                received.coll_meshes.size() == 1 && received.coll_hulls.empty(),
            "Geometry family counts changed across MPI transport");
    const auto& a = received.materials[0];
    const auto& b = expected.materials[0];
    Require(a.mu == b.mu && a.cr == b.cr && a.Y == b.Y && a.nu == b.nu &&
                a.kn == b.kn && a.gn == b.gn && a.kt == b.kt && a.gt == b.gt,
            "Material fields changed across MPI transport");
    const auto& box = received.coll_boxes[0];
    const auto& eb = expected.coll_boxes[0];
    Require(box.pos == eb.pos && box.rot == eb.rot && box.dims == eb.dims && box.matID == eb.matID,
            "Box fields changed across MPI transport");
    const auto& sphere = received.coll_spheres[0];
    const auto& es = expected.coll_spheres[0];
    Require(sphere.pos == es.pos && sphere.radius == es.radius && sphere.matID == es.matID,
            "Sphere fields changed across MPI transport");
    const auto& cylinder = received.coll_cylinders[0];
    const auto& ec = expected.coll_cylinders[0];
    Require(cylinder.pos == ec.pos && cylinder.rot == ec.rot && cylinder.radius == ec.radius &&
                cylinder.length == ec.length && cylinder.matID == ec.matID,
            "Cylinder fields changed across MPI transport");
    const auto& mesh = received.coll_meshes[0];
    const auto& em = expected.coll_meshes[0];
    Require(mesh.matID == em.matID && mesh.trimesh != em.trimesh &&
                mesh.trimesh->GetCoordsVertices() == em.trimesh->GetCoordsVertices() &&
                mesh.trimesh->GetCoordsNormals() == em.trimesh->GetCoordsNormals() &&
                mesh.trimesh->GetIndicesVertices() == em.trimesh->GetIndicesVertices() &&
                mesh.trimesh->GetIndicesNormals() == em.trimesh->GetIndicesNormals(),
            "Triangle mesh payload changed across MPI transport");
}
}  // namespace robodyna::verification
