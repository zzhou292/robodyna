// External Chrono integration check for the prepared, stationary Yaris wall.
// This is a mesh-loading/scene-data check, not a collision or dynamics solver.
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"

namespace {
void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}
}  // namespace

int main(int argc, char** argv) {
    try {
        Require(argc == 2, "Usage: crash_chrono_wall_check prepared-wall.obj");
        auto mesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(argv[1], false, false);
        Require(static_cast<bool>(mesh), "Chrono could not load the prepared wall mesh");
        Require(mesh->GetNumVertices() == 62, "Wall vertex count changed during loading");
        Require(mesh->GetNumTriangles() == 100, "Wall triangle count changed during loading");
        const auto box = mesh->GetBoundingBox();
        // This Chrono checkout's tinyobjloader defaults to float on input.
        // Allow its documented representation roundoff in the visual mesh;
        // retain the canonical double-precision asset for CUDA collision.
        constexpr double input_epsilon = std::numeric_limits<float>::epsilon();
        Require(std::fabs(box.min.x() - 0.05) < 2 * input_epsilon &&
                    std::fabs(box.max.x() - 0.05) < 2 * input_epsilon,
                "Wall is not at the expected SI position");
        double area = 0;
        for (unsigned int i = 0; i < mesh->GetNumTriangles(); ++i) {
            const auto triangle = mesh->GetTriangle(i);
            chrono::ChVector3d normal;
            Require(triangle.Normal(normal), "Chrono found a degenerate wall triangle");
            Require(normal.x() < -1 + 1e-12 && std::fabs(normal.y()) < 1e-12 && std::fabs(normal.z()) < 1e-12,
                    "Wall triangle is not oriented toward incoming vehicle motion");
            area += chrono::ChTriangle::CalcArea(triangle.p1, triangle.p2, triangle.p3);
        }
        Require(std::isfinite(area) && std::fabs(area - 3.565455298182685) < 8 * input_epsilon * 3.565455298182685,
                "Chrono mesh area differs from independently checked source geometry");

        auto shape = std::make_shared<chrono::ChVisualShapeTriangleMesh>();
        shape->SetMesh(mesh, false);
        auto wall = std::make_shared<chrono::ChBody>();
        wall->SetFixed(true);
        wall->AddVisualShape(shape);
        Require(wall->IsFixed() && shape->GetMesh() == mesh, "Fixed visual body does not retain the prepared mesh");
        // No system stepping or Chrono contact shape is created. CUDA contact
        // will consume this same compiled asset through its own device adapter.
        std::cout << std::setprecision(17)
                  << "{\"status\":\"passed\",\"vertices\":" << mesh->GetNumVertices()
                  << ",\"triangles\":" << mesh->GetNumTriangles()
                  << ",\"area_m2\":" << area
                  << ",\"wall_x_m\":" << box.min.x()
                  << ",\"wall_x_roundoff_m\":" << box.min.x() - 0.05
                  << ",\"fixed\":true,\"scope\":\"Chrono mesh and scene data only\"}\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
