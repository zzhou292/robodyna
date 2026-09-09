#include "MeshArchive.h"
#include "ArtifactIO.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/serialization/ChArchiveJSON.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <vector>

namespace crash::output {

void WriteMeshFiles(const std::filesystem::path& directory, const std::string& stem,
                    const chrono::ChTriangleMeshConnected& source) {
    const auto json = directory / (stem + ".mesh.json"), obj = directory / (stem + ".obj");
    Require(!std::filesystem::exists(json) && !std::filesystem::exists(obj),
            "Refusing to overwrite a mesh artifact");
    {
        std::ofstream output(json, std::ios::binary);
        Require(bool(output), "Could not create mesh archive");
        {
            // Existing Chrono full-precision output uses shortest-roundtrip
            // std::to_chars(double); preserve the source binary64 values.
            chrono::ChTriangleMeshConnected mesh = source;
            chrono::ChArchiveOutJSON archive(output, true);
            archive << chrono::make_ChNameValue("mesh", mesh);
        }
        output.flush();
        Require(bool(output), "Mesh archive write failed");
        output.close();
        Require(!output.fail(), "Mesh archive close failed");
    }
    chrono::ChTriangleMeshConnected restored;
    {
        std::ifstream input(json, std::ios::binary);
        Require(bool(input), "Could not reopen mesh archive");
        chrono::ChArchiveInJSON archive(input, true);
        archive >> chrono::make_ChNameValue("mesh", restored);
    }
    Require(restored.GetNumVertices() == source.GetNumVertices() &&
                restored.GetNumTriangles() == source.GetNumTriangles(),
            "Mesh archive count roundtrip failed");
    const auto& original_vertices = source.GetCoordsVertices();
    const auto& restored_vertices = restored.GetCoordsVertices();
    for (std::size_t i = 0; i < original_vertices.size(); ++i)
        for (int axis = 0; axis < 3; ++axis)
            Require(Bits(original_vertices[i][axis]) == Bits(restored_vertices[i][axis]),
                    "Mesh archive lost binary64 coordinate bits");
    const auto& original_faces = source.GetIndicesVertices();
    const auto& restored_faces = restored.GetIndicesVertices();
    for (std::size_t i = 0; i < original_faces.size(); ++i)
        for (int axis = 0; axis < 3; ++axis)
            Require(original_faces[i][axis] == restored_faces[i][axis], "Mesh archive changed topology");

    Require(chrono::ChTriangleMeshConnected::WriteWavefront(
                obj.string(), std::vector<chrono::ChTriangleMeshConnected>{source}),
            "Chrono OBJ write failed");
    // Its bool return only checks file opening in this checkout. Reload to
    // reject truncated output; OBJ remains visualization precision.
    const auto visual = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(obj.string(), false, false);
    Require(bool(visual) && visual->GetNumVertices() == source.GetNumVertices() &&
                visual->GetNumTriangles() == source.GetNumTriangles(),
            "OBJ output count validation failed");
    for (std::size_t i = 0; i < original_vertices.size(); ++i)
        for (int axis = 0; axis < 3; ++axis) {
            const double actual = visual->GetCoordsVertices()[i][axis], expected = original_vertices[i][axis];
            Require(std::isfinite(actual) && std::fabs(actual - expected) <= 8e-6 * std::max(1., std::fabs(expected)),
                    "OBJ visualization coordinate validation failed");
        }
    for (std::size_t i = 0; i < original_faces.size(); ++i)
        for (int axis = 0; axis < 3; ++axis)
            Require(visual->GetIndicesVertices()[i][axis] == original_faces[i][axis],
                    "OBJ output topology validation failed");
}

}  // namespace crash::output
