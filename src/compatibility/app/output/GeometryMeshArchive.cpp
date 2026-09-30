#include "GeometryMeshArchive.h"
#include "BoundedArrayJson.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include <cmath>
#include <set>
#include <sstream>
namespace crash::output {
namespace {
const Value& Member(const Value& value,const char* name) {
    Require(value.IsObject() && value.HasMember(name),"Missing geometry mesh member");return value[name];
}
std::string Text(const Value& value,const char* name) {return array_json::Text(Member(value,name));}
double Real(const Value& value,const char* name) {return array_json::Real(Member(value,name));}
std::uint64_t Unsigned(const Value& value,const char* name) {return array_json::UInt(Member(value,name));}
void Unique(const Value& value) {
    Require(value.IsObject(),"Expected geometry mesh object");
    std::set<std::string> names;
    for(const auto& member:value.GetObject())
        Require(names.emplace(member.name.GetString(),member.name.GetStringLength()).second,"Duplicate geometry mesh member");
}
void Version(const std::string& name, const Value& value) {
    Require(name.rfind("_version_", 0) == 0 && value.IsInt() && value.GetInt() == 0,
            "Unsupported mesh archive property/version");
}
void VectorMembers(const Value& value) {
    Unique(value);
    Require(value.IsObject(), "Invalid mesh archive vector");
    for (const auto& member : value.GetObject()) {
        const std::string name(member.name.GetString(), member.name.GetStringLength());
        if (name != "x" && name != "y" && name != "z") Version(name, member.value);
    }
}
void Preflight(const Document& document,std::size_t vertex_cap,std::size_t triangle_cap) {
    Require(document.MemberCount() == 1, "Unsupported mesh archive root");
    const auto& mesh = Member(document, "mesh");
    Require(mesh.IsObject(), "Missing mesh archive object");
    Unique(mesh);
    const std::set<std::string> arrays{"m_normals", "m_UV", "m_colors", "m_face_n_indices", "m_face_uv_indices",
        "m_face_col_indices", "m_face_mat_indices", "m_properties_per_vertex", "m_properties_per_face"};
    for (const auto& name : arrays) {
        const auto& value = Member(mesh, name.c_str());
        Require(value.IsArray() && value.Empty(), "Replay supports geometry-only archives without dynamic properties");
    }
    Require(Text(mesh, "m_filename").empty() && Text(mesh, "ChGeometry__Type") == "Type::TRIANGLEMESH_CONNECTED",
            "Mesh archive has an external filename or unsupported geometry type");
    for (const auto& member : mesh.GetObject()) {
        const std::string name(member.name.GetString(), member.name.GetStringLength());
        if (!arrays.count(name) && name != "m_vertices" && name != "m_face_v_indices" &&
            name != "m_filename" && name != "ChGeometry__Type") Version(name, member.value);
    }
    const auto& vertices = Member(mesh, "m_vertices"); const auto& faces = Member(mesh, "m_face_v_indices");
    Require(vertices.IsArray() && vertices.Size() >= 3 && vertices.Size() <= vertex_cap &&
            faces.IsArray() && faces.Size() && faces.Size() <= triangle_cap, "Mesh archive exceeds node/triangle caps");
    for (const auto& vertex : vertices.GetArray()) {
        VectorMembers(vertex);
        for (const char* axis : {"x", "y", "z"}) Real(vertex, axis);
    }
    for (const auto& face : faces.GetArray()) {
        VectorMembers(face);
        for (const char* axis : {"x", "y", "z"})
            Require(Unsigned(face, axis) < vertices.Size(), "Mesh archive connectivity is out of range");
    }
}
}  // namespace

std::shared_ptr<chrono::ChTriangleMeshConnected> ReadGeometryMesh(const std::string& bytes,
    std::size_t vertex_cap,std::size_t triangle_cap) {
    Require(vertex_cap && vertex_cap<=4096 && triangle_cap && triangle_cap<=8192, "Invalid geometry mesh caps");
    const auto document = array_json::Parse(bytes,32u<<20);
    Preflight(document,vertex_cap,triangle_cap); // Bound counts/types before Chrono allocates its arrays.
    auto mesh = std::make_shared<chrono::ChTriangleMeshConnected>();
    std::istringstream input(bytes);
    chrono::ChArchiveInJSON archive(input, true);
    archive >> chrono::make_ChNameValue("mesh", *mesh);
    const auto& source = document["mesh"];
    const auto& vertices = mesh->GetCoordsVertices(); const auto& faces = mesh->GetIndicesVertices();
    Require(vertices.size() == source["m_vertices"].Size() && faces.size() == source["m_face_v_indices"].Size(),
            "Chrono archive read changed mesh counts");
    const char* axes[]{"x", "y", "z"};
    for (std::size_t i = 0; i < vertices.size(); ++i)
        for (unsigned axis = 0; axis < 3; ++axis)
            Require(std::isfinite(vertices[i][axis]) && Bits(vertices[i][axis]) == Bits(Real(source["m_vertices"][i], axes[axis])),
                    "Chrono archive read changed coordinate bits");
    for (std::size_t i = 0; i < faces.size(); ++i) {
        const auto& face = faces[i];
        for (unsigned axis = 0; axis < 3; ++axis)
            Require(face[axis] >= 0 && static_cast<std::size_t>(face[axis]) < vertices.size() &&
                    static_cast<std::uint64_t>(face[axis]) == Unsigned(source["m_face_v_indices"][i], axes[axis]),
                    "Chrono archive read changed connectivity");
        const auto a = vertices[face[1]]-vertices[face[0]], b = vertices[face[2]]-vertices[face[0]];
        const auto cross = a.Cross(b);
        const double area = std::hypot(std::hypot(cross.x(), cross.y()), cross.z());
        Require(std::isfinite(area) && area > 0, "Replay contains a nonfinite or degenerate triangle");
    }
    return mesh;
}

} // namespace crash::output
