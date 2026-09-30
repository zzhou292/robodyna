#include "chrono/core/ChMatrix.h"
#include "Q4ContactProfile.h"
#include "chrono/GuidedPlateModel.h"
#include "case/CanonicalWallArtifacts.h"
#include "output/ArtifactIO.h"
#include <sstream>

namespace crash::profile {
ContactProfileSource ReadContactProfileSource(const std::string& wall_path,const std::string& frame_path,const std::string& expected) {
    namespace io=output;namespace cd=case_data;
    const auto bytes=cd::ReadPinnedWallManifest(wall_path);std::istringstream stream(bytes);cd::CanonicalWall wall;
    const auto loaded=wall.Load(stream);io::Require(loaded.status==cd::WallStatus::Ok,loaded.message);
    std::vector<contact::PlanarWallVertex> vertices;std::vector<contact::PlanarWallTriangle> triangles;
    for(const auto& v:wall.vertices())vertices.push_back({{v.position_m[0],v.position_m[1],v.position_m[2]},v.source_node_id,v.assembled_source_node_id});
    for(const auto& t:wall.triangles())triangles.push_back({{t.vertex_indices[0],t.vertex_indices[1],t.vertex_indices[2]},t.triangle_id,t.source_quad_id,t.assembled_source_quad_id});
    reference::GuidedPlateModel model({vertices.data(),static_cast<std::uint32_t>(vertices.size()),triangles.data(),static_cast<std::uint32_t>(triangles.size())},1);
    const auto frame=io::ReadBounded(frame_path,128*1024);
    ContactProfileSource source;source.frame_sha256=io::Sha256(frame);source.wall_sha256=io::Sha256(bytes);
    io::Require(source.frame_sha256==expected,"Prescribed frame differs from declared SHA256");
    io::Document d;d.Parse<rapidjson::kParseFullPrecisionFlag|rapidjson::kParseIterativeFlag>(frame.data(),frame.size());
    io::Require(!d.HasParseError()&&d.IsObject()&&d.HasMember("schema")&&d["schema"].IsString()&&
                std::string(d["schema"].GetString())=="robo_dyna.guided_plate_fields.v1","Invalid prescribed guided frame schema");
    auto& input=source.input;
    for(const auto& field:std::array<std::pair<const char*,double*>,2>{{{"position_xyz_m",input.position},{"velocity_xyz_m_per_s",input.velocity}}}) {
        io::Require(d.HasMember(field.first)&&d[field.first].IsArray()&&d[field.first].Size()==18,"Prescribed coordinates need six finite nodes");
        for(unsigned n=0;n<18;++n) {
            const auto& value=d[field.first][n];io::Require(value.IsNumber()&&std::isfinite(value.GetDouble()),"Nonfinite prescribed coordinate");
            field.second[n]=value.GetDouble();
        }
    }
    for(const auto* key:{"accepted_epoch","contact_evaluation_attempt"})io::Require(d.HasMember(key)&&d[key].IsUint64(),"Missing prescribed frame identity");
    io::Require(d.HasMember("accepted_time_s")&&d["accepted_time_s"].IsNumber()&&std::isfinite(d["accepted_time_s"].GetDouble()),"Missing prescribed physical time");
    input.epoch=d["accepted_epoch"].GetUint64();input.attempt=d["contact_evaluation_attempt"].GetUint64();source.accepted_time=d["accepted_time_s"].GetDouble();
    const auto& guided=model.data();const auto& shell=model.shell().data();const auto ref=model.contact_geometry().view();
    input.wall_x=ref.wall_x;input.stiffness=guided.stiffness_per_area;input.maximum_penetration=guided.maximum_penetration;input.limits=guided.integration;
    for(unsigned n=0;n<6;++n) {input.inverse_mass[n]=shell.inverse_mass[n];input.fixed[n]=guided.translation_fixed_bits[n];}
    for(unsigned p=0;p<2;++p) {input.parents[p]=guided.parents[p];input.area[p]=ref.parents[p].projected_area;}
    for(unsigned p=0;p<2;++p) {
        contact::Q4PreparedIntegration prepared;
        const auto view=input.View(p);
        const auto status=contact::PrepareQ4PlanarIntegration(ref,view.surface,view.mass,p,input.stiffness,input.maximum_penetration,input.attempt,&prepared);
        io::Require(status==contact::PlanarContactStatus::Ok&&prepared.covered,"Prescribed input violates original C3 finite-wall coverage/motion");
    }
    return source;
}
} // namespace crash::profile
