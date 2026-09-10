#include "chrono/AcceptedReplayScene.h"
#include "output/AcceptedReplay.h"
#include "output/ArtifactIO.h"
#include "output/SourcePartWallArtifactSchema.h"
#include "output/source_assembly/SourceAssemblyWallSchema.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <map>
#include <sstream>

namespace {
namespace out=crash::output;

out::Document ReadDocument(const std::filesystem::path& path,std::size_t cap) {
    const auto bytes=out::ReadBounded(path,cap);
    out::Document document;document.Parse<rapidjson::kParseFullPrecisionFlag>(bytes.data(),bytes.size());
    out::Require(!document.HasParseError()&&document.IsObject(),"Actual replay document cannot be parsed");
    return document;
}
const out::Value& Member(const out::Value& object,const char* key) {
    out::Require(object.IsObject()&&object.HasMember(key),"Actual replay member is missing");
    return object[key];
}
const out::Value& Array(const out::Value& value) {
    out::Require(value.IsArray(),"Actual replay array is missing");return value;
}
std::string AcceptedName(std::uint64_t epoch,const char* suffix) {
    std::ostringstream name;
    name << "accepted-" << std::setw(6) << std::setfill('0') << epoch << suffix;
    return name.str();
}

// Only the persisted section layout differs. Read every layer directly; do not
// use either reader's display extraction or the persisted parent maximum.
std::map<std::uint64_t,double> PointMaxima(const out::Value& fields,bool assembly) {
    const auto& container=assembly?Member(fields,"sections"):fields;
    const auto& rows=Array(Member(container,assembly?"sections":"plastic_sections"));
    const auto* parents=assembly?&Array(Member(container,"source_parents")):nullptr;
    out::Require(!parents||parents->Size()==rows.Size(),"Actual source parent count changed");
    std::map<std::uint64_t,double> expected;
    for(rapidjson::SizeType i=0;i<rows.Size();++i) {
        const auto& row=Array(rows[i]);
        out::Require(row.Size()==(assembly?10u:13u),"Actual section width changed");
        if(parents)out::Require(Array((*parents)[i]).Size()==11u,"Actual source mapping width changed");
        const auto& id=parents?(*parents)[i][1]:row[1];
        out::Require(id.IsUint64()&&id.GetUint64(),"Actual source EID is invalid");
        const auto& points=Array(row[assembly?9:12]);
        out::Require(points.Size()==3u,"Actual section must have three native points");
        double maximum=0;
        for(const auto& point:points.GetArray()) {
            out::Require(Array(point).Size()==7u&&point[5].IsNumber(),"Actual native point width changed");
            const double strain=point[5].GetDouble();
            out::Require(std::isfinite(strain)&&strain>=0,"Actual native plastic strain is invalid");
            maximum=std::max(maximum,strain);
        }
        out::Require(expected.emplace(id.GetUint64(),maximum).second,"Duplicate actual parent EID");
    }
    return expected;
}

// Independent scalar ramp oracle: no ReplayScalarColor, parent adapter or
// Chrono interpolation call participates in the expected color calculation.
std::array<float,3> PointColor(double strain,double maximum) {
    const std::array<float,3> blue{.12f,.64f,.94f},yellow{.98f,.84f,.16f},red{.90f,.12f,.10f};
    const double normalized=std::clamp(strain/maximum,0.,1.);
    const bool lower=normalized<=.5;const double blend=lower?2*normalized:2*normalized-1;
    const auto& start=lower?blue:yellow;const auto& end=lower?yellow:red;
    std::array<float,3> color{};
    for(unsigned axis=0;axis<3;++axis)color[axis]=static_cast<float>((1-blend)*start[axis]+blend*end[axis]);
    return color;
}

void CheckArchivedGeometry(const chrono::ChTriangleMeshConnected& mesh,const out::Value& document) {
    const auto& raw=Member(document,"mesh");
    const auto& vertices=Array(Member(raw,"m_vertices"));
    const auto& faces=Array(Member(raw,"m_face_v_indices"));
    ASSERT_EQ(mesh.GetCoordsVertices().size(),vertices.Size());
    ASSERT_EQ(mesh.GetIndicesVertices().size(),faces.Size());
    constexpr const char* axis_names[]={"x","y","z"};
    for(rapidjson::SizeType n=0;n<vertices.Size();++n)for(unsigned axis=0;axis<3;++axis) {
        const auto& value=Member(vertices[n],axis_names[axis]);ASSERT_TRUE(value.IsNumber());
        EXPECT_EQ(out::Bits(mesh.GetCoordsVertices()[n][axis]),out::Bits(value.GetDouble()));
    }
    for(rapidjson::SizeType t=0;t<faces.Size();++t)for(unsigned axis=0;axis<3;++axis) {
        const auto& value=Member(faces[t],axis_names[axis]);ASSERT_TRUE(value.IsInt());
        EXPECT_EQ(mesh.GetIndicesVertices()[t][axis],value.GetInt());
    }
}

void CheckActualPlasticBundle(const std::filesystem::path& fixture,out::ReplayKind kind) {
    const bool assembly=kind==out::ReplayKind::SourceAssemblyWall;
    out::AcceptedReplay reader;auto report=reader.Open(fixture);
    ASSERT_EQ(report.status,out::ReplayStatus::Ok) << report.diagnostic;
    const auto info=*reader.info();ASSERT_EQ(info.kind,kind);
    ASSERT_TRUE(info.source_plasticity);ASSERT_GT(info.source_initial_speed_m_per_s,0);
    ASSERT_EQ(info.triangle_source_parent.size(),info.triangle_count);
    if(assembly) {
        ASSERT_TRUE(info.source_assembly);ASSERT_EQ(info.node_count,1030u);ASSERT_EQ(info.triangle_count,1719u);
        EXPECT_EQ(info.source_assembly->parents,915u);EXPECT_EQ(info.source_assembly->qeph,804u);
        EXPECT_EQ(info.source_assembly->t3,111u);EXPECT_TRUE(info.material_model.empty());
    }
    const auto config=ReadDocument(fixture/"configuration.json",out::assembly::WallConfigurationBytes);
    const auto& triangle_binding=Array(Member(config,"triangle_binding"));
    ASSERT_EQ(triangle_binding.Size(),info.triangle_count);
    const auto recorded_wall=ReadDocument(fixture/"placed-wall.mesh.json",out::assembly::WallMeshBytes);
    crash::visual::AcceptedReplayScene scene;
    ASSERT_EQ(scene.Initialize(info,*reader.frame(),reader.wall()).status,crash::visual::ReplaySceneStatus::Ok);
    const auto shape=scene.moving_shape();const auto mesh=shape->GetMesh();
    ASSERT_EQ(shape->GetNumMaterials(),0);ASSERT_EQ(scene.deformation_scale(),1);
    const auto wall=scene.wall_mesh();ASSERT_TRUE(wall);
    bool observed_yield=false;double largest_point_strain=0;
    for(std::size_t i=0;i<info.frame_count;++i) {
        SCOPED_TRACE(i);
        if(i) {
            report=reader.Load(i);ASSERT_EQ(report.status,out::ReplayStatus::Ok) << report.diagnostic;
            ASSERT_EQ(scene.Publish(*reader.frame()).status,crash::visual::ReplaySceneStatus::Ok);
        }
        const auto& frame=*reader.frame();
        const auto fields=ReadDocument(fixture/AcceptedName(frame.epoch,".fields.json"),
            assembly?out::assembly::WallFieldBytes:out::SourcePartPlasticWallFieldCap);
        ASSERT_TRUE(Member(fields,"accepted_epoch").IsUint64());
        EXPECT_EQ(fields["accepted_epoch"].GetUint64(),frame.epoch);
        ASSERT_TRUE(Member(fields,"accepted_time_s").IsNumber());
        EXPECT_EQ(out::Bits(fields["accepted_time_s"].GetDouble()),out::Bits(frame.time));
        const auto expected=PointMaxima(fields,assembly);
        ASSERT_EQ(frame.parent_plastic_strain.size(),expected.size());
        if(assembly)ASSERT_EQ(expected.size(),915u);
        for(const auto& parent:frame.parent_plastic_strain) {
            const auto found=expected.find(parent.source_parent);ASSERT_NE(found,expected.end());
            EXPECT_EQ(out::Bits(parent.value),out::Bits(found->second));
            observed_yield|=found->second>0;largest_point_strain=std::max(largest_point_strain,found->second);
        }
        const auto& colors=mesh->GetFaceColors();ASSERT_EQ(colors.size(),3*info.triangle_count);
        for(std::size_t t=0;t<info.triangle_count;++t) {
            const auto& binding=Array(triangle_binding[t]);ASSERT_EQ(binding.Size(),9u);ASSERT_TRUE(binding[5].IsUint64());
            const auto eid=binding[5].GetUint64();EXPECT_EQ(info.triangle_source_parent[t],eid);
            const auto found=expected.find(eid);ASSERT_NE(found,expected.end());
            const auto color=PointColor(found->second,info.plastic_strain_color_max);
            for(unsigned corner=0;corner<3;++corner) {
                ASSERT_TRUE(binding[corner].IsUint());
                EXPECT_EQ(mesh->GetIndicesVertices()[t][corner],binding[corner].GetUint());
                EXPECT_FLOAT_EQ(colors[3*t+corner].R,color[0]);
                EXPECT_FLOAT_EQ(colors[3*t+corner].G,color[1]);
                EXPECT_FLOAT_EQ(colors[3*t+corner].B,color[2]);
            }
        }
        const auto archived_mesh=ReadDocument(fixture/AcceptedName(frame.epoch,".mesh.json"),out::assembly::WallMeshBytes);
        ASSERT_NO_FATAL_FAILURE(CheckArchivedGeometry(*mesh,archived_mesh));
        ASSERT_NO_FATAL_FAILURE(CheckArchivedGeometry(*wall,recorded_wall));
        const auto& nodal=assembly?Member(fields,"nodal_fields"):fields;
        const auto& position=Array(Member(nodal,"position_xyz_m"));ASSERT_EQ(position.Size(),3*info.node_count);
        for(std::size_t n=0;n<info.node_count;++n)for(unsigned axis=0;axis<3;++axis) {
            ASSERT_TRUE(position[3*n+axis].IsNumber());
            EXPECT_EQ(out::Bits(mesh->GetCoordsVertices()[n][axis]),out::Bits(position[3*n+axis].GetDouble()));
        }
        EXPECT_EQ(scene.moving_shape(),shape);EXPECT_EQ(scene.moving_mesh().get(),mesh.get());
        EXPECT_EQ(scene.wall_mesh(),wall);EXPECT_EQ(scene.stamp()->epoch,frame.epoch);
        EXPECT_DOUBLE_EQ(scene.stamp()->time,frame.time);
    }
    EXPECT_TRUE(observed_yield) << "This optional gate requires actual nonzero accepted plastic histories";
    EXPECT_DOUBLE_EQ(info.plastic_strain_color_max,std::max(.001,largest_point_strain));
}

TEST(AcceptedReplayScene, ActualPlasticBundleStagesRecordedPointsAndPhysicalGeometry) {
    const auto* fixture=std::getenv("ROBO_DYNA_PLASTIC_REPLAY_FIXTURE");
    if(!fixture||!*fixture)GTEST_SKIP() << "Set ROBO_DYNA_PLASTIC_REPLAY_FIXTURE to a completed yielding archive";
    CheckActualPlasticBundle(fixture,out::ReplayKind::SourcePartWall);
}
TEST(AcceptedReplayScene, ActualAssemblyBundleStagesEveryNativeParentAndPhysicalMesh) {
    const auto* fixture=std::getenv("ROBO_DYNA_SOURCE_ASSEMBLY_REPLAY_FIXTURE");
    if(!fixture||!*fixture)GTEST_SKIP() << "Set ROBO_DYNA_SOURCE_ASSEMBLY_REPLAY_FIXTURE to a completed assembly archive";
    CheckActualPlasticBundle(fixture,out::ReplayKind::SourceAssemblyWall);
}
} // namespace
