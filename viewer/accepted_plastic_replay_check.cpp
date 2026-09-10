#include "chrono/AcceptedReplayScene.h"
#include "chrono/ReplayParentScalarColors.h"
#include "output/AcceptedReplay.h"
#include "output/ArtifactIO.h"
#include "output/SourcePartWallArtifactSchema.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <map>
#include <sstream>

namespace {
crash::output::Document AcceptedPointFields(const std::filesystem::path& directory,std::uint64_t epoch) {
    std::ostringstream name;
    name << "accepted-" << std::setw(6) << std::setfill('0') << epoch << ".fields.json";
    const auto bytes=crash::output::ReadBounded(directory/name.str(),crash::output::SourcePartPlasticWallFieldCap);
    crash::output::Document document;document.Parse(bytes.data(),bytes.size());
    crash::output::Require(!document.HasParseError(),"Actual plastic point fields cannot be parsed");
    return document;
}

TEST(AcceptedReplayScene, ActualPlasticBundleStagesRecordedPointsAndPhysicalGeometry) {
    const auto* fixture=std::getenv("ROBO_DYNA_PLASTIC_REPLAY_FIXTURE");
    if(!fixture||!*fixture)GTEST_SKIP() << "Set ROBO_DYNA_PLASTIC_REPLAY_FIXTURE to a completed yielding archive";
    crash::output::AcceptedReplay reader;
    auto report=reader.Open(fixture);
    ASSERT_EQ(report.status,crash::output::ReplayStatus::Ok) << report.diagnostic;
    const auto info=*reader.info();
    ASSERT_TRUE(info.source_plasticity);ASSERT_GT(info.source_initial_speed_m_per_s,0);
    ASSERT_EQ(info.triangle_source_parent.size(),info.triangle_count);
    crash::visual::AcceptedReplayScene scene;
    ASSERT_EQ(scene.Initialize(info,*reader.frame(),reader.wall()).status,crash::visual::ReplaySceneStatus::Ok);
    const auto shape=scene.moving_shape();const auto mesh=shape->GetMesh();
    ASSERT_EQ(shape->GetNumMaterials(),0);ASSERT_EQ(scene.deformation_scale(),1);
    const auto wall=scene.wall_mesh();ASSERT_TRUE(wall);
    const auto wall_coordinates=wall->GetCoordsVertices();
    bool observed_yield=false;double largest_point_strain=0;
    for(std::size_t i=0;i<info.frame_count;++i) {
        SCOPED_TRACE(i);
        if(i) {
            report=reader.Load(i);
            ASSERT_EQ(report.status,crash::output::ReplayStatus::Ok) << report.diagnostic;
            ASSERT_EQ(scene.Publish(*reader.frame()).status,crash::visual::ReplaySceneStatus::Ok);
        }
        const auto& frame=*reader.frame();
        // Independent read of the recorded point component, not the display
        // summary or the reader's newly added parent-field extraction helper.
        const auto fields=AcceptedPointFields(fixture,frame.epoch);
        ASSERT_TRUE(fields.HasMember("plastic_sections"));
        const auto& sections=fields["plastic_sections"];ASSERT_TRUE(sections.IsArray());
        std::map<std::uint64_t,double> expected;
        for(const auto& section:sections.GetArray()) {
            ASSERT_TRUE(section.IsArray());ASSERT_EQ(section.Size(),13u);
            ASSERT_TRUE(section[1].IsUint64());ASSERT_TRUE(section[12].IsArray());ASSERT_EQ(section[12].Size(),3u);
            double maximum=0;
            for(const auto& point:section[12].GetArray()) {
                ASSERT_TRUE(point.IsArray());ASSERT_EQ(point.Size(),7u);ASSERT_TRUE(point[5].IsNumber());
                maximum=std::max(maximum,point[5].GetDouble());
            }
            ASSERT_TRUE(expected.emplace(section[1].GetUint64(),maximum).second);
            observed_yield|=maximum>0;largest_point_strain=std::max(largest_point_strain,maximum);
        }
        ASSERT_EQ(frame.parent_plastic_strain.size(),expected.size());
        for(const auto& parent:frame.parent_plastic_strain) {
            const auto found=expected.find(parent.source_parent);ASSERT_NE(found,expected.end());
            EXPECT_DOUBLE_EQ(parent.value,found->second);
        }
        const auto& face_colors=mesh->GetFaceColors();ASSERT_EQ(face_colors.size(),3*info.triangle_count);
        for(std::size_t t=0;t<info.triangle_count;++t) {
            const auto found=expected.find(info.triangle_source_parent[t]);ASSERT_NE(found,expected.end());
            const auto color=crash::visual::ReplayScalarColor(found->second/info.plastic_strain_color_max);
            for(unsigned corner=0;corner<3;++corner) {
                EXPECT_FLOAT_EQ(face_colors[3*t+corner].R,color.R);
                EXPECT_FLOAT_EQ(face_colors[3*t+corner].G,color.G);
                EXPECT_FLOAT_EQ(face_colors[3*t+corner].B,color.B);
            }
        }
        EXPECT_EQ(mesh->GetCoordsVertices(),frame.mesh->GetCoordsVertices());
        EXPECT_EQ(scene.moving_shape(),shape);EXPECT_EQ(scene.moving_mesh().get(),mesh.get());
        EXPECT_EQ(scene.wall_mesh(),wall);EXPECT_EQ(wall->GetCoordsVertices(),wall_coordinates);
        EXPECT_EQ(scene.stamp()->epoch,frame.epoch);EXPECT_DOUBLE_EQ(scene.stamp()->time,frame.time);
    }
    EXPECT_TRUE(observed_yield) << "This optional gate requires actual nonzero accepted plastic histories";
    EXPECT_DOUBLE_EQ(info.plastic_strain_color_max,std::max(.001,largest_point_strain));
}
} // namespace
