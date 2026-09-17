#include "Support.h"
#include "../RunArchive.h"
#include "../RunState.h"
#include "output/MeshArchive.h"
#include "output/GeometryMeshArchive.h"
namespace crash::output::physical_run::test {
TEST(PhysicalRunWall, NamedReceiptAndReservationsRejectUnknownMissingAndAliasedFiles) {
    const auto c=Context();
    auto request=MakeWallRequest(c,4,.5,3);
    EXPECT_EQ(request.archive.static_files.size(),10u);
    EXPECT_NO_THROW(detail::ValidateRequest(request.archive,{},true));
    EXPECT_THROW(detail::ValidateRequest(request.archive,{},false),std::exception);
    auto bad=request;bad.archive.static_files.back().file="wall/unknown.obj";
    EXPECT_THROW(detail::ValidateRequest(bad.archive,{},true),std::exception);
    bad=request;--bad.archive.static_files.back().bytes;
    EXPECT_THROW(detail::ValidateRequest(bad.archive,{},true),std::exception);
    WallReceipt receipt{c.identity().source_instance,91,c.identity().source_mapping_sha256,{}};
    for(std::size_t i=0;i<WallFiles.size();++i)receipt.files[i]={WallFiles[i],Sha256("fixture"),7};
    EXPECT_EQ(ReadWallDocument(WallDocument(receipt)).wall_binding_id,91u);
    receipt.files.back().file=receipt.files.front().file;
    EXPECT_THROW(WallDocument(receipt),std::exception);
    auto config=Config(c);config.wall=true;config.request=request.archive;
    EXPECT_TRUE(ReadConfiguration(ConfigurationDocument(config)).wall);
    const auto plan=records::activity::PlanWithActivity(c,request.archive,"parent-activity.json");
    request.archive.total_byte_cap=plan.archive.forecast_bytes;
    EXPECT_EQ(records::activity::PlanWithActivity(c,request.archive,"parent-activity.json").archive.forecast_bytes,
        plan.archive.forecast_bytes);
    --request.archive.total_byte_cap;
    EXPECT_THROW(records::activity::PlanWithActivity(c,request.archive,"parent-activity.json"),std::exception);
}
TEST(PhysicalRunWall, SharedChronoGeometryReaderRetainsBitsAndRejectsLateTopology) {
    ft::Directory directory;
    chrono::ChTriangleMeshConnected mesh;
    mesh.GetCoordsVertices()={{1.,-.125,2.},{1.,1.,2.},{1.,1.,3.},{1.,0.,3.}};
    mesh.GetIndicesVertices()={{0,1,2},{0,2,3}};
    WriteMeshFiles(directory.path,"selected-wall",mesh);
    const auto bytes=ReadBounded(directory.path/"selected-wall.mesh.json",WallFileCap);
    const auto restored=ReadGeometryMesh(bytes);
    ASSERT_EQ(restored->GetNumVertices(),4u);ASSERT_EQ(restored->GetNumTriangles(),2u);
    for(std::size_t n=0;n<4;++n)for(unsigned a=0;a<3;++a)
        EXPECT_EQ(Bits(restored->GetCoordsVertices()[n][a]),Bits(mesh.GetCoordsVertices()[n][a]));
    EXPECT_THROW(ReadGeometryMesh(bytes,3,2),std::exception);
    auto bad=array_json::Parse(bytes,WallFileCap);
    bad["mesh"]["m_face_v_indices"][1]["z"].SetInt(4);
    const auto invalid=WriteDocument(directory.path,"bad-mesh.json",bad,WallFileCap);
    EXPECT_THROW(ReadGeometryMesh(ReadFile(directory.path,invalid,WallFileCap)),std::exception);
    bad=array_json::Parse(bytes,WallFileCap);
    bad["mesh"]["m_filename"].SetString("external.obj",bad.GetAllocator());
    const auto external=WriteDocument(directory.path,"external-mesh.json",bad,WallFileCap);
    EXPECT_THROW(ReadGeometryMesh(ReadFile(directory.path,external,WallFileCap)),std::exception);
    EXPECT_EQ(ReadGeometryMesh(bytes)->GetNumTriangles(),2u);
    mesh.GetCoordsVertices()[0].y()=-0.;
    // Existing Chrono JSON encodes -0 as an integer; the owning writer rejects
    // the lost sign instead of publishing changed bits. Binary frame I/O above
    // supports signed zero independently and retains its exact native bits.
    EXPECT_THROW(WriteMeshFiles(directory.path,"negative-zero",mesh),std::exception);
}
} // namespace crash::output::physical_run::test
