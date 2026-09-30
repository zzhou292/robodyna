#include "EnvironmentMeshFixture.h"
namespace crash::output::physical_run::test {
TEST(PhysicalEnvironmentMesh, RealCodecPreservesDeclaredReferenceBitsAndKeepsCanonicalSourceSeparate) {
    DeclaredMesh fixture;const auto mesh=fixture.Read();
    ASSERT_EQ(mesh->GetNumVertices(),4u);ASSERT_EQ(mesh->GetNumTriangles(),2u);
    EXPECT_NE(Bits(mesh->GetCoordsVertices()[0].x()),Bits(array_json::Real(fixture.document["contact_front_plane_m"])));
    for(unsigned i=0;i<4;++i)for(unsigned k=0;k<3;++k)
        EXPECT_EQ(Bits(mesh->GetCoordsVertices()[i][k]),Bits(array_json::Real(fixture.document["reference_m"][i][k])));
    EXPECT_EQ(fixture.receipt.files.size(),3u);
    EXPECT_FALSE(std::filesystem::exists(fixture.directory.path/"wall/original-canonical-wall.manifest.json"));
    const auto old=ReadFile(fixture.directory.path,fixture.receipt.files[1],EnvironmentFileCap);
    records::test::Overwrite(fixture.directory.path/EnvironmentFiles[1],old+"corrupt");
    EXPECT_THROW(fixture.Read(),std::exception);
    EXPECT_EQ(mesh->GetNumTriangles(),2u);
    records::test::Overwrite(fixture.directory.path/EnvironmentFiles[1],old);
    EXPECT_NO_THROW(fixture.Read());
}
TEST(PhysicalEnvironmentMesh, AuthenticatedLateDescriptorMismatchesFailWithoutReplacingPublishedMesh) {
    DeclaredMesh fixture;const auto accepted=fixture.Read();
    for(unsigned bad=0;bad<9;++bad) {
        Document doc;doc.CopyFrom(fixture.document,doc.GetAllocator());
        if(bad==0)doc["vehicle_canonical_sha256"].SetString(Sha256("other").c_str(),doc.GetAllocator());
        if(bad==1)doc["physical_nodes"].SetUint64(14);
        if(bad==2)doc["node_ids"][3].SetUint64(1001);
        if(bad==3)doc["reference_m"][3][2].SetDouble(1.25);
        if(bad==4)doc["reference_offset_m"].SetDouble(.25);
        if(bad==5)doc["triangles"][1][2].SetUint(2);
        if(bad==6)doc["domain_nodes"][3].SetUint64(8);
        if(bad==7)doc["thickness_m"].SetDouble(.002);
        if(bad==8)doc["mesh_sha256"].SetString(Sha256("other mesh").c_str(),doc.GetAllocator());
        fixture.Replace(std::move(doc),bad);
        EXPECT_THROW(fixture.Read(),std::exception);
        EXPECT_EQ(Bits(accepted->GetCoordsVertices()[3].z()),Bits(1.));
    }
    Document retry;retry.CopyFrom(fixture.document,retry.GetAllocator());fixture.Replace(std::move(retry),9);
    EXPECT_NO_THROW(fixture.Read());
}
}
