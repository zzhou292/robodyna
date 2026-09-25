#include "ContactSource.h"
#include "output/ArtifactIO.h"
#include "lib_utest/qualification/radioss_type25_fixed_main_startup/NativeOracle.h"
#include <gtest/gtest.h>
#include <cstdlib>
namespace crash::cases::native_scene {
TEST(NativeSceneContactSource, ReadyNormalsMatchIndependentAllActiveNativeProducerPhase) {
    namespace n=tlfea::contact::radioss_type25;
    const char* path=std::getenv("ROBO_DYNA_NATIVE_SCENE_EXPORT");ASSERT_NE(path,nullptr);
    const auto declared=modelio::native_scene::DeclaredSource::Read(path,output::Sha256(output::ReadBounded(path,4u<<20)));
    const auto physical=PhysicalSource::Prepare(declared,771);const auto contact=ContactSource::Prepare(physical,{1,2,3});
    const auto& d=declared.data();std::vector<std::uint64_t> ids;std::vector<double> xyz;
    for(const auto& node:d.nodes){ids.push_back(node.id);xyz.insert(xyz.end(),{node.xyz_mm.x,node.xyz_mm.y,node.xyz_mm.z});}
    std::vector<n::startup::PrimaryFace> faces;
    for(const auto& row:d.wall){n::startup::PrimaryFace face;face.source_id=row.id;face.layout=n::ShellLayout::Triangle3;
        for(unsigned j=0;j<4;++j)face.nodes[j]=row.nodes[j];faces.push_back(face);}
    // Actual independent Fortran Starter->ready computation with every source
    // main active. Later-cycle sampled normals belong to another phase and are
    // deliberately NOT used as this phase's expected values or normalized.
    const std::vector<double> coefficients(2*faces.size(),d.material.young_n_mm2*d.thickness_mm);
    n::startup::Input input;input.profile=n::startup::Profile::OrdinaryExteriorFixedMain;
    input.node_source_ids=ids.data();input.node_count=ids.size();input.positions={xyz.data(),std::uint32_t(ids.size()),3,1};
    input.primary=faces.data();input.primary_count=faces.size();input.coordinates=n::startup::Coordinates::Native;input.source_generation=3;
    const auto expected=type25_startup_test::Oracle(input,coefficients.data(),coefficients.size());
    const auto& source=contact.source().selection;
    ASSERT_EQ(expected.ready_normals.size(),4*source.main_count);ASSERT_EQ(expected.ready_references.size(),source.normal_count);
    const auto same=[](n::StoredNormal a,n::StoredNormal b){
        EXPECT_EQ(output::Bits(double(a.x)),output::Bits(double(b.x)));
        EXPECT_EQ(output::Bits(double(a.y)),output::Bits(double(b.y)));
        EXPECT_EQ(output::Bits(double(a.z)),output::Bits(double(b.z)));
    };
    for(std::size_t i=0;i<source.main_count;++i)for(unsigned j=0;j<4;++j)same(source.mains[i].normal_slot[j],expected.ready_normals[4*i+j]);
    for(std::size_t i=0;i<source.normal_count;++i){const auto& actual=source.normals[i];const auto& reference=expected.ready_references[i];
        EXPECT_EQ(actual.boundary,reference.boundary);if(actual.boundary){same(actual.bisector[0],reference.bisector[0]);same(actual.bisector[1],reference.bisector[1]);}}
}
}
