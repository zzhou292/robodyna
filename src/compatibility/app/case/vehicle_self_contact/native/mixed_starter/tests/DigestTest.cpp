#include "../Internal.h"
#include "lib_utest/qualification/radioss_type25_post_gapm/Fixture.h"
namespace crash::cases::vehicle_self_contact::native::mixed_starter {
namespace leaf = ::type25_post_gapm_test;
namespace d = detail;
namespace {
Provenance ProvenanceValue() {
    Provenance value;
    value.source_digest=std::string(64,'a');value.post_gapm_digest=std::string(64,'b');
    return value;
}
}
// These are digest value tests over genuine numerical snapshots. Mutated
// metadata below tests identity sensitivity, never source/geometry admission.
TEST(MixedStarterDigest, CompleteDomainBitsBindEvenUnusedEnvironmentNodes) {
    leaf::Fixture f(leaf::Blocks(1,true));
    ASSERT_EQ(f.report.status,s::Status::Ok);
    const auto provenance=ProvenanceValue();
    auto ids=std::vector<std::uint64_t>(f.input.node_source_ids,f.input.node_source_ids+f.input.node_count);
    auto positions=f.mesh.points;
    ids.push_back(9000);positions.push_back({10.,20.,30.});
    const auto original_positions=positions;
    auto input=f.input;input.node_count=ids.size();input.node_source_ids=ids.data();
    input.positions={reinterpret_cast<const double*>(positions.data()),std::uint32_t(positions.size()),3,1};
    s::NodePrefixExtension prefix;
    prefix.node_source_ids=f.input.node_source_ids;prefix.positions=f.input.positions;
    prefix.node_count=f.input.node_count;prefix.coordinates=f.input.coordinates;prefix.units=f.input.units;
    const auto forecast=s::PreflightMixedStarter(input,f.sides.result,f.post,prefix,{});
    ASSERT_EQ(forecast.status,s::Status::Ok);
    tl::util::HostArena arena,scratch;
    ASSERT_TRUE(arena.Initialize(forecast.output_bytes));ASSERT_TRUE(scratch.Initialize(forecast.scratch_bytes));
    s::Snapshot snapshot;
    ASSERT_EQ(s::BuildStarter(input,f.sides.result,f.post,prefix,{},arena,scratch,&snapshot).status,s::Status::Ok);
    const auto base=d::Digest(provenance,snapshot,input,1u<<20);
    EXPECT_EQ(base,d::Digest(provenance,snapshot,input,1u<<20));
    ids.back()=9001;
    EXPECT_NE(base,d::Digest(provenance,snapshot,input,1u<<20));
    ids.back()=9000;positions.back().x+=.125;
    EXPECT_NE(base,d::Digest(provenance,snapshot,input,1u<<20));
    positions=original_positions;input.positions.data=reinterpret_cast<const double*>(positions.data());
    input.units.length_m=2;
    EXPECT_NE(base,d::Digest(provenance,snapshot,input,1u<<20));
}
TEST(MixedStarterDigest, FloatSignsOriginSupportAndOrderedIncidenceRemainDistinct) {
    leaf::Fixture f(leaf::Blocks(1,true));
    ASSERT_EQ(f.report.status,s::Status::Ok);
    const auto provenance=ProvenanceValue();
    const auto base=d::Digest(provenance,f.startup,f.input,1u<<20);
    auto value=f.startup;
    auto normals=std::vector<tlfea::contact::radioss_type25::StoredNormal>(value.starter.face_normals,value.starter.face_normals+4*value.main_count);
    value.starter.face_normals=normals.data();
    bool changed=false;
    for(auto& normal:normals)if(normal.x==0){normal.x=std::signbit(normal.x)?0.f:-0.f;changed=true;break;}
    ASSERT_TRUE(changed);
    EXPECT_NE(base,d::Digest(provenance,value,f.input,1u<<20));
    value=f.startup;
    auto origins=std::vector<s::PrimaryFaceIdentity>(value.raw_origins,value.raw_origins+value.raw_origin_count);
    origins.back().physical_parent_id++;
    value.raw_origins=origins.data();
    EXPECT_NE(base,d::Digest(provenance,value,f.input,1u<<20));
    value=f.startup;
    auto supports=f.supports;supports.back().first.source_element_id++;
    auto post=*value.post_gapm;post.final_support=supports.data();value.post_gapm=&post;
    EXPECT_NE(base,d::Digest(provenance,value,f.input,1u<<20));
    value=f.startup;
    auto incidence=std::vector<std::uint32_t>(value.normal_mains,value.normal_mains+value.normal_incidence_count);
    incidence.front()++;value.normal_mains=incidence.data();
    EXPECT_NE(base,d::Digest(provenance,value,f.input,1u<<20));
    EXPECT_THROW(d::Digest(provenance,f.startup,f.input,64),std::exception);
    EXPECT_EQ(base,d::Digest(provenance,f.startup,f.input,1u<<20));
}
}
