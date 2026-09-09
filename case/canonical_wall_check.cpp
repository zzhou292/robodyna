#include "CanonicalWall.h"
#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

namespace {
namespace cw=crash::case_data;
std::string manifest_path;
// Four vertices, one source quad, two triangles: the complete small schema is
// explicit without duplicating the 62-node production asset as a string literal.
const char* fixture=R"json({
 "schema":"tlfea.yaris_fixed_wall.v1",
 "scope":"Fixed wall geometry only; no vehicle or material dynamics imported",
 "output_length_unit":"m","source_length_unit":"mm","length_scale":0.001,
 "contact":{"representation":"fixed_triangle_mesh","front_normal":[-1,0,0],
   "velocity_m_per_s":[0,0,0],"dynamic_wall_dofs":0,"analytic_force_generation":false,
   "additional_wall_offset_m":0,"friction":0.6,"source_display_thickness_m":0.001},
 "source":{"wall_file":"source/wall.key","combine_file":"source/combine.key",
   "wall_sha256":"0000000000000000000000000000000000000000000000000000000000000000",
   "combine_sha256":"1111111111111111111111111111111111111111111111111111111111111111",
   "model_archive_reference_sha256":"2222222222222222222222222222222222222222222222222222222222222222"},
 "generator":{"sha256":"3333333333333333333333333333333333333333333333333333333333333333"},
 "artifacts":{"wall.obj":{"sha256":"4444444444444444444444444444444444444444444444444444444444444444"}},
 "transform":{"node_id_offset":100,"element_id_offset":300,"part_id_offset":500,
   "rigid_wall_id_offset":0,"transformation_id":1,"translation_mm":[-4550,0,0]},
 "counts":{"source_nodes":4,"collision_vertices":4,"source_quads":1,"collision_triangles":2},
 "vertices":[
   {"vertex_index":0,"source_node_id":1,"assembled_source_node_id":101,"position_m":[0.05,0,0]},
   {"vertex_index":1,"source_node_id":2,"assembled_source_node_id":102,"position_m":[0.05,1,0]},
   {"vertex_index":2,"source_node_id":3,"assembled_source_node_id":103,"position_m":[0.05,1,1]},
   {"vertex_index":3,"source_node_id":4,"assembled_source_node_id":104,"position_m":[0.05,0,1]}],
 "source_quads":[{"source_quad_id":21,"assembled_source_quad_id":321,"source_part_id":31,
   "assembled_source_part_id":531,"source_node_ids":[1,2,3,4]}],
 "triangles":[
   {"triangle_id":11,"source_quad_id":21,"assembled_source_quad_id":321,"vertex_indices":[0,2,1],"source_node_ids":[1,3,2]},
   {"triangle_id":12,"source_quad_id":21,"assembled_source_quad_id":321,"vertex_indices":[0,3,2],"source_node_ids":[1,4,3]}],
 "stitching":{"tolerance_m":1e-10,"edges":[]},"bounds_m":[[0.05,0,0],[0.05,1,1]],
 "validation":{"area_m2":1,"euler_characteristic":1,"unique_edges":5,"interior_edges":1,
   "boundary_edges":[[1,2],[2,3],[3,4],[1,4]]},
 "reaction_groups":{"whole_wall_triangle_ids":[11,12],"source_segment_set_1001_triangle_ids":[11,12]}
})json";
rapidjson::Document Fixture() {
    rapidjson::Document document; document.Parse<rapidjson::kParseFullPrecisionFlag>(fixture); return document;
}
std::string Json(const rapidjson::Document& document) {
    rapidjson::StringBuffer buffer; rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    EXPECT_TRUE(document.Accept(writer)); return {buffer.GetString(),buffer.GetSize()};
}
cw::WallReport Load(cw::CanonicalWall& wall,const std::string& json,cw::WallLimits limits={}) {
    std::istringstream input(json); return wall.Load(input,limits);
}
std::uint64_t Bits(double value) { std::uint64_t result=0;std::memcpy(&result,&value,sizeof(result));return result; }
void ExpectEmpty(const cw::CanonicalWall& wall) {
    EXPECT_FALSE(wall.loaded()); EXPECT_TRUE(wall.vertices().empty()); EXPECT_TRUE(wall.triangles().empty());
    EXPECT_TRUE(wall.source_quads().empty()); EXPECT_TRUE(wall.stitching().empty());
    EXPECT_TRUE(wall.reaction_groups().whole_wall_triangle_ids.empty()); EXPECT_TRUE(wall.provenance().wall_sha256.empty()); EXPECT_EQ(wall.area_m2(),0);
}
template<class Mutation> void Reject(Mutation mutation,cw::WallStatus status=cw::WallStatus::InvalidData) {
    auto document=Fixture(); mutation(document); cw::CanonicalWall wall;
    const auto report=Load(wall,Json(document)); EXPECT_EQ(report.status,status)<<report.message; ExpectEmpty(wall);
    EXPECT_EQ(Load(wall,fixture).status,cw::WallStatus::Ok); // Late failure cannot poison a clean input retry.
}

TEST(CanonicalWall, RequiredOriginalAssetPreservesAllBinary64CoordinatesAndSourceMappings) {
    cw::CanonicalWall wall; const auto report=wall.LoadFile(manifest_path);
    ASSERT_EQ(report.status,cw::WallStatus::Ok)<<report.message;
    ASSERT_EQ(wall.vertices().size(),62u); ASSERT_EQ(wall.triangles().size(),100u); ASSERT_EQ(wall.source_quads().size(),46u);
    EXPECT_EQ(wall.front_normal(),(std::array<double,3>{-1,0,0}));
    // Tight regression for this pinned asset; not a general 100-term floating
    // summation error bound. Coordinate fidelity is checked bitwise below.
    EXPECT_NEAR(wall.area_m2(),3.565455298182685,4e-15);
    EXPECT_EQ(wall.provenance().wall_sha256,"ef02a4701b37d27cec81b1f9a02ab555f55ac61f68b070e8b0c18dc23b1d5155");
    EXPECT_EQ(wall.provenance().combine_sha256,"3e0137cd8c569a71a4281cc307549dc2f20772bc67eac0658ae73682dbe242a2");
    EXPECT_EQ(wall.provenance().obj_sha256,"08cb67124534ec16fab19e9503472c339eee9f0c79cc4ca20d3d6b014d6ed4e9");
    // Independent floating conversion: RapidJSON only retains raw NUMBER TOKENS
    // here; libc strtod (not RapidJSON GetDouble) supplies each binary64 oracle.
    std::ifstream input(manifest_path,std::ios::binary); ASSERT_TRUE(input.good());
    rapidjson::IStreamWrapper stream(input); rapidjson::Document tokens;
    tokens.ParseStream<rapidjson::kParseNumbersAsStringsFlag>(stream); ASSERT_FALSE(tokens.HasParseError());
    ASSERT_EQ(tokens["vertices"].Size(),wall.vertices().size());
    bool would_lose_float_precision=false;
    for(rapidjson::SizeType i=0;i<tokens["vertices"].Size();++i) {
        const auto& vertex=wall.vertices()[i]; EXPECT_EQ(vertex.vertex_index,i);
        EXPECT_EQ(vertex.source_node_id,1001u+i); EXPECT_EQ(vertex.assembled_source_node_id,10001001u+i);
        for(unsigned axis=0;axis<3;++axis) {
            const auto& token=tokens["vertices"][i]["position_m"][axis]; ASSERT_TRUE(token.IsString());
            char* end=nullptr; const double expected=std::strtod(token.GetString(),&end);
            ASSERT_EQ(end,token.GetString()+token.GetStringLength()); ASSERT_TRUE(std::isfinite(expected));
            EXPECT_EQ(Bits(vertex.position_m[axis]),Bits(expected));
            would_lose_float_precision|=double(float(expected))!=expected;
        }
    }
    EXPECT_TRUE(would_lose_float_precision);
    for(std::size_t i=0;i<wall.triangles().size();++i) {
        const auto& triangle=wall.triangles()[i]; EXPECT_EQ(triangle.triangle_id,i+1);
        EXPECT_EQ(triangle.assembled_source_quad_id,triangle.source_quad_id+10000000u);
        for(unsigned j=0;j<3;++j)EXPECT_EQ(wall.vertices()[triangle.vertex_indices[j]].source_node_id,triangle.source_node_ids[j]);
    }
    ASSERT_EQ(wall.stitching().size(),1u); EXPECT_EQ(wall.stitching()[0].source_quad_id,1046u);
    EXPECT_EQ(wall.stitching()[0].source_edge,(std::array<std::uint64_t,2>{1006,1060}));
    EXPECT_EQ(wall.stitching()[0].inserted_source_node_ids,(std::vector<std::uint64_t>{1012,1018,1024,1030,1036,1042,1048,1054}));
    EXPECT_EQ(wall.reaction_groups().whole_wall_triangle_ids.size(),100u);
    EXPECT_EQ(wall.reaction_groups().source_segment_set_1001_triangle_ids.size(),90u);
}

TEST(CanonicalWall, UnsignedIdsAboveDoublePrecisionRemainExactInEveryMapping) {
    auto document=Fixture(); constexpr std::uint64_t base=std::uint64_t{1}<<53;
    auto shift=[](rapidjson::Value& value){value.SetUint64(value.GetUint64()+base);};
    for(auto& v:document["vertices"].GetArray()){shift(v["source_node_id"]);shift(v["assembled_source_node_id"]);}
    auto& quad=document["source_quads"][0]; for(auto key:{"source_quad_id","assembled_source_quad_id","source_part_id","assembled_source_part_id"})shift(quad[key]);
    for(auto& id:quad["source_node_ids"].GetArray())shift(id);
    for(auto& t:document["triangles"].GetArray()) {
        for(auto key:{"triangle_id","source_quad_id","assembled_source_quad_id"})shift(t[key]);
        for(auto& id:t["source_node_ids"].GetArray())shift(id);
    }
    for(auto& edge:document["validation"]["boundary_edges"].GetArray())for(auto& id:edge.GetArray())shift(id);
    for(auto key:{"whole_wall_triangle_ids","source_segment_set_1001_triangle_ids"})for(auto& id:document["reaction_groups"][key].GetArray())shift(id);
    cw::CanonicalWall wall; const auto report=Load(wall,Json(document)); ASSERT_EQ(report.status,cw::WallStatus::Ok)<<report.message;
    EXPECT_EQ(wall.vertices()[0].source_node_id,base+1); EXPECT_EQ(wall.vertices()[3].assembled_source_node_id,base+104);
    EXPECT_EQ(wall.source_quads()[0].source_quad_id,base+21); EXPECT_EQ(wall.source_quads()[0].assembled_source_part_id,base+531);
    EXPECT_EQ(wall.triangles()[0].triangle_id,base+11); EXPECT_EQ(wall.triangles()[1].source_node_ids[1],base+4);
    EXPECT_EQ(wall.reaction_groups().whole_wall_triangle_ids[0],base+11);
    EXPECT_EQ(wall.reaction_groups().whole_wall_triangle_ids[1],base+12);
}

TEST(CanonicalWall, PrecisionAndIntegerTypeErrorsAreRejectedWithoutCoercion) {
    Reject([](auto& d){d["vertices"][0]["source_node_id"].SetDouble(1.);});
    Reject([](auto& d){d["vertices"][0]["source_node_id"].SetInt64(-1);});
    Reject([](auto& d){d["vertices"][0]["source_node_id"].SetString("1");});
    Reject([](auto& d){d["vertices"][0]["source_node_id"].SetDouble(9007199254740993.);});
    Reject([](auto& d){d["vertices"][0]["source_node_id"].SetUint64(UINT64_MAX);d["vertices"][0]["assembled_source_node_id"].SetUint64(UINT64_MAX);});
    Reject([](auto& d){d["vertices"][0]["position_m"][1].SetString("NaN");});
    std::string overflow=fixture; const auto position=overflow.find("0.05"); ASSERT_NE(position,std::string::npos); overflow.replace(position,4,"1e400");
    cw::CanonicalWall wall; EXPECT_EQ(Load(wall,overflow).status,cw::WallStatus::ParseError); ExpectEmpty(wall);
}

TEST(CanonicalWall, SourceRoleAndDuplicateJsonKeysAreNotSilentlyAccepted) {
    Reject([](auto& d){d["schema"].SetString("tlfea.yaris_vehicle.v1");},cw::WallStatus::InvalidSchema);
    Reject([](auto& d){d["source"]["wall_file"].SetString("source/vehicle.key");},cw::WallStatus::InvalidSchema);
    Reject([](auto& d){d["contact"]["analytic_force_generation"].SetBool(true);});
    Reject([](auto& d){d["contact"]["dynamic_wall_dofs"].SetUint(1);});
    Reject([](auto& d){d["contact"]["additional_wall_offset_m"].SetDouble(.001);});
    Reject([](auto& d){d["contact"]["velocity_m_per_s"][0].SetDouble(1);});
    Reject([](auto& d){d.RemoveMember("counts");},cw::WallStatus::InvalidSchema);
    Reject([](auto& d){d["source"]["wall_sha256"].SetString("invalid");});
    Reject([](auto& d){d.AddMember("schema",rapidjson::Value("tlfea.yaris_fixed_wall.v1",d.GetAllocator()),d.GetAllocator());},cw::WallStatus::InvalidSchema);
}

TEST(CanonicalWall, DuplicateEntitiesBadIndicesAndInconsistentSourceMapsFail) {
    Reject([](auto& d){d["vertices"][1]["vertex_index"].SetUint(0);});
    Reject([](auto& d){d["vertices"][1]["source_node_id"].SetUint(1);d["vertices"][1]["assembled_source_node_id"].SetUint(101);});
    Reject([](auto& d){d["triangles"][1]["triangle_id"].SetUint(11);});
    Reject([](auto& d){d["triangles"][0]["vertex_indices"][0].SetUint(4);});
    Reject([](auto& d){d["triangles"][0]["source_node_ids"][0].SetUint(4);});
    Reject([](auto& d){d["triangles"][0]["assembled_source_quad_id"].SetUint(999);});
    Reject([](auto& d){d["source_quads"][0]["source_node_ids"][0].SetUint(99);});
    Reject([](auto& d){d["counts"]["collision_triangles"].SetUint(3);});
    Reject([](auto& d){d["triangles"][1].CopyFrom(d["triangles"][0],d.GetAllocator());d["triangles"][1]["triangle_id"].SetUint(12);});
}

TEST(CanonicalWall, WindingDegeneracyAndNonplanarGeometryAreRejected) {
    Reject([](auto& d){d["vertices"][0]["position_m"][0].SetDouble(std::nextafter(.05,1.));});
    Reject([](auto& d){d["vertices"][3]["position_m"].CopyFrom(d["vertices"][2]["position_m"],d.GetAllocator());});
    Reject([](auto& d){d["triangles"][0]["source_node_ids"][0].SetUint(3);d["triangles"][0]["vertex_indices"][0].SetUint(2);});
    Reject([](auto& d){auto& t=d["triangles"][0];t["source_node_ids"][1].SetUint(2);t["source_node_ids"][2].SetUint(3);t["vertex_indices"][1].SetUint(1);t["vertex_indices"][2].SetUint(2);});
    Reject([](auto& d){d["contact"]["front_normal"][0].SetDouble(1);});
}

TEST(CanonicalWall, FiniteCrossProductsCannotHideOverflowingAreaAggregation) {
    auto document=Fixture();
    // Each independently evaluated corner determinant is 1.1e308 and finite.
    // The source-quad sum of two determinants overflows before multiplication
    // by one half. A finite declared area must not pass infinity<=infinity.
    constexpr double width=1e154,height=1.1e154;
    ASSERT_TRUE(std::isfinite(width*height)); ASSERT_FALSE(std::isfinite(width*height+width*height));
    for(auto& vertex:document["vertices"].GetArray()) {
        auto& p=vertex["position_m"];p[1].SetDouble(p[1].GetDouble()*width);p[2].SetDouble(p[2].GetDouble()*height);
    }
    document["bounds_m"][1][1].SetDouble(width);document["bounds_m"][1][2].SetDouble(height);
    document["validation"]["area_m2"].SetDouble(width*height);
    cw::CanonicalWall wall; const auto report=Load(wall,Json(document)); EXPECT_EQ(report.status,cw::WallStatus::InvalidData); ExpectEmpty(wall);
}

TEST(CanonicalWall, StitchReactionAndBoundaryMapsAreValidatedBeforePublication) {
    Reject([](auto& d){d["reaction_groups"]["whole_wall_triangle_ids"][1].SetUint(11);});
    Reject([](auto& d){d["reaction_groups"]["source_segment_set_1001_triangle_ids"][0].SetUint(99);});
    Reject([](auto& d){d["validation"]["boundary_edges"][0][0].SetUint(99);});
    Reject([](auto& d){d["validation"]["unique_edges"].SetUint(6);});
    Reject([](auto& d){d["validation"]["area_m2"].SetDouble(.5);});
    Reject([](auto& d){d["bounds_m"][1][1].SetDouble(std::nextafter(1.,2.));});
    Reject([](auto& d){d["stitching"]["tolerance_m"].SetDouble(1);});
    Reject([](auto& d){
        rapidjson::Document stitch;stitch.Parse(R"({"source_quad_id":21,"source_edge":[1,3],"inserted_source_node_ids":[2]})");
        rapidjson::Value value;value.CopyFrom(stitch,d.GetAllocator());d["stitching"]["edges"].PushBack(value,d.GetAllocator());
    });
}

TEST(CanonicalWall, ByteCapsAndEntityCapsApplyBeforePublishingAnyData) {
    for(int mode=0;mode<4;++mode) {
        cw::WallLimits limits;
        if(mode==0)limits.max_json_bytes=100;
        if(mode==1)limits.max_vertices=3;
        if(mode==2)limits.max_triangles=1;
        if(mode==3)limits.max_source_quads=0;
        cw::CanonicalWall wall;EXPECT_EQ(Load(wall,fixture,limits).status,cw::WallStatus::ResourceLimit);ExpectEmpty(wall);
    }
    cw::WallLimits invalid;invalid.max_vertices=65;cw::CanonicalWall wall;
    EXPECT_EQ(Load(wall,fixture,invalid).status,cw::WallStatus::ResourceLimit);ExpectEmpty(wall);
    std::string deep(20,'[');deep+="0";deep+=std::string(20,']');
    EXPECT_EQ(Load(wall,deep).status,cw::WallStatus::ResourceLimit);ExpectEmpty(wall);
}

TEST(CanonicalWall, FailedReadAndReloadCannotExposePartialOrReplaceImmutableData) {
    cw::CanonicalWall wall;EXPECT_EQ(wall.LoadFile(manifest_path+".missing-required-asset").status,cw::WallStatus::IoError);ExpectEmpty(wall);
    EXPECT_EQ(Load(wall,"{").status,cw::WallStatus::ParseError);ExpectEmpty(wall);
    ASSERT_EQ(Load(wall,fixture).status,cw::WallStatus::Ok);
    const auto* vertices=wall.vertices().data();const auto* triangles=wall.triangles().data();const auto area=wall.area_m2();
    EXPECT_EQ(Load(wall,"{").status,cw::WallStatus::AlreadyLoaded);
    EXPECT_EQ(wall.LoadFile(manifest_path).status,cw::WallStatus::AlreadyLoaded);
    EXPECT_EQ(wall.vertices().data(),vertices);EXPECT_EQ(wall.triangles().data(),triangles);EXPECT_EQ(wall.area_m2(),area);
}
}  // namespace

int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);
    if(argc!=2) { std::cerr<<"Usage: canonical_wall_check required-canonical-manifest.json [gtest options]\n";return 2; }
    manifest_path=argv[1];std::ifstream required(manifest_path,std::ios::binary);
    if(!required) { std::cerr<<"Required canonical wall asset is missing: "<<manifest_path<<'\n';return 2; }
    return RUN_ALL_TESTS();
}
