#include "chrono/core/ChMatrix.h"
#include "WallStudyProvenance.h"
#include "CanonicalWallArtifacts.h"
#include "GuidedPlateStudyIO.h"
#include "guided_plate_study_fixture.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

namespace {
using namespace crash::case_data;
namespace fixture=crash::case_data::study_test;
namespace io=crash::output;
namespace fs=std::filesystem;
using Kind=WallTessellationKind;
std::string asset;
class TempRoot {
  public:
    TempRoot() {
        const auto pattern=(fs::temp_directory_path()/"wall-study-provenance-XXXXXX").string();
        std::vector<char> buffer(pattern.begin(),pattern.end());buffer.push_back(0);
        const char* made=::mkdtemp(buffer.data());if(!made)throw std::runtime_error("Cannot create isolated wall-study test directory");path=made;
    }
    ~TempRoot(){std::error_code ignored;fs::remove_all(path,ignored);}
    fs::path path;
};
class WallStudyProvenanceCheck:public ::testing::Test {
  protected:
    TempRoot root;CanonicalWall original;std::string canonical;
    void SetUp() override {
        ASSERT_NO_THROW(canonical=ReadPinnedWallManifest(asset));
        std::istringstream input(canonical);const auto r=original.Load(input);ASSERT_EQ(r.status,WallStatus::Ok)<<r.message;
    }
    GuidedStudyConfig Config(Kind kind) const {
        // Reuse the completed host observer fixture; only its reference footprint
        // is placed inside the actual source wall. No mechanics run is implied.
        auto c=fixture::Config(1,(std::uint64_t{1}<<63)+71);
        c.qualification_id=kGuidedPlateQualification;c.wall_binding_id=WallTessellationBindingId(kind);
        const auto bounds=original.bounds_m();c.wall_x=bounds[0][0];
        const double y=.5*bounds[0][1]+.5*bounds[1][1],z=.5*bounds[0][2]+.5*bounds[1][2];
        const double dy[]{0,-.1,-.1,0,.1,.1};
        for(unsigned n=0;n<6;++n) {
            c.reference_position[3*n]=c.wall_x-.01;c.reference_position[3*n+1]=y+dy[n];
            c.reference_position[3*n+2]=z+(n==0||n==1||n==4?.05:-.05);
        }
        for(auto& p:c.contact_reference)for(unsigned n=0;n<4;++n) {
            const auto node=p.parent.nodes[n];p.reference_projection[n]={c.wall_x,c.reference_position[3*node+1],c.reference_position[3*node+2]};
        }
        return c;
    }
    std::string StudyBytes(const GuidedStudyConfig& config,const std::string& basename) {
        const auto data=fixture::Run(config);const auto path=root.path/basename;WriteGuidedPlateStudy(path,data);
        return io::ReadBounded(path,kGuidedStudyByteCap);
    }
};
io::Document Json(const std::string& bytes) {
    io::Document d;d.Parse<rapidjson::kParseFullPrecisionFlag>(bytes.data(),bytes.size());
    io::Require(!d.HasParseError()&&d.IsObject(),"Invalid sidecar test JSON");return d;
}
std::string Bytes(const fs::path& path){return io::ReadBounded(path,kWallStudyProvenanceByteCap);}
void Bits(double a,double b){EXPECT_EQ(io::Bits(a),io::Bits(b));}
void Point(const io::Value& a,tlfea::contact::Vec3 b) {
    ASSERT_TRUE(a.IsArray());ASSERT_EQ(a.Size(),3u);Bits(a[0u].GetDouble(),b.x);Bits(a[1u].GetDouble(),b.y);Bits(a[2u].GetDouble(),b.z);
}
void SameVerified(const VerifiedWallStudyProvenance& a,const VerifiedWallStudyProvenance& b) {
    EXPECT_EQ(a.kind,b.kind);EXPECT_EQ(a.owner_id,b.owner_id);EXPECT_EQ(a.wall_binding_id,b.wall_binding_id);
    EXPECT_EQ(a.study_sha256,b.study_sha256);EXPECT_EQ(a.source_manifest_sha256,b.source_manifest_sha256);
    EXPECT_EQ(a.derived_mesh_sha256,b.derived_mesh_sha256);EXPECT_EQ(a.derived_vertices,b.derived_vertices);EXPECT_EQ(a.derived_triangles,b.derived_triangles);
}

TEST_F(WallStudyProvenanceCheck, DeterministicAllVariantRoundTripRetainsCompleteSourceAndLineage) {
    for(auto kind:{Kind::Original,Kind::FlipConvexPairs,Kind::UniformFour}) {
        SCOPED_TRACE(static_cast<unsigned>(kind));const auto name=std::to_string(static_cast<unsigned>(kind));
        WallTessellation wall;const auto initialized=wall.Initialize(original,canonical,kind);ASSERT_EQ(initialized.status,WallTessellationStatus::Ok)<<initialized.diagnostic;
        const auto study=StudyBytes(Config(kind),"study-"+name+".json");const auto path=root.path/("wall-"+name+".json"),again=root.path/("again-"+name+".json");
        ASSERT_NO_THROW(WriteWallStudyProvenance(path,original,canonical,wall,study));
        ASSERT_NO_THROW(WriteWallStudyProvenance(again,original,canonical,wall,study));
        const auto bytes=Bytes(path);EXPECT_EQ(bytes,Bytes(again));EXPECT_LT(bytes.size(),kWallStudyProvenanceByteCap);
        const auto verified=ReadWallStudyProvenance(path,original,canonical,study);
        SameVerified(verified,ParseWallStudyProvenance(bytes,original,canonical,study));
        EXPECT_EQ(verified.kind,kind);EXPECT_EQ(verified.wall_binding_id,WallTessellationBindingId(kind));EXPECT_GT(verified.owner_id,std::uint64_t{1}<<53);
        EXPECT_EQ(verified.study_sha256,io::Sha256(study));EXPECT_EQ(verified.source_manifest_sha256,kCanonicalWallManifestSha256);
        EXPECT_EQ(verified.derived_mesh_sha256,wall.metadata()->mesh_sha256);EXPECT_EQ(verified.derived_vertices,wall.view().vertex_count);
        EXPECT_EQ(verified.derived_triangles,wall.view().triangle_count);
        const auto doc=Json(bytes);const auto& m=*wall.metadata();const auto view=wall.view();
        EXPECT_EQ(doc["transform_version"].GetString(),m.transform_version);EXPECT_EQ(doc["kind_name"].GetString(),std::string(WallTessellationName(kind)));
        EXPECT_EQ(doc["original_source_provenance"]["wall_sha256"].GetString(),original.provenance().wall_sha256);
        EXPECT_EQ(doc["original_source_provenance"]["model_archive_reference_sha256"].GetString(),original.provenance().model_archive_reference_sha256);
        EXPECT_EQ(doc["original_vertices"].GetUint64(),62u);EXPECT_EQ(doc["original_triangles"].GetUint64(),100u);
        EXPECT_EQ(doc["original_boundary_edges"].GetUint64(),m.original_boundary_edges);EXPECT_EQ(doc["derived_boundary_edges"].GetUint64(),m.derived_boundary_edges);
        EXPECT_EQ(doc["exposed_boundary_exact"].GetBool(),m.exposed_boundary_exact);Bits(doc["exposed_boundary_displacement_bound_m"].GetDouble(),m.exposed_boundary_displacement_bound_m);
        ASSERT_EQ(doc["flipped_source_quads"].Size(),m.flipped_source_quads.size());
        for(unsigned i=0;i<m.flipped_source_quads.size();++i)EXPECT_EQ(doc["flipped_source_quads"][i].GetUint64(),m.flipped_source_quads[i]);
        ASSERT_EQ(doc["midpoints"].Size(),m.midpoints.size());ASSERT_EQ(doc["faces"].Size(),m.faces.size());
        for(unsigned i=0;i<m.midpoints.size();++i) {
            const auto& a=doc["midpoints"][i];const auto& b=m.midpoints[i];const auto& vertex=view.vertices[b.vertex];
            EXPECT_EQ(a["vertex"].GetUint64(),b.vertex);EXPECT_EQ(a["synthetic_source_node_id"].GetUint64(),vertex.source_node_id);
            EXPECT_EQ(a["synthetic_assembled_node_id"].GetUint64(),vertex.assembled_source_node_id);EXPECT_GE(vertex.source_node_id,kWallMidpointIdBase);
            for(unsigned n=0;n<2;++n) {
                EXPECT_EQ(a["original_edge"][n].GetUint64(),b.original_edge[n]);EXPECT_EQ(a["original_source_nodes"][n].GetUint64(),b.original_source_nodes[n]);
                EXPECT_EQ(a["original_assembled_nodes"][n].GetUint64(),b.original_assembled_nodes[n]);
            }
            Point(a["position_xyz_m"],vertex.position);Point(a["coordinate_error_m"],b.coordinate_error);
            Bits(a["exact_midpoint_y_m"][0u].GetDouble(),b.exact_midpoint_y.lower);Bits(a["exact_midpoint_y_m"][1u].GetDouble(),b.exact_midpoint_y.upper);
            Bits(a["exact_midpoint_z_m"][0u].GetDouble(),b.exact_midpoint_z.lower);Bits(a["exact_midpoint_z_m"][1u].GetDouble(),b.exact_midpoint_z.upper);
            EXPECT_EQ(a["exposed_edge"].GetBool(),b.exposed_edge);
        }
        for(unsigned i=0;i<m.faces.size();++i) {
            const auto& a=doc["faces"][i];const auto& b=m.faces[i];const auto& triangle=view.triangles[i];
            EXPECT_EQ(a["triangle_index"].GetUint64(),i);EXPECT_EQ(a["triangle_id"].GetUint64(),triangle.triangle_id);
            EXPECT_EQ(a["source_quad_id"].GetUint64(),triangle.source_quad_id);EXPECT_EQ(a["assembled_source_quad_id"].GetUint64(),triangle.assembled_source_quad_id);
            EXPECT_EQ(a["original_triangle_count"].GetUint64(),b.original_triangle_count);EXPECT_EQ(a["subtriangle"].GetUint64(),b.subtriangle);
            EXPECT_EQ(a["connectivity_changed"].GetBool(),b.connectivity_changed);
            for(unsigned n=0;n<3;++n)EXPECT_EQ(a["derived_vertex_indices"][n].GetUint64(),triangle.nodes[n]);
            for(unsigned n=0;n<2;++n)EXPECT_EQ(a["original_triangle_ids"][n].GetUint64(),b.original_triangle_ids[n]);
        }
    }
}

TEST_F(WallStudyProvenanceCheck, RegeneratedMetadataRejectsTamperedGeometryLineageSourceAndBindings) {
    WallTessellation wall;const auto r=wall.Initialize(original,canonical,Kind::UniformFour);ASSERT_EQ(r.status,WallTessellationStatus::Ok)<<r.diagnostic;
    const auto study=StudyBytes(Config(Kind::UniformFour),"study.json");const auto path=root.path/"sidecar.json";
    WriteWallStudyProvenance(path,original,canonical,wall,study);const auto original_sidecar=Bytes(path);
    for(unsigned variant=0;variant<20;++variant) {
        SCOPED_TRACE(variant);auto d=Json(original_sidecar);auto& point=d["midpoints"][0u];auto& face=d["faces"][399u];
        if(variant==0)d["kind"].SetUint(1);
        if(variant==1)d["source_manifest_sha256"].SetString(std::string(64,'b').c_str(),d.GetAllocator());
        if(variant==2)d["derived_mesh_sha256"].SetString(std::string(64,'c').c_str(),d.GetAllocator());
        if(variant==3)d["owner_id"].SetUint64(d["owner_id"].GetUint64()+1);
        if(variant==4)d["wall_binding_id"].SetUint64(WallTessellationBindingId(Kind::Original));
        if(variant==5)point["original_source_nodes"][0u].SetUint64(999);
        if(variant==6)point["position_xyz_m"][1u].SetDouble(std::nextafter(point["position_xyz_m"][1u].GetDouble(),std::numeric_limits<double>::infinity()));
        if(variant==7)face["triangle_id"].SetUint64(face["triangle_id"].GetUint64()+1);
        if(variant==8)face["original_triangle_ids"][0u].SetUint64(1);
        if(variant==9)face["derived_vertex_indices"][0u].SetUint64(999);
        if(variant==10)d["exposed_boundary_displacement_bound_m"].SetDouble(0);
        if(variant==11)d["exposed_boundary_exact"].SetBool(true);
        if(variant==12)d["original_source_provenance"]["wall_sha256"].SetString(std::string(64,'d').c_str(),d.GetAllocator());
        if(variant==13)d["original_vertices"].SetUint64(63);
        if(variant==14)d["transform_version"].SetString("unknown-version",d.GetAllocator());
        if(variant==15)d["flipped_source_quads"].PushBack(1001,d.GetAllocator());
        if(variant==16)face.AddMember("source_quad_id",1001,d.GetAllocator());
        if(variant==17)d["schema"].SetString("unknown-schema",d.GetAllocator());
        if(variant==18)d["midpoints"].PopBack();
        if(variant==19)point["exact_midpoint_y_m"][0u].SetDouble(std::nextafter(point["exact_midpoint_y_m"][0u].GetDouble(),-std::numeric_limits<double>::infinity()));
        const auto bad=root.path/("tampered-"+std::to_string(variant)+".json");io::WriteJson(bad,d);
        EXPECT_THROW(ReadWallStudyProvenance(bad,original,canonical,study),std::runtime_error);
    }
    const auto valid=ReadWallStudyProvenance(path,original,canonical,study);EXPECT_EQ(valid.derived_triangles,400u);
}

TEST_F(WallStudyProvenanceCheck, ExactStudyBytesAndAuthenticatedOriginalObjectAreRequired) {
    WallTessellation wall;const auto r=wall.Initialize(original,canonical,Kind::FlipConvexPairs);ASSERT_EQ(r.status,WallTessellationStatus::Ok)<<r.diagnostic;
    const auto study=StudyBytes(Config(Kind::FlipConvexPairs),"study.json");const auto path=root.path/"sidecar.json";
    WriteWallStudyProvenance(path,original,canonical,wall,study);
    // Semantically identical JSON with one extra whitespace byte is a different
    // numerical evidence artifact and must not match the saved source binding.
    EXPECT_THROW(ReadWallStudyProvenance(path,original,canonical,study+" "),std::runtime_error);
    EXPECT_THROW(ReadWallStudyProvenance(path,original,canonical+" ",study),std::runtime_error);
    const auto other=StudyBytes(Config(Kind::Original),"different-binding.json");
    EXPECT_THROW(ReadWallStudyProvenance(path,original,canonical,other),std::runtime_error);
    auto reordered=Json(canonical);reordered["triangles"][0u].Swap(reordered["triangles"][1u]);
    const auto changed_path=root.path/"reordered-wall.json";io::WriteJson(changed_path,reordered);CanonicalWall changed;
    const auto loaded=changed.LoadFile(changed_path.string());ASSERT_EQ(loaded.status,WallStatus::Ok)<<loaded.message;
    EXPECT_THROW(ReadWallStudyProvenance(path,changed,canonical,study),std::runtime_error);
    EXPECT_THROW(WriteWallStudyProvenance(root.path/"wrong-source.json",changed,canonical,wall,study),std::runtime_error);
    EXPECT_FALSE(fs::exists(root.path/"wrong-source.json"));
    // Fixed filenames belong to the loader schema. Syntax-valid hashes are
    // retained there, then authenticated at the shared source-binding boundary.
    // Regeneration may not copy altered hashes merely because a separately
    // supplied byte string has the pinned SHA.
    for(unsigned variant=0;variant<7;++variant) {
        SCOPED_TRACE(variant);auto d=Json(canonical);auto& allocator=d.GetAllocator();
        if(variant==0)d["source"]["wall_file"].SetString("other-wall.k",allocator);
        if(variant==1)d["source"]["combine_file"].SetString("other-combine.k",allocator);
        if(variant==2)d["source"]["wall_sha256"].SetString(std::string(64,'b').c_str(),allocator);
        if(variant==3)d["source"]["combine_sha256"].SetString(std::string(64,'b').c_str(),allocator);
        if(variant==4)d["source"]["model_archive_reference_sha256"].SetString(std::string(64,'b').c_str(),allocator);
        if(variant==5)d["generator"]["sha256"].SetString(std::string(64,'b').c_str(),allocator);
        if(variant==6)d["artifacts"]["wall.obj"]["sha256"].SetString(std::string(64,'b').c_str(),allocator);
        const auto source_path=root.path/("altered-source-"+std::to_string(variant)+".json");io::WriteJson(source_path,d);
        CanonicalWall altered;const auto load=altered.LoadFile(source_path.string());
        if(variant<2) {
            EXPECT_EQ(load.status,WallStatus::InvalidSchema)<<load.message;
            EXPECT_FALSE(altered.loaded());continue;
        }
        ASSERT_EQ(load.status,WallStatus::Ok)<<load.message;
        EXPECT_THROW(CheckCanonicalWallBinding(altered,canonical),std::runtime_error);
        WallTessellation rejected;const auto initialized=rejected.Initialize(altered,canonical,Kind::FlipConvexPairs);
        EXPECT_NE(initialized.status,WallTessellationStatus::Ok);EXPECT_FALSE(rejected.initialized());
        EXPECT_THROW(ReadWallStudyProvenance(path,altered,canonical,study),std::runtime_error);
        const auto destination=root.path/("rejected-source-sidecar-"+std::to_string(variant)+".json");
        EXPECT_THROW(WriteWallStudyProvenance(destination,altered,canonical,wall,study),std::runtime_error);
        EXPECT_FALSE(fs::exists(destination));
    }
}

TEST_F(WallStudyProvenanceCheck, WrongPlaneOutsideFootprintAndUninitializedGeometryRejectBeforeWriting) {
    WallTessellation wall,empty;const auto r=wall.Initialize(original,canonical,Kind::UniformFour);ASSERT_EQ(r.status,WallTessellationStatus::Ok)<<r.diagnostic;
    const auto correct=StudyBytes(Config(Kind::UniformFour),"correct.json");
    EXPECT_THROW(WriteWallStudyProvenance(root.path/"empty.json",original,canonical,empty,correct),std::runtime_error);
    EXPECT_FALSE(fs::exists(root.path/"empty.json"));
    for(unsigned variant=0;variant<3;++variant) {
        auto c=Config(Kind::UniformFour);
        if(variant==0) {
            c.wall_x+=.001;for(auto& p:c.contact_reference)for(auto& point:p.reference_projection)point.x=c.wall_x;
        } else if(variant==1) {
            for(unsigned n=0;n<6;++n)c.reference_position[3*n+1]+=10;
            for(auto& p:c.contact_reference)for(auto& point:p.reference_projection)point.y+=10;
        } else c.wall_binding_id=WallTessellationBindingId(Kind::Original);
        const auto bytes=StudyBytes(c,"bad-study-"+std::to_string(variant)+".json");const auto path=root.path/("rejected-"+std::to_string(variant)+".json");
        EXPECT_THROW(WriteWallStudyProvenance(path,original,canonical,wall,bytes),std::runtime_error);
        EXPECT_FALSE(fs::exists(path));
    }
}

TEST_F(WallStudyProvenanceCheck, CreateOnlyCapsAndRequiredDuplicatesPreserveVerifiedOutput) {
    WallTessellation wall;const auto r=wall.Initialize(original,canonical,Kind::Original);ASSERT_EQ(r.status,WallTessellationStatus::Ok)<<r.diagnostic;
    const auto study=StudyBytes(Config(Kind::Original),"study.json");const auto path=root.path/"sidecar.json";
    WriteWallStudyProvenance(path,original,canonical,wall,study);const auto original_bytes=Bytes(path);
    EXPECT_THROW(WriteWallStudyProvenance(path,original,canonical,wall,study),std::runtime_error);EXPECT_EQ(Bytes(path),original_bytes);
    const auto oversized=root.path/"oversized.json";io::WriteBytes(oversized,std::string(kWallStudyProvenanceByteCap+1,' '));
    EXPECT_THROW(ReadWallStudyProvenance(oversized,original,canonical,study),std::runtime_error);
    EXPECT_THROW(ParseWallStudyProvenance(std::string(kWallStudyProvenanceByteCap+1,' '),original,canonical,study),std::runtime_error);
    EXPECT_THROW(WriteWallStudyProvenance(root.path/"oversized-study.json",original,canonical,wall,std::string(kGuidedStudyByteCap+1,' ')),std::runtime_error);
    EXPECT_FALSE(fs::exists(root.path/"oversized-study.json"));
    auto extra=Json(original_bytes);extra.AddMember("observer_note",io::Value("Ignored extension",extra.GetAllocator()),extra.GetAllocator());
    io::WriteJson(root.path/"extended.json",extra);const auto expected=ReadWallStudyProvenance(path,original,canonical,study);
    SameVerified(ReadWallStudyProvenance(root.path/"extended.json",original,canonical,study),expected);
    extra.AddMember("kind",0,extra.GetAllocator());io::WriteJson(root.path/"duplicate.json",extra);
    auto output=expected;
    EXPECT_THROW(output=ReadWallStudyProvenance(root.path/"duplicate.json",original,canonical,study),std::runtime_error);
    SameVerified(output,expected);
}
} // namespace
int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);
    if(argc!=2){std::cerr<<"Required authenticated canonical wall manifest argument\n";return 2;}
    asset=argv[1];return RUN_ALL_TESTS();
}
