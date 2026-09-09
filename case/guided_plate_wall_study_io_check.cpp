#include "chrono/core/ChMatrix.h"
#include "GuidedPlateCli.h"
#include "GuidedPlateContactProtocol.h"
#include "GuidedPlateStudyIO.h"
#include "WallStudyProvenance.h"
#include "CanonicalWallArtifacts.h"
#include "guided_plate_study_fixture.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <vector>

namespace {
using namespace crash::case_data;
using Kind=WallTessellationKind;
using Backend=tlfea::contact::Q4PlanarIntegrationBackend;
namespace fixture=crash::case_data::study_test;
namespace io=crash::output;
namespace fs=std::filesystem;
std::string asset;
struct Temp {
    fs::path path;
    Temp() {
        const auto pattern=(fs::temp_directory_path()/"guided-wall-comparison-XXXXXX").string();
        std::vector<char> data(pattern.begin(),pattern.end());data.push_back(0);
        const auto* made=::mkdtemp(data.data());if(!made)throw std::runtime_error("Cannot create wall comparison test directory");path=made;
    }
    ~Temp(){std::error_code error;fs::remove_all(path,error);}
};
std::string Bytes(const fs::path& path){return io::ReadBounded(path,kGuidedStudyByteCap);}
io::Document Json(const std::string& bytes) {
    io::Document d;d.Parse<rapidjson::kParseFullPrecisionFlag>(bytes.data(),bytes.size());
    io::Require(!d.HasParseError()&&d.IsObject(),"Malformed test JSON");return d;
}
class GuidedPlateWallStudyIOCheck:public ::testing::Test {
  protected:
    Temp root;CanonicalWall original;std::string canonical;
    void SetUp() override {
        ASSERT_NO_THROW(canonical=ReadPinnedWallManifest(asset));
        std::istringstream stream(canonical);const auto loaded=original.Load(stream);ASSERT_EQ(loaded.status,WallStatus::Ok)<<loaded.message;
    }
    GuidedStudyConfig Config(Kind kind,unsigned refinement=1,Backend backend=Backend::ScalarDyadicSquares) const {
        // Synthetic completed observer history only. Actual source geometry is
        // used for provenance checks; these records are never replay artifacts.
        auto c=fixture::Config(refinement,1);c.qualification_id=kGuidedPlateQualification;
        c.wall_binding_id=WallTessellationBindingId(kind);c.integration_backend=backend;
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
    void Save(const GuidedStudyData& data,Kind kind,const fs::path& study,const fs::path& sidecar) {
        WriteGuidedPlateStudy(study,data);WallTessellation transformed;
        const auto report=transformed.Initialize(original,canonical,kind);
        io::Require(report.status==WallTessellationStatus::Ok,report.diagnostic.c_str());
        WriteWallStudyProvenance(sidecar,original,canonical,transformed,Bytes(study));
    }
    GuidedPlateWallStudyPaths Paths(const char* prefix="run") const {
        const std::string p=prefix;return {asset,root.path/(p+"-derived.json"),root.path/(p+"-derived-proof.json"),
            root.path/(p+"-canonical.json"),root.path/(p+"-canonical-proof.json"),root.path/(p+"-comparison.json")};
    }
    void Pair(const GuidedPlateWallStudyPaths& p,Kind kind=Kind::FlipConvexPairs) {
        Save(fixture::Run(Config(kind)),kind,p.derived_study,p.derived_provenance);
        Save(fixture::Run(Config(Kind::Original)),Kind::Original,p.canonical_study,p.canonical_provenance);
    }
};

TEST_F(GuidedPlateWallStudyIOCheck, BothTransformsAuthenticateExactBytesAndUseExistingSameStepMath) {
    for(auto kind:{Kind::FlipConvexPairs,Kind::UniformFour}) {
        const auto p=Paths(kind==Kind::FlipConvexPairs?"flip":"subdivide");Pair(p,kind);
        GuidedStudyComparison expected;std::string error;
        ASSERT_TRUE(CompareGuidedPlateWallStudies(ReadGuidedPlateStudy(p.derived_study),ReadGuidedPlateStudy(p.canonical_study),expected,error))<<error;
        const auto result=CompareAndWriteGuidedPlateWallStudies(p);EXPECT_TRUE(result.passed)<<result.diagnostic;
        EXPECT_EQ(result.force_ratio,expected.force_ratio);EXPECT_EQ(result.impulse_ratio,expected.impulse_ratio);
        const auto doc=Json(Bytes(p.report));EXPECT_STREQ(doc["schema"].GetString(),"robo_dyna.guided_plate_wall_comparison.v1");
        EXPECT_STREQ(doc[io::contact_metadata::BackendField].GetString(),io::contact_metadata::Scalar);
        EXPECT_EQ(doc["derived"]["owner_id"].GetUint64(),1u);EXPECT_EQ(doc["canonical"]["owner_id"].GetUint64(),1u);
        EXPECT_NE(doc["derived"]["wall_binding_id"].GetUint64(),doc["canonical"]["wall_binding_id"].GetUint64());
        EXPECT_EQ(doc["derived"]["study_sha256"].GetString(),io::Sha256(Bytes(p.derived_study)));
        EXPECT_EQ(doc["canonical"]["study_sha256"].GetString(),io::Sha256(Bytes(p.canonical_study)));
        EXPECT_EQ(doc["derived"]["provenance_sha256"].GetString(),io::Sha256(Bytes(p.derived_provenance)));
        EXPECT_EQ(doc["canonical"]["provenance_sha256"].GetString(),io::Sha256(Bytes(p.canonical_provenance)));
        EXPECT_STREQ(doc["source_manifest_sha256"].GetString(),kCanonicalWallManifestSha256);
        EXPECT_LT(Bytes(p.report).size(),kGuidedStudyByteCap);
    }
}
TEST_F(GuidedPlateWallStudyIOCheck, ValidNumericalFailureWritesAllRatiosAndReturnsExitTwo) {
    const auto p=Paths();auto derived=fixture::Run(Config(Kind::FlipConvexPairs));
    derived.samples[0].normal_displacement[0]*=1.2;std::string error;
    ASSERT_TRUE(ValidateGuidedPlateStudy(derived,error))<<error;
    Save(derived,Kind::FlipConvexPairs,p.derived_study,p.derived_provenance);
    Save(fixture::Run(Config(Kind::Original)),Kind::Original,p.canonical_study,p.canonical_provenance);
    GuidedPlateCommand command;command.kind=GuidedPlateCommandKind::CompareWall;command.wall_comparison=p;
    EXPECT_EQ(ExecuteGuidedPlateCommand(command),2);const auto doc=Json(Bytes(p.report));
    EXPECT_FALSE(doc["passed"].GetBool());EXPECT_GT(doc["displacement_ratio"].GetDouble(),1);
    for(const auto* key:{"velocity_ratio","rotation_ratio","force_ratio","impulse_ratio","energy_ratio","event_ratio","penetration_ratio"})
        EXPECT_TRUE(doc.HasMember(key)&&doc[key].IsDouble())<<key;
    EXPECT_FALSE(fs::exists(root.path/"manifest.json")); // No fake accepted replay bundle.
}
TEST_F(GuidedPlateWallStudyIOCheck, ChangedBytesRolesSourceAndSameStepIdentityRejectBeforePublication) {
    const auto valid=Paths();Pair(valid);
    for(unsigned variant=0;variant<7;++variant) {
        SCOPED_TRACE(variant);auto p=valid;p.report=root.path/("reject-"+std::to_string(variant)+".json");
        if(variant==0) {p.derived_study=root.path/"changed-study.json";io::WriteBytes(p.derived_study,Bytes(valid.derived_study)+"\n");}
        if(variant==1) {
            p.derived_provenance=root.path/"changed-proof.json";auto doc=Json(Bytes(valid.derived_provenance));
            doc["study_sha256"].SetString(std::string(64,'a').c_str(),doc.GetAllocator());io::WriteJson(p.derived_provenance,doc);
        }
        if(variant==2) {std::swap(p.derived_study,p.canonical_study);std::swap(p.derived_provenance,p.canonical_provenance);}
        if(variant==3) {p.derived_study=p.canonical_study;p.derived_provenance=p.canonical_provenance;}
        if(variant==4) {
            p.canonical_study=root.path/"different-h.json";p.canonical_provenance=root.path/"different-h-proof.json";
            Save(fixture::Run(Config(Kind::Original,2)),Kind::Original,p.canonical_study,p.canonical_provenance);
        }
        if(variant==5) {p.wall=root.path/"changed-source.json";io::WriteBytes(p.wall,canonical+"\n");}
        if(variant==6) {
            p.canonical_study=root.path/"different-backend.json";p.canonical_provenance=root.path/"different-backend-proof.json";
            Save(fixture::Run(Config(Kind::Original,1,Backend::RectangularDyadic)),Kind::Original,p.canonical_study,p.canonical_provenance);
        }
        EXPECT_THROW(CompareAndWriteGuidedPlateWallStudies(p),std::runtime_error);EXPECT_FALSE(fs::exists(p.report));
    }
    EXPECT_NO_THROW(CompareAndWriteGuidedPlateWallStudies(valid)); // Rejected attempts do not poison valid inputs.
}
TEST_F(GuidedPlateWallStudyIOCheck, ExistingOutputsAndOversizedInputsRemainUnpublished) {
    auto p=Paths();Pair(p);io::WriteBytes(p.report,"retain me");
    EXPECT_THROW(CompareAndWriteGuidedPlateWallStudies(p),std::runtime_error);EXPECT_EQ(Bytes(p.report),"retain me");
    p.report=root.path/"new-report.json";p.derived_provenance=root.path/"huge.json";
    io::WriteBytes(p.derived_provenance,std::string(kWallStudyProvenanceByteCap+1,' '));
    EXPECT_THROW(CompareAndWriteGuidedPlateWallStudies(p),std::runtime_error);EXPECT_FALSE(fs::exists(p.report));
    p=Paths();p.report=root.path/"dangling.json";fs::create_symlink(root.path/"absent",p.report);
    EXPECT_THROW(CompareAndWriteGuidedPlateWallStudies(p),std::runtime_error);EXPECT_FALSE(fs::exists(root.path/"absent"));
}
TEST_F(GuidedPlateWallStudyIOCheck, BackendSerializationIsExplicitLegacyScalarAndStrict) {
    for(auto backend:{Backend::ScalarDyadicSquares,Backend::RectangularDyadic}) {
        const auto path=root.path/(backend==Backend::ScalarDyadicSquares?"scalar.json":"rectangular.json");
        WriteGuidedPlateStudy(path,fixture::Run(Config(Kind::Original,1,backend)));
        const auto saved=ReadGuidedPlateStudy(path);EXPECT_EQ(saved.config.integration_backend,backend);
        auto doc=Json(Bytes(path));EXPECT_STREQ(doc["configuration"][io::contact_metadata::BackendField].GetString(),GuidedContactBackendName(backend));
    }
    const auto scalar=root.path/"scalar.json";auto legacy=Json(Bytes(scalar));legacy["configuration"].RemoveMember(io::contact_metadata::BackendField);
    const auto legacy_path=root.path/"legacy.json";io::WriteJson(legacy_path,legacy);
    EXPECT_EQ(ReadGuidedPlateStudy(legacy_path).config.integration_backend,Backend::ScalarDyadicSquares);
    for(unsigned variant=0;variant<5;++variant) {
        auto doc=Json(Bytes(scalar));auto& config=doc["configuration"];auto& b=config[io::contact_metadata::BackendField];
        if(variant==0)b.SetString("rectangular-dyadic",doc.GetAllocator());
        if(variant==1)b.SetString("unknown",doc.GetAllocator());
        if(variant==2)b.SetUint(1);
        if(variant==3)config.AddMember(io::Value(io::contact_metadata::BackendField,doc.GetAllocator()),
            io::Value(io::contact_metadata::Scalar,doc.GetAllocator()),doc.GetAllocator());
        if(variant==4) {
            constexpr char embedded[]="scalar-dyadic-squares\0garbage";
            b.SetString(embedded,sizeof(embedded)-1,doc.GetAllocator());
        }
        const auto path=root.path/("bad-backend-"+std::to_string(variant)+".json");io::WriteJson(path,doc);
        EXPECT_THROW(ReadGuidedPlateStudy(path),std::runtime_error);
    }
    Backend preserved=Backend::RectangularDyadic;EXPECT_FALSE(ParseGuidedContactBackend("unknown",preserved));EXPECT_EQ(preserved,Backend::RectangularDyadic);
    auto invalid=fixture::Run(Config(Kind::Original));invalid.config.integration_backend=static_cast<Backend>(99);
    EXPECT_THROW(WriteGuidedPlateStudy(root.path/"invalid-enum.json",invalid),std::runtime_error);EXPECT_FALSE(fs::exists(root.path/"invalid-enum.json"));
}
TEST_F(GuidedPlateWallStudyIOCheck, LegacyRefinementCommandRetainsSchemaHashesAndSharedFields) {
    GuidedPlateCommand command;command.kind=GuidedPlateCommandKind::CompareRefinement;
    command.coarse_study=root.path/"coarse.json";command.fine_study=root.path/"fine.json";command.refinement_report=root.path/"refinement.json";
    WriteGuidedPlateStudy(command.coarse_study,fixture::Run(Config(Kind::Original,1)));
    WriteGuidedPlateStudy(command.fine_study,fixture::Run(Config(Kind::Original,2)));
    EXPECT_EQ(ExecuteGuidedPlateCommand(command),0);const auto doc=Json(Bytes(command.refinement_report));
    EXPECT_STREQ(doc["schema"].GetString(),"robo_dyna.guided_plate_comparison.v1");
    EXPECT_EQ(doc["coarse_sha256"].GetString(),io::Sha256(Bytes(command.coarse_study)));
    EXPECT_EQ(doc["fine_sha256"].GetString(),io::Sha256(Bytes(command.fine_study)));
    for(const auto* key:{"passed","diagnostic","displacement_ratio","velocity_ratio","rotation_ratio","force_ratio","impulse_ratio",
        "energy_ratio","event_ratio","penetration_ratio","energy_envelopes","deforming_contact_evidence","events_complete"})EXPECT_TRUE(doc.HasMember(key))<<key;
    const auto before=Bytes(command.refinement_report);EXPECT_THROW(ExecuteGuidedPlateCommand(command),std::runtime_error);
    EXPECT_EQ(Bytes(command.refinement_report),before);
}
} // namespace
int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);
    if(argc!=2){std::cerr<<"Expected authenticated canonical wall manifest path\n";return 2;}
    asset=argv[1];return RUN_ALL_TESTS();
}
