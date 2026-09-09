#include "chrono/core/ChMatrix.h"
#include "GuidedPlateStudyIO.h"
#include "guided_plate_study_fixture.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

namespace {
using namespace crash::case_data;
namespace fixture=crash::case_data::study_test;
namespace io=crash::output;
namespace fs=std::filesystem;
class TempRoot {
  public:
    TempRoot() {
        const auto pattern=(fs::temp_directory_path()/"guided-study-io-XXXXXX").string();
        std::vector<char> buffer(pattern.begin(),pattern.end());buffer.push_back(0);
        const char* made=::mkdtemp(buffer.data());if(!made)throw std::runtime_error("Could not create isolated study IO test directory");
        path=made;
    }
    ~TempRoot(){std::error_code ignored;fs::remove_all(path,ignored);}
    fs::path path;
};
void Bits(double actual,double expected){EXPECT_EQ(io::Bits(actual),io::Bits(expected));}
template<std::size_t N> void Array(const std::array<double,N>& actual,const std::array<double,N>& expected) {
    for(unsigned i=0;i<N;++i){SCOPED_TRACE(i);Bits(actual[i],expected[i]);}
}
void Interval(GuidedStudyInterval actual,GuidedStudyInterval expected){Bits(actual.lower,expected.lower);Bits(actual.upper,expected.upper);}
void Certificate(const GuidedStudyCertificate& actual,const GuidedStudyCertificate& expected) {
    Bits(actual.value,expected.value);Bits(actual.lower,expected.lower);Bits(actual.upper,expected.upper);Bits(actual.error,expected.error);
}
void Event(const GuidedStudyEvent& actual,const GuidedStudyEvent& expected) {
    EXPECT_EQ(actual.observed,expected.observed);EXPECT_EQ(actual.lower_epoch,expected.lower_epoch);EXPECT_EQ(actual.upper_epoch,expected.upper_epoch);
    Bits(actual.lower_time,expected.lower_time);Bits(actual.upper_time,expected.upper_time);
}
void SameStudy(const GuidedStudyData& a,const GuidedStudyData& b) {
    EXPECT_EQ(a.complete,b.complete);const auto& c=a.config;const auto& d=b.config;
    EXPECT_EQ(c.owner_id,d.owner_id);EXPECT_EQ(c.qualification_id,d.qualification_id);EXPECT_EQ(c.wall_binding_id,d.wall_binding_id);
    EXPECT_EQ(c.base_steps,d.base_steps);EXPECT_EQ(c.refinement,d.refinement);EXPECT_EQ(c.experiment_sha256,d.experiment_sha256);
    Bits(c.fixed_dt,d.fixed_dt);Bits(c.horizon,d.horizon);Bits(c.initial_energy,d.initial_energy);Bits(c.wall_x,d.wall_x);
    Array(c.reference_position,d.reference_position);Array(c.reference_rotation,d.reference_rotation);Interval(c.total_reference_area,d.total_reference_area);
    for(unsigned p=0;p<2;++p) {
        SCOPED_TRACE(p);const auto& x=c.contact_reference[p];const auto& y=d.contact_reference[p];
        EXPECT_EQ(x.covered,y.covered);EXPECT_EQ(x.parent.feature_id,y.parent.feature_id);
        EXPECT_EQ(x.parent.parent_element_id,y.parent.parent_element_id);EXPECT_EQ(x.parent.parent_face_id,y.parent.parent_face_id);
        Bits(x.parent.half_thickness,y.parent.half_thickness);Bits(x.projected_area,y.projected_area);Interval(x.area_enclosure,y.area_enclosure);
        for(unsigned n=0;n<4;++n) {
            EXPECT_EQ(x.parent.nodes[n],y.parent.nodes[n]);Bits(x.reference_projection[n].x,y.reference_projection[n].x);
            Bits(x.reference_projection[n].y,y.reference_projection[n].y);Bits(x.reference_projection[n].z,y.reference_projection[n].z);
        }
    }
    Array(a.initial_position,b.initial_position);Array(a.initial_rotation,b.initial_rotation);
    const auto& s=a.summary;const auto& t=b.summary;
    EXPECT_EQ(s.accepted_epoch,t.accepted_epoch);EXPECT_EQ(s.last_attempt,t.last_attempt);Bits(s.accepted_time,t.accepted_time);
    EXPECT_EQ(s.sample_count,t.sample_count);Certificate(s.sampled_peak_normal_force,t.sampled_peak_normal_force);Certificate(s.normal_wall_impulse,t.normal_wall_impulse);
    Bits(s.maximum_penetration,t.maximum_penetration);Bits(s.maximum_value_energy_relative_error,t.maximum_value_energy_relative_error);
    Bits(s.maximum_certified_energy_relative_error,t.maximum_certified_energy_relative_error);Event(s.activation,t.activation);Event(s.pressure_release,t.pressure_release);
    EXPECT_EQ(s.certified_partial_area,t.certified_partial_area);EXPECT_EQ(s.certified_unequal_nodal_force,t.certified_unequal_nodal_force);
    EXPECT_EQ(s.separated_rebounding,t.separated_rebounding);EXPECT_EQ(s.separation_sample_epoch,t.separation_sample_epoch);Bits(s.separation_sample_time,t.separation_sample_time);
    Bits(s.minimum_curvature,t.minimum_curvature);Bits(s.maximum_curvature,t.maximum_curvature);Bits(s.maximum_abs_curvature,t.maximum_abs_curvature);
    Array(s.minimum_rotation,t.minimum_rotation);Array(s.maximum_rotation,t.maximum_rotation);
    for(unsigned i=0;i<kGuidedStudySamples;++i) {
        SCOPED_TRACE(i);const auto& x=a.samples[i];const auto& y=b.samples[i];EXPECT_EQ(x.epoch,y.epoch);Bits(x.time,y.time);
        Array(x.normal_displacement,y.normal_displacement);Array(x.normal_velocity,y.normal_velocity);Array(x.world_z_rotation,y.world_z_rotation);
        Bits(x.curvature_proxy,y.curvature_proxy);Interval(x.minimum_signed_gap,y.minimum_signed_gap);Interval(x.tip_normal_velocity,y.tip_normal_velocity);
        Array(x.energy,y.energy);Bits(x.shell_bending_energy,y.shell_bending_energy);Bits(x.maximum_penetration,y.maximum_penetration);
        Certificate(x.normal_wall_force,y.normal_wall_force);Certificate(x.contact_potential,y.contact_potential);
    }
}
io::Document Parse(const std::string& bytes) {
    io::Document result;result.Parse<rapidjson::kParseFullPrecisionFlag>(bytes.data(),bytes.size());
    io::Require(!result.HasParseError()&&result.IsObject(),"Malformed JSON in test fixture");return result;
}
std::string Read(const fs::path& path){return io::ReadBounded(path,kGuidedStudyByteCap);}
GuidedStudyData PreciseFixture() {
    auto config=fixture::Config(1,std::numeric_limits<std::uint64_t>::max()-1);
    config.qualification_id=std::numeric_limits<std::uint64_t>::max()-3;
    config.wall_binding_id=std::numeric_limits<std::uint64_t>::max()-5;
    const double y[]{.12345678901234568,.02345678901234568,.02345678901234568,.12345678901234568,.22345678901234568,.22345678901234568};
    for(unsigned n=0;n<6;++n) {
        config.reference_position[3*n+1]=y[n];config.reference_position[3*n+2]=-.9182736455463728+(n==0||n==1||n==4?.05:-.05);
        for(unsigned j=0;j<4;++j)config.reference_rotation[4*n+j]=.5;
    }
    for(unsigned p=0;p<2;++p) {
        auto& r=config.contact_reference[p];r.parent.feature_id=std::numeric_limits<std::uint64_t>::max()-11-p;
        r.parent.parent_element_id=std::numeric_limits<std::uint64_t>::max()-21-p;
        r.parent.parent_face_id=std::numeric_limits<std::uint32_t>::max()-p;r.parent.half_thickness=-0.;
        r.projected_area=std::nextafter(.01,std::numeric_limits<double>::infinity());
        for(unsigned n=0;n<4;++n) {
            const auto node=r.parent.nodes[n];r.reference_projection[n]={config.wall_x,config.reference_position[3*node+1],config.reference_position[3*node+2]};
        }
    }
    auto result=fixture::Run(config);result.summary.last_attempt=std::numeric_limits<std::uint64_t>::max()-31;
    return result;
}

TEST(GuidedPlateStudyIO, FullPrecisionRoundTripRetainsAllParentAndTwoHundredOneSampleFields) {
    const auto source=PreciseFixture();std::string diagnostic;ASSERT_TRUE(ValidateGuidedPlateStudy(source,diagnostic))<<diagnostic;
    TempRoot root;const auto first=root.path/"first.json",second=root.path/"second.json";
    ASSERT_NO_THROW(WriteGuidedPlateStudy(first,source));
    GuidedStudyData loaded;ASSERT_NO_THROW(loaded=ReadGuidedPlateStudy(first));
    SameStudy(loaded,source);ASSERT_EQ(loaded.summary.sample_count,201u);
    SameStudy(ParseGuidedPlateStudy(Read(first)),source);
    EXPECT_GT(loaded.config.owner_id,std::uint64_t{1}<<53);EXPECT_GT(loaded.summary.last_attempt,std::uint64_t{1}<<53);
    ASSERT_NO_THROW(WriteGuidedPlateStudy(second,loaded));
    EXPECT_EQ(Read(first),Read(second));EXPECT_LT(fs::file_size(first),kGuidedStudyByteCap);
}

TEST(GuidedPlateStudyIO, CertifiedRoundedValueOutsideTruthIntervalIsPreserved) {
    auto data=fixture::Run(1,7);const double truth=1e-9;
    data.samples[0].normal_wall_force=fixture::Cert(std::nextafter(truth,std::numeric_limits<double>::infinity()),truth,truth);
    ASSERT_GT(data.samples[0].normal_wall_force.value,data.samples[0].normal_wall_force.upper);
    std::string diagnostic;ASSERT_TRUE(ValidateGuidedPlateStudy(data,diagnostic))<<diagnostic;
    TempRoot root;const auto path=root.path/"rounded.json";ASSERT_NO_THROW(WriteGuidedPlateStudy(path,data));
    GuidedStudyData read;ASSERT_NO_THROW(read=ReadGuidedPlateStudy(path));
    SameStudy(read,data);EXPECT_GT(read.samples[0].normal_wall_force.value,read.samples[0].normal_wall_force.upper);
}

TEST(GuidedPlateStudyIO, CreateOnlyMissingPathsAndByteCapsPreserveExistingOutput) {
    const auto data=fixture::Run(1,7);TempRoot root;const auto existing=root.path/"existing.json";
    io::WriteBytes(existing,"Retain existing report\n");
    EXPECT_THROW(WriteGuidedPlateStudy(existing,data),std::runtime_error);
    EXPECT_EQ(Read(existing),"Retain existing report\n");
    EXPECT_THROW(WriteGuidedPlateStudy(root.path,data),std::runtime_error);
    EXPECT_THROW(WriteGuidedPlateStudy(root.path/"missing-parent"/"report.json",data),std::runtime_error);
    EXPECT_FALSE(fs::exists(root.path/"missing-parent"));
    EXPECT_THROW(ReadGuidedPlateStudy(root.path/"absent.json"),std::runtime_error);
    const auto oversized=root.path/"oversized.json";io::WriteBytes(oversized,std::string(kGuidedStudyByteCap+1,' '));
    EXPECT_THROW(ReadGuidedPlateStudy(oversized),std::runtime_error);
    EXPECT_THROW(ParseGuidedPlateStudy(std::string(kGuidedStudyByteCap+1,' ')),std::runtime_error);
    GuidedStudyComparison comparison;comparison.diagnostic=std::string(kGuidedStudyByteCap,'x');
    EXPECT_THROW(WriteGuidedPlateComparison(root.path/"oversized-comparison.json",comparison,std::string(64,'a'),std::string(64,'b')),std::runtime_error);
    EXPECT_FALSE(fs::exists(root.path/"oversized-comparison.json"));
}

TEST(GuidedPlateStudyIO, MalformedSchemasShapesKnownDuplicatesAndCertificatesRejectReads) {
    const auto data=fixture::Run(1,7);TempRoot root;const auto source=root.path/"valid.json";WriteGuidedPlateStudy(source,data);
    const auto bytes=Read(source);
    for(unsigned variant=0;variant<12;++variant) {
        SCOPED_TRACE(variant);auto doc=Parse(bytes);auto& c=doc["configuration"];auto& p=c["contact_reference"][1u];
        if(variant==0)doc["schema"].SetString("unrecognized.v1",doc.GetAllocator());
        if(variant==1)doc["samples"].PopBack();
        if(variant==2)p["reference_projection"].PopBack();
        if(variant==3)c["owner_id"].SetDouble(7.5);
        if(variant==4)doc.AddMember("schema",io::Value("robo_dyna.guided_plate_study.v1",doc.GetAllocator()),doc.GetAllocator());
        if(variant==5)c.AddMember("owner_id",std::uint64_t{7},doc.GetAllocator());
        if(variant==6)p.AddMember("projected_area",.01,doc.GetAllocator());
        if(variant==7)p["half_thickness"].SetDouble(.001);
        if(variant==8)doc["summary"]["sampled_peak_normal_force"][3u].SetDouble(0);
        if(variant==9)doc["samples"][51u]["contact_potential"][3u].SetDouble(0);
        if(variant==10)doc["complete"].SetBool(false);
        if(variant==11)doc["samples"][200u]["time"].SetString("NaN",doc.GetAllocator());
        const auto path=root.path/("malformed-"+std::to_string(variant)+".json");io::WriteJson(path,doc);
        EXPECT_THROW(ReadGuidedPlateStudy(path),std::runtime_error);
    }
    auto nonfinite=bytes;const auto key=nonfinite.find("\"fixed_dt\":");ASSERT_NE(key,std::string::npos);
    const auto begin=nonfinite.find(':',key)+1,end=nonfinite.find(',',begin);ASSERT_NE(end,std::string::npos);
    nonfinite.replace(begin,end-begin," NaN");io::WriteBytes(root.path/"nan-token.json",nonfinite);
    EXPECT_THROW(ReadGuidedPlateStudy(root.path/"nan-token.json"),std::runtime_error);
    SameStudy(ReadGuidedPlateStudy(source),data);
}

TEST(GuidedPlateStudyIO, MalformedInMemoryRecordsRejectBeforeCreatingFiles) {
    const auto valid=fixture::Run(1,7);TempRoot root;
    for(unsigned variant=0;variant<5;++variant) {
        SCOPED_TRACE(variant);auto bad=valid;
        if(variant==0)bad.complete=false;
        if(variant==1)bad.samples[199].curvature_proxy=std::numeric_limits<double>::quiet_NaN();
        if(variant==2)bad.summary.sampled_peak_normal_force.error=0;
        if(variant==3)bad.config.contact_reference[1].reference_projection[3].y=1;
        if(variant==4)bad.config.contact_reference[1].parent.half_thickness=.001;
        const auto path=root.path/("rejected-"+std::to_string(variant)+".json");
        EXPECT_THROW(WriteGuidedPlateStudy(path,bad),std::runtime_error);
        EXPECT_FALSE(fs::exists(path));
    }
}

TEST(GuidedPlateStudyIO, CompletedNonReboundOutcomeAndValidFailedComparisonAreWrittenTruthfully) {
    auto coarse=fixture::Run(1,7);const auto fine=fixture::Run(2,77);
    coarse.summary.separated_rebounding=false;coarse.summary.separation_sample_epoch=0;coarse.summary.separation_sample_time=0;
    std::string diagnostic;ASSERT_TRUE(ValidateGuidedPlateStudy(coarse,diagnostic))<<diagnostic;
    TempRoot root;const auto a=root.path/"coarse.json",b=root.path/"fine.json",result=root.path/"comparison.json";
    ASSERT_NO_THROW(WriteGuidedPlateStudy(a,coarse));
    ASSERT_NO_THROW(WriteGuidedPlateStudy(b,fine));
    const auto loaded=ReadGuidedPlateStudy(a);SameStudy(loaded,coarse);EXPECT_TRUE(loaded.complete);EXPECT_FALSE(loaded.summary.separated_rebounding);
    GuidedStudyComparison comparison;ASSERT_TRUE(CompareGuidedPlateStudies(loaded,ReadGuidedPlateStudy(b),comparison,diagnostic))<<diagnostic;
    ASSERT_FALSE(comparison.passed);EXPECT_FALSE(comparison.deforming_contact_evidence);EXPECT_FALSE(comparison.diagnostic.empty());
    const auto coarse_hash=io::Sha256(Read(a)),fine_hash=io::Sha256(Read(b));
    ASSERT_NO_THROW(WriteGuidedPlateComparison(result,comparison,coarse_hash,fine_hash));
    const auto json=Parse(Read(result));EXPECT_STREQ(json["schema"].GetString(),"robo_dyna.guided_plate_comparison.v1");
    EXPECT_FALSE(json["passed"].GetBool());EXPECT_FALSE(json["deforming_contact_evidence"].GetBool());
    EXPECT_EQ(json["energy_envelopes"].GetBool(),comparison.energy_envelopes);EXPECT_EQ(json["events_complete"].GetBool(),comparison.events_complete);
    EXPECT_EQ(std::string(json["coarse_sha256"].GetString()),coarse_hash);EXPECT_EQ(std::string(json["fine_sha256"].GetString()),fine_hash);
    EXPECT_EQ(std::string(json["diagnostic"].GetString()),comparison.diagnostic);
    Bits(json["displacement_ratio"].GetDouble(),comparison.displacement_ratio);Bits(json["velocity_ratio"].GetDouble(),comparison.velocity_ratio);
    Bits(json["rotation_ratio"].GetDouble(),comparison.rotation_ratio);Bits(json["force_ratio"].GetDouble(),comparison.force_ratio);
    Bits(json["impulse_ratio"].GetDouble(),comparison.impulse_ratio);Bits(json["energy_ratio"].GetDouble(),comparison.energy_ratio);
    Bits(json["event_ratio"].GetDouble(),comparison.event_ratio);Bits(json["penetration_ratio"].GetDouble(),comparison.penetration_ratio);
    const auto before=Read(result);
    EXPECT_THROW(WriteGuidedPlateComparison(result,comparison,coarse_hash,fine_hash),std::runtime_error);
    EXPECT_EQ(Read(result),before);
    EXPECT_THROW(WriteGuidedPlateComparison(root.path/"bad-hash.json",comparison,"not-a-hash",fine_hash),std::runtime_error);
    EXPECT_FALSE(fs::exists(root.path/"bad-hash.json"));
    comparison.force_ratio=std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(WriteGuidedPlateComparison(root.path/"bad-ratio.json",comparison,coarse_hash,fine_hash),std::runtime_error);
    EXPECT_FALSE(fs::exists(root.path/"bad-ratio.json"));
}
} // namespace
