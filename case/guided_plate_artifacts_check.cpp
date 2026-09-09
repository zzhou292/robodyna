#include "chrono/core/ChMatrix.h"
#include "GuidedPlateArtifacts.h"
#include "GuidedPlateFields.h"
#include "GuidedPlateIntervals.h"
#include "CanonicalWallArtifacts.h"
#include "output/ArtifactIO.h"
#include "output/ArtifactInventory.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <vector>

namespace {
using namespace crash::case_data;
namespace out=crash::output;
namespace fs=std::filesystem;
namespace ref=crash::reference;
namespace shell=tl::fea::reissner;
namespace contact=tlfea::contact;
using Frame=GuidedPlateFrame;
using Code=GuidedPlateStatus;
std::string asset;

class TempRoot {
  public:
    TempRoot() {
        const auto pattern=(fs::temp_directory_path()/"guided-plate-artifacts-XXXXXX").string();
        std::vector<char> buffer(pattern.begin(),pattern.end()); buffer.push_back(0);
        const char* made=::mkdtemp(buffer.data());
        if(!made)throw std::runtime_error("Could not create isolated guided artifact test directory");
        path=made;
    }
    ~TempRoot(){std::error_code error;fs::remove_all(path,error);}
    fs::path path;
};
class GuidedPlateArtifactsCheck:public ::testing::Test {
  protected:
    CanonicalWall wall;
    std::string bytes;
    void SetUp()override {
        int count=0;ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);ASSERT_GT(count,0);
        ASSERT_NO_THROW(bytes=ReadPinnedWallManifest(asset));
        std::istringstream input(bytes);const auto loaded=wall.Load(input);
        ASSERT_EQ(loaded.status,WallStatus::Ok)<<loaded.message;
    }
};
out::Document Json(const fs::path& path) {
    const auto bytes=out::ReadBounded(path,1024*1024);
    out::Document doc;doc.Parse<rapidjson::kParseFullPrecisionFlag>(bytes.c_str());
    out::Require(!doc.HasParseError()&&doc.IsObject(),"Invalid test JSON artifact");return doc;
}
unsigned FrameEvery(const GuidedPlateCase& run) {
    // Short prefix tests do not claim a complete experiment. Reserve only the
    // initial/final frame budget while explicitly exercising one accepted row.
    return static_cast<unsigned>(run.metrics()->required_steps);
}
void ExpectBits(double actual,double expected){EXPECT_EQ(out::Bits(actual),out::Bits(expected));}
template<std::size_t N>
void ExpectArray(const out::Value& value,const std::array<double,N>& expected) {
    ASSERT_TRUE(value.IsArray());ASSERT_EQ(value.Size(),N);
    for(std::size_t i=0;i<N;++i)ExpectBits(value[i].GetDouble(),expected[i]);
}
void ExpectVector(const out::Value& value,contact::Vec3 expected) {
    ASSERT_TRUE(value.IsArray());ASSERT_EQ(value.Size(),3u);
    ExpectBits(value[0u].GetDouble(),expected.x);ExpectBits(value[1u].GetDouble(),expected.y);ExpectBits(value[2u].GetDouble(),expected.z);
}
void ExpectCertificate(const out::Value& value,const contact::Q4CertifiedIntegral& expected) {
    ExpectArray(value,std::array<double,4>{{expected.value,expected.lower,expected.upper,expected.error}});
}
void ExpectFields(const out::Document& doc,const Frame& frame,const ref::GuidedPlateData& model) {
    EXPECT_STREQ(doc["schema"].GetString(),"robo_dyna.guided_plate_fields.v1");
    EXPECT_EQ(doc["owner_id"].GetUint64(),frame.stamp.owner_id);
    EXPECT_EQ(doc["accepted_epoch"].GetUint64(),frame.stamp.epoch);
    ExpectBits(doc["accepted_time_s"].GetDouble(),frame.stamp.time);
    ExpectBits(doc["fixed_dt_s"].GetDouble(),frame.stamp.fixed_dt);
    EXPECT_EQ(doc["reaction_base_epoch"].GetUint64(),frame.stamp.reaction_base_epoch);
    ExpectBits(doc["reaction_time_s"].GetDouble(),frame.stamp.reaction_time);
    EXPECT_EQ(doc["reactions_valid"].GetBool(),frame.stamp.reactions_valid);
    EXPECT_EQ(doc["element_evaluation_base_epoch"].GetUint64(),frame.element_association.base_epoch);
    EXPECT_EQ(doc["element_evaluation_attempt"].GetUint64(),frame.element_association.attempt);
    EXPECT_EQ(doc["contact_evaluation_base_epoch"].GetUint64(),frame.contact_association.base_epoch);
    EXPECT_EQ(doc["contact_evaluation_attempt"].GetUint64(),frame.contact_association.attempt);
    EXPECT_EQ(doc["qualification_id"].GetUint64(),frame.element_association.configuration_id);
    EXPECT_EQ(doc["wall_binding_id"].GetUint64(),frame.contact_association.wall_binding_id);
    ExpectArray(doc["position_xyz_m"],frame.position);ExpectArray(doc["orientation_wxyz"],frame.rotation);
    ExpectArray(doc["velocity_xyz_m_per_s"],frame.velocity);ExpectArray(doc["omega_world_xyz_rad_per_s"],frame.omega);
    ExpectArray(doc["reaction_force_xyz_N_at_base"],frame.reaction_force);
    ExpectArray(doc["reaction_couple_world_xyz_Nm_at_base"],frame.reaction_couple);
    ASSERT_EQ(doc["elements"].Size(),ref::kCouponElements);
    ASSERT_EQ(doc["contact_parents"].Size(),ref::kCouponElements);
    for(unsigned e=0;e<ref::kCouponElements;++e) {
        const auto& actual=doc["elements"][e];const auto& expected=frame.element[e];
        EXPECT_EQ(actual["parent_element"].GetUint64(),model.parents[e].parent_element_id);
        ExpectBits(actual["elastic_energy_J"].GetDouble(),expected.energy);
        ExpectBits(actual["bending_energy_J"].GetDouble(),expected.bending_energy);
        const auto& parent=doc["contact_parents"][e];const auto& integral=frame.parent[e].integration;
        EXPECT_EQ(parent["parent_element"].GetUint64(),model.parents[e].parent_element_id);
        EXPECT_EQ(parent["parent_face"].GetUint(),model.parents[e].parent_face_id);
        EXPECT_EQ(parent["feature_id"].GetUint64(),model.parents[e].feature_id);
        EXPECT_EQ(parent["base_epoch"].GetUint64(),integral.base_epoch);EXPECT_EQ(parent["attempt"].GetUint64(),integral.attempt);
        EXPECT_EQ(parent["covered"].GetBool(),frame.parent[e].covered);EXPECT_EQ(parent["valid"].GetBool(),integral.valid);
        ExpectCertificate(parent["resultant_magnitude_N"],integral.resultant);ExpectCertificate(parent["potential_J"],integral.potential);
        ExpectArray(parent["active_area_m2"],std::array<double,2>{{integral.active_area.lower,integral.active_area.upper}});
        EXPECT_EQ(parent["leaf_count"].GetUint(),integral.leaf_count);EXPECT_EQ(parent["visited"].GetUint(),integral.visited);
        EXPECT_EQ(parent["deepest_leaf"].GetUint(),integral.deepest_leaf);
        for(unsigned n=0;n<4;++n) {
            EXPECT_EQ(parent["connectivity_zero_based"][n].GetUint(),model.parents[e].nodes[n]);
            ExpectVector(parent["force_world_N"][n],integral.nodal.forces[n]);
            ExpectVector(parent["couple_world_Nm"][n],integral.nodal.couples[n]);
            ExpectCertificate(parent["nodal_force_magnitude_N"][n],integral.force[n]);
            for(unsigned c=0;c<3;++c) {
                ExpectBits(actual["force_world_N"][n][c].GetDouble(),shell::detail::Component(expected.force[n],c));
                ExpectBits(actual["couple_world_Nm"][n][c].GetDouble(),shell::detail::Component(expected.couple[n],c));
            }
            for(unsigned c=0;c<12;++c) {
                ExpectBits(actual["gauss_strain"][n][c].GetDouble(),expected.strain[n][c]);
                ExpectBits(actual["gauss_resultant"][n][c].GetDouble(),expected.resultant[n][c]);
            }
        }
    }
    const auto& contact=doc["contact_result"];const auto& expected=frame.contact_association;
    ExpectVector(contact["wall_reaction_N"],expected.wall_reaction);ExpectVector(contact["wall_moment_Nm"],expected.wall_moment);
    ExpectVector(contact["force_error_N"],expected.force_error);ExpectVector(contact["wall_moment_error_Nm"],expected.wall_moment_error);
    ExpectCertificate(contact["potential_J"],expected.potential);
    ExpectBits(contact["maximum_penetration_m"].GetDouble(),expected.maximum_penetration);
}
void ExpectMesh(const fs::path& path,const std::vector<std::array<double,3>>& positions,
                const std::vector<std::array<std::uint32_t,3>>& triangles) {
    chrono::ChTriangleMeshConnected mesh;
    std::ifstream file(path);chrono::ChArchiveInJSON archive(file,true);archive>>chrono::make_ChNameValue("mesh",mesh);
    ASSERT_EQ(mesh.GetCoordsVertices().size(),positions.size());ASSERT_EQ(mesh.GetIndicesVertices().size(),triangles.size());
    for(std::size_t n=0;n<positions.size();++n)for(unsigned c=0;c<3;++c)ExpectBits(mesh.GetCoordsVertices()[n][c],positions[n][c]);
    for(std::size_t n=0;n<triangles.size();++n)for(unsigned c=0;c<3;++c)EXPECT_EQ(mesh.GetIndicesVertices()[n][c],triangles[n][c]);
}
std::vector<std::string> Split(const std::string& line) {
    std::istringstream stream(line);std::vector<std::string> result;std::string field;
    while(std::getline(stream,field,','))result.push_back(field);return result;
}
std::map<std::string,std::string> OneInterval(const fs::path& path) {
    std::istringstream stream(out::ReadBounded(path,1024*1024));std::string header,line;
    out::Require(bool(std::getline(stream,header))&&bool(std::getline(stream,line)),"Missing guided interval row");
    const auto columns=Split(header),row=Split(line);
    out::Require(columns.size()==row.size()&&!std::getline(stream,line),"Unexpected guided interval columns or rows");
    std::map<std::string,std::string> result;
    for(std::size_t i=0;i<columns.size();++i)out::Require(result.emplace(columns[i],row[i]).second,"Duplicate interval column");
    return result;
}
void ExpectState(const Frame& a,const Frame& b) {
    EXPECT_EQ(a.position,b.position);EXPECT_EQ(a.rotation,b.rotation);EXPECT_EQ(a.velocity,b.velocity);EXPECT_EQ(a.omega,b.omega);
    EXPECT_EQ(a.reaction_force,b.reaction_force);EXPECT_EQ(a.reaction_couple,b.reaction_couple);
    EXPECT_EQ(a.stamp.owner_id,b.stamp.owner_id);EXPECT_EQ(a.stamp.epoch,b.stamp.epoch);ExpectBits(a.stamp.time,b.stamp.time);
    ExpectBits(a.stamp.fixed_dt,b.stamp.fixed_dt);EXPECT_EQ(a.stamp.reactions_valid,b.stamp.reactions_valid);
    EXPECT_EQ(a.stamp.reaction_base_epoch,b.stamp.reaction_base_epoch);ExpectBits(a.stamp.reaction_time,b.stamp.reaction_time);
    ExpectBits(a.metrics.work.total_energy,b.metrics.work.total_energy);
    ExpectBits(a.metrics.wall_impulse.x,b.metrics.wall_impulse.x);
}

TEST_F(GuidedPlateArtifactsCheck, ShortActualPrefixPreservesFieldsMeshWallAndIntervalIdentity) {
    GuidedPlateCase run;auto r=run.Initialize(wall);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    TempRoot root;const auto directory=root.path/"prefix";
    GuidedPlateArtifacts writer(directory.string(),bytes,wall,run,FrameEvery(run));
    Frame initial,accepted;r=run.Capture(initial);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;writer.WriteFrame(run);
    const auto base=run.metrics()->stamp;r=run.Step();ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    writer.RecordInterval(base,*run.metrics());r=run.Capture(accepted);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;writer.WriteFrame(run);
    const auto config=Json(directory/"configuration.json");
    EXPECT_STREQ(config["canonical_wall_manifest_sha256"].GetString(),kCanonicalWallManifestSha256);
    EXPECT_EQ(config["required_steps"].GetUint64(),run.metrics()->required_steps);
    ASSERT_EQ(config["vertex_binding"].Size(),6u);ASSERT_EQ(config["triangle_binding"].Size(),4u);
    ASSERT_EQ(config["contact_parent_binding"].Size(),2u);
    for(unsigned n=0;n<6;++n) {
        const auto& binding=config["vertex_binding"][n];ASSERT_EQ(binding.Size(),4u);
        EXPECT_EQ(binding[0u].GetUint64(),n);EXPECT_EQ(binding[1u].GetUint64(),3u);
        EXPECT_EQ(binding[2u].GetUint64(),1u);EXPECT_EQ(binding[3u].GetUint64(),n+1);
        EXPECT_EQ(config["translation_fixed_bits"][n].GetUint(),run.guided_data()->translation_fixed_bits[n]);
        EXPECT_EQ(config["rotation_fixed"][n].GetUint(),run.guided_data()->rotation_fixed[n]);
    }
    for(unsigned e=0;e<2;++e) {
        const auto& parent=config["contact_parent_binding"][e];const auto& expected=run.guided_data()->parents[e];
        EXPECT_EQ(parent["parent_element"].GetUint64(),expected.parent_element_id);
        EXPECT_EQ(parent["feature_id"].GetUint64(),expected.feature_id);EXPECT_EQ(parent["parent_face"].GetUint(),expected.parent_face_id);
        for(unsigned n=0;n<4;++n)EXPECT_EQ(parent["connectivity_zero_based"][n].GetUint(),expected.nodes[n]);
    }
    std::vector<std::array<std::uint32_t,3>> triangles;
    for(unsigned t=0;t<4;++t) {
        const auto& binding=config["triangle_binding"][t];ASSERT_EQ(binding.Size(),9u);
        EXPECT_EQ(binding[5u].GetUint64(),run.guided_data()->parents[t/2].parent_element_id);
        triangles.push_back({binding[0u].GetUint(),binding[1u].GetUint(),binding[2u].GetUint()});
    }
    for(unsigned epoch=0;epoch<2;++epoch) {
        const auto& frame=epoch?accepted:initial;const std::string stem=epoch?"accepted-000001":"accepted-000000";
        const auto fields=Json(directory/(stem+".fields.json"));ExpectFields(fields,frame,*run.guided_data());
        const char* phase=epoch?"prepared_candidate_subsequently_committed":"accepted_base";
        EXPECT_STREQ(fields["element_evaluation_phase"].GetString(),phase);EXPECT_STREQ(fields["contact_evaluation_phase"].GetString(),phase);
        std::vector<std::array<double,3>> positions;
        for(unsigned n=0;n<6;++n)positions.push_back({frame.position[3*n],frame.position[3*n+1],frame.position[3*n+2]});
        ExpectMesh(directory/(stem+".mesh.json"),positions,triangles);
        const auto obj=chrono::ChTriangleMeshConnected::CreateFromWavefrontFile((directory/(stem+".obj")).string(),false,false);
        ASSERT_TRUE(obj);EXPECT_EQ(obj->GetNumVertices(),6u);EXPECT_EQ(obj->GetNumTriangles(),4u);
    }
    EXPECT_EQ(out::ReadBounded(directory/"canonical-wall.manifest.json",1024*1024),bytes);
    std::vector<std::array<double,3>> wall_positions;triangles.clear();
    for(const auto& v:wall.vertices())wall_positions.push_back(v.position_m);
    for(const auto& t:wall.triangles())triangles.push_back(t.vertex_indices);
    ASSERT_EQ(wall_positions.size(),62u);ASSERT_EQ(triangles.size(),100u);
    ExpectMesh(directory/"canonical-wall.mesh.json",wall_positions,triangles);
    for(const char* file:{"accepted-intervals.csv","shell-intervals.csv","contact-intervals.csv"}) {
        const auto row=OneInterval(directory/file);
        EXPECT_EQ(std::stoull(row.at("owner_id")),base.owner_id);EXPECT_EQ(std::stoull(row.at("base_epoch")),base.epoch);
        EXPECT_EQ(std::stoull(row.at("attempt")),accepted.metrics.shell.attempt);
        EXPECT_EQ(std::stoull(row.at("accepted_epoch")),accepted.stamp.epoch);
        ExpectBits(std::stod(row.at("force_eval_time_s")),base.time);
        ExpectBits(std::stod(row.at("accepted_time_s")),accepted.stamp.time);
    }
    const auto combined=OneInterval(directory/"accepted-intervals.csv"),shell=OneInterval(directory/"shell-intervals.csv"),
               contact=OneInterval(directory/"contact-intervals.csv");
    ExpectBits(std::stod(combined.at("total_energy_J")),accepted.metrics.work.total_energy);
    ExpectBits(std::stod(combined.at("wall_impulse_x_Ns")),accepted.metrics.wall_impulse.x);
    ExpectBits(std::stod(shell.at("elastic_energy_J")),accepted.metrics.shell.elastic_energy);
    ExpectBits(std::stod(contact.at("wall_reaction_x_N_at_base")),accepted.metrics.applied_contact.wall_reaction.x);
    ExpectBits(std::stod(contact.at("wall_reaction_x_N_at_endpoint")),accepted.metrics.contact.wall_reaction.x);
    ExpectBits(std::stod(contact.at("potential_error_J")),accepted.metrics.contact.potential.error);
    EXPECT_FALSE(fs::exists(directory/"manifest.json"));EXPECT_FALSE(fs::exists(directory/"final-metrics.json"));
}

TEST_F(GuidedPlateArtifactsCheck, RejectedAttemptRefreshesBothEndpointAssociationsWithoutChangingAppliedInterval) {
    GuidedPlateCase run;auto r=run.Initialize(wall);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    TempRoot root;const auto directory=root.path/"refresh";
    GuidedPlateArtifacts writer(directory.string(),bytes,wall,run,FrameEvery(run));writer.WriteFrame(run);
    const auto base=run.metrics()->stamp;r=run.Step();ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    writer.RecordInterval(base,*run.metrics());Frame before,recovered;
    r=run.Capture(before);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    const auto candidate=GuidedPlateFrameFields(before,*run.guided_data());
    EXPECT_STREQ(candidate["contact_evaluation_phase"].GetString(),"prepared_candidate_subsequently_committed");
    r=run.Step({1e-9});ASSERT_EQ(r.status,Code::AdmissionFailure)<<r.diagnostic;
    r=run.Capture(recovered);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;ExpectState(recovered,before);
    EXPECT_GT(recovered.contact_association.attempt,before.contact_association.attempt);
    EXPECT_EQ(recovered.metrics.applied_contact.attempt,before.metrics.applied_contact.attempt);
    writer.WriteFrame(run);const auto fields=Json(directory/"accepted-000001.fields.json");ExpectFields(fields,recovered,*run.guided_data());
    EXPECT_STREQ(fields["element_evaluation_phase"].GetString(),"accepted_base");
    EXPECT_STREQ(fields["contact_evaluation_phase"].GetString(),"accepted_base");
    EXPECT_EQ(fields["contact_evaluation_base_epoch"].GetUint64(),1u);
    EXPECT_EQ(fields["reaction_base_epoch"].GetUint64(),0u);
    for(const char* file:{"accepted-intervals.csv","shell-intervals.csv","contact-intervals.csv"})
        EXPECT_EQ(std::stoull(OneInterval(directory/file).at("attempt")),before.metrics.applied_contact.attempt);
}

TEST_F(GuidedPlateArtifactsCheck, ForeignStaleSkippedAndDuplicateInputsCannotPublishRows) {
    GuidedPlateCase run,foreign;auto r=run.Initialize(wall);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    r=foreign.Initialize(wall);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    TempRoot root;const auto directory=root.path/"identity";
    GuidedPlateArtifacts writer(directory.string(),bytes,wall,run,FrameEvery(run));writer.WriteFrame(run);
    const auto initial_index=out::ReadBounded(directory/"accepted-frames.csv",1024*1024);
    EXPECT_THROW(writer.WriteFrame(run),std::runtime_error);
    EXPECT_THROW(writer.WriteFrame(foreign),std::runtime_error);
    EXPECT_EQ(out::ReadBounded(directory/"accepted-frames.csv",1024*1024),initial_index);
    const auto zero=run.metrics()->stamp;r=run.Step();ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;const auto first=*run.metrics();
    EXPECT_THROW(writer.WriteFrame(run),std::runtime_error);
    for(unsigned variant=0;variant<4;++variant) {
        auto bad=first;
        if(variant==0)++bad.contact.attempt;
        if(variant==1)++bad.applied_contact.base_epoch;
        if(variant==2)++bad.contact.wall_binding_id;
        if(variant==3)++bad.shell.configuration_id;
        EXPECT_THROW(writer.RecordInterval(zero,bad),std::runtime_error)<<variant;
    }
    const auto other_base=foreign.metrics()->stamp;r=foreign.Step();ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    EXPECT_THROW(writer.RecordInterval(other_base,*foreign.metrics()),std::runtime_error);
    const auto one=run.metrics()->stamp;r=run.Step();ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    EXPECT_THROW(writer.RecordInterval(one,*run.metrics()),std::runtime_error);
    ASSERT_NO_THROW(writer.RecordInterval(zero,first));
    EXPECT_THROW(writer.RecordInterval(zero,first),std::runtime_error);
    ASSERT_NO_THROW(writer.RecordInterval(one,*run.metrics()));
    ASSERT_NO_THROW(writer.WriteFrame(run));
    EXPECT_FALSE(fs::exists(directory/"accepted-000001.fields.json"));
    EXPECT_THROW(writer.Finish(foreign,.1),std::runtime_error);EXPECT_FALSE(fs::exists(directory/"manifest.json"));
}

TEST_F(GuidedPlateArtifactsCheck, MalformedFieldsRejectLateParentCertificatesAndMixedAssociations) {
    GuidedPlateCase run;auto r=run.Initialize(wall);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    Frame valid;r=run.Capture(valid);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    ASSERT_NO_THROW(GuidedPlateFrameFields(valid,*run.guided_data()));
    for(unsigned variant=0;variant<14;++variant) {
        auto bad=valid;
        if(variant==0)++bad.contact_association.owner_id;
        if(variant==1)++bad.contact_association.attempt;
        if(variant==2)bad.contact_association.phase=contact::Q4PlanarContactPhase::PreparedCandidate;
        if(variant==3)++bad.parent[1].integration.feature_id;
        if(variant==4)++bad.parent[1].integration.nodal.nodes[3];
        if(variant==5)bad.parent[1].integration.potential.lower=-1;
        if(variant==6)bad.parent[1].integration.force[3].error=-1;
        if(variant==7)bad.parent[1].integration.potential.upper=1; // Unchanged zero radius understates this interval.
        if(variant==8)bad.contact_association.force_error.x=-1;
        if(variant==9)bad.contact_association.wall_moment_error.z=-1;
        if(variant==10)bad.parent[1].integration.resultant.value=std::numeric_limits<double>::infinity();
        if(variant==11)bad.element[1].resultant[3][11]=std::numeric_limits<double>::quiet_NaN();
        if(variant==12)bad.stamp.reactions_valid=true;
        if(variant==13)bad.contact_association.potential.upper=1;
        EXPECT_THROW(GuidedPlateFrameFields(bad,*run.guided_data()),std::runtime_error)<<"variant "<<variant;
    }
    ExpectFields(GuidedPlateFrameFields(valid,*run.guided_data()),valid,*run.guided_data());
}

TEST_F(GuidedPlateArtifactsCheck, ConstructorRejectsUninitializedBadBytesCadenceAndExistingDirectoryBeforePublication) {
    TempRoot root;GuidedPlateCase empty;
    EXPECT_THROW(GuidedPlateArtifacts((root.path/"empty").string(),bytes,wall,empty,100),std::runtime_error);
    EXPECT_FALSE(fs::exists(root.path/"empty"));
    GuidedPlateCase run;auto r=run.Initialize(wall);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    for(unsigned variant=0;variant<3;++variant) {
        const auto directory=root.path/("invalid-"+std::to_string(variant));
        const auto source=variant==0?bytes+" ":bytes;
        const auto cadence=variant==1?0u:variant==2?1u:FrameEvery(run);
        EXPECT_THROW(GuidedPlateArtifacts(directory.string(),source,wall,run,cadence),std::runtime_error)<<variant;
        EXPECT_FALSE(fs::exists(directory));
    }
    // The loader admits this same geometry with a different triangle order;
    // authentic bytes for another object must not authenticate that object.
    auto reordered=Json(asset);reordered["triangles"][0u].Swap(reordered["triangles"][1u]);
    out::WriteJson(root.path/"reordered-wall.json",reordered);
    CanonicalWall other_wall;const auto loaded=other_wall.LoadFile((root.path/"reordered-wall.json").string());
    ASSERT_EQ(loaded.status,WallStatus::Ok)<<loaded.message;
    EXPECT_THROW(GuidedPlateArtifacts((root.path/"wrong-wall").string(),bytes,other_wall,run,FrameEvery(run)),std::runtime_error);
    EXPECT_FALSE(fs::exists(root.path/"wrong-wall"));
    const auto existing=root.path/"existing";ASSERT_TRUE(fs::create_directory(existing));
    out::WriteBytes(existing/"sentinel","Retain existing output\n");
    EXPECT_THROW(GuidedPlateArtifacts(existing.string(),bytes,wall,run,FrameEvery(run)),std::runtime_error);
    EXPECT_EQ(out::ReadBounded(existing/"sentinel",1024),"Retain existing output\n");
    EXPECT_EQ(std::distance(fs::directory_iterator(existing),fs::directory_iterator{}),1);
}

TEST(GuidedPlateArtifactForecast, EncodedFiniteExtremesFitTheDeclaredLayoutAndByteCaps) {
    constexpr std::uint64_t steps=19997;
    const auto forecast=ForecastGuidedPlateOutput(steps,100);
    ASSERT_EQ(forecast.frames,201u);
    const auto& headers=GuidedPlateIntervalHeaders();
    std::size_t total=kGuidedStaticReserve+forecast.frames*(kGuidedFieldFileCap+kGuidedMeshFileCap+kGuidedObjFileCap);
    // This is a formatter-only stress record, not an admitted physical state.
    GuidedPlateMetrics values;
    values.work.total_energy=-std::numeric_limits<double>::max();
    values.shell.elastic_energy=std::numeric_limits<double>::denorm_min();
    values.contact.potential.value=std::numeric_limits<double>::min();
    const auto rows=GuidedPlateIntervalRows({},values);
    for(unsigned n=0;n<headers.size();++n) {
        const auto columns=Split(headers[n]);const auto row=Split(rows[n]);ASSERT_EQ(row.size(),columns.size());
        ASSERT_LE(columns.size(),64u);ASSERT_LE(headers[n].size(),8192u);
        EXPECT_EQ(forecast.ledger_bytes[n],headers[n].size()+steps*columns.size()*26);
        EXPECT_LE(rows[n].size(),columns.size()*26);EXPECT_LE(forecast.ledger_bytes[n],out::kArtifactFileCap);
        total+=forecast.ledger_bytes[n];
        for(std::size_t c=0;c<columns.size();++c) {
            if(columns[c]=="total_energy_J")ExpectBits(std::strtod(row[c].c_str(),nullptr),values.work.total_energy);
            if(columns[c]=="elastic_energy_J")ExpectBits(std::strtod(row[c].c_str(),nullptr),values.shell.elastic_energy);
            if(columns[c]=="potential_J")ExpectBits(std::strtod(row[c].c_str(),nullptr),values.contact.potential.value);
        }
    }
    EXPECT_EQ(forecast.total_bytes,total);EXPECT_LE(total,out::kArtifactTotalCap);
    EXPECT_THROW(ForecastGuidedPlateOutput(steps,1),std::runtime_error); // Frame capacity.
    EXPECT_THROW(ForecastGuidedPlateOutput(2*steps,200),std::runtime_error); // Compact h/2 study, not these full ledgers.
    EXPECT_THROW(ForecastGuidedPlateOutput(0,100),std::runtime_error);
    EXPECT_THROW(ForecastGuidedPlateOutput(steps,0),std::runtime_error);
    EXPECT_THROW(ForecastGuidedPlateOutput(std::numeric_limits<std::uint64_t>::max(),100),std::runtime_error);
    values.shell.elastic_energy=std::numeric_limits<double>::infinity();
    EXPECT_THROW(GuidedPlateIntervalRows({},values),std::runtime_error);
}

TEST_F(GuidedPlateArtifactsCheck, IncompleteFinishAndOutputFailurePreserveAcceptedStateAndCannotComplete) {
    GuidedPlateCase run;auto r=run.Initialize(wall);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    TempRoot root;const auto directory=root.path/"failed";
    GuidedPlateArtifacts writer(directory.string(),bytes,wall,run,FrameEvery(run));writer.WriteFrame(run);
    const auto base=run.metrics()->stamp;r=run.Step();ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    writer.RecordInterval(base,*run.metrics());Frame accepted,after;
    r=run.Capture(accepted);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    EXPECT_THROW(writer.Finish(run,.1),std::runtime_error);
    const auto index=out::ReadBounded(directory/"accepted-frames.csv",1024*1024);
    // A deterministic ordinary filesystem conflict exercises a late multi-file
    // write failure without permissions changes, disk exhaustion or GPU faults.
    out::WriteBytes(directory/"accepted-000001.fields.json","Existing destination must survive\n");
    EXPECT_THROW(writer.WriteFrame(run),std::runtime_error);
    EXPECT_EQ(out::ReadBounded(directory/"accepted-000001.fields.json",1024),"Existing destination must survive\n");
    EXPECT_EQ(out::ReadBounded(directory/"accepted-frames.csv",1024*1024),index);
    r=run.Capture(after);ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;ExpectState(after,accepted);
    writer.Fail("Intentional accepted output failure");const auto failure=out::ReadBounded(directory/"failure.json",1024*1024);
    const auto doc=Json(directory/"failure.json");EXPECT_STREQ(doc["status"].GetString(),"failed");
    EXPECT_EQ(doc["last_recorded_epoch"].GetUint64(),1u);
    writer.Fail("This later message must not replace the first failure");
    EXPECT_EQ(out::ReadBounded(directory/"failure.json",1024*1024),failure);
    EXPECT_THROW(writer.WriteFrame(run),std::runtime_error);
    EXPECT_THROW(writer.RecordInterval(base,*run.metrics()),std::runtime_error);
    EXPECT_THROW(writer.Finish(run,.1),std::runtime_error);
    EXPECT_FALSE(fs::exists(directory/"manifest.json"));EXPECT_FALSE(fs::exists(directory/"final-metrics.json"));
    r=run.Step();ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;EXPECT_EQ(run.metrics()->stamp.epoch,2u);
}
} // namespace

int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);
    if(argc!=2){std::cerr<<"Required canonical wall manifest argument\n";return 2;}
    asset=argv[1];return RUN_ALL_TESTS();
}
