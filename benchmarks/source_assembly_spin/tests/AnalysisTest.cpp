#include "benchmarks/source_assembly_spin/AnalysisInternal.h"
#include "output/source_assembly/QephSpinTrace.h"
#include "case/source_assembly_observation/tests/QephSpinFixture.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/rapidjson/writer.h"
#include <cstdlib>

namespace crash::benchmarks::assembly_spin::test {
namespace observation=cases::source_assembly_observation;
std::string Encode(const Value& v){rapidjson::StringBuffer b;rapidjson::Writer<rapidjson::StringBuffer> w(b);Require(v.Accept(w),"Fixture serialization failed");return std::string(b.GetString(),b.GetSize())+'\n';}
struct Fixture {
    std::filesystem::path directory,inventory;Document header,row;
    Fixture() {
        const auto* path=std::getenv("ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY");Require(path&&*path,"Explicit source fixture required");inventory=path;
        std::string pattern=(std::filesystem::temp_directory_path()/"spin-native-values-XXXXXX").string();std::vector<char> p(pattern.begin(),pattern.end());p.push_back(0);
        const auto* made=mkdtemp(p.data());Require(made,"Temporary fixture directory failed");directory=made;
        // Supplied value fixture intentionally has an altered candidate point
        // and couple. Native replay must expose that difference, not certify it.
        observation::test::SpinFixture f;observation::QephSpinObservation v;Require(bool(observation::ObserveQephSpin(f.spin_input(),&v)),"Probe fixture failed");
        v.enclosing=v.base;v.enclosing.epoch=1;v.enclosing.time=v.base.fixed_dt;v.enclosing.velocity_time=.5*v.base.fixed_dt;
        v.enclosing.velocity_phase=tl::fea::NodalVelocityPhase::PreviousMidpoint;v.enclosing.reactions_valid=true;
        v.enclosing.reaction_kick_dt=.5*v.base.fixed_dt;v.completed=true;row=output::assembly::QephSpinDocument(v);
        header.SetObject();output::String(header,"schema","robo_dyna.source_assembly_qeph_spin_trace.v1");output::String(header,"record","header");
        output::String(header,"source_inventory_sha256",f.bindings.source().data().identity.sha256);output::Integer(header,"source_inventory_bytes",f.bindings.source().data().identity.bytes);
        output::Integer(header,"source_node_id",v.source_node);output::Integer(header,"source_instance_id",v.source_instance);output::Integer(header,"owner_id",v.base.owner_id);
        output::Integer(header,"configuration_id",71);output::Integer(header,"qualification_id",72);output::Integer(header,"wall_binding_id",73);
        output::Number(header,"fixed_dt_s",v.base.fixed_dt);output::Integer(header,"requested_steps",4);output::Integer(header,"cadence",2);
        output::Integer(header,"row_byte_cap",RowCap);output::Integer(header,"total_byte_cap",TraceCap);output::Integer(header,"forecast_bytes",6*RowCap);
    }
    ~Fixture(){std::error_code e;std::filesystem::remove_all(directory,e);}
    std::string Bytes(bool footer=true) {
        auto bytes=Encode(header)+Encode(row);if(!footer)return bytes;
        Document f;f.SetObject();output::String(f,"record","completion");output::Boolean(f,"horizon_complete",false);output::String(f,"stop_reason","Supplied pure-value fixture");
        output::Integer(f,"accepted_epoch",1);output::Number(f,"accepted_time_s",json::Real(json::Member(row,"enclosing_accepted_stamp"),"time"));
        output::Integer(f,"saved_force_stages",1);output::Boolean(f,"has_force_stage",true);output::Number(f,"last_observed_force_time_s",0);
        output::Integer(f,"trace_bytes_before_footer",bytes.size());return bytes+Encode(f);
    }
    Document Run(const char* name,bool footer=true) {const auto p=directory/name;output::WriteBytes(p,Bytes(footer));return Analyze(inventory,p);}
};
TEST(SourceSpinAnalysis, FullSuppliedPairReportsNativeDifferencesWithoutClaimingAcceptance) {
    Fixture f;const auto report=f.Run("valid.jsonl");EXPECT_EQ(json::Unsigned(report,"sampled_force_stages"),1u);
    EXPECT_FALSE(json::Member(report,"horizon_complete").GetBool());EXPECT_EQ(json::Unsigned(report,"incident_parents"),2u);
    const auto& differences=json::Member(report,"native_recurrence_differences");
    EXPECT_GT(json::Real(json::Member(differences,"point_PLA"),"max_abs_difference"),.009);
    EXPECT_GT(json::Real(json::Member(differences,"positive_internal_couple_xyz_N_m"),"max_abs_difference"),.12);
    EXPECT_EQ(json::Array(report,"local_observations",2,2).Size(),2u);
}
TEST(SourceSpinAnalysis, CompleteSourceAndPhaseAreCheckedBeforeNativeInterpretation) {
    Fixture f;auto& c=f.row["enclosing_candidate_parents"][1u];const auto curve=c["source_curve_id"].GetUint64();c["source_curve_id"].SetUint64(curve+1);
    EXPECT_THROW(f.Run("wrong-source.jsonl"),std::runtime_error);c["source_curve_id"].SetUint64(curve);
    auto& stamp=f.row["enclosing_accepted_stamp"];stamp["reaction_time"].SetDouble(1e-8);
    EXPECT_THROW(f.Run("wrong-phase.jsonl"),std::runtime_error);stamp["reaction_time"].SetDouble(0);
    c["position_endpoint_xyz_m"][0u].SetDouble(c["position_endpoint_xyz_m"][0u].GetDouble()+1e-8);
    EXPECT_THROW(f.Run("split-shared-node.jsonl"),std::runtime_error);
    c["position_endpoint_xyz_m"][0u].SetDouble(f.row["enclosing_candidate_parents"][0u]["position_endpoint_xyz_m"][3u].GetDouble());
    c["retained_kinematics"]["origin_endpoint_epoch"].SetUint64(2);
    EXPECT_THROW(f.Run("wrong-origin.jsonl"),std::runtime_error);
}
TEST(SourceSpinAnalysis, TruncatedMissingFooterAndAlteredFooterDoNotPublishReports) {
    Fixture f;EXPECT_THROW(f.Run("missing-footer.jsonl",false),std::runtime_error);
    auto bytes=f.Bytes();bytes.pop_back();output::WriteBytes(f.directory/"partial.jsonl",bytes);
    EXPECT_THROW(Analyze(f.inventory,f.directory/"partial.jsonl"),std::runtime_error);
    auto duplicated=Encode(f.header)+Encode(f.row)+Encode(f.row);output::WriteBytes(f.directory/"duplicate.jsonl",duplicated);
    EXPECT_THROW(Analyze(f.inventory,f.directory/"duplicate.jsonl"),std::runtime_error);
    f.header["forecast_bytes"].SetUint64(RowCap);
    EXPECT_THROW(f.Run("false-cap.jsonl"),std::runtime_error);
}
TEST(SourceSpinAnalysis, OptionalCompletedActualTraceTraversesAllNativePairs) {
    const auto* trace=std::getenv("ROBO_DYNA_SOURCE_ASSEMBLY_SPIN_TRACE");if(!trace||!*trace)GTEST_SKIP()<<"No completed actual trace supplied";
    const auto* inventory=std::getenv("ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY");ASSERT_NE(inventory,nullptr);
    const auto report=Analyze(inventory,trace);EXPECT_GT(json::Unsigned(report,"sampled_force_stages"),1u);
    EXPECT_EQ(json::Unsigned(report,"incident_parents"),2u);
    const auto& m=json::Member(json::Member(report,"native_recurrence_differences"),"point_stress");
    EXPECT_GT(json::Unsigned(m,"samples"),15u); // Report metrics; no new arbitrary parity/energy threshold.
}
}
