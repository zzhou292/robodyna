#include "accepted_replay_source_assembly_connector_fixture.h"
#include "source_assembly/SourceAssemblyKineticSchema.h"
#include "ReplayBundleTestSupport.h"
#include <gtest/gtest.h>
#include <cstdlib>
#include <limits>
namespace crash::output {
namespace {
using namespace replay_detail;
using connector_test::Fixture;using connector_test::Put;
bool HasSource() {const auto* p=std::getenv("ROBO_DYNA_SEVEN_PART_SOURCE_INVENTORY");return p&&*p;}
TEST(AcceptedReplayConnectors,SourceSpecificConfigurationAndManifestCannotMislabelSevenAsSix) {
    Document c,m;c.SetObject();m.SetObject();String(c,"scope",AssemblyConnectorScope);String(m,"scope",AssemblyConnectorScope);
    EXPECT_NO_THROW(CheckAssemblySourceScope(c,m,true));EXPECT_THROW(CheckAssemblySourceScope(c,m,false),std::runtime_error);
    m["scope"].SetString("Original connected six-part Yaris component; complete internal groups and explicit released external connections",m.GetAllocator());
    EXPECT_THROW(CheckAssemblySourceScope(c,m,true),std::runtime_error);
    c["scope"].SetString("Original six-part Yaris component, internal nodal rigid groups active, external connections explicitly released",c.GetAllocator());
    EXPECT_NO_THROW(CheckAssemblySourceScope(c,m,false));
}
TEST(AcceptedReplayConnectors,ExplicitPolicySelectsPinnedSevenAndPreservesCompleteEndpointPartitions) {
    if(!HasSource())GTEST_SKIP()<<"No explicit seven-part source inventory supplied";
    Fixture f;EXPECT_EQ(AssemblySourceIdentity(f.configuration).sha256,source::PinnedYarisSevenPartInventory().sha256);
    f.Admit();const auto& a=*f.bundle.assembly;ASSERT_EQ(a.connectors.size(),1u);EXPECT_EQ(a.connectors[0].element,2101297u);
    EXPECT_EQ(a.source.data().parents.size(),959u);EXPECT_EQ(a.source.data().nodes.size(),1093u);
    std::size_t connected=0;for(std::size_t n=0;n<a.native_nodes.size();++n) {
        const auto& values=a.native_nodes[n];const bool endpoint=a.connector_nodes[n][0]>0;connected+=endpoint;
        EXPECT_EQ(values[0],endpoint?1+.0005:1);EXPECT_EQ(values[1],endpoint?2+5e-9:2);
        EXPECT_EQ(values[2],1);EXPECT_EQ(values[3],1);
        if(endpoint)EXPECT_FALSE(a.grouped_node[n]);
    }
    EXPECT_EQ(connected,2u);EXPECT_NO_THROW(CheckAssemblyConnectors(f.bundle,f.bundle.entries[0],f.frame));
    f.configuration["input"].EraseMember("connectors");EXPECT_THROW(AssemblySourceIdentity(f.configuration),std::runtime_error);
    f.configuration.EraseMember("connector_storage_limits");f.configuration.EraseMember("connector_work_scope");
    EXPECT_EQ(AssemblySourceIdentity(f.configuration).sha256,source::PinnedYarisSixPartInventory().sha256);
}
TEST(AcceptedReplayConnectors,OriginalCardsResolvedPropertiesMassAndNodeMembershipFailClosed) {
    if(!HasSource())GTEST_SKIP()<<"No explicit seven-part source inventory supplied";
    for(unsigned fault=0;fault<12;++fault) {
        SCOPED_TRACE(fault);Fixture f;auto& c=f.configuration["input"]["connectors"];auto& weld=c["connections"][0u];
        if(fault==0)c["policy"].SetString("implicit_defaults",f.configuration.GetAllocator());
        if(fault==1)weld["source_element_id"].SetUint64(0);
        if(fault==2)weld["source_card_lines"][1u].SetUint64(weld["source_card_lines"][1u].GetUint64()+1);
        if(fault==3)c["properties"][0u]["mass_kg"].SetDouble(.002);
        if(fault==4)c["properties"][0u]["failure_exponent"][3u].SetDouble(1);
        if(fault==5)c["endpoint_contributions"][1u][6u].SetDouble(0);
        if(fault==6)c["endpoint_contributions"][1u][8u].SetDouble(2);
        if(fault==7)weld["global_nodes"][1u].SetUint64(1093);
        if(fault==8)weld["reference_positions_m"][5u].SetDouble(0);
        if(fault==9)weld["transverse_axis"][0u].SetDouble(2);
        if(fault==10)f.bundle.assembly->grouped_node[f.bundle.assembly->source.data().internal_spotwelds[0].nodes[1]]=true;
        if(fault==11)c["connection_count"].SetUint64(2);
        EXPECT_THROW(f.Admit(),std::runtime_error);
    }
}
TEST(AcceptedReplayConnectors,ActualSourceShapedInitialRowsRejectFalsePhaseLateFrameLoadAndFailure) {
    if(!HasSource())GTEST_SKIP()<<"No explicit seven-part source inventory supplied";
    for(unsigned fault=0;fault<10;++fault) {
        SCOPED_TRACE(fault);Fixture f;f.Admit();auto& c=f.frame["connectors"];auto& d=c["diagnostics"];auto& e=c["elements"][0u];
        if(fault==0)d["attempt"].SetUint64(1);
        if(fault==1)d["time_s"].SetDouble(f.bundle.fixed_dt);
        if(fault==2)e["source_element_id"].SetUint64(0);
        if(fault==3)e["history"]["transverse_axis"][2u].SetDouble(std::numeric_limits<double>::quiet_NaN());
        if(fault==4)e["frame"]["axes_row_major"][8u].SetDouble(2);
        if(fault==5)e["frame"]["midpoint_length_m"].SetDouble(0);
        if(fault==6)e["endpoint_wrenches"][1u][5u].SetDouble(1);
        if(fault==7)e["history"]["active"].SetBool(false);
        if(fault==8)e["critical_dt_s"].SetDouble(f.bundle.fixed_dt);
        if(fault==9)d["internal_work_J"][3u].SetDouble(1);
        EXPECT_THROW(CheckAssemblyConnectors(f.bundle,f.bundle.entries[0],f.frame),std::runtime_error);
    }
}
TEST(AcceptedReplayConnectors,SubtotalsAreAlreadyIncludedAndCannotBeAddedTwiceOrSmuggledIntoLegacy) {
    if(!HasSource())GTEST_SKIP()<<"No explicit seven-part source inventory supplied";
    Fixture f;f.Admit();Document k;k.SetObject();String(k,"member_columns",assembly::KineticMemberColumns);
    String(k,"aggregate_columns",assembly::KineticAggregateColumns);
    const auto row=[&](std::initializer_list<double> data){Value v(rapidjson::kArrayType);for(double x:data)v.PushBack(x,k.GetAllocator());return v;};
    Put(k,"ordinary_native_nodes",row({3,2,.5,.75,5,0}));Put(k,"grouped_native_members",row({0,0,0,0,0,0}));
    Put(k,"aggregate_groups",row({0,0,0,0,0,0,0,0,0,0,0,0,0,0}));Put(k,"connector_kinetic_J",row({1,.75}));
    Number(k,"native_total_J",5);Number(k,"effective_total_J",5);
    EXPECT_NO_THROW(CheckAssemblyKineticChannels(f.bundle,k));
    k["native_total_J"].SetDouble(6.75);EXPECT_THROW(CheckAssemblyKineticChannels(f.bundle,k),std::runtime_error);
    k["native_total_J"].SetDouble(5);k["ordinary_native_nodes"][5u].SetDouble(.75);
    EXPECT_THROW(CheckAssemblyKineticChannels(f.bundle,k),std::runtime_error);
    f.bundle.assembly->connectors.clear();EXPECT_THROW(AssemblyConnectorKinetic(f.bundle,k),std::runtime_error);
}
TEST(AcceptedReplayConnectors,InactiveHistoryCannotResurrectAndSparseWorkIsNotAnIntervalIncrement) {
    if(!HasSource())GTEST_SKIP()<<"No explicit seven-part source inventory supplied";
    Fixture f;f.Admit();auto& b=f.bundle;
    std::string pattern=(std::filesystem::temp_directory_path()/"seven-replay-history-XXXXXX").string();
    std::vector<char> path(pattern.begin(),pattern.end());path.push_back(0);const auto* created=mkdtemp(path.data());ASSERT_NE(created,nullptr);
    struct Directory {std::filesystem::path path;~Directory(){std::error_code e;std::filesystem::remove_all(path,e);}} directory{created};
    b.directory=directory.path;auto& record=f.frame["connectors"];auto& h=record["elements"][0u]["history"];auto& d=record["diagnostics"];
    h["active"].SetBool(false);h["failure_criterion"].SetDouble(1);h["internal_work_J"][0u].SetDouble(5);
    d["active_count"].SetUint64(0);d["internal_work_J"][0u].SetDouble(5);
    const auto file="prior.fields.json";WriteJson(b.directory/file,f.frame);const auto bytes=ReadBounded(b.directory/file,kFileCap);
    b.inventory[file]={Sha256(bytes),bytes.size()};b.entries[0].epoch=1;b.entries[0].mesh="prior.mesh.json";
    Entry current;current.owner=23;current.epoch=2;current.time=2*b.fixed_dt;current.interval_base_time=b.fixed_dt;
    current.interval_base_velocity_time=.5*b.fixed_dt;current.interval_attempt=2;b.entries.push_back(current);
    d["epoch"].SetUint64(2);d["base_epoch"].SetUint64(1);d["attempt"].SetUint64(2);
    d["time_s"].SetDouble(current.time);d["base_time_s"].SetDouble(current.interval_base_time);
    d["velocity_time_s"].SetDouble(1.5*b.fixed_dt);d["base_velocity_time_s"].SetDouble(.5*b.fixed_dt);
    d["kick_dt_s"].SetDouble(b.fixed_dt);d["has_completed_interval"].SetBool(true);d["accepted_force_assembled"].SetBool(true);
    EXPECT_NO_THROW(CheckAssemblyConnectors(b,current,f.frame));
    h["active"].SetBool(true);d["active_count"].SetUint64(1);EXPECT_THROW(CheckAssemblyConnectors(b,current,f.frame),std::runtime_error);
    h["active"].SetBool(false);d["active_count"].SetUint64(0);
    h["internal_work_J"][0u].SetDouble(6);d["internal_work_J"][0u].SetDouble(6);
    EXPECT_THROW(CheckAssemblyConnectors(b,current,f.frame),std::runtime_error); // Adjacent final increment is still zero.
    current.epoch=4;current.time=4*b.fixed_dt;current.interval_base_time=3*b.fixed_dt;
    current.interval_base_velocity_time=2.5*b.fixed_dt;current.interval_attempt=4;b.entries.back()=current;
    d["epoch"].SetUint64(4);d["base_epoch"].SetUint64(3);d["attempt"].SetUint64(4);
    d["time_s"].SetDouble(current.time);d["base_time_s"].SetDouble(current.interval_base_time);
    d["velocity_time_s"].SetDouble(3.5*b.fixed_dt);d["base_velocity_time_s"].SetDouble(2.5*b.fixed_dt);
    EXPECT_NO_THROW(CheckAssemblyConnectors(b,current,f.frame)); // A sparse cumulative difference is not the final increment.
    d["newly_failed_count"].SetUint64(1);EXPECT_THROW(CheckAssemblyConnectors(b,current,f.frame),std::runtime_error);
    d["newly_failed_count"].SetUint64(0);EXPECT_NO_THROW(CheckAssemblyConnectors(b,current,f.frame));
}
std::filesystem::path Actual() {
    const auto* path=std::getenv("ROBO_DYNA_SOURCE_ASSEMBLY_SEVEN_PART_REPLAY_FIXTURE");return path?path:"";
}
TEST(AcceptedReplaySevenPart,CompleteActualBundleStreams959ParentsAnd1093Nodes) {
    if(Actual().empty())GTEST_SKIP()<<"No completed actual seven-part connector archive supplied";
    AcceptedReplay replay;const auto r=replay.Open(Actual());ASSERT_EQ(r.status,ReplayStatus::Ok)<<r.diagnostic;
    const auto& info=*replay.info();ASSERT_TRUE(info.source_assembly);EXPECT_EQ(info.node_count,1093u);
    EXPECT_EQ(info.source_assembly->parents,959u);EXPECT_EQ(info.source_assembly->qeph,845u);EXPECT_EQ(info.source_assembly->t3,114u);
    EXPECT_EQ(info.source_assembly->part_ids.size(),7u);EXPECT_EQ(info.source_assembly->inventory_sha256,source::PinnedYarisSevenPartInventory().sha256);
    EXPECT_NE(info.scope.find("seven-part"),std::string::npos);ASSERT_EQ(replay.Load(info.frame_count-1).status,ReplayStatus::Ok);
    EXPECT_EQ(replay.frame()->parent_plastic_strain.size(),959u);EXPECT_EQ(replay.frame()->epoch,info.final_epoch);
}
TEST(AcceptedReplaySevenPart,RehashedActualConnectorSourceAndAcceptedRecordsCannotChange) {
    if(Actual().empty())GTEST_SKIP()<<"No completed actual seven-part connector archive supplied";
    for(unsigned fault=0;fault<9;++fault) {
        test_support::ModifiedReplayBundle b(Actual());std::string file="configuration.json";auto d=b.Read(file);
        if(fault<4) {
            auto& c=d["input"]["connectors"];
            if(fault==0)c["connections"][0u]["source_node_ids"][1u].SetUint64(0);
            if(fault==1)c["endpoint_contributions"][1u][7u].SetDouble(0);
            if(fault==2)c["properties"][0u]["stiffness"][3u].SetDouble(0);
            if(fault==3)d["input"].EraseMember("connectors");
        } else {
            auto final=b.Read("final-metrics.json");file=b.Frame(final["accepted_epoch"].GetUint64());d=b.Read(file);
            if(fault==4)d["connectors"]["diagnostics"]["attempt"].SetUint64(0);
            if(fault==5)d["connectors"]["elements"][0u]["history"]["internal_work_J"][3u].SetDouble(123);
            if(fault==6)d["connectors"]["elements"][0u]["endpoint_wrenches"][1u][5u].SetDouble(123);
            if(fault==7)d["connectors"]["elements"][0u]["critical_dt_s"].SetDouble(0);
            if(fault==8)d["diagnostics"]["motion"]["after"]["connector_kinetic_J"][1u].SetDouble(-1);
        }
        b.Replace(file,d);b.Rehash(file);AcceptedReplay reader;const auto r=reader.Open(b.directory);
        EXPECT_EQ(r.status,ReplayStatus::InvalidBundle)<<fault<<" "<<r.diagnostic;
    }
}
}
}
