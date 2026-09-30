#include "AcceptedReplay.h"
#include "AcceptedReplaySourceAssembly.h"
#include "AcceptedReplayCsv.h"
#include "ReplayBundleTestSupport.h"
#include "source_assembly/SourceAssemblyWallSchema.h"
#include <iomanip>
#include <sstream>
#include <gtest/gtest.h>
#include <cstdlib>

namespace crash::output {
namespace {
using test_support::ModifiedReplayBundle;
std::filesystem::path Source() {
    const auto* path=std::getenv("ROBO_DYNA_SOURCE_ASSEMBLY_REPLAY_FIXTURE");
    Require(path&&*path,"ROBO_DYNA_SOURCE_ASSEMBLY_REPLAY_FIXTURE must name a completed actual-source owner archive");return path;
}
std::uint64_t FirstEpoch(const ModifiedReplayBundle& b) {
    std::istringstream input(ReadBounded(b.directory/"accepted-frames.csv",32*1024*1024));std::string line;
    for(unsigned row=0;row<3;++row)Require(bool(std::getline(input,line)),"Missing initial/first accepted frame");
    return replay_detail::ReplayCsvNumber<std::uint64_t>(replay_detail::ReplayCsvRow<5>(line)[1]);
}
void Reject(const ModifiedReplayBundle& b,unsigned fault) {
    AcceptedReplay reader;const auto report=reader.Open(b.directory);
    EXPECT_EQ(report.status,ReplayStatus::InvalidBundle)<<fault<<" "<<report.diagnostic;
}
TEST(AcceptedReplaySourceAssembly,CompleteOriginalSourceAndBothNativeFamiliesStreamAtPhysicalScale) {
    AcceptedReplay reader;const auto report=reader.Open(Source());ASSERT_EQ(report.status,ReplayStatus::Ok)<<report.diagnostic;
    const auto& info=*reader.info();ASSERT_EQ(info.kind,ReplayKind::SourceAssemblyWall);ASSERT_TRUE(info.source_assembly);
    const auto& source=*info.source_assembly;EXPECT_EQ(source.inventory_bytes,1731843u);
    EXPECT_EQ(source.inventory_sha256,modelio::assembly::PinnedYarisSixPartInventory().sha256);
    EXPECT_EQ(source.part_ids,(std::vector<std::uint64_t>{2000119,2000120,2000145,2000157,2000165,2000260}));
    EXPECT_EQ(source.material_ids.size(),6u);EXPECT_EQ(source.section_ids.size(),6u);EXPECT_EQ(source.curve_ids.size(),2u);
    EXPECT_EQ(source.parents,915u);EXPECT_EQ(source.qeph,804u);EXPECT_EQ(source.t3,111u);EXPECT_EQ(source.groups,6u);EXPECT_EQ(source.members,76u);
    EXPECT_EQ(info.node_count,1030u);EXPECT_EQ(info.triangle_count,1719u);EXPECT_EQ(info.triangle_source_parent.size(),1719u);
    EXPECT_TRUE(info.material_model.empty());EXPECT_TRUE(info.material_policy.empty());ASSERT_TRUE(reader.wall());
    ASSERT_EQ(reader.frame()->parent_plastic_strain.size(),915u);
    for(const auto& p:reader.frame()->parent_plastic_strain)EXPECT_EQ(p.value,0);
    ASSERT_EQ(reader.Load(info.frame_count-1).status,ReplayStatus::Ok);
    EXPECT_EQ(reader.frame()->epoch,info.final_epoch);EXPECT_EQ(Bits(reader.frame()->time),Bits(info.final_time));
    ASSERT_EQ(reader.frame()->parent_plastic_strain.size(),915u);
    const auto source_model=modelio::assembly::SourceAssembly::Read(Source()/"source-assembly-inventory.json",modelio::assembly::PinnedYarisSixPartInventory());
    for(std::size_t i=0;i<915;++i)EXPECT_EQ(reader.frame()->parent_plastic_strain[i].source_parent,source_model.data().parents[i].source_id);
}
TEST(AcceptedReplaySourceAssembly,RehashedCompleteSourceMaterialBoundaryAndGroupFaultsAreRejected) {
    for(unsigned fault=0;fault<7;++fault) {
        ModifiedReplayBundle b(Source());auto c=b.Read("configuration.json");auto& input=c["input"];
        if(fault==0)c["vertex_binding"][1029][3].SetUint64(0);
        if(fault==1)c["triangle_binding"][1718][5].SetUint64(0);
        if(fault==2)input["materials"][5][9].SetDouble(0);
        if(fault==3)input["sections"][5][1].SetUint64(99);
        if(fault==4)input["curves"][1][2][45].SetDouble(1);
        if(fault==5)input["internal_rigid_groups"][5]["members"][0][1].SetUint64(0);
        if(fault==6)input["boundary"]["external_node_ids"].PopBack();
        b.Replace("configuration.json",c);b.Rehash("configuration.json");Reject(b,fault);
    }
}
TEST(AcceptedReplaySourceAssembly,OriginalInventoryCannotBeReplacedByRehashingTheBundle) {
    ModifiedReplayBundle b(Source());const auto bytes=ReadBounded(b.directory/"source-assembly-inventory.json",32*1024*1024);
    b.ReplaceBytes("source-assembly-inventory.json",bytes+" ");b.Rehash("source-assembly-inventory.json");Reject(b,0);
}
TEST(AcceptedReplaySourceAssembly,RehashedLateParentLayerAndAcceptedPhaseFaultsAreRejected) {
    for(unsigned fault=0;fault<7;++fault) {
        ModifiedReplayBundle b(Source());const auto file=b.Frame(FirstEpoch(b));auto f=b.Read(file);
        if(fault==0)f["stamp"]["velocity_phase"].SetString("collocated",f.GetAllocator());
        if(fault==1)f["stamp"]["rigid_groups"]["member_count"].SetUint64(75);
        if(fault==2)f["sections"]["source_parents"][914][8].SetUint64(0);
        if(fault==3)f["sections"]["sections"][914][9][2][5].SetDouble(1);
        if(fault==4)f["sections"]["point_columns"].SetString("stress_XX_Pa,stress_YY_Pa,stress_YZ_Pa,stress_XY_Pa,stress_ZX_Pa,equivalent_plastic_strain,filtered_rate_per_s",f.GetAllocator());
        if(fault==5)f["diagnostics"]["motion"]["after"]["phase"]["frame_time_s"].SetDouble(f["accepted_time_s"].GetDouble());
        if(fault==6)f["nodal_fields"]["orientation_wxyz"][4119].SetDouble(2);
        b.Replace(file,f);b.Rehash(file);Reject(b,fault);
    }
}
TEST(AcceptedReplaySourceAssembly,CompleteContactAndInitialSeparationCannotBeRelabelled) {
    for(unsigned fault=0;fault<5;++fault) {
        ModifiedReplayBundle b(Source());const auto file=b.Frame(fault==0?0:FirstEpoch(b));auto f=b.Read(file);
        if(fault==0)f["contact"].SetObject();
        if(fault==1)f["contact"]["nodes"][1029][1].SetUint64(0);
        if(fault==2)f["contact"]["parents"][914][4].SetUint64(0);
        if(fault==3)f["contact"]["nodes"][1029][7][1].SetDouble(100);
        if(fault==4)f["contact"]["attempt"].SetUint64(0);
        b.Replace(file,f);b.Rehash(file);Reject(b,fault);
    }
}
TEST(AcceptedReplaySourceAssembly,ExplicitByteFrameAndPrefixCapsRemainBounded) {
    for(unsigned fault=0;fault<5;++fault) {
        ModifiedReplayBundle b(Source());auto c=b.Read("configuration.json");
        if(fault==0)c["archive_byte_cap"].SetUint64(1024ull*1024*1024+1);
        if(fault==1)c["artifact_file_byte_cap"].SetUint64(32ull*1024*1024+1);
        if(fault==2)c["frame_cap"].SetUint64(1001);
        if(fault==3)c["frame_every"].SetUint64(c["frame_every"].GetUint64()+1);
        if(fault==4)c["forecast_bytes"].SetUint64(c["forecast_bytes"].GetUint64()-1);
        b.Replace("configuration.json",c);b.Rehash("configuration.json");Reject(b,fault);
    }
}
TEST(AcceptedReplaySourceAssembly,UndeclaredCanonicalMeshCannotOverrideTheAuthenticatedPlacedWall) {
    ModifiedReplayBundle b(Source());const auto initial=b.Frame(0);
    const auto mesh=initial.substr(0,initial.size()-12)+".mesh.json";
    const auto bytes=ReadBounded(b.directory/mesh,32*1024*1024);b.ReplaceBytes("canonical-wall.mesh.json",bytes);
    auto m=b.Read("manifest.json");Document item;item.SetObject();String(item,"file","canonical-wall.mesh.json");
    String(item,"sha256",Sha256(bytes));Integer(item,"bytes",bytes.size());Value value;value.CopyFrom(item,m.GetAllocator());
    m["artifacts"].PushBack(value,m.GetAllocator());b.Replace("manifest.json",m);Reject(b,0);
}
TEST(AcceptedReplaySourceAssembly,RehashedContactArithmeticAndPartitionDiagnosticsAreChecked) {
    for(unsigned fault=0;fault<7;++fault) {
        ModifiedReplayBundle b(Source());const auto file=b.Frame(FirstEpoch(b));auto f=b.Read(file);
        if(fault==0)f["contact"]["nodes"][1029][9][0].SetDouble(1);
        if(fault==1)f["contact"]["nodes"][1029][12].SetBool(!f["contact"]["nodes"][1029][12].GetBool());
        if(fault==2)f["contact"]["potential_increment_J"].SetDouble(1);
        if(fault==3)f["contact"]["nodes"][1029][10].SetDouble(1);
        if(fault==4)f["diagnostics"]["motion"]["after"]["grouped_native_members"][2].SetDouble(1);
        if(fault==5)f["diagnostics"]["motion"]["after"]["aggregate_groups"][9].SetDouble(1);
        if(fault==6)f["diagnostics"]["motion"]["native_residual_J"].SetDouble(1);
        b.Replace(file,f);b.Rehash(file);Reject(b,fault);
    }
}

TEST(AcceptedReplaySourceAssembly,DeclaredEightMiBFileCapDrivesSegmentedIntervalParsing) {
    // Synthetic CSV contract only: no mesh, accepted owner or mechanical result.
    namespace rd=replay_detail;ModifiedReplayBundle files(Source());
    rd::Bundle b;b.directory=files.directory;b.info.owner_id=7;b.info.node_count=1030;b.info.final_epoch=10000;
    b.fixed_dt=1./65536;b.info.final_time=10000*b.fixed_dt;b.wall_penetration_cap=.002;
    b.assembly=std::make_shared<rd::AssemblyReplayData>(modelio::assembly::SourceAssembly::Read(
        Source()/"source-assembly-inventory.json",modelio::assembly::PinnedYarisSixPartInventory()));
    auto& a=*b.assembly;a.requested_steps=10000;a.maximum_rotation=1;a.maximum_area_ratio=2;a.maximum_thickness_ratio=2;
    b.entries={{7,0,0,"",""},{7,10000,b.info.final_time,"",""}};
    constexpr std::size_t cap=8*1024*1024;
    const auto plan=PlanCsvLedger("contract-intervals.csv",assembly::WallIntervalHeader,10000,assembly::WallIntervalColumns*26,cap);
    ASSERT_EQ(plan.segments.size(),2u);
    // The reader requires the canonical logical name, while this fixture clone
    // owns private replacement entries and never changes the retained archive.
    auto canonical=PlanCsvLedger("accepted-intervals.csv",assembly::WallIntervalHeader,10000,assembly::WallIntervalColumns*26,cap);
    for(const auto& segment:canonical.segments)std::filesystem::remove(files.directory/segment.file);
    CsvLedgerWriter writer(files.directory,assembly::WallIntervalHeader,canonical,cap);
    for(std::uint64_t e=1;e<=10000;++e) {
        const double base=(e-1)*b.fixed_dt;std::ostringstream row;row<<std::setprecision(17);
        row<<7<<','<<e-1<<','<<e<<','<<base<<','<<e<<','<<e*b.fixed_dt;
        std::array<double,33> v{};v[0]=base+.5*b.fixed_dt;v[1]=e==1?.5*b.fixed_dt:b.fixed_dt;v[31]=v[32]=1;
        for(double x:v)row<<','<<x;row<<'\n';writer.Append(e,row.str());
    }
    writer.Finish();
    for(const auto& segment:canonical.segments) {
        const auto bytes=ReadBounded(files.directory/segment.file,cap);b.inventory.emplace(segment.file,rd::Artifact{Sha256(bytes),bytes.size()});
    }
    Document config,manifest;config.SetObject();manifest.SetObject();Integer(config,"artifact_file_byte_cap",cap);
    AppendCsvLedgerSegments(config,&canonical,1);AppendCsvLedgerSegments(manifest,&canonical,1);
    EXPECT_NO_THROW(rd::ReadAssemblyIntervals(b,config,manifest));EXPECT_EQ(b.entries.back().interval_attempt,10000u);
    config["artifact_file_byte_cap"].SetUint64(32*1024*1024);
    EXPECT_THROW(rd::ReadAssemblyIntervals(b,config,manifest),std::runtime_error);
}

TEST(AcceptedReplaySourceAssembly,FailedLoadAndRehashedOpenPreserveCompletePublishedFrame) {
    ModifiedReplayBundle b(Source());AcceptedReplay reader;const auto report=reader.Open(b.directory);
    ASSERT_EQ(report.status,ReplayStatus::Ok)<<report.diagnostic;
    ASSERT_EQ(reader.Load(1).status,ReplayStatus::Ok);const auto old=*reader.frame();const auto info=reader.info()->source_assembly;
    const auto file=b.Frame(reader.frame()->epoch);auto f=b.Read(file);f["stamp"]["reaction_base_epoch"].SetUint64(reader.frame()->epoch+1);
    b.Replace(file,f);EXPECT_EQ(reader.Load(1).status,ReplayStatus::InvalidFrame);EXPECT_EQ(reader.frame()->mesh,old.mesh);
    b.Rehash(file);EXPECT_EQ(reader.Open(b.directory).status,ReplayStatus::InvalidBundle);
    EXPECT_EQ(reader.frame()->mesh,old.mesh);EXPECT_EQ(reader.frame()->epoch,old.epoch);EXPECT_EQ(reader.info()->source_assembly,info);
}
} // namespace
} // namespace crash::output
