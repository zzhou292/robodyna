#include "../Internal.h"
#include "../../tied_search/tests/ActualGeometry.h"
#include "modelio/vehicle_source/tests/TestSupport.h"
#include "lib_utest/qualification/tied_shell_classification/NativeOracle.h"
#include <iostream>

namespace crash::cases::vehicle_startup::classification_actual_test {
namespace {
const TiedSearchFinalized& Finalized() {
    static const auto value = TiedSearchFinalized::Prepare(TiedSearchAssessment::Prepare(test::OriginalTiedGeometry()));
    return value;
}
const std::string& Member(const char* key, std::size_t cap) {
    // Separate immutable fixture values; no fallback to another input file.
    const auto read = [](const char* variable, std::size_t bytes) {
        const char* path = std::getenv(variable);
        output::Require(path && *path,"Missing explicit original classification fixture");
        return output::ReadBounded(path,bytes);
    };
    static const auto main = read("ROBO_STATIC_MEMBER",42846753);
    static const auto auxiliary = read("ROBO_TIED_AUX_MEMBER",44991);
    static const auto wall = read("ROBO_TIED_WALL_MEMBER",10604);
    output::Require(std::string(key) == "main" || std::string(key) == "auxiliary" || std::string(key) == "wall",
                    "Unknown original classification fixture role");
    const auto& value = std::string(key) == "main" ? main : std::string(key) == "auxiliary" ? auxiliary : wall;
    output::Require(value.size() == cap,"Original classification fixture extent differs");
    return value;
}
const tied::TiedClassificationContext& Context() {
    static const auto value = [] {
        const auto& declaration = Finalized().assessment().geometry().packing().declaration();
        namespace vehicle = modelio::vehicle;
        const auto plan = vehicle::VehicleSourcePlan::ReadBytes(declaration.canonical(),
            vehicle::test::Bytes(),vehicle::test::Identity(vehicle::test::Bytes()));
        const auto rigid = vehicle::rigid_part::RigidPartSource::Prepare(plan,Member("main",42846753));
        const auto auxiliary = tied::TiedAuxiliaryConstraints::Prepare(declaration,Member("auxiliary",44991),
            tied::OriginalWallPolicy::ReplaceWithMeshWall);
        std::cout << "Rigid source startup reservation " << rigid.data().startup_budget_bytes
                  << "; projected source preflight "
                  << tied::TiedClassificationContext::Forecast(auxiliary,rigid,10604) << " bytes\n";
        return tied::TiedClassificationContext::Prepare(auxiliary,rigid,Member("wall",10604),
            tied::OriginalWallAssemblyPolicy::ReplaceWholeOriginalWallWithMeshWall);
    }();
    return value;
}
const TiedSearchClassification& Prepared() {
    static const auto value = [] {
        const auto forecast = TiedSearchClassification::Forecast(Finalized(),Context());
        std::cout << "Complete classification preflight " << forecast.total_host_bytes << " bytes\n";
        return TiedSearchClassification::Prepare(Finalized(),Context());
    }();
    return value;
}
}
TEST(TiedClassificationActual, CompleteOriginalReadsetAndEveryObservedNativeClassificationAgree) {
    const auto& value = Prepared();
    const auto& receipt = value.context().receipt();
    EXPECT_EQ(receipt.observed_slaves,11165u);
    EXPECT_EQ(receipt.plain_groups,759u);
    EXPECT_EQ(receipt.auxiliary_groups,11u);
    EXPECT_EQ(receipt.rigid_parts,22u);
    EXPECT_EQ(receipt.joints,44u);
    EXPECT_EQ(receipt.original_type2_interfaces,1u);
    EXPECT_EQ(receipt.replaced_primitive_walls,6u);
    ASSERT_EQ(receipt.wall.node_ids.size(),62u);
    ASSERT_EQ(receipt.wall.shell_ids.size(),46u);
    EXPECT_EQ(receipt.wall.node_ids.front(),1001u);
    EXPECT_EQ(receipt.wall.node_ids.back(),1062u);
    EXPECT_EQ(receipt.wall.shell_ids.front(),1001u);
    EXPECT_EQ(receipt.wall.shell_ids.back(),1046u);
    EXPECT_EQ(receipt.wall.sha256,"ef02a4701b37d27cec81b1f9a02ab555f55ac61f68b070e8b0c18dc23b1d5155");
    const auto& geometry = Finalized().assessment().geometry();
    const auto& declaration = geometry.packing().declaration();
    EXPECT_EQ(&declaration.data(),&Context().auxiliary().declaration().data());
    EXPECT_EQ(&declaration.canonical().data(),&Context().rigid().source().canonical().data());
    auto input = tied_classification_detail::Pack(declaration.canonical().data(),declaration.data(),
        geometry.data(),Finalized().data(),Finalized().receipt());
    const auto native = ::classification_test::NativeClassify(input.View());
    const auto& d = value.data();
    ASSERT_EQ(d.slaves.size(),11165u);
    ASSERT_EQ(native.irupt.size(),d.slaves.size());
    ASSERT_EQ(native.five_blocks.size(),5*input.nodes.size());
    ASSERT_EQ(native.itf.size(),8192u);
    std::size_t cin_count = 0, penalty_count = 0;
    for (std::size_t s = 0; s < d.slaves.size(); ++s) {
        const auto& slave = d.slaves[s];
        const auto n = input.slaves[s];
        const auto count = input.nodes.size();
        EXPECT_EQ(slave.original_nsv_row,Finalized().data().slaves[s]);
        EXPECT_EQ(slave.source_node_id,Finalized().secondary(s).id);
        EXPECT_EQ(slave.irupt,native.irupt[s]);
        EXPECT_EQ(slave.kinematics.conditions,native.five_blocks[n]);
        EXPECT_EQ(slave.kinematics.translation,native.five_blocks[count+n]);
        EXPECT_EQ(slave.kinematics.rotation,native.five_blocks[2*count+n]);
        EXPECT_EQ(slave.kinematics.duplicate_conditions,native.five_blocks[3*count+n]);
        EXPECT_EQ(slave.kinematics.incompatible_conditions,native.five_blocks[4*count+n]);
        if (native.irupt[s] == 0) ++cin_count;
        else ++penalty_count;
    }
    EXPECT_EQ(d.cin_count,cin_count);
    EXPECT_EQ(d.penalty_count,penalty_count);
    for (std::size_t word = 0; word < 8192; ++word) EXPECT_EQ(d.interface_decode[word],native.itf[word]);
    EXPECT_EQ(d.kinset_warning_count,static_cast<unsigned>(native.warnings));
    EXPECT_EQ(d.penalty_warning_count,static_cast<unsigned>(native.penalties));
    EXPECT_EQ(d.phase,native_search::ClassificationPhase::InterfaceTaggedBeforeKinChk);
    EXPECT_EQ(value.post_kinchk(),TiedAssessmentReadiness::Pending);
    RecordProperty("rigid_source_startup_bytes",std::to_string(Context().rigid().data().startup_budget_bytes));
    RecordProperty("source_context_startup_bytes",std::to_string(receipt.startup_budget_bytes));
    RecordProperty("classification_forecast_bytes",std::to_string(value.forecast().total_host_bytes));
    RecordProperty("classification_cin_count",std::to_string(cin_count));
    RecordProperty("classification_penalty_count",std::to_string(penalty_count));
    std::cout << "Original classification CIN " << cin_count << "; PEN " << penalty_count
              << "; source reservation " << receipt.startup_budget_bytes << "; complete forecast "
              << value.forecast().total_host_bytes << "; owned result " << d.owned_payload_bytes << " bytes\n";
}
TEST(TiedClassificationActual, CompleteCapsLateWallFailureAndImmutableRetry) {
    const auto& saved = Prepared();
    auto copy = saved;
    auto moved = std::move(copy);
    EXPECT_EQ(&copy.data(),&moved.data());
    const auto* prior = &saved.data();
    tied::ClassificationSourceLimits source_cap;
    source_cap.host_bytes = Context().receipt().startup_budget_bytes-1;
    EXPECT_THROW(tied::TiedClassificationContext::Forecast(Context().auxiliary(),Context().rigid(),10604,source_cap),std::exception);
    ++source_cap.host_bytes;
    EXPECT_EQ(tied::TiedClassificationContext::Forecast(Context().auxiliary(),Context().rigid(),10604,source_cap),source_cap.host_bytes);
    auto wall = Member("wall",10604);
    wall.back() = 'x';
    EXPECT_THROW(tied::TiedClassificationContext::Prepare(Context().auxiliary(),Context().rigid(),wall,
        tied::OriginalWallAssemblyPolicy::ReplaceWholeOriginalWallWithMeshWall),std::exception);
    TiedClassificationLimits cap;
    cap.host_bytes = saved.forecast().total_host_bytes-1;
    EXPECT_THROW(TiedSearchClassification::Prepare(Finalized(),Context(),cap),std::exception);
    ++cap.host_bytes;
    EXPECT_EQ(TiedSearchClassification::Forecast(Finalized(),Context(),cap).total_host_bytes,cap.host_bytes);
    cap.native.max_host_bytes = 1;
    EXPECT_THROW(TiedSearchClassification::Prepare(Finalized(),Context(),cap),std::exception);
    EXPECT_EQ(&saved.data(),prior);
    cap.native.max_host_bytes = TiedClassificationLimits{}.native.max_host_bytes;
    const auto retry = TiedSearchClassification::Prepare(Finalized(),Context(),cap);
    EXPECT_EQ(retry.data().interface_decode,saved.data().interface_decode);
    EXPECT_EQ(retry.data().slaves.back().source_node_id,saved.data().slaves.back().source_node_id);
    EXPECT_EQ(retry.data().slaves.back().irupt,saved.data().slaves.back().irupt);
}
}
