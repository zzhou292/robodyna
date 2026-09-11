#include "../Internal.h"
#include "../../tied_classification/tests/ActualClassification.h"
#include "modelio/vehicle_sections/tests/GlassSourceSupport.h"
#include <iostream>
#include <set>
#ifdef ROBO_DYNA_CIN_ACTIVITY_CHECK
#include "../../TiedCinWitnessActivity.h"
#endif

namespace crash::cases::vehicle_startup::cin_actual_test {
const TiedCinAttachments& Prepared();
}
namespace crash::cases::vehicle_startup::cin_witness_actual_test {
const VehicleShellBinding& Binding() {
    static const auto value=[] {
        const auto& cin=cin_actual_test::Prepared();
        const auto& source=cin.post_kinchk().classification().context().rigid().source();
        namespace vehicle=modelio::vehicle;
        const auto& bytes=vehicle::test::GlassBytes();
        const auto glass=VehicleSectionResolution::ReadBytes(source,bytes,vehicle::test::Identity(bytes));
        const auto midlayer=VehicleSectionResolution::ResolveOriginalMidlayer(glass,vehicle::ResolutionProfile::OriginalMidlayerV1);
        const auto rigid=VehicleSectionResolution::ResolveOriginalRigidParts(midlayer,
            classification_actual_test::Member("main",42846753),vehicle::ResolutionProfile::OriginalRigidPartsV1);
        const auto references=VehicleShellReferences::Prepare(rigid);
        std::cout<<"Original complete shell-binding reservation "<<ForecastShellBinding(references).total_bytes<<" bytes\n";
        return VehicleShellBinding::Prepare(references);
    }();
    return value;
}
const TiedCinWitnessRoster& Prepared() {
    static const auto value=[] {
        const auto& cin=cin_actual_test::Prepared();
        const auto& binding=Binding();
        std::cout<<"Original CIN shell-witness preflight "
                 <<TiedCinWitnessRoster::Forecast(cin,binding).total_host_bytes<<" bytes\n";
        return TiedCinWitnessRoster::Prepare(cin,binding);
    }();
    return value;
}
TEST(TiedCinWitnessActual, AllOriginalAttachmentsRetainEverySourceShellContainmentWitness) {
    const auto& value=Prepared();
    const auto& data=value.data();
    ASSERT_EQ(data.counts.source_parents,349645u);
    ASSERT_EQ(data.counts.attachments,11165u);
    EXPECT_EQ(data.counts.rows_without_shell_witness,0u);
    EXPECT_EQ(data.counts.declared_parent_witnesses,11165u);
    const auto& rows=value.attachments().model().rows();
    const auto& domain=*value.attachments().model().domain();
    const auto& refs=value.binding().references();
    std::size_t triangles=0,triangle_quad_witnesses=0;
    for (std::size_t r=0;r<rows.count;++r) {
        const auto& range=data.ranges[r];
        const auto& patch=rows.data[r];
        const bool triangle=patch.topology==native_search::CinMasterTopology::TriangleRepeatedThird;
        triangles+=triangle;
        ASSERT_GT(range.count,0u)<<r;
        std::set<std::uint64_t> ids;
        std::uint32_t prior=0;
        bool first=true;
        for (std::size_t i=range.offset;i<range.offset+range.count;++i) {
            const auto& witness=data.witnesses[i];
            const auto& origin=data.origins[i];
            ASSERT_TRUE(ids.insert(witness.source_element_id).second)<<r;
            if (!first) EXPECT_GT(origin.source_parent_row,prior)<<r;
            prior=origin.source_parent_row;
            first=false;
            const auto& original=refs.rows().at(origin.source_parent_row);
            EXPECT_EQ(witness.source_element_id,original.element_id);
            EXPECT_EQ(origin.source_part_id,original.part_id);
            EXPECT_EQ(origin.canonical_parent,original.canonical_parent);
            EXPECT_EQ(witness.native_parent_index,original.reference_index);
            for (const auto node:patch.master_domain_nodes) {
                const auto nid=domain.nodes()[node].source_id;
                EXPECT_NE(std::find(origin.source_node_ids.begin(),origin.source_node_ids.end(),nid),
                          origin.source_node_ids.end());
            }
            triangle_quad_witnesses+=triangle && witness.family==cin_stage::WitnessFamily::ShellQuad;
        }
        EXPECT_EQ(ids.count(patch.master_source.element_id),1u)<<r;
    }
    EXPECT_EQ(triangles,125u);
    RecordProperty("complete_source_parents",std::to_string(data.counts.source_parents));
    RecordProperty("source_shell_witnesses",std::to_string(data.counts.witnesses));
    RecordProperty("maximum_witnesses_per_attachment",std::to_string(data.counts.maximum_per_row));
    RecordProperty("unmapped_witness_domain_slots",std::to_string(data.counts.missing_domain_slots));
    RecordProperty("triangle_quad_containment_witnesses",std::to_string(triangle_quad_witnesses));
    RecordProperty("runtime_roster_mappable",value.runtime_mappable() ? "true" : "false");
    RecordProperty("complete_witness_forecast_bytes",std::to_string(value.forecast().total_host_bytes));
}
TEST(TiedCinWitnessActual, ExactCapsLifetimeAndLateCountFailureLeaveImmutableRosterIntact) {
    const auto& saved=Prepared();
    auto copy=saved;
    auto moved=std::move(copy);
    EXPECT_EQ(copy.data().witnesses.data(),moved.data().witnesses.data());
    TiedCinWitnessLimits limits;
    limits.host_bytes=saved.forecast().total_host_bytes-1;
    EXPECT_THROW(TiedCinWitnessRoster::Prepare(saved.attachments(),saved.binding(),limits),std::exception);
    ++limits.host_bytes;
    EXPECT_EQ(TiedCinWitnessRoster::Forecast(saved.attachments(),saved.binding(),limits).total_host_bytes,limits.host_bytes);
    limits.witnesses=saved.data().counts.witnesses-1;
    EXPECT_THROW(TiedCinWitnessRoster::Prepare(saved.attachments(),saved.binding(),limits),std::exception);
    limits.witnesses=saved.data().counts.witnesses;
    const auto retry=TiedCinWitnessRoster::Prepare(saved.attachments(),saved.binding(),limits);
    ASSERT_EQ(retry.data().witnesses.size(),saved.data().witnesses.size());
    EXPECT_EQ(retry.data().witnesses.back().source_element_id,saved.data().witnesses.back().source_element_id);
    EXPECT_EQ(retry.data().origins.back().source_parent_row,saved.data().origins.back().source_parent_row);
}
#ifdef ROBO_DYNA_CIN_ACTIVITY_CHECK
TEST(TiedCinWitnessActual, LiveActivityWorkspaceCannotInventVirginOwnerStateOrUploadToAnUnadmittedOwner) {
    const auto& roster=Prepared();
    const auto forecast=TiedCinWitnessActivity::Forecast(roster);
    TiedCinActivityLimits limits;
    limits.workspace_bytes=forecast.workspace_bytes-1;
    EXPECT_THROW(TiedCinWitnessActivity::Create(roster,limits),std::exception);
    ++limits.workspace_bytes;
    auto activity=TiedCinWitnessActivity::Create(roster,limits);
    EXPECT_FALSE(activity->has_accepted_activity());
    EXPECT_EQ(activity->accepted_flags().count,0u);
    tl::fea::FENodalState owner;
    tl::fea::ShellBatchPublication publication;
    EXPECT_EQ(activity->CaptureAccepted(owner,publication,{}).status,TiedCinActivityStatus::StaleOwner);
    EXPECT_FALSE(activity->has_accepted_activity());
    tl::fea::NodalTrialToken token;
    EXPECT_EQ(activity->UploadAttempt(owner,token).status,TiedCinActivityStatus::StaleOwner);
    EXPECT_EQ(activity->accepted_flags().count,0u);
    EXPECT_EQ(activity->forecast().workspace_bytes,forecast.workspace_bytes);
    RecordProperty("activity_workspace_bytes",std::to_string(forecast.workspace_bytes));
    RecordProperty("activity_complete_host_reservation",std::to_string(forecast.total_host_bytes));
}
#endif

}
