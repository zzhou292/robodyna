#include "PhysicalSource.h"
#include "MovingContactSource.h"
#include "case/vehicle_startup/TiedCinWitnessRoster.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>
#include <cstdlib>
namespace crash::cases::native_scene {
namespace {
modelio::native_scene::DeclaredSource Declaration() {
    const auto* env=std::getenv("ROBO_DYNA_NATIVE_CIN_SCENE_EXPORT");if(!env)throw std::runtime_error("Explicit CIN export required");
    const std::filesystem::path path(env);return modelio::native_scene::DeclaredSource::Read(path,output::Sha256(output::ReadBounded(path,4u<<20)));
}
}
TEST(TiedScenePreparation, ActualPhysicalParentsProduceMInertiaAndCompleteCinModel) {
    const auto physical=PhysicalSource::Prepare(Declaration(),772);
    ASSERT_NE(physical.tied_source(),nullptr);ASSERT_TRUE(physical.cin().prepared());EXPECT_FALSE(physical.cin().explicitly_empty());
    EXPECT_TRUE(physical.rigid().explicitly_empty());ASSERT_EQ(physical.cin().rows().count,4u);
    EXPECT_TRUE(physical.cin().domain()->SharesStorage(*physical.physical().domain()));
    EXPECT_TRUE(physical.cin().SharesStorage(physical.tied_source()->model()));
    for(std::size_t i=0;i<17;++i) {
        const auto& c=physical.physical().coefficients()->nodes()[i].coefficients;
        EXPECT_GT(c.mass,0.);EXPECT_GT(c.isotropic_inertia,0.);
        EXPECT_EQ(physical.translation_fixed_bits()[i],i<9?7:0);
    }
    for(std::size_t row=0;row<4;++row) {
        const auto& a=physical.cin().rows().data[row];EXPECT_EQ(a.secondary_domain_node,13+row);
        EXPECT_EQ(a.master_domain_nodes,(std::array<std::uint32_t,4>{9,10,12,11}));
        EXPECT_EQ(a.master_source.element_id,9u);EXPECT_EQ(a.master_source.part_id,2u);
    }
}
TEST(TiedScenePreparation, NativeValueStagesDeriveEveryDispositionAndUseRealSecondaryThickness) {
    const auto physical=PhysicalSource::Prepare(Declaration(),773);const auto& source=*physical.tied_source();
    const auto* final=source.search().data();ASSERT_NE(final,nullptr);EXPECT_EQ(final->slaves,(std::vector<std::uint32_t>{0,1,2,3}));
    EXPECT_EQ(final->selected_masters,(std::vector<std::uint64_t>{1,1,1,1}));
    ASSERT_EQ(final->messages.size(),7u);
    constexpr unsigned flushes[]{1071,1078,1079,1873,1157,1158,1872};
    for(unsigned i=0;i<7;++i) {
        EXPECT_EQ(final->messages[i].id,flushes[i]);
        EXPECT_EQ(final->messages[i].action,tl::constraints::tied_shell::NativeMessageAction::Flush);
        EXPECT_EQ(final->messages[i].original_slave,SIZE_MAX);
    }
    for(auto disposition:final->dispositions)
        EXPECT_EQ(disposition,tl::constraints::tied_shell::FinalizationDisposition::Kept);
    RecordProperty("retained_secondary_rows",int(final->slaves.size()));
    RecordProperty("retained_main_nodes",int(final->main_nodes.size()));
    RecordProperty("empty_native_flush_records",int(final->messages.size()));
    const auto interfaces=source.classification().interfaces();ASSERT_EQ(interfaces.count,2u);
    EXPECT_FALSE(interfaces.data[0].selected);EXPECT_TRUE(interfaces.data[1].selected);
    const auto flags=source.classification().irupt();const auto offset=interfaces.data[1].slave_offset;
    for(unsigned row=0;row<4;++row) {
        EXPECT_EQ(flags.data[offset+row],0);EXPECT_EQ(source.post_kinchk().slaves().data[row].before.irupt,0);
        EXPECT_EQ(source.search_inputs()[row].master_thickness,1.);
        EXPECT_EQ(source.search_inputs()[row].secondary_shell_thickness,1.);
    }
    // The chosen geometric points differ; no uniform prescribed search S/T is used.
    EXPECT_NE(final->st[0],final->st[1]);EXPECT_NE(final->st[0],final->st[2]);
}
TEST(TiedScenePreparation, GenuinePhysicalWitnessRosterNeedsNoVehicleWrapperOrFakeActivity) {
    const auto physical=PhysicalSource::Prepare(Declaration(),774);const auto* roster=physical.cin_witnesses();
    ASSERT_NE(roster,nullptr);EXPECT_TRUE(roster->runtime_mappable());
    EXPECT_TRUE(roster->model().SharesStorage(physical.cin()));
    EXPECT_EQ(&roster->shells(),physical.physical().shells());
    const auto& d=roster->data();ASSERT_EQ(d.ranges.size(),4u);ASSERT_EQ(d.witnesses.size(),4u);
    EXPECT_EQ(d.counts.declared_parent_witnesses,4u);EXPECT_EQ(d.counts.additional_containing_parents,0u);
    for(unsigned row=0;row<4;++row) {
        EXPECT_EQ(d.ranges[row].offset,row);EXPECT_EQ(d.ranges[row].count,1u);
        EXPECT_EQ(d.witnesses[row].source_element_id,9u);EXPECT_EQ(d.witnesses[row].native_parent_index,0u);
        EXPECT_EQ(d.origins[row].source_part_id,2u);EXPECT_EQ(d.origins[row].canonical_parent,UINT32_MAX);
    }
    EXPECT_THROW(roster->attachments(),std::exception);
    EXPECT_THROW(roster->binding(),std::exception);
}
TEST(TiedScenePreparation, ForeignDomainAndExactWitnessCapAreFailureAtomic) {
    const auto source=PhysicalSource::Prepare(Declaration(),775);const auto foreign=PhysicalSource::Prepare(Declaration(),776);
    namespace v=vehicle_startup;
    EXPECT_THROW(v::TiedCinWitnessRoster::PreparePhysical(source.cin(),foreign.physical(),{8u<<20,4}),std::exception);
    const auto forecast=v::TiedCinWitnessRoster::ForecastPhysical(source.cin(),source.physical(),{8u<<20,4});
    EXPECT_NO_THROW(v::TiedCinWitnessRoster::PreparePhysical(source.cin(),source.physical(),{forecast.total_host_bytes,4}));
    EXPECT_THROW(v::TiedCinWitnessRoster::PreparePhysical(source.cin(),source.physical(),{forecast.total_host_bytes-1,4}),std::exception);
    EXPECT_THROW(TiedSource::Prepare(source.declared(),*source.physical().domain(),{1}),std::exception);
    EXPECT_EQ(source.cin().rows().count,4u);
}
TEST(TiedScenePreparation, ShippingContactAndTrajectoryAdmissionRemainClosed) {
    const auto source=PhysicalSource::Prepare(Declaration(),777);
    EXPECT_THROW(MovingContactSource::Prepare(source,{778,779,1}),std::exception);
    EXPECT_EQ(source.cin().rows().count,4u);EXPECT_GT(source.retained_host_upper_bound(),0u);
}
}
