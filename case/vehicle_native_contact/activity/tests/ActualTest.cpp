#include "../Declaration.h"
#include "../../source/OriginalSources.h"
#include <gtest/gtest.h>
#include <cstdlib>
#include <algorithm>
#include <iostream>
namespace crash::cases::vehicle_native_contact::activity {
namespace {
std::string Required(const char* key) {
    const auto* value=std::getenv(key);output::Require(value&&*value,"Missing explicit activity source fixture input");return value;
}
vehicle_run::OriginalPaths Paths() {
    return {Required("ROBO_STATIC_CANONICAL"),Required("ROBO_STATIC_SCOPE"),Required("ROBO_STATIC_MEMBER"),
        Required("ROBO_VEHICLE_DECLARATIONS"),Required("ROBO_VEHICLE_GLASS_RESOLUTION"),Required("ROBO_DYNA_TYPE13_DECLARATION"),
        Required("ROBO_SELF_CONTACT_AUX_MEMBER"),Required("ROBO_TIED_WALL_MEMBER"),Required("ROBO_NATIVE_WALL_MANIFEST"),
        Required("ROBO_SELF_CONTACT_COMBINE_MEMBER")};
}
}
TEST(NativeActivitySourceActual, AuthenticatedV6CoverageAndSourcePlanForecastNeedNoPhysicalOwner) {
    const auto bytes=Required("ROBO_NATIVE_SOLID_PACKETS_BYTES");
    output::Require(bytes.find_first_not_of("0123456789")==std::string::npos,"Packet size must be decimal");
    const modelio::solid_control_packets::Artifact artifact{Required("ROBO_NATIVE_SOLID_PACKETS"),std::stoull(bytes),
        Required("ROBO_NATIVE_SOLID_PACKETS_SHA256"),"native_v6_raw8_heph_explicit_cin28"};
    const auto original=source::OriginalSources::Prepare(Paths(),artifact);const auto inputs=original.inputs();
    const auto* post=inputs.self.snapshot().post_gapm;ASSERT_NE(post,nullptr);
    std::cout<<"V6 source reader_idel="<<inputs.controls.raw_controls().reader_idel
        <<" pre_shell_internal="<<post->pre_shell_internal_count
        <<" incoming_erosion="<<static_cast<unsigned>(post->incoming_solid_erosion)
        <<" final_erosion="<<static_cast<unsigned>(post->final_solid_erosion)<<std::endl;
    const auto declaration=Declaration::Prepare(inputs);const auto& c=declaration.coverage();
    EXPECT_EQ(c.physical_nodes,376934u);EXPECT_EQ(c.shells.executed,349645u);EXPECT_EQ(c.solids.executed,4980u);
    EXPECT_EQ(c.type13,4442u);EXPECT_EQ(c.beam18,142u);EXPECT_EQ(c.welds,2828u);EXPECT_EQ(c.joints,44u);
    namespace n=tlfea::contact::radioss_type25;
    n::MixedMovingMainSource self;self.starter=inputs.self.snapshot();
    // SourcePlan consumes identity/connectivity only. This test transcribes the
    // genuine two-side wall declaration, not a runtime contact/history source.
    const auto& starter=inputs.wall.starter();ASSERT_EQ(starter.main_count,2u);ASSERT_EQ(starter.primary_count,1u);
    std::array<n::lifecycle::Main,2> mains;
    for(unsigned i=0;i<2;++i) {
        mains[i].global_id=starter.mains[i].global_id;mains[i].segment_type=starter.mains[i].segment_type;
        std::copy_n(starter.mains[i].nodes,4,mains[i].nodes);
    }
    const auto wall_id=inputs.wall.wall().ids().shell;n::FixedMainSource wall;
    wall.selection.mains=mains.data();wall.selection.main_count=2;wall.selection.node_count=starter.node_count;
    wall.selection.generation=starter.source_generation;wall.primary_main_count=1;wall.primary_parent_ids=&wall_id;
    const auto forecasts=PreflightPlans(inputs,declaration,self,wall);
    auto document=Document(declaration);
    for(unsigned i=0;i<2;++i) {
        const auto prefix=i?"wall_plan_":"self_plan_";
        output::Integer(document,(std::string(prefix)+"output_bytes").c_str(),forecasts[i].output_bytes);
        output::Integer(document,(std::string(prefix)+"startup_bytes").c_str(),forecasts[i].startup_bytes);
        output::Integer(document,(std::string(prefix)+"parents").c_str(),forecasts[i].counts.parents);
        output::Integer(document,(std::string(prefix)+"node_incidence").c_str(),forecasts[i].counts.incidence);
        output::Integer(document,(std::string(prefix)+"emitting_capacity").c_str(),forecasts[i].counts.emitting_capacity);
    }
    output::Boolean(document,"physical_owner_created",false);
    output::WriteJson(Required("ROBO_NATIVE_ACTIVITY_REPORT"),document);
}
}
