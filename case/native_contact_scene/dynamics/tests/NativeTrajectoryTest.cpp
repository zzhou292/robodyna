#include "NativeSceneAccess.h"
#include "../../ArchiveSource.h"
#include "output/full_shell/tests/TestSupport.h"
#include "lib_utest/qualification/radioss_type25_runtime/Reference.h"
#include "ObservedScene.h" // Expected observations only, never factory inputs.
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <vector>
namespace crash::cases::native_scene {
namespace {
using Access=qualification::NativeSceneAccess;
namespace reference=native_runtime_test;
namespace expected=native_scene_fixture;
namespace ft=output::full_shell::test;
void Number(double actual,double wanted,double absolute,double relative=2e-8) {
    ASSERT_TRUE(std::isfinite(actual));ASSERT_TRUE(std::isfinite(wanted));
    EXPECT_NEAR(actual,wanted,absolute+relative*std::abs(wanted));
}
void Motion(output::physical_frames::NativeAcceptedState state,const reference::ExpectedFrame& wanted) {
    ASSERT_TRUE(state.available);ASSERT_EQ(state.nodes,18u);ASSERT_EQ(state.stamp.epoch,wanted.epoch);
    Number(state.stamp.time,wanted.time,1e-16,0);
    for(unsigned i=0;i<54;++i){SCOPED_TRACE(i);Number(state.position_xyz[i]/.001,wanted.position[i],2e-8);Number(state.velocity_xyz[i]/.001,wanted.velocity[i],2e-5);}
}
void History(const native::NativeContactRow& actual,const native::NativeContactRow& wanted) {
    for(unsigned i=0;i<4;++i)ASSERT_EQ(actual.irtlm[i],wanted.irtlm[i]);
    const auto& a=actual.history.normal;const auto& b=wanted.history.normal;
    Number(a.previous_penetration,b.previous_penetration,2e-8);Number(a.staged_penetration,b.staged_penetration,2e-8);
    Number(a.previous_stiffness,b.previous_stiffness,2e-3);Number(a.staged_stiffness,b.staged_stiffness,2e-3);
    Number(a.damping_half_force,b.damping_half_force,2e-5);
    const double x[]{actual.history.previous_force.x,actual.history.previous_force.y,actual.history.previous_force.z,
        actual.history.staged_force.x,actual.history.staged_force.y,actual.history.staged_force.z};
    const double y[]{wanted.history.previous_force.x,wanted.history.previous_force.y,wanted.history.previous_force.z,
        wanted.history.staged_force.x,wanted.history.staged_force.y,wanted.history.staged_force.z};
    for(unsigned i=0;i<6;++i)Number(x[i],y[i],2e-5);
    Number(actual.penetration_auxiliary,wanted.penetration_auxiliary,2e-8);Number(actual.penetration_offset,wanted.penetration_offset,2e-8);
    for(unsigned i=0;i<2;++i){if(std::abs(wanted.selection_metric[i])==1e20)EXPECT_EQ(actual.selection_metric[i],wanted.selection_metric[i]);
        else Number(actual.selection_metric[i],wanted.selection_metric[i],2e-8);}
}
void Packets(const native::runtime_qualification::Observation& actual,const reference::ExpectedFrame& wanted) {
    ASSERT_EQ(actual.force_base_epoch,wanted.epoch);std::vector<int> secondary,main;
    for(const auto& item:actual.occurrences)if(item.secondary>0){secondary.push_back(item.secondary);main.push_back(item.selected.local_main);}
    ASSERT_EQ(actual.cohort_ends.size(),wanted.geometry_packets.size());std::size_t first=0;
    for(std::size_t i=0;i<actual.cohort_ends.size();++i){const auto last=actual.cohort_ends[i];ASSERT_GE(last,first);ASSERT_LE(last,secondary.size());
        EXPECT_EQ(std::vector<int>(secondary.begin()+first,secondary.begin()+last),wanted.geometry_packets[i].secondary);
        EXPECT_EQ(std::vector<int>(main.begin()+first,main.begin()+last),wanted.geometry_packets[i].main);first=last;}
    EXPECT_EQ(first,secondary.size());
}
std::filesystem::path Export(){const char* p=std::getenv("ROBO_DYNA_NATIVE_SCENE_EXPORT");if(!p)throw std::runtime_error("Explicit native declared export required");return p;}
}
TEST(NativeSceneDynamicsCuda,GeneratedSourceFullThousandIntervalTrajectoryRetainsOriginalNativeGate) {
    const auto file=Export();const auto declared=modelio::native_scene::DeclaredSource::Read(file,output::Sha256(output::ReadBounded(file,4u<<20)));
    const auto physical=PhysicalSource::Prepare(declared,771);const auto contact=ContactSource::Prepare(physical,{1,1,1});
    ft::Directory directory;const auto archive=ArchiveSource::Write(physical,directory.path);
    DynamicsConfig config;config.configuration=881;config.qualification=882;config.fixed_dt=3e-7;
    auto dynamics=NativeSceneDynamics::Prepare(contact,config);output::full_shell::Identity identity;identity.run=883;
    auto capture=dynamics.MakeCapture(archive.mapping(),identity);
    ASSERT_NO_THROW(capture->Capture());auto state=capture->native_state();ASSERT_TRUE(state.available);ASSERT_EQ(state.nodes,18u);
    ASSERT_EQ(capture->secondary_count(),18u);ASSERT_EQ(physical.declared().data().nodes.size(),18u);
    // Physical order is proven against independently exported source IDs before
    // using the captured nodal trajectory order. No captured values initialize it.
    for(unsigned i=0;i<18;++i){ASSERT_EQ(physical.physical().domain()->nodes()[i].source_id,expected::NodeIds[i]);
        Number(state.mass_kg[i],expected::ObservedMass[i]*1000,0,128*std::numeric_limits<double>::epsilon());
        Number(state.inertia_kg_m2[i],expected::ObservedInertia[i]*.001,0,128*std::numeric_limits<double>::epsilon());}
    EXPECT_EQ(state.numerical_mass_kg,0);EXPECT_FALSE(Access::Contact(dynamics).force_phase_available);
    reference::ReferenceReader native_reference(TYPE25_NATIVE_REFERENCE_FILE);
    std::array<bool,18> prior{};std::array<unsigned,18> episodes{};std::size_t active=0,rebuilds=0;
    RecordProperty("physical_source","shipping DeclaredSource/PhysicalSource/ContactSource");
    RecordProperty("coordinate_absolute_tolerance_native_mm","2e-8");RecordProperty("velocity_absolute_tolerance_native_mm_per_s","2e-5");
    RecordProperty("force_absolute_tolerance_native_n","2e-5");RecordProperty("stiffness_absolute_tolerance_native_n_per_mm","2e-3");RecordProperty("relative_tolerance","2e-8");
    for(std::uint64_t step=0;step<1000;++step){
        SCOPED_TRACE(step);const auto wanted=native_reference.Read(step);ASSERT_NO_FATAL_FAILURE(Motion(capture->native_state(),wanted));
        ASSERT_NO_THROW(Access::BeginMaterials(dynamics));native::runtime_qualification::Observation observation;
        EXPECT_FALSE(Access::Observe(dynamics,observation));std::array<double,54> before{},after{};std::array<double,18> k0{},k1{};
        ASSERT_NO_THROW(Access::Force(dynamics,before,k0));ASSERT_NO_THROW(Access::AssembleContact(dynamics));ASSERT_TRUE(Access::Observe(dynamics,observation));
        ASSERT_NO_FATAL_FAILURE(Packets(observation,wanted));ASSERT_NO_THROW(Access::Force(dynamics,after,k1));
        for(unsigned i=0;i<54;++i){SCOPED_TRACE(i);Number(after[i]-before[i],wanted.outgoing_force[i]-wanted.incoming_force[i],2e-5);}
        for(unsigned i=0;i<18;++i)Number((k1[i]-k0[i])/1000,wanted.outgoing_stiffness[i]-wanted.incoming_stiffness[i],2e-3);
        ASSERT_NO_THROW(Access::FinishPrepare(dynamics));ASSERT_NO_THROW(dynamics.CommitStep());EXPECT_FALSE(Access::Observe(dynamics,observation));
        const auto& diagnostics=dynamics.last_accepted_step().contact;active+=diagnostics.active_forces!=0;rebuilds+=diagnostics.reference_rebuilt;
        ASSERT_NO_THROW(capture->Capture());const auto published=Access::Contact(dynamics);ASSERT_TRUE(published.force_phase_available);
        EXPECT_EQ(published.force_base_stamp.epoch,step);EXPECT_EQ(published.generation,step+1);EXPECT_EQ(capture->native_state().stamp.epoch,step+1);
        const auto* history=capture->native_history();const auto* flags=capture->initial_contact_flags();ASSERT_NE(history,nullptr);ASSERT_NE(flags,nullptr);
        for(unsigned row=0;row<18;++row){SCOPED_TRACE(row);EXPECT_EQ(history[row].secondary_source_id,expected::NodeIds[expected::SecondaryNodes[row]]);
            ASSERT_NO_FATAL_FAILURE(History(history[row].row,wanted.rows[row]));EXPECT_EQ(flags[row],wanted.initial_contact[row]);
            const bool hit=history[row].row.irtlm[0]!=0;if(hit&&!prior[row])++episodes[row];prior[row]=hit;}
        if(HasFailure())return;
    }
    const auto terminal=native_reference.Read(1000);ASSERT_NO_FATAL_FAILURE(Motion(capture->native_state(),terminal));native_reference.Finish();
    EXPECT_EQ(active,35u);EXPECT_EQ(episodes[12],2u);EXPECT_GE(rebuilds,1u);
    for(unsigned row=0;row<18;++row)EXPECT_EQ(capture->native_history()[row].row.irtlm[0],0);
    RecordProperty("accepted_physical_intervals","1000");RecordProperty("reference_rebuilds",std::to_string(rebuilds));
    RecordProperty("angular_trajectory_comparison","not_available_in_full_native_fixture");
}
}
