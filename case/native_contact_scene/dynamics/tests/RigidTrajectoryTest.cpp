#include "Fixture.h"
#include "NativeTrajectoryAssertions.h"
#include "CaptureAssertions.h"
#include "RigidReference.h"
#include <array>
#include <algorithm>
#include <limits>
namespace crash::cases::native_scene {
namespace {
using Access=qualification::NativeSceneAccess;
namespace reference=native_runtime_test;
using trajectory_test::Number;using trajectory_test::Motion;using trajectory_test::History;using trajectory_test::Packets;
output::Document Metadata() {
    auto metadata=output::array_json::Parse(output::ReadBounded(TYPE25_RIGID_NATIVE_REFERENCE_METADATA,128u<<10),128u<<10);
    output::Require(metadata["intervals"].GetUint64()==1000&&metadata["frames"].GetUint64()==1001&&
        metadata["dt_s"].GetDouble()==3e-7,"Moving trajectory fixture clock domain differs");
    output::Require(output::Sha256(output::ReadBounded(TYPE25_RIGID_NATIVE_REFERENCE_FILE,16u<<20))==
        metadata["reference_sha256"].GetString(),"Moving expected binary hash differs");
    for(const char* key:{"node_source_ids","secondary_source_ids","mass_native_tonne","inertia_native_tonne_mm2","episodes_by_secondary"})
        output::Require(metadata[key].IsArray()&&metadata[key].Size()==18,"Moving trajectory fixture node/row domain differs");
    output::Require(output::Sha256(output::ReadBounded(TYPE25_RIGID_NATIVE_GROUP_FILE,16u<<20))==
        metadata["rigid_reference_sha256"].GetString(),"Rigid expected binary hash differs");
    return metadata;
}
void SameNormals(const native::runtime_qualification::NormalObservation& a,
    const native::runtime_qualification::NormalObservation& b) {
    ASSERT_EQ(a.face.size(),b.face.size());ASSERT_EQ(a.references.size(),b.references.size());
    const auto same=[](native::StoredNormal x,native::StoredNormal y) {
        EXPECT_EQ(output::Bits(double(x.x)),output::Bits(double(y.x)));
        EXPECT_EQ(output::Bits(double(x.y)),output::Bits(double(y.y)));
        EXPECT_EQ(output::Bits(double(x.z)),output::Bits(double(y.z)));
    };
    for(std::size_t i=0;i<a.face.size();++i)same(a.face[i],b.face[i]);
    for(std::size_t i=0;i<a.references.size();++i) {
        EXPECT_EQ(a.references[i].boundary,b.references[i].boundary);
        for(unsigned j=0;j<2;++j)same(a.references[i].bisector[j],b.references[i].bisector[j]);
    }
    EXPECT_EQ(a.accepted.generation,b.accepted.generation);
    EXPECT_EQ(a.accepted.selectors.history,b.accepted.selectors.history);
}
}
TEST(NativeRigidSceneDynamicsCuda, SourceBoundRigidThousandIntervalsAndActiveDiscardMatchExternalNative) {
    const auto metadata=Metadata();test::SourceFixture source("ROBO_DYNA_NATIVE_RIGID_SCENE_EXPORT");
    ASSERT_EQ(source.contact.kind(),ContactSelection::Kind::MovingShells);
    auto dynamics=source.MakeDynamics();auto capture=source.MakeCapture(dynamics);
    ASSERT_NO_THROW(capture->Capture());ASSERT_EQ(capture->native_state().nodes,18u);
    const auto& initial=capture->native_state();
    ASSERT_EQ(source.contact.config().response_mass,native::ResponseMassPolicy::AcceptedOwnerCoefficients);
    ASSERT_EQ(initial.stamp.rigid_groups.group_count,1u);
    rigid_trajectory_test::Reader group_reference(TYPE25_RIGID_NATIVE_GROUP_FILE);
    auto current_group=group_reference.Read(0);auto force_group=current_group;
    for(unsigned i=0;i<18;++i) {
        ASSERT_EQ(source.physical.physical().domain()->nodes()[i].source_id,metadata["node_source_ids"][i].GetUint64());
        Number(initial.mass_kg[i],metadata["mass_native_tonne"][i].GetDouble()*1000,0,128*std::numeric_limits<double>::epsilon());
        Number(initial.inertia_kg_m2[i],metadata["inertia_native_tonne_mm2"][i].GetDouble()*.001,0,128*std::numeric_limits<double>::epsilon());
    }
    const std::vector<double> initial_mass(initial.mass_kg,initial.mass_kg+18),initial_inertia(initial.inertia_kg_m2,initial.inertia_kg_m2+18);
    ASSERT_EQ(initial.numerical_mass_kg,0);ASSERT_EQ(capture->secondary_count(),18u);
    reference::ReferenceReader expected(TYPE25_RIGID_NATIVE_REFERENCE_FILE);
    const auto first_active=metadata["first_active_step"].GetUint64();ASSERT_LT(first_active,1000u);
    std::array<bool,18> prior{};std::array<unsigned,18> episodes{};std::size_t active=0,rebuilds=0;bool retried=false;
    RecordProperty("physical_source","shipping DeclaredSource/PhysicalSource/MovingContactSource");
    RecordProperty("normal_profile",source.contact.profile_name());
    RecordProperty("coordinate_absolute_tolerance_native_mm","2e-8");RecordProperty("velocity_absolute_tolerance_native_mm_per_s","2e-5");
    RecordProperty("force_absolute_tolerance_native_n","2e-5");RecordProperty("stiffness_absolute_tolerance_native_n_per_mm","2e-3");
    RecordProperty("relative_tolerance","2e-8");
    for(std::uint64_t step=0;step<1000;++step) {
        SCOPED_TRACE(step);const auto wanted=expected.Read(step);
        ASSERT_NO_FATAL_FAILURE(Motion(capture->native_state(),wanted));
        ASSERT_NO_FATAL_FAILURE(rigid_trajectory_test::Motion(capture->native_state(),Access::Rigid(dynamics),current_group,force_group));
        const auto before_group=Access::Rigid(dynamics);
        const auto before_state=capture_test::Copy(capture->native_state());
        const auto before_history=std::vector<native::NativeGeometryHistory>(capture->native_history(),capture->native_history()+18);
        const auto before_flags=std::vector<int>(capture->initial_contact_flags(),capture->initial_contact_flags()+18);
        native::runtime_qualification::NormalObservation before_normals;
        if(step==first_active)ASSERT_TRUE(Access::AcceptedNormals(dynamics,before_normals));
        const auto assemble=[&] {
            Access::BeginMaterials(dynamics);native::runtime_qualification::Observation observed;
            EXPECT_FALSE(Access::Observe(dynamics,observed));std::array<double,54> f0{},f1{};std::array<double,18> k0{},k1{};
            Access::Force(dynamics,f0,k0);Access::AssembleContact(dynamics);
            ASSERT_TRUE(Access::Observe(dynamics,observed));ASSERT_NO_FATAL_FAILURE(Packets(observed,wanted));
            if(step==first_active)EXPECT_TRUE(std::any_of(observed.response.begin(),observed.response.end(),
                [](const auto& value){return value.contact_active;}));
            Access::Force(dynamics,f1,k1);
            for(unsigned i=0;i<54;++i)Number(f1[i]-f0[i],wanted.outgoing_force[i]-wanted.incoming_force[i],2e-5);
            for(unsigned i=0;i<18;++i)Number((k1[i]-k0[i])/1000,wanted.outgoing_stiffness[i]-wanted.incoming_stiffness[i],2e-3);
            Access::FinishPrepare(dynamics);
        };
        ASSERT_NO_FATAL_FAILURE(assemble());
        if(step==first_active) {
            EXPECT_THROW(dynamics.PrepareStep(),std::exception);EXPECT_TRUE(dynamics.has_prepared_step());
            capture->Capture();capture_test::Same(before_state,capture->native_state());
            native::runtime_qualification::NormalObservation unchanged;ASSERT_TRUE(Access::AcceptedNormals(dynamics,unchanged));
            SameNormals(before_normals,unchanged);
            rigid_trajectory_test::SameGroup(before_group,Access::Rigid(dynamics));dynamics.DiscardStep();ASSERT_FALSE(dynamics.has_prepared_step());
            capture->Capture();capture_test::Same(before_state,capture->native_state());
            rigid_trajectory_test::SameGroup(before_group,Access::Rigid(dynamics));
            for(unsigned i=0;i<18;++i) {
                capture_test::SameHistory(before_history[i],capture->native_history()[i]);EXPECT_EQ(before_flags[i],capture->initial_contact_flags()[i]);
            }
            ASSERT_TRUE(Access::AcceptedNormals(dynamics,unchanged));SameNormals(before_normals,unchanged);
            ASSERT_NO_FATAL_FAILURE(assemble());retried=true;
        }
        ASSERT_NO_THROW(dynamics.CommitStep());
        ASSERT_NO_THROW(capture->Capture());
        force_group=current_group;current_group=group_reference.Read(step+1);
        for(unsigned i=0;i<18;++i) {
            EXPECT_EQ(output::Bits(capture->native_state().mass_kg[i]),output::Bits(initial_mass[i]));
            EXPECT_EQ(output::Bits(capture->native_state().inertia_kg_m2[i]),output::Bits(initial_inertia[i]));
        }
        const auto published=Access::Contact(dynamics);EXPECT_EQ(published.generation,step+1);EXPECT_EQ(published.force_base_stamp.epoch,step);
        const auto& diagnostics=dynamics.last_accepted_step().contact;active+=diagnostics.active_forces!=0;rebuilds+=diagnostics.reference_rebuilt;
        for(unsigned row=0;row<18;++row) {
            SCOPED_TRACE(row);const auto& history=capture->native_history()[row];
            EXPECT_EQ(history.secondary_source_id,metadata["secondary_source_ids"][row].GetUint64());
            ASSERT_NO_FATAL_FAILURE(History(history.row,wanted.rows[row]));EXPECT_EQ(capture->initial_contact_flags()[row],wanted.initial_contact[row]);
            const bool hit=history.row.irtlm[0]!=0;if(hit&&!prior[row])++episodes[row];prior[row]=hit;
        }
        if(HasFailure())return;
    }
    const auto terminal=expected.Read(1000);ASSERT_NO_FATAL_FAILURE(Motion(capture->native_state(),terminal));expected.Finish();
    ASSERT_NO_FATAL_FAILURE(rigid_trajectory_test::Motion(capture->native_state(),Access::Rigid(dynamics),current_group,force_group));group_reference.Finish();
    EXPECT_TRUE(retried);EXPECT_EQ(active,metadata["active_steps"].GetUint64());EXPECT_GE(rebuilds,1u);
    for(unsigned i=0;i<18;++i)EXPECT_EQ(episodes[i],metadata["episodes_by_secondary"][i].GetUint());
    RecordProperty("accepted_physical_intervals","1000");RecordProperty("active_force_discard_retry","passed");
    RecordProperty("angular_trajectory_comparison","native physical spin + primary omega + force-phase principal frame; no native nodal quaternion claim");
    RecordProperty("rigid_source","actual source PART2; separate native primary19; full member M/J and converter regularizers");
    RecordProperty("spin_absolute_tolerance_per_s","2e-8");RecordProperty("frame_absolute_tolerance","2e-10");
}
}
