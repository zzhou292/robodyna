#include "Support.h"
#include "../RunState.h"
#include <array>
#include <limits>

namespace crash::output::physical_run::test {
namespace {
Profile SelfProfile(bool bound=true) {return {true,bound,true,true};}
Values SelfRow(const records::Context& c,std::uint64_t epoch,bool bound=true) {
    auto row=Row(c,epoch,epoch*3,bound);
    SelfContactValues v;
    v.source_id=UINT64_C(9007199254740997);v.selected_parents=4;
    v.events=5;v.vertex_face_events=3;v.boundary_vertex_edge_events=1;v.edge_edge_events=2;v.active_events=4;
    v.accepted_parent_pairs=4;v.accepted_facet_pairs=9;v.discovered_features=12;v.regularity_generation=epoch+7;
    v.candidate_parent_pairs=6;v.candidate_facet_pairs=15;v.policy_outcomes=15;
    v.certified_separated=3;v.same_rigid_exclusions=2;v.local_intersections=5;
    v.represented_vf=2;v.represented_ee=3;v.policy_digest=UINT64_C(5411954021770061371)+epoch;
    v.active_parents=2;v.removing_parents=1;v.skipped_parents=1;
    v.base_velocity_time=epoch>1?Row(c,epoch-1).stamp.velocity_time:0.;
    v.potential_j=.125*epoch;v.maximum_force_n=12.;v.maximum_sti_n_m=45.;v.maximum_represented_stiffness_n_m=46.;
    v.endpoint_a_n={1.,-2.,-0.};v.endpoint_b_n={-1.,2.,0.};v.equal_opposite_residual_n={0.,-0.,0.};
    v.global_moment_n_m={.1,-.2,0.};
    row.self_contact=v;return row;
}
void Exact(const SelfContactValues& a,const SelfContactValues& b) {
    std::array<std::uint64_t,SelfContactIntegerCount> ai{},bi{};
    std::array<double,SelfContactRealCount> ar{},br{};
    EncodeSelfContact(a,ai.data(),ar.data());EncodeSelfContact(b,bi.data(),br.data());
    EXPECT_EQ(ai,bi);
    for(std::size_t i=0;i<ar.size();++i)EXPECT_EQ(Bits(ar[i]),Bits(br[i]))<<i;
}
}

TEST(PhysicalRunSelfContact, TypedChunksRoundTripLargeIntegersSignedZeroAndBothPhases) {
    for(bool bound:{false,true}) {
        const auto c=Context();const auto p=SelfProfile(bound);ft::Directory dir;
        const auto row_bytes=8*(IntegerFields(p).size()+RealFields(p).size());
        ASSERT_EQ(interval::RowBytes+ExtraIntervalBytes(p),row_bytes);
        IntervalWriter writer(dir.path,c,p,5,2*row_bytes,3*2*row_bytes);
        EXPECT_EQ(writer.buffer_bytes(),2*row_bytes);
        for(unsigned epoch=1;epoch<=5;++epoch)writer.Append(SelfRow(c,epoch,bound));
        const auto segments=writer.Finish();ASSERT_EQ(segments.size(),3u);
        std::size_t actual_bytes=0;
        for(const auto& segment:segments) {
            EXPECT_EQ(segment.integers.layout.columns,4+SelfContactIntegerCount);
            EXPECT_EQ(segment.reals.layout.columns,4+unsigned(bound)+SelfContactRealCount);
            actual_bytes+=std::filesystem::file_size(dir.path/segment.integers.file);
            actual_bytes+=std::filesystem::file_size(dir.path/segment.reals.file);
        }
        EXPECT_EQ(actual_bytes,5*row_bytes);
        unsigned seen=0;
        const auto end=ReadIntervals(dir.path,c,p,5,5,segments,2*row_bytes,3*2*row_bytes,[&](const Values& row) {
            const auto expected=SelfRow(c,++seen,bound);
            ASSERT_TRUE(row.self_contact);Exact(*expected.self_contact,*row.self_contact);
            EXPECT_TRUE(records::SameStamp(expected.stamp,row.stamp));
        });
        EXPECT_EQ(seen,5u);EXPECT_EQ(end.last.epoch,5u);
        auto bad=segments;bad[0].integers.layout.fields[4]="wrong_source_id";
        EXPECT_THROW(ReadIntervals(dir.path,c,p,5,5,bad,2*row_bytes,3*2*row_bytes,{}),std::exception);
        EXPECT_THROW(ReadIntervals(dir.path,c,{true,bound,true},5,5,segments,2*row_bytes,3*2*row_bytes,{}),std::exception);
    }
}

TEST(PhysicalRunSelfContact, ProfileAndConfigurationRequireExactVersionAvailabilityAndReservation) {
    const auto p=SelfProfile();auto c=Config(Context(),p);
    c.request.extra_interval_bytes=ExtraIntervalBytes(p);
    auto profile=ProfileDocument(p);EXPECT_TRUE(SameProfile(ReadProfile(profile),p));
    const auto config=ConfigurationDocument(c);
    EXPECT_EQ(ReadConfiguration(config).request.extra_interval_bytes,ExtraIntervalBytes(p));
    EXPECT_EQ(array_json::Text(profile["contact_work"]),"unavailable");
    for(unsigned mutation=0;mutation<5;++mutation) {
        auto bad=ConfigurationDocument(c);
        switch(mutation) {
            case 0:bad["extra_interval_bytes"].SetUint64(ExtraIntervalBytes(p)-1);break;
            case 1:bad["extra_interval_bytes"].SetUint64(ExtraIntervalBytes(p)+1);break;
            case 2:bad.RemoveMember("extra_interval_bytes");break;
            case 3:bad["profile"]["schema"].SetString(ProfileSchema,bad.GetAllocator());break;
            case 4:bad["profile"]["self_contact"].SetString("friction_applied",bad.GetAllocator());break;
        }
        EXPECT_THROW(ReadConfiguration(bad),std::exception)<<mutation;
    }
    auto under=c.request;--under.extra_interval_bytes;
    EXPECT_THROW(detail::ValidateRequest(under,p,false),std::exception);
    EXPECT_NO_THROW(detail::ValidateRequest(c.request,p,false));
    const auto plan=records::PlanArchive(c.request);
    EXPECT_EQ(plan.interval_bytes,c.request.intervals*(interval::RowBytes+ExtraIntervalBytes(p)));
    auto exact=c.request;exact.total_byte_cap=plan.forecast_bytes;
    EXPECT_NO_THROW(records::PlanArchive(exact));--exact.total_byte_cap;
    EXPECT_THROW(records::PlanArchive(exact),std::exception);
}

TEST(PhysicalRunSelfContact, InvalidCensusNonfiniteForcesAndHiddenParticipantRejectWithoutAdvancing) {
    const auto c=Context();const auto p=SelfProfile();const auto valid=SelfRow(c,1);
    for(unsigned mutation=0;mutation<13;++mutation) {
        auto bad=valid;auto& v=*bad.self_contact;
        switch(mutation) {
            case 0:v.source_id=0;break;
            case 1:++v.events;break;
            case 2:v.boundary_vertex_edge_events=v.vertex_face_events+1;break;
            case 3:++v.policy_outcomes;break;
            case 4:++v.certified_separated;break;
            case 5:++v.skipped_parents;break;
            case 6:v.global_moment_n_m[1]=std::numeric_limits<double>::quiet_NaN();break;
            case 7:v.potential_j=-1.;break;
            case 8:v.regularity_generation=0;break;
            case 9:v.vertex_face_events=UINT64_MAX;v.edge_edge_events=1;v.events=0;break;
            case 10:v.base_velocity_time=.0625;break;
            case 11:v.vertex_face_events=0;v.boundary_vertex_edge_events=0;v.events=v.edge_edge_events;
                v.active_events=v.events;break;
            case 12:v.edge_edge_events=0;v.events=v.vertex_face_events;v.active_events=v.events;break;
        }
        EXPECT_THROW(Advance(c,p,3,{},bad),std::exception)<<mutation;
    }
    EXPECT_THROW(CheckValues(c,{true,true,true},valid),std::exception);
    auto missing=valid;missing.self_contact.reset();EXPECT_THROW(CheckValues(c,p,missing),std::exception);
    const auto first=Advance(c,p,3,{},valid);
    auto bad=SelfRow(c,2);bad.self_contact->source_id++;
    EXPECT_THROW(Advance(c,p,3,first,bad),std::exception);
    bad=SelfRow(c,2);bad.self_contact->base_velocity_time=bad.stamp.base_time;
    EXPECT_THROW(Advance(c,p,3,first,bad),std::exception);
    EXPECT_EQ(first.last.epoch,1u);
    EXPECT_EQ(Advance(c,p,3,first,SelfRow(c,2)).last.epoch,2u);
}

TEST(PhysicalRunSelfContact, InitialOnlyPrefixAndSampledActivityReplayUseExpandedLedgerShape) {
    const auto c=Context();const auto p=SelfProfile();auto config=Config(c,p);
    config.request.extra_interval_bytes=ExtraIntervalBytes(p);
    for(unsigned accepted:{0u,3u}) {
        ft::Directory dir;IntervalWriter writer(dir.path,c,p,4,config.request.file_byte_cap,64u<<20);
        Index index;index.planned_intervals=4;index.accepted_intervals=accepted;
        index.stop_reason="qualified contact prefix";index.frames.push_back(Frame(dir.path,c,{}));
        const auto plan=records::PlanArchive(config.request);
        for(unsigned epoch=1;epoch<=accepted;++epoch) {
            const auto row=SelfRow(c,epoch);writer.Append(row);index.final=row.stamp;
            if(epoch==accepted || std::binary_search(plan.frame_epochs.begin(),plan.frame_epochs.end(),epoch))
                index.frames.push_back(Frame(dir.path,c,row.stamp));
        }
        index.segments=writer.Finish();
        const auto restored=ReadIndex(c,config,IndexDocument(config,index));
        EXPECT_NO_THROW(ValidateRecords(dir.path,c,config,restored,64u<<20));
    }
}

TEST(PhysicalRunSelfContact, LegacyProfileBytesAndColumnsRemainUnchanged) {
    ft::Directory dir;const auto document=ProfileDocument({});
    const auto file=WriteDocument(dir.path,"legacy.json",document,MetadataCap);
    EXPECT_EQ(ReadFile(dir.path,file,MetadataCap),
        "{\"schema\":\"robo_dyna.physical_observation_profile.v1\","
        "\"purpose\":\"selected_physical_model_accepted_visualization_not_restart\","
        "\"participants\":\"qeph,t3,qbat,type25,type13,solids\",\"structural_limit\":\"unavailable\","
        "\"kinetic_energy\":\"unavailable\",\"total_energy\":\"unavailable\",\"internal_work\":\"unavailable\","
        "\"contact_force\":\"unavailable\",\"contact_penetration\":\"unavailable\","
        "\"contact_work\":\"unavailable\",\"joint_work\":\"unavailable\"}");
    EXPECT_EQ(IntegerFields().size(),4u);EXPECT_EQ(RealFields({}).size(),4u);
    EXPECT_EQ(ExtraIntervalBytes({true,true,true}),0u);
    auto fake=ProfileDocument({});String(fake,"self_contact","frictionless_fixed_triangles_v1_accepted_base_force_candidate_policy");
    EXPECT_THROW(ReadProfile(fake),std::exception);
}
} // namespace crash::output::physical_run::test
