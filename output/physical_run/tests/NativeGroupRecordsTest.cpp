#include "Support.h"
namespace crash::output::physical_run::test {
namespace {
Profile GroupProfile() {Profile p;p.type45=true;p.beam18=true;p.structural_limit=true;p.native_group=true;return p;}
Values GroupRow(const records::Context& context,std::uint64_t epoch) {
    auto row=Row(context,epoch,3*epoch,true);NativeGroupValues group;group.count=2;
    const auto velocity=epoch==1?0.:(double(epoch)-1.5)*context.fixed_dt();
    group.entries[0]={NativeContactRole::Self,{91,92,93,context.nodes()+4,context.nodes(),3,5,
        epoch,(epoch+1)/2,epoch-1,row.stamp.base_time,velocity}};
    group.entries[1]={NativeContactRole::MeshWall,{101,102,103,context.nodes()+4,context.nodes()+4,1,2,
        epoch,1,epoch-1,row.stamp.base_time,velocity}};
    row.native_group=group;return row;
}
}
TEST(NativeGroupRecords, ExplicitFullVehicleProfilePreservesLegacyAndEnvironmentSchemas) {
    const auto profile=GroupProfile();const auto doc=ProfileDocument(profile);
    EXPECT_EQ(array_json::Text(doc["schema"]),NativeGroupProfileSchema);
    EXPECT_EQ(array_json::Text(doc["participants"]),"qeph,t3,qbat,type25,type13,solids,type45,beam18,native_type25_group");
    EXPECT_TRUE(SameProfile(ReadProfile(doc),profile));
    const auto context=Context();auto config=Config(context,profile);config.environment=true;
    config.request.extra_interval_bytes=ExtraIntervalBytes(profile);
    const auto encoded=ConfigurationDocument(config);
    EXPECT_EQ(array_json::Text(encoded["schema"]),"robo_dyna.physical_run_configuration.v5");
    const auto decoded=ReadConfiguration(encoded);EXPECT_TRUE(decoded.environment);EXPECT_TRUE(SameProfile(decoded.profile,profile));
    auto bad=profile;bad.native_contact=true;EXPECT_THROW(ProfileDocument(bad),std::exception);
    bad=profile;bad.self_contact=true;EXPECT_THROW(ProfileDocument(bad),std::exception);
    EXPECT_EQ(IntegerFields(profile).size(),4+NativeGroupIntegerCount);
    EXPECT_EQ(RealFields(profile).size(),5+NativeGroupRealCount);
    auto row=GroupRow(context,1);EXPECT_NO_THROW(CheckValues(context,profile,row));
    EXPECT_THROW(CheckNativeContactValues(row.native_group->entries[0].publication),std::exception);
    EXPECT_NO_THROW(CheckNativeContactValues(row.native_group->entries[0].publication,NativeContactLayout::ExpandedSurface));
}
TEST(NativeGroupRecords, TwoNativePublicationsRoundTripInDeclaredOrderAcrossChunks) {
    ft::Directory dir;const auto context=Context();const auto profile=GroupProfile();
    IntervalWriter writer(dir.path,context,profile,4,624,32768);
    for(unsigned step=1;step<=4;++step)writer.Append(GroupRow(context,step));
    const auto segments=writer.Finish();ASSERT_EQ(segments.size(),2u);unsigned read=0;
    const auto final=ReadIntervals(dir.path,context,profile,4,4,segments,624,32768,[&](const Values& actual) {
        const auto expected=GroupRow(context,++read);ASSERT_TRUE(actual.native_group);
        std::uint64_t a[NativeGroupIntegerCount],b[NativeGroupIntegerCount];double x[NativeGroupRealCount],y[NativeGroupRealCount];
        EncodeNativeGroup(*actual.native_group,a,x);EncodeNativeGroup(*expected.native_group,b,y);
        for(unsigned i=0;i<NativeGroupIntegerCount;++i)EXPECT_EQ(a[i],b[i]);
        for(unsigned i=0;i<NativeGroupRealCount;++i)EXPECT_EQ(Bits(x[i]),Bits(y[i]));
    });
    EXPECT_EQ(read,4u);ASSERT_TRUE(final.native_group);EXPECT_EQ(final.native_group->entries[0].publication.reference_generation,2u);
    EXPECT_EQ(final.native_group->entries[1].publication.reference_generation,1u);
}
TEST(NativeGroupRecords, SourceRolePhaseAndOneChildGenerationCorruptionCannotAdvance) {
    const auto context=Context();const auto profile=GroupProfile();Sequence sequence;
    sequence=Advance(context,profile,4,sequence,GroupRow(context,1));const auto saved=sequence;
    const auto reject=[&](Values bad) {EXPECT_THROW(Advance(context,profile,4,sequence,bad),std::exception);
        EXPECT_TRUE(records::SameStamp(sequence.last,saved.last));};
    auto bad=GroupRow(context,2);std::swap(bad.native_group->entries[0],bad.native_group->entries[1]);reject(bad);
    bad=GroupRow(context,2);++bad.native_group->entries[1].publication.publication_generation;reject(bad);
    bad=GroupRow(context,2);++bad.native_group->entries[0].publication.source_id;reject(bad);
    bad=GroupRow(context,2);bad.native_group->entries[1].role=NativeContactRole::Self;reject(bad);
    bad=GroupRow(context,2);bad.native_group->entries[1].publication.nodes--;reject(bad);
    bad=GroupRow(context,2);bad.native_group->entries[0].publication.reference_generation=2;
    bad.native_group->entries[0].publication.force_base_velocity_time=0;reject(bad);
    bad=GroupRow(context,2);bad.native_group.reset();reject(bad);
    EXPECT_NO_THROW(Advance(context,profile,4,sequence,GroupRow(context,2)));
}
TEST(NativeGroupRecords, SingleInterfaceEncodingRequiresCanonicalUnusedStorage) {
    auto group=*GroupRow(Context(),1).native_group;group.count=1;group.entries[1]={};
    std::uint64_t ints[NativeGroupIntegerCount];double reals[NativeGroupRealCount];
    EncodeNativeGroup(group,ints,reals);EXPECT_EQ(DecodeNativeGroup(ints,reals).count,1u);
    const auto unused=1+(1+NativeContactIntegerCount);ints[unused]=2;
    EXPECT_THROW(DecodeNativeGroup(ints,reals),std::exception);ints[unused]=0;
    reals[NativeContactRealCount]=-0.;EXPECT_THROW(DecodeNativeGroup(ints,reals),std::exception);
    group.entries[1].publication.source_id=4;EXPECT_THROW(CheckNativeGroupValues(group),std::exception);
}
}
