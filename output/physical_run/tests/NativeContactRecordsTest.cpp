#include "Support.h"
namespace crash::output::physical_run::test {
namespace {
Profile NativeProfile() {Profile p;p.native_contact=true;return p;}
Values NativeRow(const records::Context& c,std::uint64_t epoch,std::uint64_t reference) {
    auto row=Row(c,epoch,3*epoch,false);
    row.native_contact=NativeContactValues{91,92,93,c.nodes(),c.nodes(),2,4,epoch,reference,epoch-1,
        row.stamp.base_time,epoch==1?0.:(double(epoch)-1.5)*c.fixed_dt()};
    return row;
}
}
TEST(NativeContactRecords, ExplicitProfileAndConfigurationKeepLegacySchemasSeparate) {
    const auto p=NativeProfile();auto d=ProfileDocument(p);
    EXPECT_EQ(array_json::Text(d["schema"]),NativeContactProfileSchema);
    EXPECT_EQ(array_json::Text(d["participants"]),"qeph,t3,native_type25");
    EXPECT_EQ(array_json::Text(d["contact_force"]),"unavailable");
    EXPECT_TRUE(SameProfile(ReadProfile(d),p));
    EXPECT_EQ(IntegerFields(p).size(),14u);EXPECT_EQ(RealFields(p).size(),6u);EXPECT_EQ(ExtraIntervalBytes(p),0u);
    const auto c=Context();auto config=Config(c,p);config.request.extra_interval_bytes=ExtraIntervalBytes(p);
    const auto json=ConfigurationDocument(config);
    EXPECT_EQ(array_json::Text(json["schema"]),"robo_dyna.physical_run_configuration.v3");
    EXPECT_TRUE(SameProfile(ReadConfiguration(json).profile,p));
    auto bad=p;bad.self_contact=true;EXPECT_THROW(ProfileDocument(bad),std::exception);
    bad=p;bad.type45=true;EXPECT_THROW(ProfileDocument(bad),std::exception);
    d["contact_force"].SetString("native_N",d.GetAllocator());EXPECT_THROW(ReadProfile(d),std::exception);
    EXPECT_EQ(array_json::Text(ProfileDocument({})["schema"]),ProfileSchema);
    Profile old;old.self_contact=true;
    EXPECT_EQ(array_json::Text(ProfileDocument(old)["schema"]),SelfContactProfileSchema);
}
TEST(NativeContactRecords, NeutralHistoryPayloadWritesAndLegacyFixedPayloadReadsWithoutChangingColumns) {
    const auto profile=NativeProfile();auto doc=ProfileDocument(profile);
    EXPECT_EQ(array_json::Text(doc["native_contact"]),"native_type25_accepted_base_force_history_v1");
    EXPECT_TRUE(SameProfile(ReadProfile(doc),profile));
    const auto integers=IntegerFields(profile),reals=RealFields(profile);
    doc["native_contact"].SetString("fixed_main_type25_accepted_base_force_history_v1",doc.GetAllocator());
    const auto legacy=ReadProfile(doc);EXPECT_TRUE(SameProfile(legacy,profile));
    EXPECT_EQ(IntegerFields(legacy),integers);EXPECT_EQ(RealFields(legacy),reals);
    EXPECT_EQ(array_json::Text(ProfileDocument(legacy)["native_contact"]),"native_type25_accepted_base_force_history_v1");
    doc["native_contact"].SetString("moving_exact_geometry_or_restart",doc.GetAllocator());
    EXPECT_THROW(ReadProfile(doc),std::exception);
}
TEST(NativeContactRecords, NativeGenerationAndForceBaseRoundTripAcrossChunks) {
    ft::Directory dir;const auto c=Context();const auto p=NativeProfile();
    IntervalWriter writer(dir.path,c,p,4,624,16384);
    for(unsigned i=1;i<=4;++i)writer.Append(NativeRow(c,i,(i+1)/2));
    const auto segments=writer.Finish();ASSERT_EQ(segments.size(),2u);
    unsigned read=0;
    const auto final=ReadIntervals(dir.path,c,p,4,4,segments,624,16384,[&](const Values& v){
        ++read;const auto want=NativeRow(c,read,(read+1)/2);ASSERT_TRUE(v.native_contact);
        std::uint64_t a[NativeContactIntegerCount]{},b[NativeContactIntegerCount]{};
        double x[NativeContactRealCount]{},y[NativeContactRealCount]{};
        EncodeNativeContact(*v.native_contact,a,x);EncodeNativeContact(*want.native_contact,b,y);
        for(unsigned j=0;j<NativeContactIntegerCount;++j)EXPECT_EQ(a[j],b[j]);
        for(unsigned j=0;j<NativeContactRealCount;++j)EXPECT_EQ(Bits(x[j]),Bits(y[j]));
        EXPECT_TRUE(records::SameStamp(v.stamp,want.stamp));
    });
    EXPECT_EQ(read,4u);EXPECT_EQ(final.last.epoch,4u);EXPECT_EQ(final.native_contact->reference_generation,2u);
}
TEST(NativeContactRecords, CorruptEpochReferenceAndSourceRejectWithoutAdvancingSequence) {
    const auto c=Context();const auto p=NativeProfile();Sequence s;
    s=Advance(c,p,4,s,NativeRow(c,1,1));const auto saved=s;
    const auto reject=[&](Values bad){EXPECT_THROW(Advance(c,p,4,s,bad),std::exception);EXPECT_TRUE(records::SameStamp(s.last,saved.last));};
    auto bad=NativeRow(c,2,1);bad.native_contact->publication_generation=3;reject(bad);
    bad=NativeRow(c,2,1);bad.native_contact->reference_generation=0;reject(bad);
    bad=NativeRow(c,2,1);bad.native_contact->reference_generation=3;reject(bad);
    bad=NativeRow(c,2,1);++bad.native_contact->source_generation;reject(bad);
    bad=NativeRow(c,2,1);bad.native_contact->force_base_velocity_time=0;reject(bad);
    bad=NativeRow(c,2,1);++bad.native_contact->force_base_epoch;reject(bad);
    s=Advance(c,p,4,s,NativeRow(c,2,1));
    bad=NativeRow(c,3,3);EXPECT_THROW(Advance(c,p,4,s,bad),std::exception); // Valid isolated generation, invalid jump.
    s=Advance(c,p,4,s,NativeRow(c,3,2));
    bad=NativeRow(c,4,1);EXPECT_THROW(Advance(c,p,4,s,bad),std::exception);
    auto first=NativeRow(c,1,1);first.native_contact->publication_generation=UINT64_MAX;
    EXPECT_THROW(CheckValues(c,p,first),std::exception);
    EXPECT_NO_THROW(Advance(c,p,4,s,NativeRow(c,4,3)));
}
}
