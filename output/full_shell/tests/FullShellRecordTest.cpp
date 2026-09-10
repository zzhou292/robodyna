#include "TestSupport.h"
#include <limits>
#include <type_traits>

namespace crash::output::full_shell::test {
static_assert(!std::is_copy_assignable_v<Context>&&!std::is_move_assignable_v<Context>);
TEST(FullShellRecords,ExactRoundTripAndExplicitFieldApplicability) {
    Directory dir;auto context=MakeContext();auto copy=context;auto moved=std::move(copy);
    EXPECT_EQ(context.point_layout_sha256(),copy.point_layout_sha256()); // Immutable move-as-copy.
    EXPECT_EQ(context.points(),4u);EXPECT_EQ(context.point_offsets(),(std::vector<std::size_t>{0,3,4,4,4}));
    auto original=Frame();original.position_xyz.back()=std::numeric_limits<double>::denorm_min();
    const auto file=WriteFrame(dir.path,"frame-2",context,View(original));
    const auto restored=ReadFrame(dir.path,moved,file,original.stamp);
    for(std::size_t i=0;i<original.position_xyz.size();++i)EXPECT_EQ(Bits(restored.position_xyz[i]),Bits(original.position_xyz[i]));
    for(std::size_t i=0;i<original.plastic_points.size();++i)EXPECT_EQ(Bits(restored.plastic_points[i]),Bits(original.plastic_points[i]));
    const auto maximum=ParentPlasticMaxima(context,restored);ASSERT_EQ(maximum.size(),4u);
    EXPECT_EQ(maximum[0],.3);EXPECT_EQ(maximum[1],.2);EXPECT_FALSE(maximum[2]);EXPECT_FALSE(maximum[3]);
    auto initial=Frame(true);const auto first=WriteFrame(dir.path,"frame-0",context,View(initial));
    EXPECT_NO_THROW(ReadFrame(dir.path,context,first,initial.stamp));
}
TEST(FullShellRecords,ContextRejectsLateSourceDefectsAndCapsBeforePointers) {
    auto parents=Parents();const auto good=Context::Create(Id(),4,parents.data(),parents.size(),.125);
    for(int bad=0;bad<4;++bad) {
        auto p=parents;
        if(bad==0)p.back().source_part=0;
        if(bad==1)p.back().source_element=p.front().source_element;
        if(bad==2)p.back().plastic=static_cast<PlasticField>(999);
        if(bad==3){p.back().plastic=PlasticField::NativeEquivalentPlasticStrain;p.back().native_points=0;}
        EXPECT_THROW(Context::Create(Id(),4,p.data(),p.size(),.125),std::exception);
    }
    RecordLimits limits;limits.host_bytes=1;
    EXPECT_THROW(Context::Create(Id(),4,reinterpret_cast<const ParentPoints*>(1),4,.125,limits),std::exception);
    EXPECT_THROW(Context::Create(Id(),1048577,reinterpret_cast<const ParentPoints*>(1),4,.125),std::exception);
    EXPECT_THROW(Context::Create(Id(),4,parents.data(),parents.size(),std::numeric_limits<double>::denorm_min()),std::exception);
    EXPECT_EQ(good.parents().back().source_element,74u);
    auto changed=parents;changed.back().source_part+=1;
    const auto other=Context::Create(Id(),4,changed.data(),changed.size(),.125);
    EXPECT_NE(good.point_layout_sha256(),other.point_layout_sha256());
}
TEST(FullShellRecords,LateContentAndExistingMetadataRejectBeforeAnyWrite) {
    Directory dir;const auto c=MakeContext();auto f=Frame();f.plastic_points.back()=-1;
    EXPECT_THROW(WriteFrame(dir.path,"late",c,View(f)),std::exception);EXPECT_TRUE(std::filesystem::is_empty(dir.path));
    f=Frame();f.position_xyz.back()=std::numeric_limits<double>::infinity();
    EXPECT_THROW(WriteFrame(dir.path,"late",c,View(f)),std::exception);EXPECT_TRUE(std::filesystem::is_empty(dir.path));
    f=Frame();auto poisoned=View(f);poisoned.position_xyz=reinterpret_cast<const double*>(1);--poisoned.position_values;
    EXPECT_THROW(WriteFrame(dir.path,"late",c,poisoned),std::exception);EXPECT_TRUE(std::filesystem::is_empty(dir.path));
    WriteBytes(dir.path/"late.frame.json","existing");
    EXPECT_THROW(WriteFrame(dir.path,"late",c,View(f)),std::exception);
    EXPECT_FALSE(std::filesystem::exists(dir.path/"late.positions.bin"));
    EXPECT_FALSE(std::filesystem::exists(dir.path/"late.plastic.bin"));
    std::filesystem::remove(dir.path/"late.frame.json");EXPECT_NO_THROW(WriteFrame(dir.path,"late",c,View(f)));
}
TEST(FullShellRecords,RehashedFalseIdentityAndPhasePreservePriorFrame) {
    Directory dir;const auto c=MakeContext();const auto original=Frame();
    const auto file=WriteFrame(dir.path,"frame",c,View(original));
    const auto bytes=ReadBounded(dir.path/file.file,FrameMetadataByteCap);
    auto visible=ReadFrame(dir.path,c,file,original.stamp);
    for(int bad=0;bad<7;++bad) {
        auto d=array_json::Parse(bytes,FrameMetadataByteCap);
        if(bad==0)d["identity"]["owner"].SetUint64(2);
        if(bad==1)d["attempt"].SetUint64(5); // Valid number, false index association.
        if(bad==2)d["velocity_time_s"].SetDouble(.25);
        if(bad==3)d["point_layout_sha256"].SetString(std::string(64,'c').c_str(),d.GetAllocator());
        if(bad==4)d["nodes"].SetUint64(5);
        if(bad==5)d["plastic"]["fields"][0].SetString("invented_stress",d.GetAllocator());
        if(bad==6)d.AddMember("undeclared",1,d.GetAllocator());
        const auto changed=Rewrite(dir.path,file,d);
        EXPECT_THROW(visible=ReadFrame(dir.path,c,changed,original.stamp),std::exception);
        EXPECT_EQ(visible.stamp.attempt,original.stamp.attempt);EXPECT_EQ(visible.plastic_points,original.plastic_points);
    }
    Overwrite(dir.path/file.file,bytes);EXPECT_NO_THROW(visible=ReadFrame(dir.path,c,file,original.stamp));
}
TEST(FullShellRecords,RehashedNegativeLastNativePointAndTruncationRejectAtomically) {
    Directory dir;const auto c=MakeContext();const auto original=Frame();
    const auto file=WriteFrame(dir.path,"frame",c,View(original));
    const auto metadata=ReadBounded(dir.path/file.file,FrameMetadataByteCap);
    auto d=array_json::Parse(metadata,FrameMetadataByteCap);const auto descriptor=arrays::ParseDescriptor(d["plastic"]);
    const auto old=ReadBounded(dir.path/descriptor.file,descriptor.bytes);
    auto visible=ReadFrame(dir.path,c,file,original.stamp);auto bad=old;
    const auto bits=Bits(-.125);for(unsigned j=0;j<8;++j)bad[bad.size()-8+j]=static_cast<char>(bits>>(8*j));
    Overwrite(dir.path/descriptor.file,bad);const auto hash=Sha256(bad);
    d["plastic"]["sha256"].SetString(hash.c_str(),d.GetAllocator());const auto rehashed=Rewrite(dir.path,file,d);
    EXPECT_THROW(visible=ReadFrame(dir.path,c,rehashed,original.stamp),std::exception);
    EXPECT_EQ(visible.plastic_points,original.plastic_points);
    Overwrite(dir.path/descriptor.file,old.substr(0,old.size()-1));
    EXPECT_THROW(visible=ReadFrame(dir.path,c,rehashed,original.stamp),std::exception);
    Overwrite(dir.path/descriptor.file,old);Overwrite(dir.path/file.file,metadata);
    EXPECT_NO_THROW(visible=ReadFrame(dir.path,c,file,original.stamp));
}
TEST(FullShellRecords,UnavailableFieldsUseEmptyPoolAndInvalidInitialPhaseRejects) {
    auto parents=Parents();
    for(auto& p:parents){p.plastic=PlasticField::Unavailable;p.native_points=0;}
    parents.back().plastic=PlasticField::NotApplicable; // Resolved rigid parent: no invented quadrature.
    auto c=Context::Create(Id(),4,parents.data(),parents.size(),.125);EXPECT_EQ(c.points(),0u);
    auto f=Frame(true);f.plastic_points.clear();Directory dir;
    const auto file=WriteFrame(dir.path,"empty",c,View(f));
    const auto out=ReadFrame(dir.path,c,file,f.stamp);EXPECT_TRUE(out.plastic_points.empty());
    for(const auto& value:ParentPlasticMaxima(c,out))EXPECT_FALSE(value);
    parents.back().plastic=PlasticField::NativeEquivalentPlasticStrain;
    EXPECT_THROW(Context::Create(Id(),4,parents.data(),parents.size(),.125),std::exception);
    auto bad=f;bad.stamp.attempt=1;EXPECT_THROW(CheckFrame(c,View(bad)),std::exception);
    bad=f;bad.stamp.time=std::numeric_limits<double>::max();EXPECT_THROW(CheckFrame(c,View(bad)),std::exception);
    bad=f;bad.stamp.epoch=1;bad.stamp.attempt=1;bad.stamp.time=.125;bad.stamp.velocity_time=.0625;bad.stamp.kick_dt=.0625;
    EXPECT_NO_THROW(CheckFrame(c,View(bad)));bad.stamp.kick_dt=.125;EXPECT_THROW(CheckFrame(c,View(bad)),std::exception);
}
TEST(FullShellRecords,SyntheticFullSourceCountsRoundTripAndLastPointFailure) {
    // These are actual source-scale COUNTS with synthetic identities/values.
    // This is not original geometry or evidence of a physical vehicle interval.
    constexpr std::size_t nodes=359785,parents_count=349645;
    std::vector<ParentPoints> parents(parents_count);
    for(std::size_t i=0;i<parents.size();++i)
        parents[i]={i+1,100+i%867,2,1,3,PlasticField::NativeEquivalentPlasticStrain};
    const auto c=Context::Create(Id(),nodes,parents.data(),parents.size(),.125);
    FrameRecord f{{2,1,4,.25,.125,.1875,.125},std::vector<double>(3*nodes),std::vector<double>(3*parents_count)};
    for(std::size_t i=0;i<f.position_xyz.size();++i)f.position_xyz[i]=.125*static_cast<double>(i);
    f.position_xyz[1]=-0.;f.plastic_points.back()=.03125;
    Directory dir;const auto file=WriteFrame(dir.path,"full",c,View(f));
    const auto out=ReadFrame(dir.path,c,file,f.stamp);
    ASSERT_EQ(out.position_xyz.size(),f.position_xyz.size());ASSERT_EQ(out.plastic_points.size(),f.plastic_points.size());
    for(std::size_t i=0;i<f.position_xyz.size();++i)ASSERT_EQ(Bits(out.position_xyz[i]),Bits(f.position_xyz[i]));
    for(std::size_t i=0;i<f.plastic_points.size();++i)ASSERT_EQ(Bits(out.plastic_points[i]),Bits(f.plastic_points[i]));
    auto invalid=View(f);f.plastic_points.back()=std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(WriteFrame(dir.path,"rejected",c,invalid),std::exception);
    EXPECT_FALSE(std::filesystem::exists(dir.path/"rejected.positions.bin"));
    RecordProperty("synthetic_nodes",nodes);RecordProperty("synthetic_parents",parents_count);
    RecordProperty("binary_value_bytes",8*(f.position_xyz.size()+f.plastic_points.size()));
}
} // namespace crash::output::full_shell::test
