#pragma once
#include "TestSupport.h"
#include "output/full_shell/IntervalSegments.h"

namespace crash::output::full_shell::test {
inline IntervalContext Intervals(std::uint64_t count=5) {
    auto id=Id();id.owner=UINT64_C(9007199254740993);
    return {id,.125,count,{4,4,4,.5,1.,2.,2.,2*interval::RowBytes,4096}};
}
inline interval::Values Row(const IntervalContext& c,std::uint64_t epoch) {
    using namespace interval;
    Values v;
    v.integers={c.identity.owner,epoch-1,UINT64_C(18014398509481984)+2*epoch,epoch};
    auto& r=v.reals;
    r[BaseTime]=(epoch-1)*c.fixed_dt;
    r[Time]=r[BaseTime]+c.fixed_dt;
    r[VelocityTime]=r[BaseTime]+.5*c.fixed_dt;
    r[KickDt]=epoch==1?.5*c.fixed_dt:c.fixed_dt;
    r[NativeKinetic]=10;
    r[EffectiveKinetic]=10;
    r[NativeInternalWork]=-0.;
    r[CumulativePlasticWork]=.125*epoch;
    r[NativeKickDelta]=-.125*epoch;
    r[EffectiveDelta]=-.125*epoch;
    r[AppliedKickWork]=-.125*epoch;
    r[RoundoffBudget]=1e-12;
    r[MaximumPlasticStrain]=.001*epoch;
    r[MaximumAreaRatio]=1;
    r[MaximumThicknessRatio]=1;
    if(epoch==2||epoch==3) {
        r[ActiveNodes]=1;
        r[MaximumPenetration]=.01;
        r[WallResultant]=3;
        r[WallPotential]=.5;
    }
    return v;
}
inline void Append(IntervalWriter& writer,const interval::Values& row) {
    writer.Append(row,IntervalStamp(row));
}
inline void Equal(const interval::Values& a,const interval::Values& b) {
    EXPECT_EQ(a.integers,b.integers);
    for(std::size_t i=0;i<a.reals.size();++i)EXPECT_EQ(Bits(a.reals[i]),Bits(b.reals[i]))<<i;
}
inline IntervalLedger Complete(const std::filesystem::path& dir,const IntervalContext& c) {
    IntervalWriter writer(dir,"intervals",c);
    for(std::uint64_t epoch=1;epoch<=c.planned_intervals;++epoch)Append(writer,Row(c,epoch));
    return writer.Finish();
}
} // namespace crash::output::full_shell::test
