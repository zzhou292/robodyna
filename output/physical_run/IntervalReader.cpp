#include "IntervalIO.h"
#include <algorithm>
#include <limits>
namespace crash::output::physical_run {
namespace {
std::size_t StagingBytes(Profile profile,const interval::ChunkPlan& plan) {
    const auto columns=IntegerFields(profile).size()+RealFields(profile).size();
    const auto rows=std::min(plan.rows_per_chunk,plan.intervals);
    constexpr std::size_t CopiesAndScalarBytes=3*8;
    Require(columns && columns<=std::numeric_limits<std::size_t>::max()/CopiesAndScalarBytes &&
        rows<=std::numeric_limits<std::size_t>::max()/(CopiesAndScalarBytes*columns),
        "Physical interval read staging size is unrepresentable");
    return static_cast<std::size_t>(rows)*CopiesAndScalarBytes*columns;
}
} // namespace
std::size_t IntervalReadStagingBytes(Profile profile,std::uint64_t planned,std::size_t file_cap) {
    return StagingBytes(profile,interval::PlanChunks(
        planned,file_cap,kArtifactMaximumTotalCap,ExtraIntervalBytes(profile)));
}
Sequence ReadIntervals(const std::filesystem::path& root,const records::Context& c,Profile p,
    std::uint64_t planned,std::uint64_t accepted,const std::vector<Segment>& segments,
    std::size_t file_cap,std::size_t host_cap,const std::function<void(const Values&)>& visit) {
    const auto plan=interval::PlanChunks(planned,file_cap,kArtifactMaximumTotalCap,ExtraIntervalBytes(p));
    Require(accepted<=planned && segments.size()==accepted/plan.rows_per_chunk+(accepted%plan.rows_per_chunk!=0),
        "Incomplete physical interval segment coverage");
    const auto cols=RealFields(p).size();
    const auto ints=IntegerFields(p).size();
    Require(host_cap && host_cap<=256u<<20 && StagingBytes(p,plan)<=host_cap,
        "Physical interval read staging exceeds host cap");
    const arrays::Limits limits{file_cap,UINT32_MAX,64};
    Sequence sequence;
    for(std::size_t k=0;k<segments.size();++k) {
        const auto& s=segments[k];
        const auto count=std::min(plan.rows_per_chunk,accepted-sequence.last.epoch);
        const auto stem="interval-"+std::to_string(k);
        Require(s.first_epoch==sequence.last.epoch+1 && s.rows==count &&
            s.integers.file==stem+".integers.bin" && s.reals.file==stem+".reals.bin" &&
            s.integers.layout.scalar==arrays::Scalar::UInt64 && s.integers.layout.rows==count &&
            s.integers.layout.columns==ints && s.integers.layout.fields==IntegerFields(p) &&
            s.reals.layout.scalar==arrays::Scalar::Float64 && s.reals.layout.rows==count &&
            s.reals.layout.columns==cols && s.reals.layout.fields==RealFields(p),"Physical interval layout/order differs");
        const auto integers=arrays::Read<std::uint64_t>(root,s.integers,limits);
        const auto reals=arrays::Read<double>(root,s.reals,limits);
        for(std::size_t i=0;i<count;++i) {
            const auto* a=integers.data()+ints*i;const auto* b=reals.data()+cols*i;
            Values value{a[0],{a[3],a[1],a[2],b[1],b[0],b[2],b[3]},std::nullopt};
            if(p.structural_limit)value.structural_limit_s=b[4];
            if(p.self_contact)value.self_contact=DecodeSelfContact(a+4,b+4+std::size_t(p.structural_limit));
            if(p.native_contact)value.native_contact=DecodeNativeContact(a+4,b+4+std::size_t(p.structural_limit));
            if(p.native_group)value.native_group=DecodeNativeGroup(a+4,b+4+std::size_t(p.structural_limit));
            sequence=Advance(c,p,planned,sequence,value);
            if(visit)visit(value);
        }
    }
    Require(sequence.last.epoch==accepted,"Physical ledger final count differs");return sequence;
}
} // namespace crash::output::physical_run
