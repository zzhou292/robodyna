#include "IntervalSegments.h"
#include <algorithm>
#include <type_traits>

namespace crash::output::full_shell {
struct IntervalWriter::Impl {
    std::filesystem::path root;
    std::string stem;
    IntervalContext context;
    interval::ChunkPlan plan;
    IntervalCursor cursor,chunk_before;
    std::size_t buffered=0;
    std::vector<std::uint64_t> integers;
    std::vector<double> reals;
    std::vector<IntervalSegment> segments;
    bool failed=false,finished=false;
};

IntervalWriter::IntervalWriter(const std::filesystem::path& root,std::string stem,const IntervalContext& context) {
    CheckIntervalContext(context);
    const auto plan=interval::PlanChunks(context.planned_intervals,context.limits.file_bytes);
    for(std::size_t i=0;i<plan.chunks;++i) {
        arrays::CheckedPath(root,IntervalArrayName(stem,i,true),false);
        arrays::CheckedPath(root,IntervalArrayName(stem,i,false),false);
    }
    auto state=std::make_unique<Impl>();
    state->root=root;
    state->stem=std::move(stem);
    state->context=context;
    state->plan=plan;
    const auto active_rows=std::min(plan.rows_per_chunk,context.planned_intervals);
    state->integers.resize(active_rows*interval::IntegerCount);
    state->reals.resize(active_rows*interval::RealCount);
    state->segments.reserve(plan.chunks);
    impl_=std::move(state);
}

IntervalWriter::~IntervalWriter()=default;
void IntervalWriter::Append(const interval::Values& values,const FrameStamp& expected) {
    auto& s=*impl_;
    Require(!s.failed&&!s.finished,"Interval writer is closed or poisoned");
    CheckStamp(s.context.fixed_dt,expected);
    Require(SameStamp(expected,IntervalStamp(values)),"Interval values do not match accepted common phase");
    const auto after=AdvanceInterval(s.context,s.cursor,values);
    std::copy(values.integers.begin(),values.integers.end(),s.integers.begin()+s.buffered*interval::IntegerCount);
    std::copy(values.reals.begin(),values.reals.end(),s.reals.begin()+s.buffered*interval::RealCount);
    s.cursor=after;
    ++s.buffered;
    if(s.buffered==s.plan.rows_per_chunk) {
        try {Flush();} catch(...) {Abort();throw;}
    }
}

void IntervalWriter::Flush() {
    auto& s=*impl_;
    if(!s.buffered)return;
    IntervalSegment segment;
    segment.identity=s.context.identity;
    segment.fixed_dt=s.context.fixed_dt;
    segment.index=s.segments.size();
    segment.first_epoch=s.chunk_before.epoch+1;
    segment.row_count=s.buffered;
    segment.before=s.chunk_before;
    segment.after=s.cursor;
    const arrays::Limits limits{s.context.limits.file_bytes,UINT32_MAX,64};
    const auto integer_name=IntervalArrayName(s.stem,segment.index,true);
    const auto real_name=IntervalArrayName(s.stem,segment.index,false);
    // Recheck both destinations before creating either file.
    arrays::CheckedPath(s.root,integer_name,false);
    arrays::CheckedPath(s.root,real_name,false);
    segment.integers=arrays::Write(s.root,integer_name,
        {arrays::Scalar::UInt64,s.buffered,interval::IntegerCount,interval::IntegerFields()},
        s.integers.data(),s.buffered*interval::IntegerCount,limits);
    segment.reals=arrays::Write(s.root,real_name,
        {arrays::Scalar::Float64,s.buffered,interval::RealCount,interval::RealFields()},
        s.reals.data(),s.buffered*interval::RealCount,limits);
    static_assert(std::is_nothrow_move_constructible_v<IntervalSegment>);
    s.segments.push_back(std::move(segment)); // Startup-reserved slots.
    s.chunk_before=s.cursor;
    s.buffered=0;
}

IntervalLedger IntervalWriter::Close(bool prefix) {
    auto& s=*impl_;
    Require(!s.failed&&!s.finished&&s.cursor.epoch&&
        (prefix||s.cursor.epoch==s.context.planned_intervals),"Interval ledger has no complete declared horizon");
    try {Flush();} catch(...) {Abort();throw;}
    s.finished=true;
    return {s.context.planned_intervals,s.cursor.epoch,s.cursor.epoch==s.context.planned_intervals,std::move(s.segments)};
}
IntervalLedger IntervalWriter::Finish() {return Close(false);}
IntervalLedger IntervalWriter::FinishPrefix() {return Close(true);}
void IntervalWriter::Abort() noexcept {if(impl_&&!impl_->finished)impl_->failed=true;}
bool IntervalWriter::failed() const noexcept {return impl_->failed;}
std::uint64_t IntervalWriter::rows_received() const noexcept {return impl_->cursor.epoch;}
std::size_t IntervalWriter::buffer_bytes() const noexcept {
    return impl_->integers.size()*sizeof(std::uint64_t)+impl_->reals.size()*sizeof(double);
}
} // namespace crash::output::full_shell
