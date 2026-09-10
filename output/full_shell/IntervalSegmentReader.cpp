#include "IntervalSegments.h"
#include <algorithm>

namespace crash::output::full_shell {
interval::Values ValidatedIntervalSegment::row(std::size_t index) const {
    Require(index<rows(),"Interval row index exceeds staged segment");
    interval::Values result;
    std::copy_n(integers_.data()+index*interval::IntegerCount,interval::IntegerCount,result.integers.begin());
    std::copy_n(reals_.data()+index*interval::RealCount,interval::RealCount,result.reals.begin());
    return result;
}

ValidatedIntervalSegment ReadIntervalSegment(const std::filesystem::path& root,const IntervalContext& c,
        std::uint64_t accepted,bool complete,std::size_t index,const IntervalCursor& before,const IntervalSegment& s) {
    CheckSegment(c,s);
    Require(accepted&&accepted<=c.planned_intervals&&complete==(accepted==c.planned_intervals),
        "Interval ledger is not the declared complete run/prefix");
    const auto completed=interval::PlanChunks(accepted,c.limits.file_bytes);
    Require(index<completed.chunks&&s.index==index&&SameCursor(s.before,before),
        "Interval segment does not follow the expected accepted cursor");
    const auto rows=std::min(completed.rows_per_chunk,accepted-index*completed.rows_per_chunk);
    Require(s.row_count==rows,"Interval segment is missing rows or has an undeclared tail");
    const arrays::Limits limits{c.limits.file_bytes,UINT32_MAX,64};
    ValidatedIntervalSegment staged;
    staged.integers_=arrays::Read<std::uint64_t>(root,s.integers,limits);
    staged.reals_=arrays::Read<double>(root,s.reals,limits);
    auto cursor=before;
    for(std::size_t row=0;row<staged.rows();++row)cursor=AdvanceInterval(c,cursor,staged.row(row));
    Require(SameCursor(cursor,s.after),"Interval segment terminal phase/history changed");
    staged.after_=cursor;
    return staged;
}
} // namespace crash::output::full_shell
