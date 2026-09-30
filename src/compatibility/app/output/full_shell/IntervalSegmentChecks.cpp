#include "IntervalSegments.h"
#include <iomanip>
#include <sstream>

namespace crash::output::full_shell {
std::string IntervalArrayName(const std::string& stem,std::size_t index,bool integers) {
    arrays::CheckRelativeName(stem);
    Require(stem.size()<=128&&stem.find('/')==std::string::npos&&index<64,"Invalid interval output name");
    std::ostringstream name;
    name<<stem<<'-'<<std::setfill('0')<<std::setw(4)<<index<<(integers?".ids.bin":".values.bin");
    return name.str();
}

void CheckSegment(const IntervalContext& c,const IntervalSegment& s) {
    CheckIntervalContext(c);
    const auto plan=interval::PlanChunks(c.planned_intervals,c.limits.file_bytes);
    Require(SameIdentity(c.identity,s.identity)&&Bits(c.fixed_dt)==Bits(s.fixed_dt)&&
        s.index<plan.chunks&&s.row_count&&s.row_count<=plan.rows_per_chunk&&
        s.first_epoch==s.index*plan.rows_per_chunk+1&&
        s.row_count<=c.planned_intervals-s.first_epoch+1,"Interval segment source/range identity mismatch");
    CheckCursor(c,s.before);
    CheckCursor(c,s.after);
    Require(s.before.epoch==s.first_epoch-1&&s.after.epoch==s.first_epoch+s.row_count-1&&
        s.after.attempt>s.before.attempt,"Interval segment cursor range mismatch");
    const arrays::Limits limits{c.limits.file_bytes,UINT32_MAX,64};
    arrays::CheckDescriptor(s.integers,limits);
    arrays::CheckDescriptor(s.reals,limits);
    Require(s.integers.layout.scalar==arrays::Scalar::UInt64&&s.integers.layout.rows==s.row_count&&
        s.integers.layout.columns==interval::IntegerCount&&s.integers.layout.fields==interval::IntegerFields()&&
        s.reals.layout.scalar==arrays::Scalar::Float64&&s.reals.layout.rows==s.row_count&&
        s.reals.layout.columns==interval::RealCount&&s.reals.layout.fields==interval::RealFields()&&
        s.integers.file!=s.reals.file&&s.integers.bytes+s.reals.bytes==s.row_count*interval::RowBytes,
        "Interval segment array representation changed");
}
} // namespace crash::output::full_shell
