#pragma once
#include "IntervalSequence.h"
#include <memory>

namespace crash::output::full_shell {
inline constexpr const char* IntervalSegmentSchema="robo_dyna.accepted_interval_segment.v1";
struct IntervalSegment {
    Identity identity;
    double fixed_dt=0;
    std::size_t index=0;
    std::uint64_t first_epoch=0,row_count=0;
    IntervalCursor before,after;
    arrays::Descriptor integers,reals;
};
struct IntervalLedger {
    std::uint64_t planned_intervals=0,accepted_intervals=0;
    bool horizon_complete=false;
    std::vector<IntervalSegment> segments;
};
Document SegmentDocument(const IntervalContext&,const IntervalSegment&);
IntervalSegment ParseSegmentDocument(const IntervalContext&,const Value&);
// Metadata checks only; caller authenticates this description via its manifest.
void CheckSegment(const IntervalContext&,const IntervalSegment&);
std::string IntervalArrayName(const std::string& stem,std::size_t index,bool integers);

class ValidatedIntervalSegment {
  public:
    ValidatedIntervalSegment(ValidatedIntervalSegment&&) noexcept=default;
    ValidatedIntervalSegment& operator=(ValidatedIntervalSegment&&) noexcept=default;
    ValidatedIntervalSegment(const ValidatedIntervalSegment&)=delete;
    ValidatedIntervalSegment& operator=(const ValidatedIntervalSegment&)=delete;
    std::size_t rows() const noexcept {return integers_.size()/interval::IntegerCount;}
    interval::Values row(std::size_t) const;
    const IntervalCursor& after() const noexcept {return after_;}
  private:
    ValidatedIntervalSegment()=default;
    std::vector<std::uint64_t> integers_;
    std::vector<double> reals_;
    IntervalCursor after_;
    friend ValidatedIntervalSegment ReadIntervalSegment(const std::filesystem::path&,const IntervalContext&,
        std::uint64_t,bool,std::size_t,const IntervalCursor&,const IntervalSegment&);
};
// Expected completion/range/prior cursor comes from caller-authenticated index
// metadata. The complete segment is checked before a staged value is returned.
ValidatedIntervalSegment ReadIntervalSegment(const std::filesystem::path&,const IntervalContext&,
    std::uint64_t expected_accepted_intervals,bool horizon_complete,std::size_t expected_index,
    const IntervalCursor& expected_before,const IntervalSegment&);

// One preallocated active chunk pair. No solver, acceptance token or complete
// run manifest. Invalid input is retryable before mutation; write failure is
// sticky and leaves incomplete create-only files as evidence.
class IntervalWriter {
  public:
    IntervalWriter(const std::filesystem::path&,std::string stem,const IntervalContext&);
    ~IntervalWriter();
    IntervalWriter(const IntervalWriter&)=delete;
    IntervalWriter& operator=(const IntervalWriter&)=delete;
    void Append(const interval::Values&,const FrameStamp& expected_accepted_phase);
    IntervalLedger Finish();
    IntervalLedger FinishPrefix();
    void Abort() noexcept;
    bool failed() const noexcept;
    // Received rows include the buffered chunk. Persistence is established only
    // by a successful Finish/FinishPrefix result, never by this counter.
    std::uint64_t rows_received() const noexcept;
    std::size_t buffer_bytes() const noexcept;
  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    void Flush();
    IntervalLedger Close(bool prefix);
};
} // namespace crash::output::full_shell
