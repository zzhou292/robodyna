#pragma once
#include "AcceptedInterval.h"
#include "Metadata.h"
#include "output/physical_frames/Archive.h"
namespace crash::output::physical_run {
struct Limits {std::size_t host_bytes=256u<<20;};
struct Forecast {
    records::activity::ActivityPlan archive;
    std::size_t interval_staging_bytes=0,peak_host_bytes=0;
};
// Caller supplies its actual fixed-step horizon and desired sample count.
// A larger ceiling is always explicit; default admission remains two GiB.
records::source::BundleRequest MakeRequest(const records::Context&,std::uint64_t planned_intervals,
    double requested_duration,std::size_t samples,std::size_t total_byte_cap=records::TotalByteCap);
// One externally serialized output session. Append follows common acceptance;
// it never prepares, advances or commits physics. I/O failure poisons this run.
class RunArchive {
  public:
    static Forecast Preflight(const records::source::PreparedSourceMapping&,const records::Context&,
        records::source::BundleRequest,Profile,Limits={});
    static RunArchive Prepare(const std::filesystem::path&,const records::source::PreparedSourceMapping&,
        const records::Context&,records::source::BundleRequest,Profile,Limits={});
    ~RunArchive();
    RunArchive(RunArchive&&) noexcept;
    RunArchive& operator=(RunArchive&&) noexcept;
    RunArchive(const RunArchive&)=delete;
    RunArchive& operator=(const RunArchive&)=delete;
    void Append(const AcceptedInterval&);
    void Sample(const records::FrameRecord&,const records::activity::ActivityRecord&);
    records::RecordFile Finish();
    records::RecordFile FinishPrefix(const std::string& stop_reason);
    const Forecast& forecast() const noexcept;
    std::uint64_t accepted_intervals() const noexcept;
    bool failed() const noexcept;
  private:
    struct Data;
    explicit RunArchive(std::unique_ptr<Data>);
    records::RecordFile Close(bool,const std::string&);
    std::unique_ptr<Data> data_;
};
} // namespace crash::output::physical_run
