#pragma once
#include "output/full_shell/activity/ActivityRecord.h"
#include "output/full_shell/static_bundle/SourceBundle.h"
namespace crash::output::physical_frames {
namespace records=full_shell;
namespace source=full_shell::source;
struct FrameFiles {
    records::RecordFile frame,activity;
    records::FrameStamp stamp;
};
// Bounded per-frame/source publisher. The plan reserves the complete future run
// (all intervals, static obligations, sampled frames and one prefix frame), but
// this object emits no run completion marker or interval/trajectory claim.
class Archive {
  public:
    static Archive Prepare(const source::PreparedSourceMapping&,const records::Context&,
        source::BundleRequest,std::size_t host_bytes=128u<<20);
    const source::PreparedSourceBundle& source_bundle() const noexcept;
    const records::activity::ActivityPlan& plan() const noexcept;
    std::size_t startup_host_bytes() const noexcept;
    std::size_t written_frames() const noexcept;
    FrameFiles Write(const std::filesystem::path&,const std::string& stem,
        const records::FrameRecord&,const records::activity::ActivityRecord&);
  private:
    struct Data;
    explicit Archive(std::unique_ptr<Data>);
  public:
    ~Archive();
    Archive(Archive&&) noexcept;
    Archive& operator=(Archive&&) noexcept;
    Archive(const Archive&)=delete;
    Archive& operator=(const Archive&)=delete;
  private:
    std::unique_ptr<Data> data_;
};
} // namespace crash::output::physical_frames
