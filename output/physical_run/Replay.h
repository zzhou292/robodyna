#pragma once
#include "Metadata.h"
#include "output/full_shell/static_bundle/SourceBundle.h"
#include <memory>
namespace crash::output::physical_run {
struct ReplayLimits {
    records::source::SourceLimits source;
    records::RecordLimits records;
    std::size_t host_bytes=512u<<20;
};
struct Sample {
    records::FrameRecord frame;
    records::activity::ActivityRecord activity;
};
// Full source + closed-run reader. Caller supplies expected source/mapping and
// outer manifest identity. Every interval and sampled frame/activity is checked
// before this immutable result is returned; seek revalidates payload hashes.
class Replay {
  public:
    static Replay Open(const std::filesystem::path&,const records::RecordFile& expected_manifest,
        const records::source::SourceInputs&,const std::string& expected_mapping_sha256,ReplayLimits={});
    const records::source::PreparedSourceMapping& mapping() const noexcept;
    const records::Context& context() const noexcept;
    const Configuration& configuration() const noexcept;
    const Index& index() const noexcept;
    std::size_t peak_host_bytes() const noexcept;
    Sample ReadSample(std::size_t) const;
    const WallReceipt* wall() const noexcept;
    std::shared_ptr<const chrono::ChTriangleMeshConnected> wall_mesh() const noexcept;
  private:
    struct Data;
    explicit Replay(std::shared_ptr<const Data> data):data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
// Reusable record-only validation for an already authenticated source Context;
// it creates neither source authority nor live acceptance. Visits are staged in
// this function and no caller state is published on a late error.
void ValidateRecords(const std::filesystem::path&,const records::Context&,const Configuration&,const Index&,
    std::size_t workspace_cap);
} // namespace crash::output::physical_run
