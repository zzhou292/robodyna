#pragma once
#include "output/physical_run/Replay.h"

namespace crash::output::recovered_frames {
inline constexpr const char* Schema = "robo_dyna.recovered_sampled_review.v1";
inline constexpr const char* EnvironmentSchema = "robo_dyna.recovered_sampled_review.v2";
inline constexpr const char* DescriptorFilename = "recovered-samples.json";
using Limits = physical_run::ReplayLimits;

// Read an interrupted source archive, validate its source and complete scheduled
// frame/activity pairs, and copy only named immutable payloads into a real empty
// destination. Publish the distinct recovery descriptor last. No interval rows,
// ordinary run manifest, accepted-step continuity or completion are fabricated.
full_shell::RecordFile Recover(const std::filesystem::path& interrupted_archive,
    const std::filesystem::path& empty_destination_archive,
    const full_shell::source::SourceInputs& expected_source,
    const std::string& expected_mapping_sha256, const std::string& stop_reason, Limits = {});

// Caller selects and authenticates the outer descriptor. Its embedded source
// authority then feeds the existing typed source/frame/activity/wall readers.
// Recorded stamps are preserved; intervening accepted interval history is
// unavailable. This immutable handle has no normal run Index/completion API.
class Replay {
  public:
    static Replay Open(const std::filesystem::path& archive,
        const full_shell::RecordFile& expected_descriptor, Limits = {});
    const full_shell::source::PreparedSourceMapping& mapping() const noexcept;
    const full_shell::Context& context() const noexcept;
    const physical_run::Configuration& configuration() const noexcept;
    std::size_t peak_host_bytes() const noexcept;
    const std::vector<physical_run::FrameFiles>& frames() const noexcept;
    physical_run::Sample ReadSample(std::size_t) const;
    const physical_run::WallReceipt* wall() const noexcept;
    const physical_run::EnvironmentReceipt* environment() const noexcept;
    const physical_run::WallComposition* wall_composition() const noexcept;
    std::shared_ptr<const chrono::ChTriangleMeshConnected> wall_mesh() const noexcept;
    const std::string& stop_reason() const noexcept;
    const full_shell::RecordFile& descriptor() const noexcept;
  private:
    struct Data;
    explicit Replay(std::shared_ptr<const Data> data):data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
} // namespace crash::output::recovered_frames
