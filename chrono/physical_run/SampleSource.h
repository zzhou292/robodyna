#pragma once
#include "output/physical_run/Replay.h"
#include "output/recovered_frames/Replay.h"
#include <variant>

namespace crash::visual::physical_run {
// Display-only source union. Recovery never manufactures a normal run Index.
class SampleSource {
  public:
    explicit SampleSource(const output::physical_run::Replay& replay):value_(replay) {}
    explicit SampleSource(const output::recovered_frames::Replay& replay):value_(replay) {}
    const output::physical_run::Replay* normal() const noexcept;
    const output::recovered_frames::Replay* recovered() const noexcept;
    const output::full_shell::source::PreparedSourceMapping& mapping() const noexcept;
    const output::full_shell::Context& context() const noexcept;
    const output::physical_run::Configuration& configuration() const noexcept;
    std::size_t peak_host_bytes() const noexcept;
    const std::vector<output::physical_run::FrameFiles>& frames() const noexcept;
    output::physical_run::Sample ReadSample(std::size_t) const;
    const output::physical_run::WallReceipt* wall() const noexcept;
    const output::physical_run::EnvironmentReceipt* environment() const noexcept;
    std::shared_ptr<const chrono::ChTriangleMeshConnected> wall_mesh() const noexcept;
    const std::string& stop_reason() const noexcept;
  private:
    std::variant<output::physical_run::Replay, output::recovered_frames::Replay> value_;
};
} // namespace crash::visual::physical_run
