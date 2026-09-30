#include "SampleSource.h"
namespace crash::visual::physical_run {
const output::physical_run::Replay* SampleSource::normal() const noexcept {
    return std::get_if<output::physical_run::Replay>(&value_);
}
const output::recovered_frames::Replay* SampleSource::recovered() const noexcept {
    return std::get_if<output::recovered_frames::Replay>(&value_);
}
const output::full_shell::source::PreparedSourceMapping& SampleSource::mapping() const noexcept {
    return std::visit([](const auto& r) -> const output::full_shell::source::PreparedSourceMapping& {return r.mapping();}, value_);
}
const output::full_shell::Context& SampleSource::context() const noexcept {
    return std::visit([](const auto& r) -> const output::full_shell::Context& {return r.context();}, value_);
}
const output::physical_run::Configuration& SampleSource::configuration() const noexcept {
    return std::visit([](const auto& r) -> const output::physical_run::Configuration& {return r.configuration();}, value_);
}
std::size_t SampleSource::peak_host_bytes() const noexcept {
    return std::visit([](const auto& r) {return r.peak_host_bytes();}, value_);
}
const std::vector<output::physical_run::FrameFiles>& SampleSource::frames() const noexcept {
    if (const auto* r = normal()) return r->index().frames;
    return recovered()->frames();
}
output::physical_run::Sample SampleSource::ReadSample(std::size_t index) const {
    return std::visit([&](const auto& r) {return r.ReadSample(index);}, value_);
}
const output::physical_run::WallReceipt* SampleSource::wall() const noexcept {
    return std::visit([](const auto& r) {return r.wall();}, value_);
}
const output::physical_run::EnvironmentReceipt* SampleSource::environment() const noexcept {
    return std::visit([](const auto& source){return source.environment();},value_);
}
std::shared_ptr<const chrono::ChTriangleMeshConnected> SampleSource::wall_mesh() const noexcept {
    return std::visit([](const auto& r) {return r.wall_mesh();}, value_);
}
const std::string& SampleSource::stop_reason() const noexcept {
    if (const auto* r = normal()) return r->index().stop_reason;
    return recovered()->stop_reason();
}
} // namespace crash::visual::physical_run
