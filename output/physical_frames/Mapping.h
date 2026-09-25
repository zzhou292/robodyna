#pragma once
#include "case/vehicle_runtime/VehiclePhysicalStartup.h"
#include "FieldTypes.h"
#include "output/full_shell/static_bundle/PreparedSourceMapping.h"
namespace crash::output::physical_frames {
namespace records=full_shell;
namespace source=full_shell::source;
using Run=cases::vehicle_runtime::VehiclePhysicalStartup;
using Execution=cases::vehicle_runtime::Execution;
// Original shell/render ordering with an exact physical-owner node scatter.
class Mapping {
  public:
    static Mapping Prepare(const Execution&,std::size_t host_bytes=512u<<20);
    const Execution& execution() const noexcept;
    const source::PreparedSourceMapping& source_mapping() const noexcept;
    const std::vector<std::uint32_t>& physical_nodes() const noexcept;
    const std::vector<ParentField>& parents() const noexcept;
    std::size_t physical_node_count() const noexcept;
    std::size_t payload_bytes() const noexcept;
  private:
    struct Data;
    explicit Mapping(std::shared_ptr<const Data> data):data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
} // namespace crash::output::physical_frames
