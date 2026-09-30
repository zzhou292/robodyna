#pragma once
#include "PhysicalSource.h"
#include "output/full_shell/static_bundle/PreparedSourceMapping.h"
#include "output/physical_frames/FieldTypes.h"
namespace crash::cases::native_scene {
// Create-only static source publication, not a physical accepted-state token.
// It binds the existing binary mapping to the exact immutable physical source.
class ArchiveSource {
  public:
    static ArchiveSource Write(const PhysicalSource&,const std::filesystem::path& empty_directory);
    const PhysicalSource& physical_source() const noexcept;
    const output::full_shell::source::PreparedSourceMapping& mapping() const noexcept;
    const std::vector<std::uint32_t>& physical_nodes() const noexcept;
    const std::vector<output::physical_frames::ParentField>& parents() const noexcept;
  private:
    struct Data;
    explicit ArchiveSource(std::shared_ptr<const Data> data):data_(std::move(data)){}
    std::shared_ptr<const Data> data_;
};
}
