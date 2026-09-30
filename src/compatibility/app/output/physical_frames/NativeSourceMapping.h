#pragma once
#include "FieldTypes.h"
#include "output/full_shell/static_bundle/PreparedSourceMapping.h"
#include "lib_src/assembly/ShellPhysicalBinding.h"
namespace crash::output::physical_frames::detail {
struct NativeSourceMapping {
    std::vector<std::uint32_t> nodes;
    std::vector<ParentField> parents;
};
// Immutable source checks only. No owner/publication authority is conferred.
// Bound includes returned rows, decoded arrays and their temporary byte strings.
std::size_t NativeMappingBytes(const records::source::PreparedSourceMapping&,std::size_t cap);
NativeSourceMapping BindNativeSource(const records::source::PreparedSourceMapping&,
    const tl::fea::ShellPhysicalBinding&,std::size_t cap);
}
