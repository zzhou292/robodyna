#pragma once
#include <cstddef>

namespace tl::fea {
// Compatibility defaults. Other contributors retain their own admission.
inline constexpr std::size_t MaxTranslationNodes=128;
inline constexpr std::size_t MaxNodalStateNodes=2048;
inline constexpr std::size_t MaxTranslationDeviceBytes=1024*1024;
// Explicit active owner admission only; this is not a shell/contact/model gate.
inline constexpr std::size_t MaxActiveNodalStateNodes=524288;
inline constexpr std::size_t MaxActiveNodalStateDeviceBytes=256*1024*1024;
} // namespace tl::fea
