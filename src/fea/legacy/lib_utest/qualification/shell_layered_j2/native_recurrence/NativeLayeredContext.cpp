#include "NativeLayeredInput.h"
namespace tl::qualification::layered_native::detail {
std::mutex& SectionContext() noexcept { static std::mutex context;return context; }
}
