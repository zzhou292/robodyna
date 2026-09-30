#pragma once
#include "lib_src/collision/radioss_type25/runtime/Types.h"
#include <stdexcept>
#include <string_view>
namespace crash::cases::vehicle_native_contact::test {
inline tlfea::contact::radioss_type25::ContactActivityPolicy ParseContactActivity(const char* value) {
    using Policy = tlfea::contact::radioss_type25::ContactActivityPolicy;
    if (!value || std::string_view(value) == "all_active_prefix") return Policy::AllActivePrefix;
    if (std::string_view(value) == "shell_removal") return Policy::ShellRemoval;
    throw std::invalid_argument("ROBO_NATIVE_CONTACT_ACTIVITY must be all_active_prefix or shell_removal");
}
}
