#pragma once

namespace crash::cases::vehicle_run {

// Contact composition is independent of the retained structural model profile.
enum class ContactProfile { WallOnly, WallSelfContactV1 };

const char* ContactProfileName(ContactProfile);

}  // namespace crash::cases::vehicle_run
