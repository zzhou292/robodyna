#include "ContactProfile.h"

#include <stdexcept>

namespace crash::cases::vehicle_run {

const char* ContactProfileName(ContactProfile profile) {
    switch (profile) {
        case ContactProfile::WallOnly:
            return "wall-only";
        case ContactProfile::WallSelfContactV1:
            return "wall-self-contact-v1";
    }
    throw std::invalid_argument("Unknown vehicle contact profile");
}

}  // namespace crash::cases::vehicle_run
