#pragma once
#include <filesystem>
namespace crash::cases::vehicle_run {
struct OriginalPaths {
    std::filesystem::path canonical,scope,member,declarations,glass_resolution;
    std::filesystem::path type13,auxiliary_member,original_wall_member,wall_manifest;
    // Required only for the explicit wall+self-contact profile. The original
    // auxiliary member above supplies its part sets; this supplies the card.
    std::filesystem::path self_contact_combine_member;
};
}
