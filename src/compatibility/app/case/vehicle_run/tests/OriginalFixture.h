#pragma once
#include "../source/OriginalYaris.h"
#include "output/full_shell/tests/TestSupport.h"
#include <cstdlib>
namespace crash::cases::vehicle_run::test {
inline OriginalCase Source(PhysicalProfile profile=PhysicalProfile::RetainedShellAssembliesV1,
                           const vehicle_wall::Settings* declared_settings=nullptr,
                           ContactProfile contact_profile=ContactProfile::WallOnly) {
    const auto path=[](const char* name) {
        const auto value=std::getenv(name);
        output::Require(value && *value,"Explicit complete original source path required");
        return std::filesystem::path(value);
    };
    OriginalPaths paths{path("ROBO_STATIC_CANONICAL"),path("ROBO_STATIC_SCOPE"),path("ROBO_STATIC_MEMBER"),
        path("ROBO_VEHICLE_DECLARATIONS"),path("ROBO_VEHICLE_GLASS_RESOLUTION"),path("ROBO_DYNA_TYPE13_DECLARATION"),
        path("ROBO_TIED_AUX_MEMBER"),path("ROBO_TIED_WALL_MEMBER"),path("ROBO_VEHICLE_WALL")};
    if(contact_profile==ContactProfile::WallSelfContactV1)
        paths.self_contact_combine_member=path("ROBO_SELF_CONTACT_COMBINE_MEMBER");
    auto settings=vehicle_wall::LoadedWallSettings();
    settings.leading_gap_m=1e-6; // Explicit short contact gate, distinct from ordinary20mm gap.
    settings.requested_duration_s=.005;
    if(declared_settings) settings=*declared_settings;
    return PrepareOriginalYaris(paths,settings,profile,contact_profile);
}
inline records::Identity Identity() {
    records::Identity value;
    value.run=0x52554e5941524953ULL;
    value.topology=0x5941524953ULL;
    return value;
}
void CheckLoadedPrefix(PhysicalProfile);
}
