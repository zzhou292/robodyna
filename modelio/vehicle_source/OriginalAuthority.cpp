#include "OriginalAuthority.h"
namespace crash::modelio::vehicle {
void CheckOriginalYarisAuthority(const output::full_shell::source::CanonicalData& source) {
    output::Require(source.archive_sha256 == "aff8194c456726a678d6cc11f644316ca70f3d9b37c4db622726b7b2985b0451" &&
        source.inputs.canonical_manifest.sha256 == "c82f1886b8935d69ff7db4c29c700370e3a057579fab80d02664a253bc7af1c8" &&
        source.inputs.source_member.sha256 == "67208317e6c8eb1dd43b80001508915ccaace7bc0a745e1aa5a3b33f394df301" &&
        source.inputs.source_member.bytes == 42846753 && source.canonical_nodes == 393165,
        "Original solid source authority changed");
    const auto& units = source.inputs.units;
    output::Require(units.mass == "t" && units.length == "mm" && units.time == "s" &&
        units.mass_to_kg == 1000 && units.length_to_m == .001 && units.time_to_s == 1,
        "Original solid source units changed");
}
}
