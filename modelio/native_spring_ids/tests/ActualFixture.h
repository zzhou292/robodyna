#pragma once
#include "../Resolve.h"
#include "case/vehicle_startup/physical_model/tests/supports/Support.h"
#include <cstdlib>
namespace crash::modelio::native_spring_ids::test {
namespace physical = cases::vehicle_startup::physical_model::supports_test;
inline std::string FixtureMember(const char* variable, std::size_t limit) {
    const auto* path=std::getenv(variable); output::Require(path && *path,"Missing authenticated SPRING source fixture member");
    return output::ReadBounded(path,limit);
}
// Reuse the existing complete canonical/include fixture, not a second loader.
// The three env paths are supplied by self_contact/tests/actual_fixture.py.
inline const ImportContext& ActualImportContext() {
    static const auto context=[] {
        const auto auxiliary=FixtureMember("ROBO_SELF_CONTACT_AUX_MEMBER",1u<<20);
        const auto combine=FixtureMember("ROBO_SELF_CONTACT_COMBINE_MEMBER",64u<<10);
        const auto wall=FixtureMember("ROBO_TIED_WALL_MEMBER",64u<<10);
        ImportMembers input; input.profile=Profile::DirectKeywordR14FreshRadiossPoSortById;
        input.entry_member="combine.key";
        input.members={{"yaris-coarse-v1l.key",physical::Inputs().member},
            {"set-yaris-coarse-v1l.key",auxiliary},{"combine.key",combine},{"wall.key",wall}};
        return ImportContext::Prepare(vehicle::test::Canonical(),input);
    }();
    return context;
}
inline const Resolution& ActualResolution() {
    static const auto value=Resolve(physical::Inputs().beams,physical::Model().welds(),
        physical::JointSource(),ActualImportContext());
    return value;
}
} // namespace crash::modelio::native_spring_ids::test
