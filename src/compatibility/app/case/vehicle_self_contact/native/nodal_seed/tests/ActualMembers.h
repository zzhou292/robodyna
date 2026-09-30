#pragma once
#include "../Internal.h"
#include "modelio/native_spring_ids/tests/ActualFixture.h"

namespace crash::cases::vehicle_self_contact::native::nodal_seed::test {
// Lifetime adapter over the resolver's existing bounded actual-member fixture.
// No second directory walker, metadata reader or source declaration is added.
class ActualMembers {
  public:
    explicit ActualMembers(const output::full_shell::source::CanonicalSource& source)
        : auxiliary_(modelio::native_spring_ids::test::FixtureMember("ROBO_SELF_CONTACT_AUX_MEMBER", 1u << 20)),
          combine_(modelio::native_spring_ids::test::FixtureMember("ROBO_SELF_CONTACT_COMBINE_MEMBER", 64u << 10)),
          wall_(modelio::native_spring_ids::test::FixtureMember("ROBO_TIED_WALL_MEMBER", 64u << 10)) {
        output::Require(&source.data() == &modelio::vehicle::test::Canonical().data(),
            "Actual seed/import fixture canonical authority differs");
    }
    detail::ids::ImportMembers Input() const {
        detail::ids::ImportMembers result;
        result.profile = detail::ids::Profile::DirectKeywordR14FreshRadiossPoSortById;
        result.entry_member = "combine.key";
        result.members = {{"yaris-coarse-v1l.key", vehicle_startup::physical_model::supports_test::Inputs().member},
            {"set-yaris-coarse-v1l.key", auxiliary_}, {"combine.key", combine_}, {"wall.key", wall_}};
        return result;
    }
  private:
    std::string auxiliary_, combine_, wall_;
};
} // namespace crash::cases::vehicle_self_contact::native::nodal_seed::test
