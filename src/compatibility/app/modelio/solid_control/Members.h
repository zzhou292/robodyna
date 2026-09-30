#pragma once
#include "Types.h"
#include "output/ArtifactIO.h"
namespace crash::modelio::solid_control::detail {
using output::Require;
inline std::string ReadMember(const ids::ImportMembers& members, const char* name, std::size_t cap) {
    const ids::Member* found = nullptr;
    for (const auto& member : members.members) {
        if (member.filename != name) continue;
        Require(!found && member.bytes.size() <= cap, "Duplicate or oversized contact context member");
        found = &member;
    }
    Require(found, "Missing authenticated contact context member");
    return std::string(found->bytes);
}
}
