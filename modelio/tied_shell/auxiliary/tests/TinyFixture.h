#pragma once
#include "../Internal.h"
#include "../../tests/TinyFixture.h"

namespace crash::modelio::tied_shell::auxiliary_test {
struct Tiny {
    test::TinyFixture base;
    std::string member;
    Tiny();
    AuxiliaryData Prepare(OriginalWallPolicy policy = OriginalWallPolicy::ReplaceWithMeshWall,
                          AuxiliaryLimits limits = {}) const;
    void AlterMember(const std::function<void(std::string&)>&, bool rehash_blocks = false);
    void AlterCanonical(const std::function<void(output::Document&)>&);
};
} // namespace crash::modelio::tied_shell::auxiliary_test
