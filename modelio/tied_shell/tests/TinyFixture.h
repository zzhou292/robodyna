#pragma once
#include "../Internal.h"
#include <gtest/gtest.h>
#include <functional>

namespace crash::modelio::tied_shell::test {
struct TinyFixture {
    source::CanonicalData canonical;
    std::string member;
    explicit TinyFixture(bool set_options = false);
    void AlterScope(const std::function<void(output::Document&)>&);
    Data Prepare(Limits limits = {}) const { return detail::Build(canonical, member, limits); }
};
std::string Json(const output::Document&);
} // namespace crash::modelio::tied_shell::test
