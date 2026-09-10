#pragma once
// Test-only native T3 input shared by startup and prescribed-rate checks.
#include "SourceShellReferenceInput.h"
#include "T3Reference.h"
#include <stdexcept>

namespace crash::qualification::source_contact {
inline tl::qualification::t3::ReferenceInput T3ReferenceInput(
    const SourcePartContactFixture& fixture,unsigned p) {
  return shell_input_detail::Reference<tl::qualification::t3::ReferenceInput,3>(fixture,p);
}
}  // namespace crash::qualification::source_contact
