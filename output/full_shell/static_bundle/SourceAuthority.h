#pragma once
#include "Types.h"
namespace crash::output::full_shell::source::detail {
Document AuthorityDocument(const SourceInputs&);
void CheckAuthority(const SourceInputs&, const Value&);
} // namespace crash::output::full_shell::source::detail
