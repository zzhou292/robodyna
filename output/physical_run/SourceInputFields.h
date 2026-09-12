#pragma once
#include "output/full_shell/static_bundle/Types.h"
namespace crash::output::physical_run {
// Shared named source fields for caller-selected replay/recovery receipts.
// The enclosing codec checks its exact key set. No filesystem roots are stored.
void AppendSourceInputs(Document&, const full_shell::source::SourceInputs&);
full_shell::source::SourceInputs ParseSourceInputs(const Value&);
} // namespace crash::output::physical_run
