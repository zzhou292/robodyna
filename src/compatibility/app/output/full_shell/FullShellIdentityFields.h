#pragma once
#include "FullShellVisualizationSchema.h"
namespace crash::output::full_shell {
Document IdentityDocument(const Identity&);
Identity ParseIdentity(const Value&);
}
