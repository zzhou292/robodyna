#pragma once

#include <string>

namespace robodyna::parsers {
std::string ResolveRunfile(const std::string& name);
// Set the declared package path before the retained ChPythonEngine initializes
// CPython. The caller still owns the single interpreter lifetime.
void ConfigureEmbeddedPackage(const std::string& manifest_runfile);
void ConfigureDemoData(const std::string& solidworks_model_runfile);
}  // namespace robodyna::parsers
