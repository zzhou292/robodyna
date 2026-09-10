#pragma once
#include "SourceAssemblyBindings.h"

namespace crash::cases::source_assembly::startup {
std::vector<PartNativeMassLedger> PartLedger(const source::Data&, const tl::fea::ShellBatchBinding&);
void RigidGroups(const source::Data&, const tl::fea::ShellBatchBinding&,
                 const SourceAssemblyBindingOptions&, tl::fea::NodalRigidGroupModel&);
} // namespace crash::cases::source_assembly::startup
