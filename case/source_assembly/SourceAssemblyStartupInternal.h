#pragma once
#include "SourceAssemblyBindings.h"

namespace crash::cases::source_assembly::startup {
std::vector<PartNativeMassLedger> PartLedger(const source::Data&, const tl::fea::ShellBatchBinding&);
void RigidGroups(const source::Data&, const tl::fea::ShellBatchBinding&,
                 const SourceAssemblyBindingOptions&, tl::fea::NodalRigidGroupModel&);
void Connectors(const source::SourceAssembly&,const tl::fea::ShellBatchBinding&,
                const SourceAssemblyBindingOptions&,tl::fea::type25::Model&,tl::fea::NodalMassBinding&);
} // namespace crash::cases::source_assembly::startup
