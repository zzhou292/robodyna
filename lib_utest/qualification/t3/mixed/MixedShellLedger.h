#pragma once
#include "MixedShellFixture.h"

namespace mixed_shell_test {
void CheckTargets(const Rig&,unsigned interval,const Snapshot&);
void CheckLedgers(const Rig&,const Snapshot& base,const Staged& accepted,
                  const Prepared&,const Staged& next);
std::array<double,6*Nodes> Assembly(const fe::NodalAssemblyView&);
void CheckNativeScatter(const Rig&,const Staged& cache,const Loads& seed,const fe::NodalAssemblyView&);
} // namespace mixed_shell_test
