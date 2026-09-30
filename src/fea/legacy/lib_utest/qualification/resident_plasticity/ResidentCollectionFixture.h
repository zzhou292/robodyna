#pragma once
#include "ResidentPlasticityFixture.h"
#include "../plasticity_binding/PlasticityBindingFixture.h"

namespace resident_plasticity_test {
// The same authenticated collection owns two distinct materials and curves.
// The returned oracle owns independent host storage; all caller input expires.
bool InitializeCollection(Rig&,fe::ShellBatchPlasticityBinding& oracle,
                          bool mismatch_other_family=false,bool legacy_triangle=false);
bool InitializeAnalyticCollection(Rig&,fe::ShellBatchPlasticityBinding& oracle,bool all_analytic=false);
bool AssembleCollectionForBinding(Rig&);
} // namespace resident_plasticity_test
