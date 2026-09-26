#pragma once
#include "lib_src/collision/RadiossType25FixedMainStartup.h"
#include "../radioss_type25_fixed_main_startup/NativeOracle.h"
namespace type25_post_gapm_test {
namespace n=tlfea::contact::radioss_type25;
namespace s=n::startup;
using NativeResult=type25_startup_test::NativeResult;
// Whole original SH2/NEIGH (includingIDEL1), CSR and Starter/ready normals.
// Supplied post-GAPM support/permutations are explicit caller operands; no
// production support/topology/normal numerical helper provides expected output.
NativeResult Oracle(const s::Input&,const s::PostGapmTopology&,const double* coefficients,std::size_t);
}
