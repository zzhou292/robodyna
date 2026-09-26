#pragma once
#include "lib_src/collision/RadiossType25TiedRemoval.h"
#include "lib_src/collision/RadiossType25SearchStartup.h"
#include "lib_utest/qualification/radioss_type25_fixed_main_startup/NativeOracle.h"
#include <array>
#include <vector>
namespace initial_state_test {
namespace n=tlfea::contact::radioss_type25;
struct NativeInitialHistory {
    std::vector<n::tied_removal::History> rows;
    std::vector<int> initial_contact;
    std::vector<n::tied_removal::History> before_pwr;
    std::vector<double> nearest_distance;
};
// Serial/nonreentrant private Fortran COMMON; C++ entry serializes callers.
// Expected data only: native_topology is the independent whole Starter oracle,
// not the production snapshot. Candidates are explicit one-based(row,main).
NativeInitialHistory InitialHistory(const n::search_startup::Input&,
    const type25_startup_test::NativeResult& native_topology,
    const std::vector<std::array<double,4>>& corner_gaps,
    const std::vector<std::array<int,2>>& candidates,int sharp=1);
}
