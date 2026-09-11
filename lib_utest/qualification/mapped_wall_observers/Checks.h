#pragma once
#include "lib_src/collision/NodalWallContactStorage.h"
namespace wall_observer_test {
void CheckGlobalTruth(const tlfea::contact::NodalWallPointResult*,unsigned,const tlfea::contact::NodalWallDiagnostics&);
void CheckDerivedTruth(const tlfea::contact::nodal_wall_device_detail::Storage&,const tl::fea::NodalPreparedView&);
}
