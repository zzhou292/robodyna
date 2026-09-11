#include "Checks.h"
#include "IntervalTruth.h"
namespace wall_observer_test {
void CheckGlobalTruth(const c::NodalWallPointResult* nodes,unsigned count,const c::NodalWallDiagnostics& actual) {
  CheckTruth(nodes,count,actual);
}
void CheckDerivedTruth(const d::Storage& s,const tl::fea::NodalPreparedView& view) {CheckIntervalTruth(s,view);}
}
