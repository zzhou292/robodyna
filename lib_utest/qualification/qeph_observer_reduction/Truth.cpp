#include "Truth.h"
namespace qeph_observer_test {
void CheckDeviceTruth(Fixture& fixture,const b::Control& actual,unsigned epoch,bool assembled) {
  CheckTruth(fixture,actual,epoch,assembled);
}
} // namespace qeph_observer_test
