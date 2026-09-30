#include "Truth.h"
namespace t3_observer_test {
void CheckDeviceTruth(Fixture& fixture,const b::Control& actual,unsigned epoch,bool assembled) {
  CheckTruth(fixture,actual,epoch,assembled);
}
} // namespace t3_observer_test
