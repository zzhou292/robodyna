#include "lib_src/solvers/NodalCinRuntime.h"
#include <type_traits>
static_assert(std::is_same_v<decltype(tl::fea::NodalAcceptedRawMassView{}.mass_kg),const double*>);
int main() {
  const tl::fea::FENodalState owner;
  const tl::fea::NodalTrialToken token;
  tl::fea::NodalAcceptedRawMassView view;
  return owner.BorrowAcceptedRawMass(token,&view).status==tl::fea::NodalStatus::NotInitialized &&
    owner.AuthenticateAcceptedRawMass(token,view).status==tl::fea::NodalStatus::NotInitialized ? 0 : 1;
}
