// This header must compile first with only its declared foundation dependency.
#include "robodyna/mechanics/RbParticipantServices.h"

#include <type_traits>

using Services = robodyna::mechanics::RbParticipantServices;
static_assert(std::is_abstract_v<Services>);
static_assert(!std::is_destructible_v<Services>, "A borrowed service must not be deleted through its interface");
static_assert(std::is_same_v<decltype(&Services::GetParticipantGravity),
                             const chrono::ChVector3d& (Services::*)() const>);
static_assert(std::is_same_v<decltype(&Services::GetParticipantAssemblyThreads), int (Services::*)() const>);

// This is a header/standalone-link gate, not a numerical runtime test.
int main() { return 0; }
