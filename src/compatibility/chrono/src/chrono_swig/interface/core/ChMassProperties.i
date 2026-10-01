%{
#include "chrono/physics/ChMassProperties.h"
%}

// Parse the canonical definitions once while retaining existing binding names.
%rename(ChMassProperties) robodyna::mechanics::RbMassProperties;
%rename(ChInertiaUtils) robodyna::mechanics::RbInertiaUtils;
%rename(CompositeInertia) robodyna::mechanics::CompositeInertia;
%shared_ptr(robodyna::mechanics::RbMassProperties)
%include "robodyna/mechanics/RbMassProperties.h"

// The reverse aliases are real C++ declarations used by other inherited APIs.
%include "../../../chrono/physics/ChMassProperties.h"
