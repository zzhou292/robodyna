// Robodyna public API. These aliases preserve the inherited implementation.
#ifndef ROBODYNA_CONTACT_RBCONTACTMATERIAL_H
#define ROBODYNA_CONTACT_RBCONTACTMATERIAL_H

#include "chrono/physics/ChContactMaterial.h"

namespace robodyna::contact {
using RbContactMethod = ::chrono::ChContactMethod;
using RbContactMaterial = ::chrono::ChContactMaterial;
}  // namespace robodyna::contact

#endif
