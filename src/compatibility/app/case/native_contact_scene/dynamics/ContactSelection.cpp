#include "ContactSelection.h"
#include "output/ArtifactIO.h"
namespace crash::cases::native_scene {
ContactSelection ContactSelection::Prepare(const PhysicalSource& physical,ContactIdentity identity,ContactLimits limits) {
    switch(physical.declared().data().contact_surface) {
      case modelio::native_scene::DeclaredContactSurface::FixedWall:
        return ContactSource::Prepare(physical,identity,limits);
      case modelio::native_scene::DeclaredContactSurface::AllShells:
        return MovingContactSource::Prepare(physical,identity,limits);
    }
    throw std::invalid_argument("Unrecognized declared contact surface");
}
}
