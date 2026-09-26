#include "../../InitialSurfaceSource.h"
int main() {
    namespace s = crash::cases::vehicle_self_contact::native::initial_surfaces;
    s::Preparation unavailable;
    const auto report = s::ResultDocument(unavailable);
    return report.HasMember("schema") && !unavailable.source ? 0 : 1;
}
