#include "../../MixedInterfaceSource.h"
int main() {
    namespace source = crash::cases::vehicle_self_contact::native::mixed_interface;
    // Volatile function address retains the actual composition symbol without
    // a fabricated source or an unreachable null-reference invocation.
    auto (*volatile prepare)(const source::Initial&, source::Limits) = &source::MixedInterfaceSource::Prepare;
    source::Preparation empty;
    const auto document = source::ResultDocument(empty);
    return prepare && document.HasMember("schema") && !empty.source ? 0 : 1;
}
