// Qualification consumer: link the complete production facade without GTest,
// fixtures or native oracles; rejection must precede any input/owner operation.
#include "../OriginalSources.h"
int main() {
    namespace source=crash::cases::vehicle_native_contact::source;
    try { (void)source::OriginalSources::Prepare({},{}); }
    catch(const std::runtime_error&) { return 0; }
    return 1;
}
