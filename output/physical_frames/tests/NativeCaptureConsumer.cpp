#include "../NativeAcceptedFrames.h"
// Links the complete live adapter against production TL targets, with no
// GTest or reference oracle. It creates no owner and performs no CUDA work.
int main() {
    const auto p=crash::output::physical_frames::NativeAcceptedFrames::ObservationProfile();
    return !p.native_contact||p.self_contact||p.structural_limit||p.type45||p.beam18;
}
