#pragma once
#include "Buffers.h"
#include "lib_src/collision/RadiossType25Transaction.h"
#include "output/physical_run/NativeContactValues.h"
namespace crash::output::physical_frames::detail {
struct NativeCaptureScope {
    tl::fea::NodalStamp stamp;
    tl::fea::ShellPhysicalDiagnostics diagnostics;
    tl::fea::NativeContactPublicationSnapshot contact;
    tlfea::contact::radioss_type25::TransactionSourceInfo source;
};
// Pure formatting/phase checks. Only the live capture class calls them on
// actual owner/publisher reads; caller-authored values confer no authority.
records::FrameStamp NativePhase(const NativeCaptureScope&);
void CheckSameNativeScope(const NativeCaptureScope&,const NativeCaptureScope&);
physical_run::NativeContactValues NativeContactObservation(const NativeCaptureScope&);
}
