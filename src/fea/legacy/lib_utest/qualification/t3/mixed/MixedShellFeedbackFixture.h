#pragma once
#include "MixedShellLedger.h"

namespace mixed_feedback_test {
using namespace mixed_shell_test;
constexpr std::uint64_t FeedbackQualification=0x4d42314645454431ULL;
constexpr std::uint64_t FeedbackConfiguration=0x4d4231464d4f4431ULL;
bool InitializeFeedback(Rig&);
// Bit0 QEPH, bit1 T3. Missing contributors are used only by rejection tests.
// Prepared.load retains the ACTUAL total RHS, after the ordered cache scatter.
bool PrepareFeedback(Rig&,const Loads&,Prepared&,unsigned contributors=3);
bool PublishFeedback(Rig&,const Prepared&,const Staged&);
void CheckFeedback(const Rig&,const Snapshot&,const Staged&,const Prepared&,const Staged&);
void PreservedFeedback(Rig&,const Snapshot&,const Staged&,const fe::ShellBatchDiagnostics&);
} // namespace mixed_feedback_test
