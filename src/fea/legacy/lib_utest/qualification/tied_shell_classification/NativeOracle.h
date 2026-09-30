// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"

namespace classification_test {
struct NativeResult {
  std::vector<int> five_blocks,irupt,itf;
  int warnings=0,penalties=0;
};
NativeResult NativeClassify(const ClassificationInput&);
NativeResult NativeRegister(const RigidRegistrationInput&);
void Compare(const ClassificationResult&,const NativeResult&);
} // namespace classification_test
