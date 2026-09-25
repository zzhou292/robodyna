// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Fixture.h"
namespace type25_startup_test {
struct NativeSelection {int winner=0,warning=0,calls=0;double em20=0;std::vector<double> angles,sides;};
// Qualification-only original REMOVEALLBUT1 with read-only score observations.
// Candidate/self IDs are one-based indices in the supplied primary face table.
NativeSelection SelectorOracle(const Case&,int self,unsigned edge,const std::vector<int>& candidates);
} // namespace type25_startup_test
