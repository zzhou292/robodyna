#pragma once
#include "WallResponseData.h"

namespace tl::qualification::qeph::wall_response {
// Run from the owning GTest/command scope: retained native/ledger assertions
// control every receipt. Caller prepares the immutable model/config and reserves
// exactly SampleCount records. Failure leaves the last accepted prefix available
// for the owning report; no failed candidate is published as an accepted sample.
void Execute(Run&);
} // namespace tl::qualification::qeph::wall_response
