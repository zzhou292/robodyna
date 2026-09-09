#pragma once
#include "ResponseData.h"

namespace tl::qualification::qeph::response {
// Called from a GTest body so the reused, frozen BQ3 numerical assertions are
// part of the actual acceptance path. The first failed assertion stops receipts.
// Config/model/dictionary/sample capacity are prepared before calling this.
void Execute(Run&);
} // namespace tl::qualification::qeph::response
