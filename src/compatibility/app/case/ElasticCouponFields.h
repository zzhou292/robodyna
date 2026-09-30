#pragma once
#include "ElasticCouponCase.h"
#include "chrono_thirdparty/rapidjson/document.h"

namespace crash::case_data {
// Output schema only. Forces/strains/resultants are copied device evaluations
// associated with the recorded accepted configuration, never inferred from OBJ.
rapidjson::Document CouponFrameFields(const ElasticCouponFrame&);
rapidjson::Document CouponConfiguration(const ElasticCouponCase&, unsigned frame_every);
}  // namespace crash::case_data
