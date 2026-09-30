#pragma once
#include "../TiedSearchPostKinChk.h"
namespace crash::cases::vehicle_startup::post_kinchk_detail {
struct Inputs {
    std::vector<native_search::KinChkSlave> slaves;
    native_search::KinChkInput View(const TiedClassificationData&, const TiedPostKinChkReceipt&) const;
};
TiedPostKinChkReceipt Receipt(const tied::source::CanonicalData&, const tied::Data&,
    const TiedFinalizationReceipt&, const tied::ClassificationSourceReceipt&);
Inputs Pack(const TiedClassificationData&, const TiedPostKinChkReceipt&);
TiedPostKinChkForecast Budget(const TiedClassificationForecast&, std::size_t count,
    std::size_t receipt_storage_bytes, TiedPostKinChkLimits);
}
