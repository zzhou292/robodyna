#pragma once
#include "ResponseData.h"
namespace tl::qualification::qeph::response {
bool ValidateSampleFields(const Run&,const Sample&,std::string&);
bool SameSample(const Sample&,const Sample&) noexcept;
}
