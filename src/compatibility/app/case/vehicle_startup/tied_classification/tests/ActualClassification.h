#pragma once
#include "../../TiedSearchClassification.h"
namespace crash::cases::vehicle_startup::classification_actual_test {
const TiedSearchFinalized& Finalized();
const tied::TiedClassificationContext& Context();
const TiedSearchClassification& Prepared();
const std::string& Member(const char*,std::size_t);
}
