#pragma once
#include "output/ArtifactIO.h"
#include <cmath>
namespace crash::cases::source_part_elastic::field_detail {
using namespace output;
inline void UInt(Value& object, Document& doc, const char* key, std::uint64_t value) {
    object.AddMember(Value(key,doc.GetAllocator()),Value().SetUint64(value),doc.GetAllocator());
}
inline void Scalar(Value& object, Document& doc, const char* key, double value) {
    Require(std::isfinite(value),"Nonfinite source-part artifact scalar");
    Value name(key,doc.GetAllocator());
    object.AddMember(name,value,doc.GetAllocator());
}
}
