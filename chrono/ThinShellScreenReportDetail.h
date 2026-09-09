#pragma once

#include "ThinShellScreenReport.h"

namespace crash::reference::screen_report {
using namespace crash::output;
inline Document Object() { Document d; d.SetObject(); return d; }
inline void Nested(Document& d,const char* key,const Value& value) {
    Value copy; copy.CopyFrom(value,d.GetAllocator());
    d.AddMember(Value(key,d.GetAllocator()),copy,d.GetAllocator());
}
template<class Range,class Convert>
void Objects(Document& d,const char* key,const Range& values,Convert convert) {
    Value array(rapidjson::kArrayType);
    for (const auto& value:values) {
        const auto object=convert(value);
        Value copy; copy.CopyFrom(object,d.GetAllocator());
        array.PushBack(copy,d.GetAllocator());
    }
    d.AddMember(Value(key,d.GetAllocator()),array,d.GetAllocator());
}
template<class Range>
void Indices(Document& d,const char* key,const Range& values,std::size_t count) {
    Require(count<=values.size(),"Thin-shell index count exceeds its fixed capacity");
    Value array(rapidjson::kArrayType);
    for(std::size_t i=0;i<count;++i) array.PushBack(static_cast<std::uint64_t>(values[i]),d.GetAllocator());
    d.AddMember(Value(key,d.GetAllocator()),array,d.GetAllocator());
}
template<class Matrix>
void MatrixRows(Document& d,const char* key,const Matrix& matrix) {
    Require(matrix.allFinite(),"Thin-shell report matrix is nonfinite");
    Value array(rapidjson::kArrayType);
    for(Eigen::Index i=0;i<matrix.rows();++i) {
        Value row(rapidjson::kArrayType);
        for(Eigen::Index j=0;j<matrix.cols();++j) row.PushBack(matrix(i,j),d.GetAllocator());
        array.PushBack(row,d.GetAllocator());
    }
    d.AddMember(Value(key,d.GetAllocator()),array,d.GetAllocator());
}
Document Kinetic(const ShellPatchKineticEnergy&);
Document Spectrum(const ThinShellSpectrumDiagnostic&);
Document Match(const ThinShellModeMatchDiagnostic&);
} // namespace crash::reference::screen_report
