#include "Metrics.h"
#include <algorithm>

namespace crash::benchmarks::assembly_pilot {
void Metric::Distance(double reference,double distance,std::size_t index,std::uint64_t source) {
    output::Require(std::isfinite(reference)&&reference>=0&&std::isfinite(distance)&&distance>=0,"Nonfinite pilot difference");
    if(!count||distance>maximum){maximum=distance;worst_index=index;worst_source_id=source;}
    reference_maximum=std::max(reference_maximum,reference);++count;nonzero+=distance>0;
    square+=static_cast<long double>(distance)*distance;reference_square+=static_cast<long double>(reference)*reference;
    output::Require(std::isfinite(square)&&std::isfinite(reference_square),"Pilot RMS accumulation overflows");
}
void Metric::Add(double a,double b,std::size_t index,std::uint64_t source) {
    output::Require(std::isfinite(a)&&std::isfinite(b),"Nonfinite pilot input");
    Distance(std::abs(a),static_cast<double>(std::abs(static_cast<long double>(a)-b)),index,source);
}
double RotationDistance(const std::array<double,4>& a,const std::array<double,4>& b) {
    long double dot=0,as=0,bs=0;
    for(unsigned i=0;i<4;++i) {
        output::Require(std::isfinite(a[i])&&std::isfinite(b[i]),"Nonfinite pilot quaternion");
        dot+=static_cast<long double>(a[i])*b[i];as+=static_cast<long double>(a[i])*a[i];bs+=static_cast<long double>(b[i])*b[i];
    }
    output::Require(as>0&&bs>0&&std::isfinite(as)&&std::isfinite(bs),"Invalid pilot quaternion norm");
    long double minus=0,plus=0;const auto an=std::sqrt(as),bn=std::sqrt(bs);const auto sign=dot<0?-1:1;
    for(unsigned i=0;i<4;++i) {
        const auto x=a[i]/an,y=sign*b[i]/bn;minus+=(x-y)*(x-y);plus+=(x+y)*(x+y);
    }
    return static_cast<double>(4*std::atan2(std::sqrt(minus),std::sqrt(plus)));
}
output::Value MetricDocument(output::Document& d,const Metrics& metrics) {
    output::Value values(rapidjson::kObjectType);auto& a=d.GetAllocator();
    for(const auto& [name,m]:metrics) {
        output::Require(m.count>0,"Empty pilot metric");output::Value v(rapidjson::kObjectType);
        v.AddMember("units",output::Value(m.units.c_str(),a),a);v.AddMember("samples",m.count,a);v.AddMember("nonzero_differences",m.nonzero,a);
        v.AddMember("max_abs_difference",m.maximum,a);v.AddMember("rms_difference",static_cast<double>(std::sqrt(m.square/m.count)),a);
        v.AddMember("reference_max_abs",m.reference_maximum,a);
        v.AddMember("reference_rms",static_cast<double>(std::sqrt(m.reference_square/m.count)),a);
        v.AddMember("worst_flat_index",m.worst_index,a);v.AddMember("worst_source_id",m.worst_source_id,a);
        values.AddMember(output::Value(name.c_str(),a),v,a);
    }
    return values;
}
} // namespace crash::benchmarks::assembly_pilot
