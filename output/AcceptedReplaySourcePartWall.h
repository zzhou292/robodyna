#pragma once
#include "AcceptedReplayData.h"
#include <cmath>
namespace crash::output::replay_detail {
inline const Value& WallArray(const Value& object,const char* key,std::size_t size) {
    const auto& value=Member(object,key);Require(value.IsArray()&&value.Size()==size,"Wall replay array shape mismatch");return value;
}
inline const Value& WallNumbers(const Value& object,const char* key,std::size_t size) {
    const auto& a=WallArray(object,key,size);for(const auto& v:a.GetArray())Require(v.IsNumber()&&std::isfinite(v.GetDouble()),"Nonfinite wall array value");return a;
}
inline void WallBool(const Value& object,const char* key,bool expected) {
    const auto& v=Member(object,key);Require(v.IsBool()&&v.GetBool()==expected,"Wall replay boolean association failed");
}
inline std::array<double,4> WallCertificate(const Value& a) {
    Require(a.IsArray()&&a.Size()==4,"Wall certificate shape mismatch");std::array<double,4> v{};
    for(unsigned i=0;i<4;++i){Require(a[i].IsNumber()&&std::isfinite(a[i].GetDouble()),"Wall certificate is nonfinite");v[i]=a[i].GetDouble();}
    Require(v[0]>=0&&v[1]>=0&&v[2]>=v[1]&&v[3]>=0&&v[3]>=std::abs(v[0]-v[1])&&v[3]>=std::abs(v[0]-v[2]),"Wall certificate enclosure is malformed");return v;
}
void ReadSourcePartWallIntervals(Bundle&,const Document&,const Document&);
void CheckPlacedSourcePartWall(Bundle&,const Document& configuration);
unsigned CheckSourcePartWallContact(const Bundle&,const Entry&,const Value&);
}
