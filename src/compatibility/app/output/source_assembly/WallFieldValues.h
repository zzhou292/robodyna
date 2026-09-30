#pragma once
#include "output/ArtifactIO.h"
#include "lib_src/collision/NodalWallContact.h"
#include <initializer_list>

namespace crash::output::assembly::wall_fields {
inline void Child(Document& d,const char* key,const Document& child) {
    Value v;v.CopyFrom(child,d.GetAllocator());d.AddMember(Value(key,d.GetAllocator()),v,d.GetAllocator());
}
inline Value Values(Document& d,std::initializer_list<double> values) {return FiniteArray(d,values.begin(),values.size());}
inline Value Ids(Document& d,std::initializer_list<std::uint64_t> ids) {
    Value a(rapidjson::kArrayType);for(auto x:ids)a.PushBack(Value().SetUint64(x),d.GetAllocator());return a;
}
template<class V> Value Vector(Document& d,const V& v) {return Values(d,{v.x,v.y,v.z});}
inline Value Certificate(Document& d,const tlfea::contact::Q4CertifiedIntegral& c) {
    Require(tlfea::contact::nodal_wall_detail::Certificate(c),"Invalid assembly output certificate");
    return Values(d,{c.value,c.lower,c.upper,c.error});
}
inline void Put(Document& d,const char* key,Value v) {d.AddMember(Value(key,d.GetAllocator()),v,d.GetAllocator());}
template<class M> Value Matrix(Document& d,const M& m) {
    return FiniteArray(d,m.v,9);
}
} // namespace crash::output::assembly::wall_fields
