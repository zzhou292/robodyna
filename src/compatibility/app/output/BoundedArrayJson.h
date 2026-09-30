#pragma once
#include "ArtifactIO.h"
#include <cmath>
#include <initializer_list>
#include <string_view>

// Small strict JSON value helpers shared by array descriptors and visualization
// records. Their callers bound document bytes and each collection before reads.
namespace crash::output::array_json {
inline void Keys(const Value& v,std::initializer_list<const char*> names) {
    Require(v.IsObject()&&v.MemberCount()==names.size(),"Unexpected record fields");
    for(const auto* name:names) {
        unsigned count=0;
        for(auto it=v.MemberBegin();it!=v.MemberEnd();++it)
            count+=std::string_view(it->name.GetString(),it->name.GetStringLength())==name;
        Require(count==1,"Missing or duplicate record field");
    }
}
inline std::string Text(const Value& v) {
    Require(v.IsString(),"Expected record text");return {v.GetString(),v.GetStringLength()};
}
inline std::uint64_t UInt(const Value& v) {
    Require(v.IsUint64(),"Expected record unsigned integer");return v.GetUint64();
}
inline double Real(const Value& v) {
    Require(v.IsNumber()&&std::isfinite(v.GetDouble()),"Expected finite record value");return v.GetDouble();
}
inline void Child(Document& d,const char* name,const Document& child) {
    Value v;v.CopyFrom(child,d.GetAllocator());d.AddMember(Value(name,d.GetAllocator()),v,d.GetAllocator());
}
inline Document Parse(const std::string& bytes,std::size_t cap) {
    Require(bytes.size()<=cap,"Record JSON exceeds byte cap");
    Document d;d.Parse<rapidjson::kParseFullPrecisionFlag|rapidjson::kParseIterativeFlag>(bytes.data(),bytes.size());
    Require(!d.HasParseError()&&d.IsObject(),"Malformed record JSON");return d;
}
} // namespace crash::output::array_json
