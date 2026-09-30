#pragma once

#include "ArtifactIO.h"
#include <algorithm>
#include <array>
#include <string_view>

namespace crash::output::contact_metadata {
// Archive protocol only; no contact evaluation or dependency on CUDA/TL state.
inline constexpr const char* Scalar="scalar-dyadic-squares";
inline constexpr const char* Rectangular="dyadic-rectangles";
inline constexpr const char* BackendField="contact_integration_backend";
inline bool Known(std::string_view name) { return name==Scalar || name==Rectangular; }
inline std::string Backend(const Value& object,bool required=false) {
    Require(object.IsObject(),"Contact execution metadata must be an object");
    unsigned count=0;
    for(const auto& member:object.GetObject())
        if(std::string_view(member.name.GetString(),member.name.GetStringLength())==BackendField)++count;
    Require(count<=1,"Repeated contact integration backend field");
    const auto at=object.FindMember(BackendField);
    if(at==object.MemberEnd()) {
        Require(!required,"Explicit contact integration backend is missing");
        return Scalar; // Original v1 records predate selectable integration.
    }
    Require(at->value.IsString(),"Contact integration backend must be a string");
    std::string name(at->value.GetString(),at->value.GetStringLength());
    Require(Known(name),"Unknown contact integration backend"); return name;
}
inline std::array<unsigned,2> Axes(const Value& object,std::string_view backend,
                                 std::uint64_t deepest,unsigned limit,bool required) {
    Require(object.IsObject(),"Contact axis metadata must be an object");
    Require(Known(backend)&&limit<=16&&deepest<=limit,"Invalid contact integration depth limit");
    unsigned u_count=0,v_count=0;
    for(const auto& member:object.GetObject()) {
        const std::string_view name(member.name.GetString(),member.name.GetStringLength());
        u_count+=name=="deepest_u"; v_count+=name=="deepest_v";
    }
    Require(u_count<=1&&v_count<=1,"Repeated contact axis depth field");
    const auto u=object.FindMember("deepest_u"),v=object.FindMember("deepest_v");
    if(u==object.MemberEnd()&&v==object.MemberEnd()) {
        Require(!required&&backend==Scalar,"Explicit contact axis depths are missing");
        return {static_cast<unsigned>(deepest),static_cast<unsigned>(deepest)};
    }
    Require(u!=object.MemberEnd()&&v!=object.MemberEnd()&&u->value.IsUint()&&v->value.IsUint(),
            "Invalid contact axis depth metadata");
    const std::array<unsigned,2> result{u->value.GetUint(),v->value.GetUint()};
    Require(result[0]<=limit&&result[1]<=limit&&std::max(result[0],result[1])==deepest&&
            (backend!=Scalar||result[0]==result[1]),"Contact axis depths disagree with backend or maximum depth");
    return result;
}
} // namespace crash::output::contact_metadata
