#include "Types.h"
#include <cmath>
namespace crash::output::physical_run {
void CheckValues(const records::Context& c,Profile p,const Values& v) {
    records::CheckStamp(c,v.stamp);
    Require(v.owner==c.identity().owner && v.stamp.epoch &&
        v.structural_limit_s.has_value()==p.structural_limit && v.self_contact.has_value()==p.self_contact && v.native_contact.has_value()==p.native_contact &&
        !(p.native_contact&&(p.self_contact||p.type45||p.beam18)),
        "Physical accepted row source/profile differs");
    if(v.structural_limit_s)
        Require(std::isfinite(*v.structural_limit_s) && *v.structural_limit_s>0 &&
            c.fixed_dt()<=*v.structural_limit_s,"Invalid admitted structural interval limit");
    if(v.self_contact) {
        CheckSelfContactValues(*v.self_contact);
        Require(v.self_contact->selected_parents<=c.parents().size() &&
            v.self_contact->base_velocity_time<=v.stamp.base_time &&
            (v.stamp.base_epoch || Bits(v.self_contact->base_velocity_time)==Bits(0.)),
            "Self-contact selected source or accepted-base velocity phase differs");
    }
    if(v.native_contact) {
        const auto& n=*v.native_contact;CheckNativeContactValues(n);
        Require(n.nodes==c.nodes()&&n.primary_mains<=c.parents().size()&&n.publication_generation==v.stamp.epoch&&n.force_base_epoch==v.stamp.base_epoch&&
            Bits(n.force_base_time)==Bits(v.stamp.base_time)&&(!v.stamp.base_epoch?Bits(n.force_base_velocity_time)==Bits(0.):true),
            "Native contact force source or phase differs from accepted endpoint");
    }
}
Sequence Advance(const records::Context& c,Profile p,std::uint64_t planned,const Sequence& s,const Values& v) {
    CheckValues(c,p,v);
    Require(s.last.epoch<planned && v.stamp.base_epoch==s.last.epoch && v.stamp.epoch==s.last.epoch+1 &&
        v.stamp.attempt>s.last.attempt && Bits(v.stamp.base_time)==Bits(s.last.time),
        "Physical interval is not the next accepted owner endpoint");
    if(v.native_contact) {
        const auto& n=*v.native_contact;
        Require(Bits(n.force_base_velocity_time)==Bits(s.last.velocity_time)&&
            (!s.last.epoch || (s.native_contact && SameNativeSource(*s.native_contact,n)&&
             s.native_contact->publication_generation<UINT64_MAX&&
             n.publication_generation==s.native_contact->publication_generation+1&&
             n.reference_generation>=s.native_contact->reference_generation&&
             n.reference_generation-s.native_contact->reference_generation<=1)),
            "Native contact source, publication generation or force velocity phase changed");
        return {v.stamp,0,0,v.native_contact};
    }
    if(v.self_contact) {
        Require(Bits(v.self_contact->base_velocity_time)==Bits(s.last.velocity_time),
            "Self-contact base velocity time differs from previous accepted endpoint");
        Require(!s.last.epoch || (s.self_source_id==v.self_contact->source_id &&
            s.self_selected_parents==v.self_contact->selected_parents),
            "Self-contact source changes across accepted intervals");
        return {v.stamp,v.self_contact->source_id,v.self_contact->selected_parents};
    }
    return {v.stamp};
}
std::vector<std::string> IntegerFields(Profile p) {
    std::vector<std::string> fields{"owner_id","base_epoch","attempt","accepted_epoch"};
    if(p.self_contact) {const auto extra=SelfContactIntegerFields();fields.insert(fields.end(),extra.begin(),extra.end());}
    if(p.native_contact){const auto extra=NativeContactIntegerFields();fields.insert(fields.end(),extra.begin(),extra.end());}
    return fields;
}
std::vector<std::string> RealFields(Profile p) {
    std::vector<std::string> fields{"base_time_s","accepted_time_s","velocity_time_s","kick_dt_s"};
    if(p.structural_limit) fields.push_back("post_cin_structural_limit_s");
    if(p.self_contact) {const auto extra=SelfContactRealFields();fields.insert(fields.end(),extra.begin(),extra.end());}
    if(p.native_contact){const auto extra=NativeContactRealFields();fields.insert(fields.end(),extra.begin(),extra.end());}
    return fields;
}
std::size_t ExtraIntervalBytes(Profile p) noexcept {
    if(p.native_contact) {
        const auto bytes=8*(4+NativeContactIntegerCount+4+std::size_t(p.structural_limit)+NativeContactRealCount);
        return bytes>interval::RowBytes?bytes-interval::RowBytes:0;
    }
    return p.self_contact ? 8*(4+SelfContactIntegerCount+4+std::size_t(p.structural_limit)+SelfContactRealCount)-interval::RowBytes : 0;
}
} // namespace crash::output::physical_run
