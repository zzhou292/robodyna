#include "Types.h"
#include <cmath>
namespace crash::output::physical_run {
void CheckValues(const records::Context& c,Profile p,const Values& v) {
    records::CheckStamp(c,v.stamp);
    Require(v.owner==c.identity().owner && v.stamp.epoch &&
        v.structural_limit_s.has_value()==p.structural_limit && v.self_contact.has_value()==p.self_contact,
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
}
Sequence Advance(const records::Context& c,Profile p,std::uint64_t planned,const Sequence& s,const Values& v) {
    CheckValues(c,p,v);
    Require(s.last.epoch<planned && v.stamp.base_epoch==s.last.epoch && v.stamp.epoch==s.last.epoch+1 &&
        v.stamp.attempt>s.last.attempt && Bits(v.stamp.base_time)==Bits(s.last.time),
        "Physical interval is not the next accepted owner endpoint");
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
    return fields;
}
std::vector<std::string> RealFields(Profile p) {
    std::vector<std::string> fields{"base_time_s","accepted_time_s","velocity_time_s","kick_dt_s"};
    if(p.structural_limit) fields.push_back("post_cin_structural_limit_s");
    if(p.self_contact) {const auto extra=SelfContactRealFields();fields.insert(fields.end(),extra.begin(),extra.end());}
    return fields;
}
std::size_t ExtraIntervalBytes(Profile p) noexcept {
    return p.self_contact ? 8*(4+SelfContactIntegerCount+4+std::size_t(p.structural_limit)+SelfContactRealCount)-interval::RowBytes : 0;
}
} // namespace crash::output::physical_run
