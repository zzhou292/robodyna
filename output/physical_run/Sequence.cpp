#include "Types.h"
#include <cmath>
namespace crash::output::physical_run {
void CheckValues(const records::Context& c,Profile p,const Values& v) {
    records::CheckStamp(c,v.stamp);
    Require(v.owner==c.identity().owner && v.stamp.epoch &&
        v.structural_limit_s.has_value()==p.structural_limit,"Physical accepted row source/profile differs");
    if(v.structural_limit_s)
        Require(std::isfinite(*v.structural_limit_s) && *v.structural_limit_s>0 &&
            c.fixed_dt()<=*v.structural_limit_s,"Invalid admitted structural interval limit");
}
Sequence Advance(const records::Context& c,Profile p,std::uint64_t planned,const Sequence& s,const Values& v) {
    CheckValues(c,p,v);
    Require(s.last.epoch<planned && v.stamp.base_epoch==s.last.epoch && v.stamp.epoch==s.last.epoch+1 &&
        v.stamp.attempt>s.last.attempt && Bits(v.stamp.base_time)==Bits(s.last.time),
        "Physical interval is not the next accepted owner endpoint");
    return {v.stamp};
}
std::vector<std::string> IntegerFields() {return {"owner_id","base_epoch","attempt","accepted_epoch"};}
std::vector<std::string> RealFields(Profile p) {
    std::vector<std::string> fields{"base_time_s","accepted_time_s","velocity_time_s","kick_dt_s"};
    if(p.structural_limit) fields.push_back("post_cin_structural_limit_s");
    return fields;
}
} // namespace crash::output::physical_run
