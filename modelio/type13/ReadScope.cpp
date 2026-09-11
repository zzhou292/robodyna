#include "ReadInternal.h"

namespace crash::modelio::type13::reader {
void ReadScope(const Value& document) {
    Require(document.MemberCount()==10,"TYPE13 source declaration shape changed");
    TextIs(document,"schema","robo_dyna.type13_source_startup.v1");Flag(document,"simulation_ready",false);
    TextIs(document,"scope","Original PID2000486 startup only; tied endpoint pairing and recurrence remain unqualified");
    const auto& source=Member(document,"source");
    Require(source.IsObject()&&source.MemberCount()==7,"TYPE13 source authority shape changed");
    TextIs(source,"archive_sha256","aff8194c456726a678d6cc11f644316ca70f3d9b37c4db622726b7b2985b0451");
    TextIs(source,"member_sha256","67208317e6c8eb1dd43b80001508915ccaace7bc0a745e1aa5a3b33f394df301");
    TextIs(source,"member","2010-toyota-yaris-coarse-v1l/yaris-coarse-v1l.key");
    Require(Unsigned(source,"member_bytes")==42846753,"TYPE13 original member extent changed");
    TextIs(source,"canonical_manifest_sha256","c82f1886b8935d69ff7db4c29c700370e3a057579fab80d02664a253bc7af1c8");
    ReadSourceArrays(Member(source,"arrays"));
    const auto& units=Member(document,"units");Require(units.IsObject()&&units.MemberCount()==3,"TYPE13 unit declaration shape changed");
    Same(Real(units,"mass_to_kg"),1000.);Same(Real(units,"length_to_m"),.001);Same(Real(units,"time_to_s"),1.);
    const auto& authority=Member(source,"unit_authority");
    Require(authority.IsObject()&&authority.MemberCount()==2,"TYPE13 unit authority shape changed");
    TextIs(authority,"sha256","f5cd0934d505e0bfd81de95d867e1b2614d804499e130d39885083af5c52349e");
    Require(output::Sha256(Text(authority,"raw_text"))==Text(authority,"sha256"),"TYPE13 README unit evidence changed");
    const auto& policy=Member(document,"policy");Require(policy.IsObject()&&policy.MemberCount()==11,"TYPE13 native policy shape changed");
    TextIs(policy,"revision","a62b27e6baa555d222a580d6218867d0be4d70b5");
    TextIs(policy,"converter_sha256","87ce68f7e6d9d9186bb2ed5bbc74d1ca97226b8ebaffd48408e2becaae46852d");
    TextIs(policy,"converter","ConvertSectionBeamToSpringBeam");TextIs(policy,"section","ELFORM9/CST1");
    TextIs(policy,"unit_authority","original README t/mm/s; missing CONTROL_UNITS is not a converter unit declaration");
    TextIs(policy,"ignored_supplied_TFAIL","native converter sensor construction is commented out");
    TextIs(policy,"native_conversion_owner","robo-dyna C++ modelio/type13/ConvertProperty.cpp");
    for(const char* key:{"recurrence_qualified","endpoint_ties_qualified","simulation_ready"})Flag(policy,key,false);
    const auto& resolved=Member(policy,"resolved");Require(resolved.IsObject()&&resolved.MemberCount()==13,"TYPE13 resolved defaults changed");
    for(const char* key:{"TT1","TT2","Ifail2","damping","sensor","rate_failure"})Same(Real(resolved,key),0);
    for(const char* key:{"Ileng","H","Ifail","A","LSCALE","alpha"})Same(Real(resolved,key),1);
    Same(Real(resolved,"beta"),2);
    const auto& counts=Member(document,"counts");
    Require(counts.IsObject()&&counts.MemberCount()==3&&Unsigned(counts,"nodes")==7494&&Unsigned(counts,"beams")==4442&&
        Unsigned(counts,"physical_endpoints")==7493,"TYPE13 complete source counts changed");
}
}
