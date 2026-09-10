#include "AcceptedReplaySourceAssembly.h"

namespace crash::output::replay_detail {
void ReadAssemblyRotationDomain(AssemblyReplayData& a,const Value& config) {
    a.native_rotation_domain=false;
    if(!config.HasMember("rotation_domain")) return;
    const auto& domain=Member(config,"rotation_domain");
    Require(domain.IsObject()&&domain.MemberCount()==3&&
        Text(domain,"policy")=="native_shell_geometry_v1"&&
        Text(domain,"native_qualification_commit")=="70e7f7bf738eb087053816cc908da35a6f81399a",
        "Unknown assembly native rotation qualification");
    AssemblyEqual(Real(domain,"maximum_supported_rotation_rad"),1.5);
    Require(a.maximum_rotation>0&&a.maximum_rotation<=1.5,
        "Assembly geometry rotation limit exceeds the qualified native domain");
    a.native_rotation_domain=true;
}
void CheckAssemblyRotation(const Bundle& b,const Entry& e,const Value& diagnostics,const Value* nodal) {
    const auto& a=*b.assembly;
    const double maximum=Real(diagnostics,"maximum_rotation_rad");
    Require(maximum>=0&&maximum<=AssemblyQuaternionLimit(a),"Assembly quaternion diagnostic exceeds its domain");
    if(!a.native_rotation_domain) {
        Require(!diagnostics.HasMember("native_rotation_domain"),
            "Legacy assembly cannot silently change its rotation domain");
        return;
    }
    const auto& domain=Member(diagnostics,"native_rotation_domain");
    Require(domain.IsObject()&&domain.MemberCount()==3,"Native rotation diagnostics are incomplete");
    for(const auto* key:{"maximum_frame_rotation_rad","maximum_nodal_normal_rotation_rad",
                         "maximum_rigid_member_rotation_rad"}) {
        const double angle=Real(domain,key);
        Require(angle>=0&&angle<=a.maximum_rotation&& (e.epoch||angle==0),
            "Native geometry or rigid member exceeded the declared rotation domain");
    }
    if(!nodal)return; // Final metrics are checked against their complete saved frame.
    const auto& q=WallNumbers(*nodal,"orientation_wxyz",4*b.info.node_count);
    Require(a.grouped_node.size()==b.info.node_count,"Native rotation group membership is incomplete");
    double measured=0,grouped=0;
    for(std::size_t n=0;n<b.info.node_count;++n) {
        const double angle=2*std::atan2(std::hypot(std::hypot(q[4*n+1].GetDouble(),
            q[4*n+2].GetDouble()),q[4*n+3].GetDouble()),std::abs(q[4*n].GetDouble()));
        measured=std::max(measured,angle);
        if(a.grouped_node[n])grouped=std::max(grouped,angle);
    }
    Require(grouped<=a.maximum_rotation,"Actual rigid member quaternion exceeded the original bound");
    AssemblyEqual(measured,maximum);
    AssemblyEqual(grouped,Real(domain,"maximum_rigid_member_rotation_rad"));
}
} // namespace crash::output::replay_detail
