#include "NativeSolidSupport.h"
#include <climits>
#include <cmath>
#include <mutex>
namespace crash::cases::vehicle_self_contact::native::main_coefficients::test {
namespace {
std::mutex solid_lock;
extern "C" void rd_solid_support(const int*,const int*,const int*,const int*,const int*,
    const double*,const int*,int*,double*,int*);
}
NativeSolidSupportResult NativeSolidSupport(const coated::Inputs& input,
    const std::vector<std::uint8_t>& flags,const coated::s::Main& main,const std::array<int,2>& prior) {
    output::Require(!input.nodes.empty() && input.nodes.size()<=256 && input.solids.size()<=64 &&
        flags.size()==input.solids.size(),"Native solid support fixture bounds");
    std::vector<int> nodes,eids,observed_flags;
    std::vector<double> points;
    for (const auto& node:input.nodes) {
        const auto p=node.native_position;
        output::Require(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z),"Native support fixture coordinates");
        points.insert(points.end(),{p.x,p.y,p.z});
    }
    for (std::size_t i=0;i<input.solids.size();++i) {
        const auto& row=input.solids[i];
        output::Require(row.source_id && row.source_id<=INT_MAX && flags[i]<=1,
            "Native support sourceEID or flag domain");
        eids.push_back(int(row.source_id));
        observed_flags.push_back(flags[i]);
        for (const auto node:row.nodes) {
            output::Require(node<input.nodes.size(),"Native solid support raw node domain");
            nodes.push_back(int(node)+1);
        }
    }
    int face[4];
    for (unsigned k=0;k<4;++k) {
        output::Require(main.nodes[k]<input.nodes.size(),"Native solid support face domain");
        face[k]=int(main.nodes[k])+1;
    }
    // Empty physical family is genuinely unread in the original early return.
    if (nodes.empty()) nodes.push_back(0);
    if (eids.empty()) eids.push_back(0);
    if (observed_flags.empty()) observed_flags.push_back(0);
    const int counts[]{int(input.nodes.size()),int(input.solids.size())};
    int out[11]{},linked[64]{};
    double area=0;
    std::lock_guard<std::mutex> serial(solid_lock);
    rd_solid_support(counts,nodes.data(),eids.data(),observed_flags.data(),face,points.data(),prior.data(),out,&area,linked);
    output::Require(out[0]>=0 && std::size_t(out[0])<=input.solids.size() && out[3]>=0 &&
        std::size_t(out[3])<=input.solids.size() && out[4]>=0 && out[6]>=0,"Native solid support observed bounds");
    NativeSolidSupportResult result;
    if (out[0]) result.chosen=std::size_t(out[0]-1);
    result.raw_owner_words[0]=out[1];
    result.raw_owner_words[1]=out[2];
    result.effective_count=unsigned(out[4]);
    result.warning=out[5]!=0;
    result.inversions=unsigned(out[6]);
    result.geometry_defined=out[0]!=0;
    result.area=area;
    for (int i=0;i<out[3];++i) {
        output::Require(linked[i]>0 && std::size_t(linked[i])<=input.solids.size(),"Native support observed matched row");
        result.native_matches.push_back(std::size_t(linked[i]-1));
    }
    for (unsigned k=0;k<4;++k) {
        output::Require(out[7+k]>0 && std::size_t(out[7+k])<=input.nodes.size(),"Native support observed face node");
        result.final_nodes[k]=std::uint32_t(out[7+k]-1);
    }
    return result;
}
}
