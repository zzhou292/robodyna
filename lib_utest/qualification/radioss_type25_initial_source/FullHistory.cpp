// Qualification-only full128-lane caller; reused whole COR3/PEN3/PWR3 numerical leaves.
#include "NativeOracle.h"
#include <cmath>
#include <climits>
#include <mutex>
#include <stdexcept>
extern "C" void initial_state_full_history_native(const int*,const double*,const int*,const int*,const int*,
    const int*,const int*,const int*,const int*,const float*,const int*,const float*,
    const double*,const double*,const int*,int*,double*,double*,int*,const int*,const int*,int*,double*,const double*);
namespace initial_source_test {
namespace n=tlfea::contact::radioss_type25;
using NativeInitialHistory=initial_state_test::NativeInitialHistory;
namespace {
void Check(bool condition) {if(!condition)throw std::runtime_error("Bounded independent initial-history packet rejected");}
std::mutex native_mutex;
}
NativeInitialHistory FullInitialHistory(const n::search_startup::Input& input,
    const type25_startup_test::NativeResult& native,const std::vector<std::array<double,4>>& corner_gaps,
    const std::vector<std::array<int,2>>& candidates,int sharp) {
    const auto nodes=input.mesh.node_count,secondaries=input.secondary_count,mains=native.mains.size();
    const auto references=native.starter_references.size();
    Check((sharp==1||sharp==2)&&nodes&&nodes<=256&&secondaries&&secondaries<=128&&mains&&mains<=160&&references&&references<=640&&
        candidates.size()<=4096&&input.mesh.coordinates==n::startup::Coordinates::Native&&
        input.mesh.positions.valid()&&input.mesh.positions.node_count==nodes&&input.mesh.node_source_ids&&input.secondary&&input.main_gaps&&input.main_count==mains&&
        native.starter_normals.size()==4*mains&&corner_gaps.size()==mains);
    const int dims[]{int(nodes),int(secondaries),int(mains),int(references),int(candidates.size())};
    std::vector<double> x(3*nodes),gaps(secondaries),gapnm(4*mains),penetration(5*secondaries),time(2*secondaries);
    std::vector<int> ids(nodes),irect(4*mains),nsv(secondaries),candn,cande,admsr(4*mains),neighbors(4*mains);
    std::vector<int> boundary(references),types(mains),global_ids(mains),irtlm(4*secondaries),contact(secondaries),prior_irtlm(4*secondaries);
    std::vector<double> prior_pene(5*secondaries),main_gap(mains);
    std::vector<float> normals(12*mains),bisectors(6*references);
    for(std::size_t i=0;i<nodes;++i) {
        Check(input.mesh.node_source_ids[i]&&input.mesh.node_source_ids[i]<=INT_MAX);ids[i]=int(input.mesh.node_source_ids[i]);
        const auto p=input.mesh.positions.at(std::uint32_t(i));
        Check(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z));
        x[3*i]=p.x;x[3*i+1]=p.y;x[3*i+2]=p.z;
    }
    for(std::size_t i=0;i<secondaries;++i) {
        Check(input.secondary[i].node<nodes&&std::isfinite(input.secondary[i].gap)&&input.secondary[i].gap>=0);
        nsv[i]=int(input.secondary[i].node+1);gaps[i]=input.secondary[i].gap;
    }
    for(std::size_t i=0;i<mains;++i) {
        Check(std::isfinite(input.main_gaps[i])&&input.main_gaps[i]>=0);main_gap[i]=input.main_gaps[i];
        const auto& main=native.mains[i];types[i]=main.segment_type;
        Check(main.global_id>0&&std::size_t(main.global_id)<=mains);global_ids[i]=main.global_id;
        for(unsigned k=0;k<4;++k) {
            const auto at=4*i+k;Check(main.nodes[k]<nodes&&main.normal_reference[k]>0&&
                std::size_t(main.normal_reference[k])<=references&&std::isfinite(corner_gaps[i][k])&&corner_gaps[i][k]>=0);
            irect[at]=int(main.nodes[k]+1);admsr[at]=main.normal_reference[k];neighbors[at]=main.neighbors[k];gapnm[at]=corner_gaps[i][k];
            const auto v=native.starter_normals[at];normals[3*at]=v.x;normals[3*at+1]=v.y;normals[3*at+2]=v.z;
            Check(std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z));
        }
    }
    for(std::size_t i=0;i<references;++i) {
        const auto& ref=native.starter_references[i];boundary[i]=ref.boundary;
        if(!ref.boundary)continue; // Native unbound VTX storage is unconsumed.
        for(unsigned k=0;k<2;++k) {
            const auto v=ref.bisector[k];bisectors[6*i+3*k]=v.x;bisectors[6*i+3*k+1]=v.y;bisectors[6*i+3*k+2]=v.z;
            Check(std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z));
        }
    }
    for(const auto& pair:candidates) {
        Check(pair[0]>=1&&std::size_t(pair[0])<=secondaries&&pair[1]>=1&&std::size_t(pair[1])<=mains);
        candn.push_back(pair[0]);cande.push_back(pair[1]);
    }
    {std::lock_guard<std::mutex> lock(native_mutex);
      initial_state_full_history_native(dims,x.data(),ids.data(),irect.data(),nsv.data(),candn.data(),cande.data(),
          admsr.data(),neighbors.data(),normals.data(),boundary.data(),bisectors.data(),gaps.data(),gapnm.data(),
          types.data(),irtlm.data(),penetration.data(),time.data(),contact.data(),global_ids.data(),&sharp,prior_irtlm.data(),prior_pene.data(),main_gap.data());}
    NativeInitialHistory result;result.rows.resize(secondaries);result.before_pwr.resize(secondaries);
    result.nearest_distance.assign(time.begin(),time.begin()+secondaries);result.initial_contact=std::move(contact);
    for(std::size_t i=0;i<secondaries;++i) {
        auto& row=result.rows[i];auto& prior=result.before_pwr[i];
        for(unsigned k=0;k<4;++k)prior.irtlm[k]=prior_irtlm[4*i+k];
        for(unsigned k=0;k<5;++k)prior.penetration[k]=prior_pene[5*i+k];
        for(unsigned k=0;k<4;++k)row.irtlm[k]=irtlm[4*i+k];
        for(unsigned k=0;k<5;++k){Check(std::isfinite(penetration[5*i+k]));row.penetration[k]=penetration[5*i+k];}
        for(unsigned k=0;k<2;++k){Check(std::isfinite(time[2*i+k]));row.time[k]=time[2*i+k];}
    }
    return result;
}
}
