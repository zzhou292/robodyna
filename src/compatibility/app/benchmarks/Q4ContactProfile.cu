#include "Q4ContactProfileBackend.h"
#include <cuda_runtime.h>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace crash::profile {
namespace {
template<class Backend> struct Storage {
    ContactProfileInput input;
    typename Backend::Cell leaves[contact::MaxQ4IntegrationLeaves];
    std::uint32_t heap[contact::MaxQ4IntegrationLeaves];
    typename Backend::Result result;
    contact::Q4IntegrationReport report;
    TL_SURFACE_HD typename Backend::Scratch Scratch() {
        return {leaves,heap,contact::MaxQ4IntegrationLeaves,contact::MaxQ4IntegrationLeaves};
    }
};
static_assert(sizeof(Storage<detail::ScalarContactBackend>)<512*1024 &&
              sizeof(Storage<detail::RectangularContactBackend>)<512*1024,
              "Keep selected prescribed contact profile below half a MiB device storage");
void Check(cudaError_t status) {if(status!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(status));}
template<class Backend> struct Device {
    Storage<Backend>* storage=nullptr;cudaEvent_t begin=nullptr,end=nullptr;
    ~Device() {if(begin)cudaEventDestroy(begin);if(end)cudaEventDestroy(end);if(storage)cudaFree(storage);}
};
template<class Backend> __global__ void Evaluate(Storage<Backend>* storage,unsigned parent) {
    storage->result={};storage->report=Backend::Integrate(storage->input.View(parent),storage->input.limits,
                                                       storage->Scratch(),&storage->result);
}
template<class Backend> TL_SURFACE_HD contact::Q4IntegrationReport FinishSaved(Storage<Backend>& storage,unsigned parent) {
    const auto input=storage.input.View(parent);contact::Q4IntegralInterval gap[4];double nominal[4];
    for(unsigned n=0;n<4;++n) {
        const double x=storage.input.position[3*input.surface.parents[parent].nodes[n]];
        nominal[n]=x-input.wall_x;
        if(!contact::q4_bounds::Difference(x,input.wall_x,&gap[n]))return {};
    }
    return Backend::Finish(input,storage.input.limits,storage.Scratch(),gap,nominal,&storage.result);
}
template<class Backend> __global__ void FinishOnly(Storage<Backend>* storage,unsigned parent) {storage->report=FinishSaved(*storage,parent);}
void Require(bool test) {if(!test)throw std::runtime_error("Prescribed CPU/CUDA integration result differs");}
void Same(const contact::Q4CertifiedIntegral& a,const contact::Q4CertifiedIntegral& b) {
    Require(a.value==b.value&&a.lower==b.lower&&a.upper==b.upper&&a.error==b.error);
}
void Same(const contact::Q4IntegrationReport& a,const contact::Q4IntegrationReport& b) {
    Require(a.status==b.status&&a.cause==b.cause&&a.cell==b.cell&&a.depth==b.depth&&a.visited==b.visited&&a.leaves==b.leaves);
}
void Same(const contact::Q4IntegrationResult& a,const contact::Q4IntegrationResult& b) {
    Require(a.valid&&b.valid&&a.feature_id==b.feature_id&&a.parent_element_id==b.parent_element_id&&
            a.base_epoch==b.base_epoch&&a.attempt==b.attempt&&a.leaf_count==b.leaf_count&&a.visited==b.visited&&a.deepest_leaf==b.deepest_leaf);
    Same(a.resultant,b.resultant);Same(a.potential,b.potential);
    Require(a.active_area.lower==b.active_area.lower&&a.active_area.upper==b.active_area.upper);
    for(unsigned i=0;i<4;++i) {
        Same(a.force[i],b.force[i]);Require(a.nodal.nodes[i]==b.nodal.nodes[i]);
        const auto x=a.nodal.forces[i],y=b.nodal.forces[i],c=a.nodal.couples[i],d=b.nodal.couples[i];
        Require(x.x==y.x&&x.y==y.y&&x.z==y.z&&c.x==d.x&&c.y==d.y&&c.z==d.z);
    }
}
template<class Backend> void SameSelected(const ContactParentProfile& expected,const typename Backend::Result& actual) {
    Same(expected.result,Backend::Base(actual));
    Require(expected.deepest_u==Backend::U(actual)&&expected.deepest_v==Backend::V(actual));
}
template<class Backend> std::array<std::uint32_t,3> LeafKinds(const Storage<Backend>& storage,unsigned count) {
    Require(count>0&&count<=contact::MaxQ4IntegrationLeaves);
    std::array<std::uint32_t,3> result{};
    for(unsigned i=0;i<count;++i) {
        const auto kind=Backend::Kind(storage.leaves[i]);
        const unsigned index=kind==contact::Q4IntegrationCellKind::Inactive?0:
            kind==contact::Q4IntegrationCellKind::Active?1:kind==contact::Q4IntegrationCellKind::Mixed?2:3;
        Require(index<3);++result[index];
    }
    Require(result[0]+result[1]+result[2]==count);return result;
}
void CompareCertificate(const contact::Q4CertifiedIntegral& candidate,const contact::Q4CertifiedIntegral& scalar,
                        double& difference,double& combined_error,double budget) {
    Require(candidate.lower<=scalar.upper&&scalar.lower<=candidate.upper&&candidate.error<=budget&&scalar.error<=budget);
    // Raise the estimate difference and lower the sum for the pass decision;
    // an upward-rounded tolerance sum cannot admit a threshold excess. Keep
    // the upward sum too, explicitly labeled as a reporting enclosure.
    double error_lower=0;
    Require(contact::q4_bounds::AbsoluteDifferenceUpper(candidate.value,scalar.value,&difference)&&
            contact::q4_bounds::AddScalar(candidate.error,scalar.error,true,&combined_error)&&
            contact::q4_bounds::AddScalar(candidate.error,scalar.error,false,&error_lower));
    Require(difference<=error_lower);
}
void ScalarReference(const ContactProfileInput& input,unsigned p,ContactParentProfile& profile) {
    using Scalar=detail::ScalarContactBackend;
    auto host=std::make_unique<Storage<Scalar>>();host->input=input;
    const auto start=std::chrono::steady_clock::now();
    profile.scalar_report=Scalar::Integrate(input.View(p),input.limits,host->Scratch(),&host->result);
    profile.scalar_host_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    Require(profile.scalar_report.status==contact::Q4IntegrationStatus::Ok);
    profile.scalar_result=host->result;profile.scalar_leaf_kinds=LeafKinds(*host,host->result.leaf_count);
    const auto& a=profile.result;const auto& b=profile.scalar_result;
    Require(a.valid&&b.valid&&a.feature_id==b.feature_id&&a.parent_element_id==b.parent_element_id&&
            a.base_epoch==b.base_epoch&&a.attempt==b.attempt&&
            a.active_area.lower<=b.active_area.upper&&b.active_area.lower<=a.active_area.upper);
    for(unsigned n=0;n<4;++n) {
        Require(a.nodal.nodes[n]==b.nodal.nodes[n]);
        CompareCertificate(a.force[n],b.force[n],profile.scalar_difference_upper[n],profile.scalar_error_sum_upper[n],input.limits.force_error);
    }
    CompareCertificate(a.resultant,b.resultant,profile.scalar_difference_upper[4],profile.scalar_error_sum_upper[4],input.limits.force_error);
    CompareCertificate(a.potential,b.potential,profile.scalar_difference_upper[5],profile.scalar_error_sum_upper[5],input.limits.energy_error);
    profile.scalar_comparison=true;
}
template<class Backend> ContactProfileResult Measure(const ContactProfileInput& input) {
    auto host=std::make_unique<Storage<Backend>>();host->input=input;Device<Backend> device;
    Check(cudaMalloc(&device.storage,sizeof(Storage<Backend>)));Check(cudaEventCreate(&device.begin));Check(cudaEventCreate(&device.end));
    Check(cudaMemcpy(&device.storage->input,&input,sizeof(input),cudaMemcpyHostToDevice));
    ContactProfileResult output;output.device_bytes=sizeof(Storage<Backend>);output.backend=Backend::kind;
    output.scratch_bytes=Backend::scratch_bytes;
    for(unsigned p=0;p<2;++p) {
        auto& result=output.parents[p];const auto start=std::chrono::steady_clock::now();
        result.report=Backend::Integrate(input.View(p),input.limits,host->Scratch(),&host->result);
        result.host_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        Require(result.report.status==contact::Q4IntegrationStatus::Ok);
        result.result=Backend::Base(host->result);result.deepest_u=Backend::U(host->result);result.deepest_v=Backend::V(host->result);
        result.leaf_kinds=LeafKinds(*host,result.result.leaf_count);
        const auto finish_start=std::chrono::steady_clock::now();
        const auto finished=FinishSaved(*host,p);
        result.host_finish_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-finish_start).count();
        Same(result.report,finished);SameSelected<Backend>(result,host->result);
        if constexpr(Backend::kind==ContactProfileBackend::Rectangular) ScalarReference(input,p,result);
        for(unsigned trial=0;trial<4;++trial) { // One warm call, three measured calls.
            Check(cudaEventRecord(device.begin));Evaluate<<<1,1>>>(device.storage,p);Check(cudaGetLastError());
            Check(cudaEventRecord(device.end));Check(cudaEventSynchronize(device.end));
            float elapsed=0;Check(cudaEventElapsedTime(&elapsed,device.begin,device.end));
            Check(cudaMemcpy(&host->result,&device.storage->result,sizeof(host->result),cudaMemcpyDeviceToHost));
            Check(cudaMemcpy(&host->report,&device.storage->report,sizeof(host->report),cudaMemcpyDeviceToHost));
            Same(result.report,host->report);SameSelected<Backend>(result,host->result);
            if(trial)result.gpu_ms[trial-1]=elapsed;
        }
        for(unsigned trial=0;trial<4;++trial) {
            Check(cudaEventRecord(device.begin));FinishOnly<<<1,1>>>(device.storage,p);Check(cudaGetLastError());
            Check(cudaEventRecord(device.end));Check(cudaEventSynchronize(device.end));
            float elapsed=0;Check(cudaEventElapsedTime(&elapsed,device.begin,device.end));
            Check(cudaMemcpy(&host->result,&device.storage->result,sizeof(host->result),cudaMemcpyDeviceToHost));
            Check(cudaMemcpy(&host->report,&device.storage->report,sizeof(host->report),cudaMemcpyDeviceToHost));
            Same(result.report,host->report);SameSelected<Backend>(result,host->result);
            if(trial)result.gpu_finish_ms[trial-1]=elapsed;
        }
    }
    return output;
}
} // namespace
ContactProfileResult MeasureContactProfile(const ContactProfileInput& input,ContactProfileBackend backend) {
    switch(backend) {
        case ContactProfileBackend::Scalar:return Measure<detail::ScalarContactBackend>(input);
        case ContactProfileBackend::Rectangular:return Measure<detail::RectangularContactBackend>(input);
    }
    throw std::runtime_error("Invalid prescribed contact profile backend");
}
} // namespace crash::profile
