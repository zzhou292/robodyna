#include "SourceNodalWallCuda.h"
#include "SourceNodalWallProfile.h"
#include "SourceNodalWallReduction.h"
#include "collision/NodalWallContactPoint.h"
#include <chrono>

namespace crash::qualification::source_contact::nodal {
namespace {
__device__ bool ValidInput(const Input& input) {
    if (!input.prepared || !input.attempt || !input.face_count || input.face_count>MaxFaces ||
        input.share_count!=4*Q4Count+3*T3Count || input.first_share[0]!=0 ||
        input.first_share[NodeCount]!=input.share_count || !sc::IsFinite(input.wall_tolerance) || input.wall_tolerance<=0 ||
        input.config.stiffness_per_area!=cf::Stiffness || input.config.maximum_penetration!=cf::Cap ||
        input.config.parent_force_error!=cf::ForceBudget || input.config.parent_energy_error!=cf::EnergyBudget) return false;
    bool seen[MaxShares]{}; unsigned quads=0,triangles=0;
    for (unsigned p=0;p<ParentCount;++p) {
        const auto& parent=input.parents[p];
        if (!parent.parent_element_id || !parent.feature_id ||
            !sc::nodal_wall_detail::Certificate(parent.area,true) ||
            !sc::nodal_wall_detail::Certificate(parent.share,true)) return false;
        if (parent.arity==4 && parent.family==sc::NodalWallParentFamily::Q4CenterArea) ++quads;
        else if (parent.arity==3 && parent.family==sc::NodalWallParentFamily::T3Native) ++triangles;
        else return false;
        for (unsigned l=0;l<parent.arity;++l) if (parent.nodes[l]>=NodeCount) return false;
    }
    if (quads!=Q4Count || triangles!=T3Count) return false;
    for (unsigned n=0;n<NodeCount;++n) {
        if (input.nodes[n].node!=n || !sc::nodal_wall_detail::Certificate(input.nodes[n].area,true) ||
            input.first_share[n]>=input.first_share[n+1] || input.first_share[n+1]>input.share_count) return false;
        for (unsigned i=input.first_share[n];i<input.first_share[n+1];++i) {
            const unsigned slot=input.share_slot[i],p=slot/4,l=slot%4;
            if (slot>=MaxShares || p>=ParentCount || l>=input.parents[p].arity ||
                input.parents[p].nodes[l]!=n || seen[slot]) return false;
            seen[slot]=true;
        }
    }
    return true; // Exact native share count + uniqueness implies no omitted slot.
}
template<bool Profiled>
__global__ void EvaluateKernel(Storage* storage,ProfileClocks* clocks) {
    const unsigned n=threadIdx.x;
    if constexpr (Profiled) if (n==0) clocks->kernel_begin=clock64();
    auto& input=storage->input; auto& trial=storage->trial;
    storage->node_report[n]={Status::Ok}; storage->parent_report[n]={Status::Ok};
    trial.contact.nodes[n]={}; trial.contact.parents[n]={}; trial.wall_face[n]=0;
    if (n==0) {
        if constexpr (Profiled) clocks->validation_begin=clock64();
        storage->report={ValidInput(input)?Status::Ok:Status::InvalidInput};
        if constexpr (Profiled) clocks->validation_end=clock64();
    }
    __syncthreads();
    if (storage->report.status!=Status::Ok) return;
    const sc::VectorView position{input.position,NodeCount,3,1},velocity{input.velocity,NodeCount,3,1};
    const sc::LumpedTranslationMassView mass{input.inverse_mass,input.fixed,NodeCount,input.base_epoch,
        sc::TranslationMassModel::kIsotropicLumped};
    if (n<NodeCount) {
        if constexpr (Profiled) clocks->query_begin[n]=clock64();
        unsigned owner=UINT32_MAX; sc::TrianglePointGeometry point;
        const auto x=position.at(n);
        const auto query=storage->query.FindOwner({input.config.wall_x,x.y,x.z},&owner,&point);
        if constexpr (Profiled) clocks->query_end[n]=clock64();
        if (query!=sc::Status::kOk || owner==UINT32_MAX) storage->node_report[n]={Status::OutsideWall,{},n};
        else {
            if constexpr (Profiled) clocks->law_begin[n]=clock64();
            sc::NodalWallPointResult node;
            trial.wall_face[n]=input.faces[owner].geometry.face_id;
            #pragma unroll 1
            for (unsigned i=input.first_share[n];i<input.first_share[n+1];++i) {
                const auto slot=input.share_slot[i]; const unsigned p=slot/4;
                sc::NodalWallPointResult share;
                const auto report=sc::EvaluateNodalWallPoint({n,input.parents[p].share},x,velocity.at(n),
                    mass,input.config,input.attempt,&share);
                if (report.status!=sc::NodalWallStatus::Ok) {
                    storage->node_report[n]={Status::PointFailure,report,n,p}; break;
                }
                storage->shares[slot]=share;
                if (!reduction::AddShare(node,share)) {
                    storage->node_report[n]={Status::Nonfinite,{},n,p}; break;
                }
            }
            if (storage->node_report[n].status==Status::Ok) {
                if (!reduction::CompleteNode(node,velocity.at(n))) storage->node_report[n]={Status::Nonfinite,{},n};
                else trial.contact.nodes[n]=node;
            }
            if constexpr (Profiled) clocks->law_end[n]=clock64();
        }
    }
    __syncthreads();
    if (n==0) {
        if constexpr (Profiled) clocks->node_scan_begin=clock64();
        for (unsigned i=0;i<NodeCount;++i) if (storage->node_report[i].status!=Status::Ok) {
            storage->report=storage->node_report[i]; break;
        }
        if constexpr (Profiled) clocks->node_scan_end=clock64();
    }
    __syncthreads();
    if (storage->report.status!=Status::Ok) return;
    if (n<ParentCount) {
        if constexpr (Profiled) clocks->parent_begin[n]=clock64();
        const auto& parent=input.parents[n]; sc::NodalWallParentResult next;
        next.parent_element_id=parent.parent_element_id; next.feature_id=parent.feature_id;
        next.parent_face_id=parent.parent_face_id; next.family=parent.family; next.arity=parent.arity;
        for (unsigned local=0;local<parent.arity;++local) {
            const auto& share=storage->shares[4*n+local]; next.force[local]=share.force;
            if (!reduction::Sum(next.resultant,share.force) || !reduction::Sum(next.potential,share.potential))
                storage->parent_report[n]={Status::Nonfinite,{},UINT32_MAX,n};
        }
        bool accurate=next.resultant.error<=input.config.parent_force_error &&
            next.potential.error<=input.config.parent_energy_error;
        for (unsigned local=0;local<parent.arity;++local)
            accurate=accurate && next.force[local].error<=input.config.parent_force_error;
        if (!accurate && storage->parent_report[n].status==Status::Ok)
            storage->parent_report[n]={Status::Accuracy,{},UINT32_MAX,n};
        next.valid=storage->parent_report[n].status==Status::Ok; trial.contact.parents[n]=next;
        if constexpr (Profiled) clocks->parent_end[n]=clock64();
    }
    __syncthreads();
    if (n==0) {
        if constexpr (Profiled) clocks->reduction_begin=clock64();
        for (unsigned p=0;p<ParentCount;++p) if (storage->parent_report[p].status!=Status::Ok) {
            storage->report=storage->parent_report[p]; break;
        }
        auto& contact=trial.contact;
        contact.resultant={}; contact.potential={}; contact.wall_reaction={}; contact.wall_moment={};
        contact.surface_power=0; contact.base_epoch=input.base_epoch; contact.attempt=input.attempt;
        contact.parent_count=ParentCount; contact.node_count=NodeCount; contact.valid=false;
        if (storage->report.status==Status::Ok) for (unsigned i=0;i<NodeCount;++i)
            if (!reduction::AddNode(contact,contact.nodes[i])) {
                storage->report={Status::Nonfinite,{},i}; break;
            }
        contact.valid=storage->report.status==Status::Ok;
        if constexpr (Profiled) clocks->reduction_end=clock64();
    }
    __syncthreads();
    if (storage->report.status!=Status::Ok) return;
    // Parallel copy of the complete successful candidate. Failed trials never
    // overwrite device-published results; no global atomics or double springs.
    if constexpr (Profiled) clocks->publish_begin[n]=clock64();
    storage->published.contact.nodes[n]=trial.contact.nodes[n];
    storage->published.contact.parents[n]=trial.contact.parents[n];
    storage->published.wall_face[n]=trial.wall_face[n];
    if (n==0) {
        auto& out=storage->published.contact; const auto& in=trial.contact;
        out.resultant=in.resultant; out.potential=in.potential; out.wall_reaction=in.wall_reaction;
        out.wall_moment=in.wall_moment; out.surface_power=in.surface_power; out.base_epoch=in.base_epoch;
        out.attempt=in.attempt; out.parent_count=in.parent_count; out.node_count=in.node_count; out.valid=true;
    }
    if constexpr (Profiled) clocks->publish_end[n]=clock64();
}
__global__ void Noop() {}
} // namespace

Device::Device() {
    status_=cudaMalloc(&storage_,sizeof(Storage)); if (status_!=cudaSuccess) return;
    status_=cudaMemset(storage_,0,sizeof(Storage)); if (status_!=cudaSuccess) return;
    status_=cudaEventCreate(&begin_); if (status_!=cudaSuccess) return;
    status_=cudaEventCreate(&end_);
}
Device::~Device() {
    if (end_) cudaEventDestroy(end_);
    if (begin_) cudaEventDestroy(begin_);
    if (storage_) cudaFree(storage_);
}
cudaError_t Device::Fail(cudaError_t error) { if (error!=cudaSuccess) status_=error; return error; }
cudaError_t Device::Evaluate(const Input& input,Result& output,Report& report,Timing& timing) {
    if (status_!=cudaSuccess) return status_;
    auto error=cudaGetLastError(); if (error!=cudaSuccess) return Fail(error);
    const auto start=std::chrono::steady_clock::now();
    sc::PreparedPlanarWallQuery query;
    // Bind this upload's actual faces, including negative test packets. Invalid
    // counts remain for the original device ValidInput rejection, without a
    // host read beyond Input::faces. Ineligible geometry keeps the full scan.
    if (input.face_count && input.face_count<=MaxFaces)
        query.Initialize(input.faces,input.face_count,input.wall_tolerance);
    error=cudaMemcpy(&storage_->input,&input,sizeof(Input),cudaMemcpyHostToDevice); if (error!=cudaSuccess) return Fail(error);
    error=cudaMemcpy(&storage_->query,&query,sizeof(query),cudaMemcpyHostToDevice); if (error!=cudaSuccess) return Fail(error);
    error=cudaEventRecord(begin_); if (error!=cudaSuccess) return Fail(error);
    EvaluateKernel<false><<<1,Workers>>>(storage_,nullptr);
    error=cudaGetLastError(); if (error!=cudaSuccess) return Fail(error);
    error=cudaEventRecord(end_); if (error!=cudaSuccess) return Fail(error);
    error=cudaEventSynchronize(end_); if (error!=cudaSuccess) return Fail(error);
    Report checked; Timing measured;
    error=cudaMemcpy(&checked,&storage_->report,sizeof(Report),cudaMemcpyDeviceToHost); if (error!=cudaSuccess) return Fail(error);
    error=cudaEventElapsedTime(&measured.kernel_ms,begin_,end_); if (error!=cudaSuccess) return Fail(error);
    Result next;
    if (checked.status==Status::Ok) {
        error=cudaMemcpy(&next,&storage_->published,sizeof(Result),cudaMemcpyDeviceToHost); if (error!=cudaSuccess) return Fail(error);
    }
    measured.checked_end_to_end_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    if (checked.status==Status::Ok) output=next;
    report=checked; timing=measured; return cudaSuccess;
}
cudaError_t InjectInvalidLaunchForCheck() { Noop<<<1,0>>>(); return cudaPeekAtLastError(); }

cudaError_t InspectKernels(KernelFacts& output) {
    KernelFacts next; int device=0;
    auto error=cudaGetDevice(&device); if (error!=cudaSuccess) return error;
    error=cudaFuncGetAttributes(&next.original,EvaluateKernel<false>); if (error!=cudaSuccess) return error;
    error=cudaFuncGetAttributes(&next.instrumented,EvaluateKernel<true>); if (error!=cudaSuccess) return error;
    error=cudaOccupancyMaxActiveBlocksPerMultiprocessor(&next.original_max_blocks_per_sm,EvaluateKernel<false>,Workers,0);
    if (error!=cudaSuccess) return error;
    error=cudaOccupancyMaxActiveBlocksPerMultiprocessor(&next.instrumented_max_blocks_per_sm,EvaluateKernel<true>,Workers,0);
    if (error!=cudaSuccess) return error;
    error=cudaDeviceGetAttribute(&next.multiprocessors,cudaDevAttrMultiProcessorCount,device); if (error!=cudaSuccess) return error;
    error=cudaDeviceGetAttribute(&next.max_threads_per_sm,cudaDevAttrMaxThreadsPerMultiProcessor,device); if (error!=cudaSuccess) return error;
    error=cudaDeviceGetLimit(&next.stack_limit_bytes,cudaLimitStackSize); if (error!=cudaSuccess) return error;
    output=next; return cudaSuccess;
}
Profiler::Profiler() {
    status_=cudaMalloc(&storage_,sizeof(ProfileStorage)); if (status_!=cudaSuccess) return;
    status_=cudaMemset(storage_,0,sizeof(ProfileStorage)); if (status_!=cudaSuccess) return;
    status_=cudaEventCreate(&begin_); if (status_!=cudaSuccess) return;
    status_=cudaEventCreate(&end_);
}
Profiler::~Profiler() {
    if (end_) cudaEventDestroy(end_);
    if (begin_) cudaEventDestroy(begin_);
    if (storage_) cudaFree(storage_);
}
cudaError_t Profiler::Fail(cudaError_t error) { if (error!=cudaSuccess) status_=error; return error; }
cudaError_t Profiler::Evaluate(const Input& input,Result& output,Report& report,Timing& timing,ProfileClocks& clocks) {
    if (status_!=cudaSuccess) return status_;
    auto error=cudaGetLastError(); if (error!=cudaSuccess) return Fail(error);
    const auto start=std::chrono::steady_clock::now();
    sc::PreparedPlanarWallQuery query;
    if (input.face_count && input.face_count<=MaxFaces)
        query.Initialize(input.faces,input.face_count,input.wall_tolerance);
    error=cudaMemcpy(&storage_->work.input,&input,sizeof(Input),cudaMemcpyHostToDevice); if (error!=cudaSuccess) return Fail(error);
    error=cudaMemcpy(&storage_->work.query,&query,sizeof(query),cudaMemcpyHostToDevice); if (error!=cudaSuccess) return Fail(error);
    error=cudaEventRecord(begin_); if (error!=cudaSuccess) return Fail(error);
    EvaluateKernel<true><<<1,Workers>>>(&storage_->work,&storage_->clocks);
    error=cudaGetLastError(); if (error!=cudaSuccess) return Fail(error);
    error=cudaEventRecord(end_); if (error!=cudaSuccess) return Fail(error);
    error=cudaEventSynchronize(end_); if (error!=cudaSuccess) return Fail(error);
    Report checked; Timing measured; Result next; ProfileClocks trace;
    error=cudaMemcpy(&checked,&storage_->work.report,sizeof(Report),cudaMemcpyDeviceToHost); if (error!=cudaSuccess) return Fail(error);
    error=cudaEventElapsedTime(&measured.kernel_ms,begin_,end_); if (error!=cudaSuccess) return Fail(error);
    if (checked.status==Status::Ok) {
        error=cudaMemcpy(&next,&storage_->work.published,sizeof(Result),cudaMemcpyDeviceToHost); if (error!=cudaSuccess) return Fail(error);
        error=cudaMemcpy(&trace,&storage_->clocks,sizeof(ProfileClocks),cudaMemcpyDeviceToHost); if (error!=cudaSuccess) return Fail(error);
    }
    measured.checked_end_to_end_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
    if (checked.status==Status::Ok) { output=next; clocks=trace; }
    report=checked; timing=measured; return cudaSuccess;
}
} // namespace crash::qualification::source_contact::nodal
