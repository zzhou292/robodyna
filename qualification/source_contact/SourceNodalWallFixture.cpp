#include "SourceNodalWallFixture.h"
#include "collision/PrescribedSurfaceInterval.h"
#include <algorithm>

namespace crash::qualification::source_contact::nodal {
struct PreparedSource::Data {
    std::array<sc::Q4ParametricReference,ParentCount> quad;
    std::array<sc::T3MaterialMeasure,ParentCount> triangle;
    std::array<sc::NodalWallParentInput,ParentCount> inputs{};
    sc::NodalWallWeights weights;
    cf::FixtureMass mass;
};
PreparedSource::PreparedSource()=default;
PreparedSource::~PreparedSource()=default;
bool PreparedSource::prepared() const noexcept { return data_!=nullptr; }
const sc::NodalWallWeights& PreparedSource::weights() const { return data_->weights; }
const cf::FixtureMass& PreparedSource::mass() const { return data_->mass; }
bool PreparedSource::ParentWeights(unsigned source_parent,sc::NodalWallWeights* output) const {
    return data_ && output && source_parent<ParentCount &&
        output->Initialize(NodeCount,&data_->inputs[source_parent],1).status==sc::NodalWallStatus::Ok;
}
bool PreparedSource::Initialize(const SourcePartContactFixture& source,std::string& diagnostic) {
    diagnostic.clear();
    if (data_ || !source.prepared()) { diagnostic="Source preparation requires a fresh object and authenticated input"; return false; }
    auto next=std::make_unique<Data>();
    if (!next->mass.Initialize(source)) { diagnostic="Chosen source fixture mass failed"; return false; }
    for (unsigned p=0;p<ParentCount;++p) {
        if (source.parents()[p].arity==4) {
            sc::SurfaceQ4 parent;
            if (!source.q4_parent(p,parent) ||
                next->quad[p].Initialize(source.positions(),&parent,1).status!=sc::Q4ParametricStatus::Ok) {
                diagnostic="Original Q4 immutable area failed"; return false;
            }
            next->inputs[p]={&next->quad[p],0,nullptr};
        } else {
            sc::SurfaceTriangle parent;
            if (!source.t3_parent(p,parent) ||
                sc::PrepareT3MaterialMeasure(source.positions(),parent,&next->triangle[p])!=sc::SurfaceMeasureStatus::Ok) {
                diagnostic="Original native T3 immutable area failed"; return false;
            }
            next->inputs[p]={nullptr,0,&next->triangle[p]};
        }
    }
    if (next->weights.Initialize(NodeCount,next->inputs.data(),ParentCount).status!=sc::NodalWallStatus::Ok ||
        next->weights.node_count()!=NodeCount || next->weights.parent_count()!=ParentCount) {
        diagnostic="Source nodal contact weights failed or omitted a physical node"; return false;
    }
    for (unsigned n=0;n<NodeCount;++n) if (next->weights.node(n).node!=n) {
        diagnostic="Source dense node order changed"; return false;
    }
    data_=std::move(next); return true;
}

Report BuildInput(const PreparedSource& prepared,const cf::Coordinates& base,const cf::Coordinates& endpoint,
    const cf::Coordinates& velocity,const cf::FixtureMass& mass,const sc::PlanarWallGeometry& wall,
    std::uint64_t attempt,Input* output,std::string& diagnostic) {
    diagnostic.clear();
    if (!output || !prepared.prepared() || !attempt || !wall.initialized() ||
        wall.faces().empty() || wall.faces().size()>MaxFaces) return {};
    Input next;
    next.config={wall.wall_x(),cf::Stiffness,cf::Cap,cf::ForceBudget,cf::EnergyBudget};
    next.attempt=attempt; next.base_epoch=mass.view().base_epoch;
    next.wall_tolerance=wall.tolerance(); next.face_count=static_cast<unsigned>(wall.faces().size());
    const auto& weights=prepared.weights();
    for (unsigned p=0;p<ParentCount;++p) {
        const auto& parent=weights.parent(p);
        sc::PrescribedParentInterval interval;
        const auto checked=sc::CheckPrescribedSurfaceInterval(cf::View(base),cf::View(velocity),
            cf::View(endpoint),cf::View(velocity),mass.view(),parent.nodes,parent.arity,
            wall.wall_x(),cf::Cap,&interval);
        if (checked.status!=sc::PrescribedNodeStatus::Ok) {
            diagnostic="Shared mass/fixed-motion/endpoint preflight failed";
            return {Status::PreflightFailure,{},checked.node,p};
        }
        for (unsigned n=0;n<parent.arity;++n) if (mass.fixed[parent.nodes[n]] &&
            (interval.base_gaps[n].upper>0 || interval.endpoint_gaps[n].upper>0)) {
            diagnostic="Fixed penetrating or unresolved node is outside the nodal model";
            return {Status::PreflightFailure,{},parent.nodes[n],p};
        }
        sc::PlanarWallBoxCoverage coverage;
        const auto covered=sc::CheckPlanarWallBox(wall,interval.physical,cf::Clearance,parent.feature_id,
            sc::PlanarWallBoxMode::ConservativeExpansion,&coverage);
        if (covered.status!=sc::PlanarContactStatus::Ok || !coverage.covered) {
            diagnostic="Actual finite-wall box coverage failed, including exposed edges and holes";
            return {Status::OutsideWall,{},UINT32_MAX,p};
        }
        next.parents[p]=parent;
    }
    unsigned count=0;
    for (unsigned n=0;n<NodeCount;++n) {
        next.nodes[n]=weights.node(n); next.first_share[n]=count;
        for (unsigned p=0;p<ParentCount;++p) for (unsigned local=0;local<next.parents[p].arity;++local)
            if (next.parents[p].nodes[local]==n) next.share_slot[count++]=4*p+local;
    }
    next.first_share[NodeCount]=count; next.share_count=count;
    if (count!=4*Q4Count+3*T3Count) { diagnostic="Native contribution coverage changed"; return {}; }
    std::copy(wall.faces().begin(),wall.faces().end(),next.faces);
    std::copy(endpoint.begin(),endpoint.end(),next.position);
    std::copy(velocity.begin(),velocity.end(),next.velocity);
    std::copy(mass.inverse.begin(),mass.inverse.end(),next.inverse_mass);
    std::copy(mass.fixed.begin(),mass.fixed.end(),next.fixed);
    next.prepared=true; *output=next; return {Status::Ok};
}

Report EvaluateHost(const PreparedSource& prepared,const Input& input,Result* output) {
    if (!output || !prepared.prepared() || !input.prepared || !input.face_count || input.face_count>MaxFaces) return {};
    Result next;
    const sc::VectorView position{input.position,NodeCount,3,1},velocity{input.velocity,NodeCount,3,1};
    const sc::LumpedTranslationMassView mass{input.inverse_mass,input.fixed,NodeCount,input.base_epoch,
        sc::TranslationMassModel::kIsotropicLumped};
    const auto report=sc::EvaluateNodalWallContact(prepared.weights(),position,velocity,mass,input.config,input.attempt,&next.contact);
    if (report.status!=sc::NodalWallStatus::Ok) return {Status::PointFailure,report,report.node,report.parent};
    for (unsigned n=0;n<NodeCount;++n) {
        unsigned owner=UINT32_MAX; sc::TrianglePointGeometry point;
        const auto status=sc::planar_detail::FindOwner(next.contact.nodes[n].wall_point,input.faces,input.face_count,
            input.wall_tolerance,&owner,&point);
        if (status!=sc::Status::kOk || owner==UINT32_MAX) return {Status::OutsideWall,{},n};
        next.wall_face[n]=input.faces[owner].geometry.face_id;
    }
    *output=next; return {Status::Ok};
}
} // namespace crash::qualification::source_contact::nodal
