#include "TiedSource.h"
#include "output/ArtifactIO.h"
#include "lib_src/assembly/NodalDomainIdentity.h"
#include "lib_src/constraints/tied_shell/TiedSearch.h"
#include "lib_utils/BoundedArena.h"
#include <algorithm>
#include <sstream>
namespace crash::cases::native_scene {
namespace t=tl::constraints::tied_shell;
using output::Require;
namespace {
void CheckFinalizedRows(const t::FinalizationMaps& maps) {
    // Whole native I2TID3 always flushes these message accumulators, including
    // the empty error1078 accumulator. Flush is not an emitted warning/error.
    constexpr unsigned flushes[]{1071,1078,1079,1873,1157,1158,1872};
    bool valid=maps.slaves==std::vector<std::uint32_t>{0,1,2,3}&&
        maps.main_nodes==std::vector<std::uint32_t>{0,1,2,3}&&
        maps.selected_masters==std::vector<std::uint64_t>{1,1,1,1}&&maps.dispositions.size()==4&&
        maps.messages.size()==7;
    for(auto disposition:maps.dispositions)valid=valid&&disposition==t::FinalizationDisposition::Kept;
    for(std::size_t i=0;i<maps.messages.size();++i) {
        const auto& message=maps.messages[i];
        valid=valid&&i<7&&message.id==flushes[i]&&message.action==t::NativeMessageAction::Flush&&
            message.original_slave==SIZE_MAX&&message.ordered_master==0&&
            message.s==0&&message.t==0&&message.selection_distance==0;
    }
    if(valid)return;
    std::ostringstream diagnostic;
    diagnostic<<"Named CIN finalized scope differs: slaves=";
    for(auto value:maps.slaves)diagnostic<<value<<',';
    diagnostic<<" mains=";for(auto value:maps.main_nodes)diagnostic<<value<<',';
    diagnostic<<" dispositions=";for(auto value:maps.dispositions)diagnostic<<int(value)<<',';
    diagnostic<<" messages(id/action/severity/row)=";
    for(const auto& message:maps.messages)diagnostic<<message.id<<'/'<<int(message.action)<<'/'<<
        int(message.severity)<<'/'<<message.original_slave<<',';
    throw std::runtime_error(diagnostic.str());
}
}
struct TiedSource::Data {
    std::array<t::WorkingSearchInput,4> inputs;
    t::FinalizedSearch finalized;
    t::ClassificationResult classified;
    t::PostKinChkResult post;
    t::TiedCinAttachmentModel model;
    std::size_t bytes=0;
};
TiedSource TiedSource::Prepare(const modelio::native_scene::DeclaredSource& source,
    const tl::fea::NodalNodeDomain& domain,TiedSourceLimits limits) {
    const auto& d=source.data();
    Require(limits.host_bytes&&limits.host_bytes<=8u<<20&&d.tied_patch&&!d.rigid_patch&&d.nodes.size()==17&&
        d.wall.size()==8&&d.patch.size()==2&&domain.prepared()&&domain.node_count()==17,
        "Declared CIN source requires its complete bounded physical domain");
    const auto& declared=*d.tied_patch;
    Require(declared.resolved_level==28&&declared.resolved_search==2&&declared.resolved_hierarchy==0&&
        declared.ignore==2&&declared.search_distance_mm==0&&declared.tied_removal==1,
        "Declared CIN source controls are outside the qualified value stages");
    constexpr double length=.001;
    for(std::size_t i=0;i<d.nodes.size();++i) {
        const auto& p=d.nodes[i].xyz_mm;const tl::math::Vec3 si{p.x*length,p.y*length,p.z*length};
        Require(domain.nodes()[i].source_id==d.nodes[i].id&&
            tl::fea::nodal_domain_detail::SamePosition(domain.nodes()[i].position,si),
            "CIN source/domain identity or original working-coordinate conversion differs");
    }
    tl::util::BoundedArenaLayout retained(limits.host_bytes);tl::util::ArenaRegion ignored;
    Require(retained.Append<std::byte>(sizeof(Data)+8192,ignored),"CIN source object exceeds host cap");
    const auto remaining=[&](){return limits.host_bytes-retained.bytes();};
    auto out=std::make_shared<Data>();
    std::array<t::SearchChoice,4> choices{};
    const std::array<std::array<std::uint32_t,4>,1> masters{declared.master_nodes};
    auto main_nodes=declared.master_nodes;std::sort(main_nodes.begin(),main_nodes.end());
    // I2COR3 scans complete Q4 then T3 incidence. Each dependent has exactly
    // one physical Q4 contributor, while this main's unique INCOQ match is EID9.
    // Both consumed thicknesses therefore come from the actual source property.
    for(std::size_t row=0;row<4;++row) {
        auto& in=out->inputs[row];in.topology=t::MasterTopology::Quad;in.working_length_to_m=length;
        in.geometry.secondary_position=d.nodes[declared.secondary_nodes[row]].xyz_mm;
        unsigned secondary_incidence=0;
        for(const auto* family:{&d.patch,&d.wall})for(const auto& parent:*family) {
            const auto end=parent.nodes.begin()+parent.corners;
            if(std::find(parent.nodes.begin(),end,declared.secondary_nodes[row])!=end) {
                Require(parent.id==declared.dependent_parent_id&&parent.part==declared.dependent_part_id,
                    "Named CIN secondary has an undeclared competing shell incidence");
                ++secondary_incidence;in.secondary_shell_thickness=d.thickness_mm;
            }
        }
        Require(secondary_incidence==1,"Declared dependent lacks its real physical shell contribution");
        in.master_thickness=d.thickness_mm;
        for(unsigned slot=0;slot<4;++slot)in.geometry.master_position[slot]=d.nodes[declared.master_nodes[slot]].xyz_mm;
        Require(t::ConsiderCandidate(in,1,choices[row])==t::Status::Success,
            "Original working-coordinate TYPE2 projection rejected");
    }
    t::FinalizationInput final;
    final.masters=masters.data();final.master_count=1;final.slaves=declared.secondary_nodes.data();final.slave_count=4;
    final.main_nodes=main_nodes.data();final.main_node_count=4;final.choices=choices.data();final.node_count=17;
    Require(bool(t::FinalizeSearch(final,&out->finalized,{17,1,4,remaining()})),"TYPE2 finalization rejected");
    const auto& maps=*out->finalized.data();
    CheckFinalizedRows(maps);
    Require(retained.Append<std::byte>(maps.owned_payload_bytes,ignored),"Finalized source retention exceeds cap");
    std::array<t::ClassificationNode,17> nodes{};
    std::array<std::uint32_t,17> all{};
    for(unsigned i=0;i<17;++i) {
        nodes[i].source_id=std::uint32_t(d.nodes[i].id);all[i]=i;
        // KININI condition bit1 is BCS; this source has one global-frame
        // six-direction fixed condition per wall node and no repeated condition.
        if(i<9)nodes[i].kinematics={1,7,7,0,0};
    }
    std::array<std::uint32_t,4> finalized_slaves{};
    for(std::size_t row=0;row<4;++row) {
        Require(maps.slaves[row]<4&&maps.selected_masters[row]==1,"Finalized source row/rank differs");
        finalized_slaves[row]=declared.secondary_nodes[maps.slaves[row]];
    }
    // TYPE25 is explicitly present but unselected by ITAGSL2; it does not
    // manufacture a constraint. Complete source nodes retain the real wall BCs.
    const std::array<t::ClassificationInterface,2> interfaces{{
        {1,25,1,{all.data(),17},{all.data(),17}},
        {2,2,28,{finalized_slaves.data(),4},{main_nodes.data(),4}}}};
    t::ClassificationInput input;input.context={domain.source_instance_id(),{nodes.data(),nodes.size()}};
    input.interfaces={interfaces.data(),interfaces.size()};
    t::ClassificationLimits classification_limits;classification_limits.max_nodes=17;
    classification_limits.max_interfaces=2;classification_limits.max_roles=64;classification_limits.max_occurrences=128;
    classification_limits.max_host_bytes=remaining();
    Require(bool(t::Classify(input,&out->classified,classification_limits)),"Source-derived TYPE2 classification rejected");
    Require(retained.Append<std::byte>(out->classified.owned_payload_bytes(),ignored),"Classification retention exceeds cap");
    const auto classified=out->classified.interfaces();
    Require(classified.count==2&&classified.data[1].selected&&classified.data[1].source_id==2&&
        classified.data[1].slave_count==4,"Complete selected TYPE2 classification scope differs");
    std::array<t::KinChkSlave,4> slaves{};
    const auto code=out->classified.irupt();const auto changed=out->classified.nodes();
    for(unsigned row=0;row<4;++row) {
        const auto node=finalized_slaves[row];const auto irupt=code.data[classified.data[1].slave_offset+row];
        Require(irupt==0,"The first named CIN source does not admit a penalty fallback");
        slaves[row]={std::uint32_t(d.nodes[node].id),irupt,changed.data[node].kinematics};
    }
    const t::KinChkInput post{t::KinChkProfile::NoWallRbeOrCyclic,out->classified.phase(),domain.source_instance_id(),
        declared.interface_id,{slaves.data(),slaves.size()},out->classified.interface_decode()};
    Require(bool(t::PostKinChk(post,&out->post,{4,remaining()})),"Post-KINCHK source disposition rejected");
    std::array<t::CinAttachmentDeclaration,4> attachments{};
    for(unsigned row=0;row<4;++row) {
        auto& a=attachments[row];a.original_nsv_row=maps.slaves[row];a.ordered_master_rank=maps.selected_masters[row];
        a.master_source={t::CinMasterSourceKind::DeclaredShellElement,declared.master_parent_id,declared.master_part_id};
        a.topology=t::CinMasterTopology::Quad;a.secondary_source_id=d.nodes[finalized_slaves[row]].id;
        a.reference_positions[0]=domain.nodes()[finalized_slaves[row]].position;
        for(unsigned k=0;k<4;++k) {
            a.master_source_ids[k]=d.nodes[declared.master_nodes[k]].id;
            a.reference_positions[k+1]=domain.nodes()[declared.master_nodes[k]].position;
        }
    }
    // The attachment factory charges its borrowed post-KINCHK and domain
    // backing, as well as its own scratch, inside this remaining envelope.
    Require(bool(t::PrepareCinAttachments(out->post,domain,{attachments.data(),attachments.size()},&out->model,{4,remaining()})),
        "Genuine CIN attachment source/domain association rejected");
    for(auto bytes:{out->post.forecast().owned_payload_bytes,
        out->model.forecast().model_payload_bytes,out->model.forecast().domain_payload_bytes})
        Require(retained.Append<std::byte>(bytes,ignored),"Complete immutable CIN source exceeds host cap");
    out->bytes=retained.bytes();return TiedSource(std::move(out));
}
const t::TiedCinAttachmentModel& TiedSource::model() const noexcept{return data_->model;}
const t::FinalizedSearch& TiedSource::search() const noexcept{return data_->finalized;}
const t::ClassificationResult& TiedSource::classification() const noexcept{return data_->classified;}
const t::PostKinChkResult& TiedSource::post_kinchk() const noexcept{return data_->post;}
const std::array<t::WorkingSearchInput,4>& TiedSource::search_inputs() const noexcept{return data_->inputs;}
std::size_t TiedSource::retained_host_upper_bound() const noexcept{return data_->bytes;}
}
