#include "Values.h"
#include "modelio/vehicle_source/SourceCards.h"
#include "modelio/source_assembly/AuxiliarySourceCards.h"
namespace crash::cases::vehicle_self_contact::native::initial_controls::detail {
RawControls ResolveOriginalControls(const modelio::self_contact::Data& selected) {
    using output::Require;
    Require(selected.profile==modelio::self_contact::OriginalSelectionProfile::AutomaticSingleSurfacePartSetV1&&
        selected.sources.size()==3&&selected.source_fields.slave_set_id&&selected.source_fields.master_set_id==0&&
        selected.source_fields.slave_set_type==2,"Initial controls require the actual original single-surface selection");
    const auto& source=selected.sources[0];
    Require(source.block.keyword=="*CONTACT_AUTOMATIC_SINGLE_SURFACE"&&source.cards.size()==8,
        "Initial controls selected source shape differs");
    const auto scalar=[&](std::size_t row,unsigned slot){return modelio::vehicle::detail::SourceScalar(source.cards[row].second,slot);};
    const auto zero=[&](std::size_t row,unsigned first){for(unsigned k=first;k<8;++k){const auto v=scalar(row,k);
        Require(!v||*v==0,"Initial controls have an unsupported nonzero source option");}};
    Require(scalar(0,0)&&*scalar(0,0)==double(selected.source_fields.slave_set_id)&&scalar(0,1)&&*scalar(0,1)==0&&
        scalar(0,2)&&*scalar(0,2)==2,"Initial surface/source set controls differ");zero(0,3);zero(1,3);
    for(unsigned row:{2u,4u,6u,7u})Require(modelio::assembly::reader::auxiliary::Trim(source.cards[row].second).empty(),
        "Initial controls require absent optional gap/load cards");
    Require(scalar(3,0)&&*scalar(3,0)==1&&!scalar(5,0)&&scalar(5,1)&&*scalar(5,1)==1,
        "Initial controls require literal SOFT1 and IGNORE1 source cards");zero(3,1);zero(5,2);
    // Pinned ConvertContacts emits rawIGAP2/IDEL1/INACTI5 and SOFT1->IGSTI4.
    // Fresh reader defaults supply Irem_i2=1/ISHARP1/ITHK0; HM_READ normalizes
    // rawIGAP2 to IGAP1+FLAGREMN2. IDEL's later support state stays on main().
    RawControls out{1,1,2,1,0,0,1,0.};
    out.source_death_blank=!scalar(1,7).has_value();
    return out;
}
}
