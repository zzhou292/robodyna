#include "Coverage.h"
#include "modelio/solid_source/SourcePolicy.h"
#include "lib_src/collision/radioss_type25/runtime/physical_main/Index.h"
namespace crash::cases::vehicle_native_contact::activity::coverage {
namespace {
tl::fea::SolidCoefficientFamily PhysicalFamily(modelio::solid_source::Family family) {
    using A=modelio::solid_source::Family;using B=tl::fea::SolidCoefficientFamily;
    switch(family) {
        case A::Solid18:return B::Solid18;
        case A::Solid24:return B::Solid24;
        case A::Solid6z:return B::Solid6z;
        case A::Solid18Law44:return B::Solid18Law44;
        case A::Solid18Law90:return B::Solid18Law90;
    }
    output::Require(false,"Unknown source solid support family");return B::Solid18;
}
}
Population Solids(const detail::SourceInputs& in,const Canonical& canonical) {
    const auto& source=in.owner.execution_source().mechanical().embedding().source().solid_source().data();
    output::Require(source.policy==modelio::solid_source::Policy::NativeConvertedSupportsV6,
        "Activity support requires the explicit native V6 raw8 solid source");
    const auto& array=output::full_shell::source::FindArray(canonical,"solids_records");
    output::Require(array.descriptor.layout.columns==10,"Canonical solid record width differs");
    const auto records=output::arrays::Decode<std::uint64_t>(array.descriptor,array.bytes);
    std::vector<std::uint8_t> selected(records.size()/10);
    const auto& physical=in.owner.physical();const auto* solids=physical.coefficients()->solids();
    output::Require(solids&&solids->parents().size()==source.rows.size(),"Activity solid coverage differs from the actual physical roster");
    tlfea::contact::radioss_type25::runtime_detail::physical_main::Index index;
    const auto indexed=index.Initialize(physical,32u<<20);output::Require(indexed.status==tlfea::contact::radioss_type25::TransactionStatus::Ok,indexed.message);
    for(const auto& row:source.rows) {
        const auto ordinal=index.SolidOrdinal(row.element_id);output::Require(ordinal<solids->parents().size(),"Selected solid has no physical support row");
        const auto& actual=solids->parents()[ordinal];
        output::Require(actual.source_part_id==row.part_id&&actual.node_count==8&&
            actual.family==PhysicalFamily(row.family),"Native solid family/source support differs");
        CheckRecord(records,10,row.canonical_row,actual.source_element_id,actual.source_node_id,8,selected);
    }
    for(std::size_t row=0;row<selected.size();++row)
        output::Require(bool(selected[row])==modelio::solid_source::detail::Selected(records[10*row+1],source.policy),
            "Solid activity exclusion differs from the compiled executed-source policy");
    return Finish(records,10,8,selected,*physical.domain());
}
}
