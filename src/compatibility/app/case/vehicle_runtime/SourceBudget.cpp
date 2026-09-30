#include "Forecast.h"
#include "lib_utils/BoundedArena.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_runtime::detail {
std::size_t SourceBytes(const Execution& execution,const Attachments& attachments,std::size_t cap) {
    // CheckSource established actual shared app/native backing before any discount.
    // Keep older immutable app producer bounds conservatively; replace the model
    // constructor's independent native caps/temporary packing by its complete
    // actual physical handle plus retained solid reference model. TYPE13,
    // beam18's Model and all nodal coefficient producers are already retained
    // by that physical graph.
    const auto& model = execution.model().forecast();
    const auto& cin = attachments.attachments().forecast();
    const auto& roster = attachments.witnesses().data();
    tl::util::BoundedArenaLayout budget(cap);
    tl::util::ArenaRegion unused;
    for (auto bytes : {model.shell_source,model.physical_source,model.producer_source,
        execution.physical().owned_payload_bytes(),execution.model().solids().owned_payload_bytes(),
        cin.total_host_bytes,attachments.witnesses().forecast().fixed_bytes,
        attachments.forecast().fixed_bytes,execution.forecast().fixed_bytes,std::size_t{4096}})
        output::Require(budget.Append<std::byte>(bytes,unused),"Retained original source graph exceeds runtime host cap");
    output::Require(budget.Append<vehicle_startup::cin_stage::WitnessRange>(roster.ranges.capacity(),unused) &&
        budget.Append<vehicle_startup::cin_stage::ActiveWitness>(roster.witnesses.capacity(),unused) &&
        budget.Append<vehicle_startup::TiedCinWitnessOrigin>(roster.origins.capacity(),unused),
        "Actual retained CIN roster capacity exceeds runtime host cap");
    return budget.bytes();
}
} // namespace crash::cases::vehicle_runtime::detail
