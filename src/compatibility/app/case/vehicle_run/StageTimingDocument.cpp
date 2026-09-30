#include "StageTimingDocument.h"
#include <initializer_list>
namespace crash::cases::vehicle_run::detail {
output::Document StageTimingDocument(const vehicle_dynamics::StepTimingSnapshot& timing) {
    using namespace output;
    Document document;
    document.SetObject();
    Boolean(document,"enabled",timing.enabled);
    Boolean(document,"counter_saturated",timing.counter_saturated);
    Integer(document,"clock_failures",timing.clock_failures);
    Integer(document,"backward_samples",timing.backward_samples);
    for(bool last:{false,true}) {
        Value rows(rapidjson::kArrayType);
        const auto& counters=last?timing.last_step:timing.total;
        for(std::size_t i=0;i<counters.size();++i) {
            const auto& counter=counters[i];
            Document row;
            row.SetObject();
            String(row,"stage",vehicle_dynamics::StepStageNames[i]);
            Integer(row,"calls",counter.calls);
            Integer(row,"failures",counter.failures);
            Integer(row,"valid_samples",counter.valid_samples);
            Integer(row,"wall_ns",counter.wall_ns);
            Integer(row,"maximum_ns",counter.maximum_ns);
            Value value;
            value.CopyFrom(row,document.GetAllocator());
            rows.PushBack(value,document.GetAllocator());
        }
        document.AddMember(rapidjson::StringRef(last?"last_attempt":"total"),rows,document.GetAllocator());
    }
    return document;
}
} // namespace crash::cases::vehicle_run::detail
