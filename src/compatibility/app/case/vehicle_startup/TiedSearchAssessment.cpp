#include "TiedSearchAssessment.h"
#include "tied_search/Internal.h"
namespace crash::cases::vehicle_startup {
struct TiedSearchAssessment::Storage {
    explicit Storage(const tied::TiedShellSearchGeometry& value) : geometry(value) {}
    tied::TiedShellSearchGeometry geometry;
    native_search::SearchDriverResult result;
    TiedAssessmentForecast forecast;
};
TiedAssessmentForecast TiedSearchAssessment::Forecast(const tied::TiedShellSearchGeometry& g,
        TiedAssessmentLimits limits) {
    const auto& p = g.packing();
    const auto& d = p.declaration();
    return tied_assessment_detail::Preflight(d.canonical().data(), d.data(), p.data(), g.data(), limits);
}
TiedSearchAssessment TiedSearchAssessment::Prepare(const tied::TiedShellSearchGeometry& g,
        TiedAssessmentLimits limits) {
    static_assert(sizeof(Storage) == sizeof(tied::TiedShellSearchGeometry) +
        sizeof(native_search::SearchDriverResult) + sizeof(TiedAssessmentForecast),
        "Update tied assessment fixed payload accounting for changed storage");
    const auto forecast = Forecast(g, limits);
    const auto& declaration = g.packing().declaration().data();
    const auto inputs = tied_assessment_detail::Pack(declaration, g.packing().data(), g.data());
    auto draft = std::make_shared<Storage>(g);
    draft->forecast = forecast;
    const auto report = native_search::AssessSearch(inputs.View(g.data()), limits.driver, draft->result);
    if (!report) {
        const auto node = report.secondary < declaration.slave_nodes.size() ? declaration.slave_nodes[report.secondary].id : 0;
        const auto master = report.master < g.data().masters.size() ?
            declaration.masters.at(g.data().masters[report.master].declaration_row).id : 0;
        throw TiedAssessmentError(report, node, master);
    }
    tied_assessment_detail::CheckResult(declaration, g.data(), draft->result);
    return TiedSearchAssessment(std::move(draft));
}
const tied::TiedShellSearchGeometry& TiedSearchAssessment::geometry() const noexcept { return data_->geometry; }
const native_search::SearchDriverResult& TiedSearchAssessment::result() const noexcept { return data_->result; }
const TiedAssessmentForecast& TiedSearchAssessment::forecast() const noexcept { return data_->forecast; }
const tied::Node& TiedSearchAssessment::secondary(std::size_t s) const {
    return geometry().packing().declaration().data().slave_nodes.at(s);
}
const tied::Element* TiedSearchAssessment::selected_master(std::size_t s) const {
    const auto row = tied_assessment_detail::SelectedDeclarationRow(geometry().data(), result(), s);
    return row == SIZE_MAX ? nullptr : &geometry().packing().declaration().data().masters.at(row);
}
tied::SourceId TiedSearchAssessment::selected_part_id(std::size_t s) const {
    const auto* master = selected_master(s);
    return master ? geometry().packing().declaration().data().parts.at(master->part_index).id : 0;
}
} // namespace crash::cases::vehicle_startup
