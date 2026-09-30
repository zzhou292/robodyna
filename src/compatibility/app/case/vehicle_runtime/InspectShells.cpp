#include "Storage.h"
#include "Reports.h"
#include "lib_src/elements/ShellBatchLayeredSection.h"
namespace crash::cases::vehicle_runtime {
namespace fe = tl::fea;
namespace {
void Virgin(const tl::material::TabulatedShellPlasticityHistory& point) {
    output::Require(point.plastic_strain==0 && point.filtered_rate_per_s==0,"Initial point history has advanced");
    for (double stress:point.stress) output::Require(stress==0,"Initial shell point stress is nonzero");
}
template<class Batch,class Diagnostics> void Sections(Batch& batch,fe::ShellBindingFamily family,
    const Source& source,const fe::NodalStamp& stamp,std::size_t count,InitialInspection& out) {
    std::vector<fe::ShellBatchLayeredSection> rows(count);
    Diagnostics diagnostics;
    detail::RequireSuccess(batch.CopyAcceptedLayeredSectionHistory(stamp,rows.data(),rows.size(),&diagnostics));
    output::Require(diagnostics.valid && !diagnostics.has_completed_interval,"Initial layered readback has an interval");
    for (std::size_t i=0;i<count;++i) {
        const auto* role = (*source.physical().execution()).parent(family,i);
        output::Require(role && rows[i].law()==role->law,"Accepted section differs from immutable source role");
        out.shell_parents++;
        out.material_points += role->material_points;
        if (role->material_points==0) {
            output::Require(!rows[i].plastic() && !rows[i].elastic() && !rows[i].one_point(),
                "Zero-point law exposes fabricated point history");
            if(rows[i].law()==fe::ShellSectionLaw::RigidSkin) ++out.rigid_skins;
            else {
                output::Require(rows[i].law()==fe::ShellSectionLaw::GlobalLaw1Npt0,
                    "Unknown zero-point constitutive role");
                ++out.zero_point_parents;
            }
        } else if (const auto* p=rows[i].one_point()) {
            output::Require(role->material_points==1,"One-point role has a fabricated layer count");
            Virgin(p->point.saved);
            ++out.one_point_parents;
        } else if (const auto* p=rows[i].plastic()) {
            output::Require(role->material_points==3,"Plastic section point shape differs");
            for (const auto& point:p->history.point) Virgin(point);
            ++out.three_point_parents;
        } else {
            output::Require(rows[i].elastic() && role->material_points==3,"Elastic section point shape differs");
            ++out.three_point_parents;
        }
    }
}
}
void VehiclePhysicalStartup::Storage::InspectShells(InitialInspection& out) {
    Sections<fe::qeph::QephBatch,fe::qeph::BatchDiagnostics>(qeph,fe::ShellBindingFamily::Qeph,
        source,out.stamp,source.physical().shells()->qeph_count(),out);
    Sections<fe::t3::T3Batch,fe::t3::BatchDiagnostics>(t3,fe::ShellBindingFamily::T3,
        source,out.stamp,source.physical().shells()->t3_count(),out);
    std::vector<fe::qbat::BatchResult> rows(source.physical().shells()->qbat_count());
    fe::qbat::BatchDiagnostics diagnostics;
    detail::RequireSuccess(qbat.CopyAcceptedResults(out.stamp,rows.data(),rows.size(),&diagnostics));
    output::Require(diagnostics.valid && !diagnostics.has_completed_interval,"Initial QBAT readback has an interval");
    for (std::size_t i=0;i<rows.size();++i) {
        const auto* role = (*source.physical().execution()).parent(fe::ShellBindingFamily::Qbat,i);
        output::Require(role && role->material_points==4 && role->law==fe::ShellSectionLaw::Law44QbatFourInPlane &&
            rows[i].stamp.time==0 && rows[i].stamp.sample_index==0,"Initial QBAT source/point stamp differs");
        for (const auto& point:rows[i].history.point) Virgin(point.material);
        ++out.shell_parents;
        ++out.four_point_parents;
        out.material_points += 4;
    }
    const auto counts = (*source.physical().execution()).counts();
    output::Require(out.material_points==counts.material_points && out.rigid_skins==counts.rigid_skin &&
        out.shell_parents==counts.constitutive+counts.rigid_skin,"Complete accepted shell census differs from source");
}
} // namespace crash::cases::vehicle_runtime
