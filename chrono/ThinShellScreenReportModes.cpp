#include "ThinShellScreenReportDetail.h"

namespace crash::reference::screen_report {
Document Kinetic(const ShellPatchKineticEnergy& v) {
    auto d=Object();
    Number(d,"translation",v.translation); Number(d,"physical_rotation",v.physical_rotation);
    Number(d,"original_artificial_drilling",v.original_artificial_drilling);
    Number(d,"added_tangential",v.added_tangential); Number(d,"added_drilling",v.added_drilling);
    return d;
}
namespace {
Document Mode(const ThinShellModeDiagnostic& v) {
    auto d=Object(); Boolean(d,"positive_frequency",v.positive_frequency);
    Number(d,"angular_frequency_rad_per_s",v.angular_frequency);
    Number(d,"out_of_plane_mass_fraction",v.out_of_plane_mass_fraction);
    Number(d,"normal_translation_mass_fraction",v.normal_translation_mass_fraction);
    Number(d,"translation_mass_fraction",v.translation_mass_fraction);
    Number(d,"transverse_rotation_mass_fraction",v.transverse_rotation_mass_fraction);
    Number(d,"drilling_mass_fraction",v.drilling_mass_fraction); Nested(d,"modal_kinetic",Kinetic(v.kinetic));
    Boolean(d,"kinetic_available",v.kinetic_available);
    Integer(d,"kinetic_status",static_cast<unsigned>(v.kinetic_status));
    String(d,"kinetic_diagnostic",v.kinetic_diagnostic);
    Boolean(d,"finite_amplitude_available",v.finite_amplitude_available);
    Integer(d,"finite_amplitude_status",static_cast<unsigned>(v.finite_amplitude_status));
    String(d,"finite_amplitude_diagnostic",v.finite_amplitude_diagnostic);
    FiniteArray(d,"amplitude_m",v.amplitude_m.data(),2);
    FiniteArray(d,"total_energy_J",v.total_energy.data(),2);
    FiniteArray(d,"bending_energy_J",v.bending_energy.data(),2);
    FiniteArray(d,"shear_energy_J",v.shear_energy.data(),2);
    FiniteArray(d,"shear_fraction",v.shear_fraction.data(),2);
    return d;
}
Document Cluster(const ThinShellClusterDiagnostic& v) {
    auto d=Object(); Integer(d,"label",v.label); Integer(d,"count",v.count);
    Indices(d,"modes",v.modes,v.count);
    Number(d,"mean_out_of_plane_fraction",v.mean_out_of_plane_fraction);
    Number(d,"mean_normal_translation_fraction",v.mean_normal_translation_fraction);
    Number(d,"tip_a_row_norm",v.tip_a_row_norm); Number(d,"tip_b_row_norm",v.tip_b_row_norm);
    Number(d,"mean_tip_row_norm",v.mean_tip_row_norm); Boolean(d,"bending_eligible",v.bending_eligible);
    String(d,"diagnostic",v.diagnostic); return d;
}
Document Pool(const ShellModeSet& v) {
    Require(v.mode_count<=kShellModeCapacity&&v.coordinate_count<=kShellTranslationCapacity,
            "Thin-shell mode pool exceeds its fixed capacity");
    auto d=Object(); Integer(d,"coordinate_count",v.coordinate_count); Integer(d,"mode_count",v.mode_count);
    FiniteArray(d,"frequency_rad_per_s",v.frequency.data(),v.mode_count);
    Indices(d,"cluster",v.cluster,v.mode_count);
    Value rows(rapidjson::kArrayType);
    for(std::size_t i=0;i<v.mode_count;++i)
        rows.PushBack(FiniteArray(d,v.translation[i].data(),v.coordinate_count),d.GetAllocator());
    d.AddMember("physical_translation",rows,d.GetAllocator()); return d;
}
Document Comparison(const ShellModeComparison& v) {
    Require(v.cluster_count<=kShellModeCapacity&&v.mode_count<=kShellModeCapacity,
            "Thin-shell comparison exceeds its fixed capacity");
    auto d=Object(); Integer(d,"mode_count",v.mode_count); Integer(d,"cluster_count",v.cluster_count);
    Number(d,"minimum_squared_cosine",v.minimum_squared_cosine);
    Number(d,"maximum_relative_frequency_change",v.maximum_relative_frequency_change);
    Indices(d,"singleton_candidate",v.singleton_candidate,v.mode_count);
    Value array(rapidjson::kArrayType);
    for(std::size_t i=0;i<v.cluster_count;++i) {
        const auto& c=v.clusters[i]; auto item=Object();
        Integer(item,"reference_cluster",c.reference_cluster); Integer(item,"candidate_cluster",c.candidate_cluster);
        Integer(item,"mode_count",c.mode_count); Indices(item,"reference_modes",c.reference_modes,c.mode_count);
        Indices(item,"candidate_modes",c.candidate_modes,c.mode_count);
        Number(item,"minimum_squared_cosine",c.minimum_squared_cosine);
        Number(item,"maximum_relative_frequency_change",c.maximum_relative_frequency_change);
        Value copy; copy.CopyFrom(item,d.GetAllocator()); array.PushBack(copy,d.GetAllocator());
    }
    d.AddMember("clusters",array,d.GetAllocator()); return d;
}
} // namespace
Document Spectrum(const ThinShellSpectrumDiagnostic& v) {
    Require(v.cluster_count<=kCouponFreeDofs,"Thin-shell spectrum cluster count is invalid");
    auto d=Object(); Boolean(d,"available",v.available); Boolean(d,"positive_spectrum",v.positive_spectrum);
    String(d,"diagnostic",v.diagnostic); FiniteArray(d,"squared_frequency",v.squared_frequency.data(),kCouponFreeDofs);
    MatrixRows(d,"mass_modes_columns",v.mass_modes); Number(d,"symmetry_error",v.symmetry_error);
    Number(d,"eigen_residual",v.eigen_residual); Objects(d,"modes",v.modes,Mode);
    Integer(d,"cluster_count",v.cluster_count);
    Value clusters(rapidjson::kArrayType);
    for(std::size_t i=0;i<v.cluster_count;++i) {
        const auto item=Cluster(v.clusters[i]); Value copy; copy.CopyFrom(item,d.GetAllocator());
        clusters.PushBack(copy,d.GetAllocator());
    }
    d.AddMember("clusters",clusters,d.GetAllocator()); return d;
}
Document Match(const ThinShellModeMatchDiagnostic& v) {
    auto d=Object(); Boolean(d,"attempted",v.attempted); Boolean(d,"matched",v.matched);
    Boolean(d,"frequency_refined",v.frequency_refined); Integer(d,"status",static_cast<unsigned>(v.status));
    Number(d,"maximum_fd_relative_frequency_change",v.maximum_fd_relative_frequency_change);
    String(d,"diagnostic",v.diagnostic);
    Indices(d,"reference_full_indices",v.reference_modes,v.reference_pool.mode_count);
    Indices(d,"candidate_full_indices",v.candidate_modes,v.candidate_pool.mode_count);
    Nested(d,"reference_pool",Pool(v.reference_pool)); Nested(d,"candidate_pool",Pool(v.candidate_pool));
    Nested(d,"comparison",Comparison(v.comparison)); return d;
}
} // namespace crash::reference::screen_report
