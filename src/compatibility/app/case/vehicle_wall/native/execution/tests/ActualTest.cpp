#include "ActualFixture.h"
#include "../Internal.h"
#include "output/ArtifactIO.h"
#include <algorithm>
#include <cstdlib>
#include <filesystem>
namespace crash::cases::vehicle_wall::native::execution_test {
namespace fe=tl::fea;
namespace {
constexpr std::size_t ExportBytes = 2u << 20;
constexpr std::size_t GuardBytes = std::size_t{10} << 30;
std::size_t Sum(std::initializer_list<std::size_t> values) {
    std::size_t total = 0;
    for (const auto value : values) {
        output::Require(value <= SIZE_MAX-total,"Qualification budget overflow");
        total += value;
    }
    return total;
}
std::size_t ExecutionQualification(const EnvelopeExecutionForecast& f,bool with_inputs) {
    return Sum({f.total_bytes,with_inputs ? Post().forecast().total_host_bytes : 0,
        with_inputs ? JointSource().forecast().total_bytes : 0,ExportBytes});
}
std::size_t WitnessBaselineQualification() {
    // Old binding/witness comparison is sequential and retires before the new
    // execution graph. Keep the shared source and each old producer cap charged.
    return Sum({base::Wall().forecast().peak_bytes,base::References().forecast().total_bytes,
        vehicle_startup::VehicleShellBindingLimits{}.host_bytes,
        vehicle_startup::TiedPostKinChkLimits{}.host_bytes,
        vehicle_startup::TiedCinAttachmentLimits{}.host_bytes,
        vehicle_startup::TiedCinWitnessLimits{}.host_bytes,ExportBytes});
}
std::size_t OwnerQualification(const EnvelopeOwnerForecast& f) {
    return std::max(Sum({f.peak_bytes,ExportBytes}),WitnessBaselineQualification());
}
std::filesystem::path Destination() {
    const auto* value=std::getenv("ROBO_ENVELOPE_EXECUTION_OUTPUT");
    output::Require(value&&*value,"Missing create-only execution source output");
    const std::filesystem::path path(value);
    output::Require(std::filesystem::create_directory(path),"Execution source output already exists");return path;
}
void ExecutionFields(output::Document& doc,const EnvelopeExecutionForecast& f) {
    output::Integer(doc,"retained_mechanical_bytes",f.source_bytes);
    output::Integer(doc,"execution_fixed_bytes",f.fixed_bytes);output::Integer(doc,"execution_packing_bytes",f.packing_bytes);
    output::Integer(doc,"execution_native_reservation_bytes",f.native_reservation);
    output::Integer(doc,"previous_mechanical_peak_bytes",f.previous_source_peak);
    output::Integer(doc,"execution_current_phase_bytes",f.current_phase);output::Integer(doc,"execution_chain_peak_bytes",f.total_bytes);
}
void OwnerFields(output::Document& doc,const EnvelopeOwnerForecast& f) {
    output::Integer(doc,"retained_execution_bytes",f.execution_source);output::Integer(doc,"cin_bytes",f.cin);
    output::Integer(doc,"witness_reservation_bytes",f.witness_reservation);output::Integer(doc,"joint_source_bytes",f.joint_source);
    output::Integer(doc,"joint_model_bytes",f.joint_model);output::Integer(doc,"joint_packing_bytes",f.joint_packing);
    output::Integer(doc,"role_bytes",f.roles);output::Integer(doc,"owner_packing_bytes",f.owner_packing);
    output::Integer(doc,"owner_source_fixed_bytes",f.fixed);output::Integer(doc,"previous_execution_peak_bytes",f.previous_execution_peak);
    output::Integer(doc,"owner_source_current_phase_bytes",f.current_phase);output::Integer(doc,"owner_source_chain_peak_bytes",f.peak_bytes);
}
void SourceCounts() {
    ASSERT_EQ(base::Wall().domain().node_count(),376934u);
    ASSERT_EQ(base::References().counts().parents,349645u);
}
}
TEST(EnvelopeExecutionActual, ForecastRetainedGraphAndExactLocalCapBeforeCatalogAllocation) {
    ASSERT_NO_FATAL_FAILURE(SourceCounts());
    const auto f=EnvelopeExecutionSource::Preflight(Mechanical());
    auto exact=EnvelopeExecutionLimits{};exact.host_bytes=f.total_bytes;
    EXPECT_EQ(EnvelopeExecutionSource::Preflight(Mechanical(),exact).total_bytes,f.total_bytes);
    --exact.host_bytes;
    EXPECT_THROW(EnvelopeExecutionSource::Prepare(Mechanical(),exact),std::exception);
    EXPECT_LE(f.source_bytes,Mechanical().forecast().peak_bytes);
    output::Document doc;doc.SetObject();output::String(doc,"schema","robo_dyna.envelope_execution_forecast.v1");
    ExecutionFields(doc,f);
    const auto qualification=ExecutionQualification(f,false);
    output::Require(qualification<=GuardBytes,"Execution forecast exceeds unchanged10GiB qualification guard");
    output::Integer(doc,"qualification_export_bytes",ExportBytes);
    output::Integer(doc,"execution_qualification_peak_bytes",qualification);
    output::WriteJson(Destination()/"forecast.json",doc);
}
TEST(EnvelopeExecutionActual, EveryMaterialFailureAndSourceRowSurvivesWithOnlyDeclaredWallAppended) {
    ASSERT_NO_FATAL_FAILURE(SourceCounts());
    const auto preflight=EnvelopeExecutionSource::Preflight(Mechanical());
    const auto qualification=ExecutionQualification(preflight,true);
    output::Require(qualification<=GuardBytes,"Execution composition exceeds unchanged10GiB qualification guard");
    const auto& source=Executed();const auto& resolved=*base::References().resolution();
    ASSERT_EQ(source.catalog().parent_count(),349646u);
    ASSERT_EQ(source.failure().parent_count(),349646u);
    std::size_t global=0,mandatory=0;
    for(std::size_t i=0;i<349645;++i) {
        const auto* parent=source.catalog().parent(i);const auto* failure=source.failure().parent(i);
        const auto* raw=resolved.native_parent(i);
        ASSERT_TRUE(parent&&failure&&raw);
        EXPECT_EQ(parent->source_parent_id,raw->source.source_parent_id);
        EXPECT_EQ(parent->source_part_id,raw->source.source_part_id);
        EXPECT_EQ(parent->family,raw->source.family);EXPECT_EQ(parent->family_index,raw->source.family_index);
        EXPECT_EQ(parent->material_id,raw->source.material_id);EXPECT_EQ(parent->section_id,raw->source.section_id);
        EXPECT_EQ(failure->policy,raw->policy);
        EXPECT_EQ(output::Bits(failure->constant.failure_strain),output::Bits(raw->constant.failure_strain));
        EXPECT_EQ(failure->tab1.parent_policy,raw->tab1.parent_policy);
        mandatory+=failure->policy!=fe::ShellFailurePolicy::None;
        const auto part=resolved.parents()[i].part_index;
        if(resolved.material(part)->source.keyword=="*MAT_ELASTIC") {
            EXPECT_EQ(parent->execution.policy,fe::ShellParentExecutionPolicy::GlobalLaw1Npt0);
            EXPECT_EQ(source.execution().parent(i)->material_points,0u);++global;
        } else EXPECT_TRUE(execution_detail::original::detail::Same(*parent,raw->source));
    }
    EXPECT_EQ(global,27177u);EXPECT_GT(mandatory,0u);
    const auto* wall=source.catalog().parent(349645);ASSERT_NE(wall,nullptr);
    EXPECT_EQ(wall->source_parent_id,base::Wall().ids().shell);EXPECT_EQ(wall->family_index,324094u);
    EXPECT_EQ(wall->execution.policy,fe::ShellParentExecutionPolicy::GlobalLaw1Npt0);
    EXPECT_EQ(source.failure().parent(349645)->policy,fe::ShellFailurePolicy::None);
    EXPECT_EQ(source.execution().parent(349645)->material_points,0u);
    EXPECT_EQ(source.execution().counts().material_points,1037877u-3*27177u);
    const auto forecast=EnvelopeOwnerSource::Preflight(source,Post(),JointSource());
    auto exact=EnvelopeOwnerLimits{};exact.host_bytes=forecast.peak_bytes;
    EXPECT_EQ(EnvelopeOwnerSource::Preflight(source,Post(),JointSource(),exact).peak_bytes,forecast.peak_bytes);
    --exact.host_bytes;
    EXPECT_THROW(EnvelopeOwnerSource::Prepare(source,Post(),JointSource(),exact),std::exception);
    output::Document doc;doc.SetObject();output::String(doc,"schema","robo_dyna.envelope_execution_source.v1");
    output::Integer(doc,"parents",source.catalog().parent_count());output::Integer(doc,"original_global_law1",global);
    output::Integer(doc,"original_failure_declarations",mandatory);output::Boolean(doc,"owner_initialized",false);
    ExecutionFields(doc,source.forecast());OwnerFields(doc,forecast);
    output::Integer(doc,"execution_qualification_peak_bytes",qualification);
    output::Integer(doc,"qualification_export_bytes",ExportBytes);
    output::Integer(doc,"old_witness_baseline_peak_bytes",WitnessBaselineQualification());
    output::Integer(doc,"owner_qualification_peak_bytes",OwnerQualification(forecast));
    output::Boolean(doc,"owner_qualification_fits_unchanged_guard",OwnerQualification(forecast)<=GuardBytes);
    output::WriteJson(Destination()/"execution.json",doc);
}
TEST(EnvelopeOwnerActual, RealCinJointGraphAndFixedPackingRetainEveryOriginalWitness) {
    ASSERT_NO_FATAL_FAILURE(SourceCounts());
    output::Require(WitnessBaselineQualification()<=GuardBytes,
        "Sequential old witness comparison exceeds unchanged10GiB guard");
    std::string expected_witnesses;
    {
        // Independent old source wrapper is a qualification reference only.
        // It retires before the new complete execution/owner-source is prepared.
        const auto binding=vehicle_startup::VehicleShellBinding::Prepare(base::References());
        const auto cin=vehicle_startup::TiedCinAttachments::Prepare(Post(),base::Wall().vehicle_origin().domain());
        const auto witnesses=vehicle_startup::TiedCinWitnessRoster::Prepare(cin,binding);
        expected_witnesses=WitnessDigest(witnesses);
    }
    const auto execution_forecast=EnvelopeExecutionSource::Preflight(Mechanical());
    output::Require(ExecutionQualification(execution_forecast,true)<=GuardBytes,
        "Execution input stage exceeds unchanged10GiB guard");
    const auto forecast=EnvelopeOwnerSource::Preflight(Executed(),Post(),JointSource());
    output::Require(OwnerQualification(forecast)<=GuardBytes,
        "Complete owner-source qualification exceeds unchanged10GiB guard");
    const auto source=EnvelopeOwnerSource::Prepare(Executed(),Post(),JointSource());
    ASSERT_EQ(source.attachments().model().rows().count,11165u);
    ASSERT_EQ(source.joints().joints().size(),44u);
    EXPECT_EQ(WitnessDigest(source.witnesses()),expected_witnesses);
    EXPECT_TRUE(source.attachments().model().domain()->SharesStorage(Mechanical().domain()));
    EXPECT_TRUE(source.joints().domain()->SharesStorage(Mechanical().domain()));
    for(std::size_t i=0;i<44;++i) {
        EXPECT_EQ(source.joint_source_rows()[i],i);
        EXPECT_EQ(source.joints().joints()[i].geometry.source_joint_id,JointSource().data().rows[i].source_id);
    }
    auto packed=source.PackOwner(source.forecast().owner_packing);
    const auto& coefficients=Mechanical().coefficients();
    for(std::size_t i=0;i<376934;++i) {
        EXPECT_EQ(output::Bits(packed.mass[i]),output::Bits(coefficients.nodes()[i].coefficients.mass));
        EXPECT_EQ(output::Bits(packed.inertia[i]),output::Bits(coefficients.nodes()[i].coefficients.isotropic_inertia));
        ASSERT_GT(packed.mass[i],0);
        if(i<376930) {
            EXPECT_EQ(packed.fixed[i],0);EXPECT_EQ(packed.rotation_fixed[i],0);
            EXPECT_EQ(output::Bits(packed.velocity[3*i]),output::Bits(vehicle_runtime::InitialSpeedMps));
        } else {
            EXPECT_EQ(packed.fixed[i],7);EXPECT_EQ(packed.rotation_fixed[i],1);EXPECT_EQ(packed.rotation_present[i],1);
            EXPECT_EQ(packed.inverse_mass[i],0);EXPECT_EQ(packed.inverse_inertia[i],0);
            for(unsigned k=0;k<3;++k)EXPECT_EQ(output::Bits(packed.velocity[3*i+k]),output::Bits(0.));
        }
    }
    EXPECT_EQ(source.startup().kind,fe::ShellBatchStartupKind::ReferenceConstrainedUniformTranslation);
    EXPECT_EQ(source.witness_source().model,&source.attachments().model());
    EXPECT_THROW(source.PackOwner(source.forecast().owner_packing-1),std::exception);
    output::Document doc;doc.SetObject();output::String(doc,"schema","robo_dyna.envelope_owner_source.v1");
    output::String(doc,"witness_digest",expected_witnesses);output::Integer(doc,"cin_rows",11165);
    output::Integer(doc,"joints",44);output::Integer(doc,"combined_nodes",376934);output::Integer(doc,"fixed_environment_nodes",4);
    output::Integer(doc,"witnesses",source.witnesses().data().counts.witnesses);
    output::Boolean(doc,"owner_initialized",false);output::Boolean(doc,"contact_runtime_ready",false);
    OwnerFields(doc,source.forecast());
    output::Integer(doc,"old_witness_baseline_peak_bytes",WitnessBaselineQualification());
    output::Integer(doc,"qualification_export_bytes",ExportBytes);
    output::Integer(doc,"owner_qualification_peak_bytes",OwnerQualification(forecast));
    output::WriteJson(Destination()/"owner-source.json",doc);
}
}
