#include <algorithm>
#include "ActualFixture.h"
#include "../Internal.h"
#include "case/vehicle_self_contact/native/TopologyDigestFields.h"
#include "output/ArtifactIO.h"
#include <filesystem>

namespace crash::cases::vehicle_wall::native::physical_test {
namespace fe=tl::fea;
namespace {
constexpr std::size_t ExportBytes=128u<<10;
std::filesystem::path Destination() {
    const auto* value=std::getenv("ROBO_ENVELOPE_PHYSICAL_OUTPUT");
    output::Require(value&&*value,"Missing create-only combined physical output");
    const std::filesystem::path path(value);
    output::Require(std::filesystem::create_directory(path),"Combined physical output already exists");
    return path;
}
std::size_t QualificationBytes(const EnvelopePhysicalForecast& f) {
    // Original-model comparison is sequential: hash its complete named prefix
    // fields, release it, then construct the combined source. No second model
    // is a production input. Charge its complete6GiB cap plus live wall backing.
    const auto baseline=Wall().forecast().peak_bytes+vehicle_startup::physical_model::Limits::VehicleSupports().host_bytes;
    const auto total=std::max(f.peak_bytes,baseline)+(2u<<20)+ExportBytes;
    output::Require(total<10ull<<30,"Complete parity assessment exceeds its unchanged10GiB guard");
    return total;
}
std::string PrefixDigest(const fe::NodalCoefficientLedger& ledger,const fe::ShellBatchBinding& shells,
    std::size_t nodes,std::size_t q,std::size_t t,std::size_t b) {
    namespace digest=vehicle_self_contact::native::detail::digest;
    output::Require(ledger.nodes().size()>=nodes&&shells.qeph_count()>=q&&shells.t3_count()>=t&&shells.qbat_count()>=b,
        "Incomplete coefficient comparison prefix");
    digest::Fields fields("complete-original-vehicle-coefficient-prefix-v1",1u<<20);
    fields.Add<std::uint64_t>("original_nids",nodes,1,[&](auto i){return ledger.domain()->nodes()[i].source_id;});
    fields.Add<double>("native_coefficient_values",nodes,19,[&](auto i) {
        const auto& c=ledger.nodes()[i/19].coefficients;
        const double values[]{c.mass,c.isotropic_inertia,c.shell.mass,c.shell.isotropic_inertia,
            c.shell.physical_inertia,c.shell.added_inertia,c.type25.mass,c.type25.isotropic_inertia,
            c.type13.mass,c.type13.isotropic_inertia,c.type13.added_inertia,c.element_mass,
            c.solid18_mass,c.solid24_mass,c.solid6z_mass,c.solid18_law44_mass,c.solid18_law90_mass,
            c.beam18.mass,c.beam18.isotropic_inertia};
        return values[i%19];
    });
    fields.Add<std::uint64_t>("coefficient_occurrences",nodes,12,[&](auto i) {
        const auto& c=ledger.nodes()[i/12].occurrences;
        const std::uint64_t values[]{c.qeph,c.t3,c.qbat,c.type25,c.type13,c.element_mass,c.solid18,
            c.solid24,c.solid6z,c.solid18_law44,c.solid18_law90,c.beam18};return values[i%12];
    });
    fields.Add<std::uint64_t>("original_qeph_rows",q,5,[&](auto i) {
        return i%5?std::uint64_t(shells.qeph_nodes(i/5)[i%5-1]):shells.qeph_source_id(i/5);
    });
    fields.Add<std::uint64_t>("original_t3_rows",t,4,[&](auto i) {
        return i%4?std::uint64_t(shells.t3_nodes(i/4)[i%4-1]):shells.t3_source_id(i/4);
    });
    fields.Add<std::uint64_t>("original_qbat_rows",b,5,[&](auto i) {
        return i%5?std::uint64_t(shells.qbat_nodes(i/5)[i%5-1]):shells.qbat_source_id(i/5);
    });
    return fields.Finish().sha256;
}
void SourceCounts() {
    ASSERT_EQ(Wall().vehicle_prefix().nodes,376930u);
    ASSERT_EQ(Wall().domain().node_count(),376934u);
    ASSERT_EQ(References().counts().parents,349645u);
}
void WriteForecast(output::Document& doc,const EnvelopePhysicalForecast& f) {
    output::Integer(doc,"wall_source_bytes",f.wall_source);output::Integer(doc,"embedding_bytes",f.embedding);
    output::Integer(doc,"embedding_prior_peak_bytes",f.embedding_prior_peak);
    output::Integer(doc,"point_mass_current_bytes",f.point_mass_current);
    output::Integer(doc,"point_mass_chain_peak_bytes",f.point_mass_chain_peak);
    output::Integer(doc,"type25_current_bytes",f.type25_current);
    output::Integer(doc,"type25_chain_peak_bytes",f.type25_chain_peak);
    output::Integer(doc,"shell_binding_bytes",f.shell_binding);output::Integer(doc,"contributor_sources_bytes",f.contributor_sources);
    output::Integer(doc,"native_components_bytes",f.native_components);output::Integer(doc,"component_packing_bytes",f.component_packing);
    output::Integer(doc,"fixed_bytes",f.fixed_bytes);output::Integer(doc,"source_peak_bytes",f.peak_bytes);
    output::Integer(doc,"qualification_peak_bytes",QualificationBytes(f));
}
}
TEST(EnvelopePhysicalActual, CompleteForecastRejectsOneByteShortBeforePublication) {
    ASSERT_NO_FATAL_FAILURE(SourceCounts());
    const auto f=EnvelopePhysicalSource::Preflight(Wall(),References());
    EXPECT_GT(f.embedding_prior_peak,modelio::point_mass::Limits{}.host_bytes);
    EXPECT_LE(f.point_mass_current,modelio::point_mass::Limits{}.host_bytes);
    EXPECT_LE(f.type25_current,modelio::type25::Limits{}.host_bytes);
    EXPECT_EQ(f.point_mass_chain_peak,std::max(f.point_mass_current,f.embedding_prior_peak));
    EXPECT_EQ(f.type25_chain_peak,std::max(f.type25_current,f.embedding_prior_peak));
    // The additive path does not weaken either original-domain constructor.
    EXPECT_THROW(modelio::point_mass::VehiclePointMassSource::Prepare(
        Wall().vehicle_origin().source(),Wall().domain()),std::exception);
    EXPECT_THROW(modelio::type25::VehicleType25Source::Prepare(Wall().vehicle_origin().source(),Wall().domain(),
        vehicle_startup::physical_model::detail::WeldDeclaration()),std::exception);
    auto exact=EnvelopePhysicalLimits{};exact.host_bytes=f.peak_bytes;
    EXPECT_EQ(EnvelopePhysicalSource::Preflight(Wall(),References(),exact).peak_bytes,f.peak_bytes);
    --exact.host_bytes;
    EXPECT_THROW(EnvelopePhysicalSource::Prepare(Wall(),References(),exact),std::exception);
    output::Document doc;doc.SetObject();output::String(doc,"schema","robo_dyna.envelope_physical_forecast.v1");
    WriteForecast(doc,f);output::WriteJson(Destination()/"forecast.json",doc);
}
TEST(EnvelopePhysicalActual, EveryOriginalCoefficientAndFamilyIndexSurvivesTheGenuineNewWall) {
    ASSERT_NO_FATAL_FAILURE(SourceCounts());
    const auto forecast=EnvelopePhysicalSource::Preflight(Wall(),References());
    (void)QualificationBytes(forecast);
    std::string before;
    {
        const auto shells=vehicle_startup::VehicleShellBinding::Prepare(References());
        const auto baseline=vehicle_startup::physical_model::VehiclePhysicalModel::Prepare(
            Wall().vehicle_origin(),shells,vehicle_startup::physical_model::Limits::VehicleSupports());
        before=PrefixDigest(baseline.coefficients(),shells.shells(),376930,324094,21301,4250);
    }
    const auto source=EnvelopePhysicalSource::Prepare(Wall(),References());
    const auto after=PrefixDigest(source.coefficients(),source.shells(),376930,324094,21301,4250);
    ASSERT_EQ(after,before);
    ASSERT_EQ(source.domain().node_count(),376934u);
    EXPECT_EQ(source.shells().qeph_count(),324095u);
    EXPECT_EQ(source.shells().t3_count(),21301u);
    EXPECT_EQ(source.shells().qbat_count(),4250u);
    ASSERT_TRUE(source.welds().embedding());
    ASSERT_TRUE(source.point_masses().embedding());
    EXPECT_TRUE(source.welds().domain().SharesStorage(source.domain()));
    EXPECT_TRUE(source.coefficients().domain()->SharesStorage(source.domain()));
    const auto& env=source.environment_parent();
    EXPECT_EQ(env.qeph_index,324094u);
    EXPECT_EQ(env.catalog_append_ordinal,349645u);
    double added_mass=0;
    for(unsigned k=0;k<4;++k) {
        ASSERT_EQ(env.domain_nodes[k],376930u+k);
        const auto& value=source.coefficients().nodes()[env.domain_nodes[k]];
        EXPECT_EQ(output::Bits(value.coefficients.mass),output::Bits(Wall().geometry().reference.nodal_mass[k]));
        EXPECT_EQ(output::Bits(value.coefficients.isotropic_inertia),output::Bits(Wall().geometry().reference.isotropic_inertia[k]));
        EXPECT_EQ(value.occurrences.qeph,1u);
        EXPECT_EQ(value.occurrences.type25+value.occurrences.type13+value.occurrences.element_mass,0u);
        added_mass+=value.coefficients.mass;
    }
    EXPECT_EQ(output::Bits(added_mass),output::Bits(Wall().geometry().wall_mass_kg));
    for(const auto& node:source.coefficients().nodes())ASSERT_GT(node.coefficients.mass,0);
    output::Document doc;doc.SetObject();output::String(doc,"schema","robo_dyna.envelope_physical_source.v1");
    output::String(doc,"scope","Complete immutable coefficients and rigid source; execution/CIN/contact/runtime remain separate");
    output::String(doc,"original_prefix_digest",after);output::String(doc,"wall_source_digest",Wall().digest());
    output::Integer(doc,"vehicle_nodes",376930);output::Integer(doc,"combined_nodes",source.domain().node_count());
    output::Integer(doc,"vehicle_parents",349645);output::Integer(doc,"combined_shell_parents",349646);
    output::Number(doc,"added_wall_mass_kg",added_mass);output::Boolean(doc,"runtime_ready",false);
    WriteForecast(doc,source.forecast());output::WriteJson(Destination()/"source.json",doc);
}
}
