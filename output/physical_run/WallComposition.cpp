#include "WallComposition.h"
#include "output/BoundedArrayJson.h"
namespace crash::output::physical_run {
namespace {
struct Names {const char* domain;const char* solids;const char* ledger;};
Names ProfileNames(CompositionProfile profile) {
    if(profile==CompositionProfile::RetainedV1)
        return {"RetainedShellAssembliesV1","OriginalAdhesive18RubberHephS6zV1",
            "PreparedSI_Q_T_B_Type25_Type13_ElementMass_Solid18_24_6z_V3"};
    Require(profile==CompositionProfile::ExtendedSolidsV4,"Unknown wall physical composition profile");
    return {"RetainedShellAssembliesExtendedSolidsV4","OriginalExtendedSolidsV4",
        "PreparedSI_Q_T_B_Type25_Type13_ElementMass_Solid18_24_6z_Law44_Law90_V4"};
}
constexpr std::array<const char*,5> Families{{"law36_solid18","heph24","s6z","law44_solid18","law90_solid18"}};
void Check(const WallComposition& c) {
    ProfileNames(c.profile);
    const bool extended=c.profile==CompositionProfile::ExtendedSolidsV4;
    const std::array<std::uint64_t,5> counts=extended?
        std::array<std::uint64_t,5>{908,1991,350,306,1345}:std::array<std::uint64_t,5>{908,1309,195,0,0};
    Require(c.solid_parents==counts && c.solid_parts==(extended?16u:9u) &&
        c.point_mass_records==(extended?150u:148u),"Wall physical composition source census differs");
    Require(c.physical_nodes && c.physical_nodes<=524288 && c.part_roots==20 &&
        c.plain_complete<=759 && c.plain_restricted<=759-c.plain_complete &&
        c.plain_omitted==759-c.plain_complete-c.plain_restricted &&
        c.rigid_groups==c.part_roots+c.plain_complete+c.plain_restricted &&
        c.rigid_members>=c.rigid_groups && c.rigid_members<=16384,
        "Wall physical composition domain or rigid census differs");
    Require(std::isfinite(c.initial_mass_kg) && c.initial_mass_kg>0 &&
        std::isfinite(c.point_mass_kg) && c.point_mass_kg>0 && c.point_mass_kg<=c.initial_mass_kg,
        "Wall physical composition mass is invalid");
}
}
Document WallCompositionDocument(const WallComposition& c) {
    Check(c);const auto names=ProfileNames(c.profile);
    Document d;d.SetObject();
    String(d,"schema","robo_dyna.wall_physical_composition.v1");
    String(d,"domain_profile",names.domain);String(d,"solid_source_profile",names.solids);
    String(d,"coefficient_order",names.ledger);
    String(d,"scope","immutable_initial_source_coefficients_not_current_owner_or_energy");
    String(d,"joint_census","unavailable_from_wall_setup");
    Integer(d,"physical_nodes",c.physical_nodes);Integer(d,"solid_parts",c.solid_parts);
    Integer(d,"point_mass_records",c.point_mass_records);
    Document solids;solids.SetObject();
    for(unsigned i=0;i<Families.size();++i)Integer(solids,Families[i],c.solid_parents[i]);
    array_json::Child(d,"solid_parents",solids);
    Integer(d,"part_roots",c.part_roots);Integer(d,"rigid_groups",c.rigid_groups);
    Integer(d,"rigid_members",c.rigid_members);Integer(d,"plain_complete",c.plain_complete);
    Integer(d,"plain_restricted",c.plain_restricted);Integer(d,"plain_omitted",c.plain_omitted);
    Number(d,"initial_mass_kg",c.initial_mass_kg);Integer(d,"initial_mass_binary64",Bits(c.initial_mass_kg));
    Number(d,"point_mass_kg",c.point_mass_kg);Integer(d,"point_mass_binary64",Bits(c.point_mass_kg));
    return d;
}
WallComposition ReadWallComposition(const Value& d) {
    using namespace array_json;
    Keys(d,{"schema","domain_profile","solid_source_profile","coefficient_order","scope","joint_census",
        "physical_nodes","solid_parts","point_mass_records","solid_parents","part_roots","rigid_groups",
        "rigid_members","plain_complete","plain_restricted","plain_omitted","initial_mass_kg",
        "initial_mass_binary64","point_mass_kg","point_mass_binary64"});
    Require(Text(d["schema"])=="robo_dyna.wall_physical_composition.v1" &&
        Text(d["scope"])=="immutable_initial_source_coefficients_not_current_owner_or_energy" &&
        Text(d["joint_census"])=="unavailable_from_wall_setup","Unsupported wall composition receipt");
    WallComposition c;
    const auto profile=Text(d["domain_profile"]);
    c.profile=profile=="RetainedShellAssembliesV1"?CompositionProfile::RetainedV1:CompositionProfile::ExtendedSolidsV4;
    const auto names=ProfileNames(c.profile);
    Require(profile==names.domain && Text(d["solid_source_profile"])==names.solids &&
        Text(d["coefficient_order"])==names.ledger,"Wall physical composition profiles disagree");
    c.physical_nodes=UInt(d["physical_nodes"]);c.solid_parts=UInt(d["solid_parts"]);
    c.point_mass_records=UInt(d["point_mass_records"]);
    const auto& solids=d["solid_parents"];
    Keys(solids,{"law36_solid18","heph24","s6z","law44_solid18","law90_solid18"});
    for(unsigned i=0;i<Families.size();++i)c.solid_parents[i]=UInt(solids[Families[i]]);
    c.part_roots=UInt(d["part_roots"]);c.rigid_groups=UInt(d["rigid_groups"]);
    c.rigid_members=UInt(d["rigid_members"]);c.plain_complete=UInt(d["plain_complete"]);
    c.plain_restricted=UInt(d["plain_restricted"]);c.plain_omitted=UInt(d["plain_omitted"]);
    c.initial_mass_kg=Real(d["initial_mass_kg"]);c.point_mass_kg=Real(d["point_mass_kg"]);
    Require(Bits(c.initial_mass_kg)==UInt(d["initial_mass_binary64"]) &&
        Bits(c.point_mass_kg)==UInt(d["point_mass_binary64"]),"Wall composition mass representation differs");
    Check(c);return c;
}
std::optional<WallComposition> ReadSetupComposition(const Value& setup,
    std::uint64_t surface_nodes,std::uint64_t canonical_nodes) {
    using namespace array_json;
    Require(setup.IsObject() && setup.HasMember("schema") && setup.HasMember("physical_nodes"),
        "Missing wall setup physical source fields");
    const auto nodes=UInt(setup["physical_nodes"]);
    Require(surface_nodes<=nodes && nodes<=canonical_nodes,"Wall physical domain is outside source bounds");
    const auto schema=Text(setup["schema"]);
    if(schema=="robo_dyna.vehicle_wall_setup.v1") {
        Require(!setup.HasMember("physical_composition"),"Legacy wall setup cannot claim a typed composition");
        return std::nullopt;
    }
    Require(schema=="robo_dyna.vehicle_wall_setup.v2" && setup.HasMember("physical_composition"),
        "Wall setup requires its versioned physical composition");
    auto c=ReadWallComposition(setup["physical_composition"]);
    Require(c.physical_nodes==nodes,"Wall composition and setup domain counts differ");
    return c;
}
} // namespace crash::output::physical_run
