#pragma once
#include "SourcePartContactFixture.h"
#include "collision/PrescribedSurfaceContact.h"
#include <array>
#include <memory>
#include <string>

namespace crash::qualification::source_contact::force {
namespace sf=crash::qualification::source_contact;
namespace sc=tlfea::contact;
inline constexpr double Stiffness=4e5,Depth=.00025,Cap=.0005,Clearance=1e-6;
inline constexpr double ForceBudget=5e-7,EnergyBudget=1.2500000000000005e-12;
inline constexpr const char* MassPolicy="qualification-e2a16-equal-native-node-lump";
using Coordinates=std::array<double,3*sf::NodeCount>;
inline sc::VectorView View(const Coordinates& a) {return {a.data(),sf::NodeCount,3,1};}
struct FixtureMass {
    std::array<double,sf::NodeCount> mass{},inverse{};
    std::array<std::uint8_t,sf::NodeCount> fixed{};
    bool Initialize(const sf::SourcePartContactFixture&);
    sc::LumpedTranslationMassView view() const {
        return {inverse.data(),fixed.data(),sf::NodeCount,17,sc::TranslationMassModel::kIsotropicLumped};
    }
};
struct ParentForce {
    std::uint64_t source_id=0,feature_id=0,base_epoch=0,attempt=0;
    unsigned arity=0;
    std::array<std::uint32_t,4> nodes{}; // Capacity only: native T3 uses exactly3.
    std::array<sc::Vec3,4> force{};
    std::array<sc::Q4CertifiedIntegral,4> magnitude{};
    sc::Q4CertifiedIntegral area,resultant,potential;
    sc::Q4IntegralInterval active_area;
    unsigned cells=0,visits=0,depth_u=0,depth_v=0;
    bool valid=false;
};
struct Aggregate {
    std::array<ParentForce,sf::ParentCount> parents{};
    std::array<sc::Vec3,sf::NodeCount> forces{};
    sc::Vec3 wall_reaction,wall_moment;
    double power=0;
    bool valid=false;
};
struct Scratch {
    std::array<sc::Q4RectangularCell,sc::MaxQ4IntegrationLeaves> cells;
    std::array<std::uint32_t,sc::MaxQ4IntegrationLeaves> heap;
    sc::Q4RectangularScratch view() {return {cells.data(),heap.data(),sc::MaxQ4IntegrationLeaves,sc::MaxQ4IntegrationLeaves};}
};
static_assert(sizeof(Scratch)+sizeof(Aggregate)+sizeof(sf::SourcePartContactFixture)<1024*1024,
              "Source qualification storage must stay below1MiB excluding input parser/wall");

struct Harness {
    const sf::SourcePartContactFixture& source;
    FixtureMass mass;
    std::unique_ptr<Scratch> scratch=std::make_unique<Scratch>();
    std::string diagnostic;
    unsigned failed_parent=UINT32_MAX;
    explicit Harness(const sf::SourcePartContactFixture& source):source(source) {}
    bool EvaluateParent(unsigned,const Coordinates&,const Coordinates&,const Coordinates&,
                        const sc::PlanarWallGeometry&,ParentForce*);
    bool EvaluateAll(const Coordinates&,const Coordinates&,const Coordinates&,
                     const sc::PlanarWallGeometry&,Aggregate*);
};
Coordinates Shift(const sf::SourcePartContactFixture&,double);
Coordinates Velocity(double);
double ParentShift(const sf::SourcePartContactFixture&,unsigned,double);
double WholeShift(const sf::SourcePartContactFixture&,double);
} // namespace crash::qualification::source_contact::force
