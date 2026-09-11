#pragma once
#include "VehicleMassInput.h"
#include "lib_utest/qualification/nodal_vehicle/VehicleOwnerFixture.h"
#include "lib_utest/qualification/type25/EvaluationValues.h"
#include "lib_src/elements/type25/Type25BatchIdentity.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

namespace type25_batch_test {
namespace vehicle=fe::vehicle_test;
inline constexpr double VehicleStep=0x1p-20;
struct VehicleRig {
  VehicleMassInput input;
  vehicle::Initial initial;
  fe::FENodalState owner;
  spring::Batch batch;
  VehicleRig(std::size_t nodes=524288,std::size_t connections=4096):input(nodes,connections),initial(nodes) {}
  bool Initialize();
  bool Prepare(fe::NodalTrialToken&,fe::NodalPreparedView&);
  bool Read(std::vector<spring::Evaluation>&,spring::BatchDiagnostics&);
  void Discard() {owner.Discard();batch.DiscardTrial();}
};
void ExactVehicle(const spring::Evaluation&,const spring::Evaluation&);
void VehicleAgainstHost(VehicleRig&,const fe::NodalTrialToken&,const fe::NodalPreparedView&,
                        const std::vector<spring::Evaluation>&,const std::vector<spring::Evaluation>&);
} // namespace type25_batch_test
