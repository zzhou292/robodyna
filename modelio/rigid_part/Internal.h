#pragma once
#include "RigidPartSource.h"
#include "modelio/tied_shell/Internal.h"
namespace crash::modelio::vehicle::rigid_part::detail {
using output::Require;
using namespace assembly::reader;
void CheckOriginal(const VehicleSourcePlan&);
SourceData Declarations(const VehicleSourcePlan&,Limits);
void ReadSources(SourceData&,const source::CanonicalData&,const std::string&,Limits);
void Geometry(SourceData&,const source::CanonicalData&,Limits);
void Connections(SourceData&,Limits);
void InitializeTopology(const SourceData&,tl::fea::rigid::NodalRigidPartTopology&,Limits);
}
