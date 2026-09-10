#pragma once
// Qualification runtime values, not a production batch or commit framework.
#include "WallResponseData.h"
#include "../wall_coupled/ScreenedWallFixture.h"

namespace tl::qualification::qeph::wall_response::runtime_detail {
namespace c=qeph_wall_test;
struct Participants {
  const Config& config;
  const Model& model;
  c::WallRig wall;
  c::ScreenedVelocity velocity{};
  c::LedgerScales scales;
  c::NativeSequence native;
  c::PortResults cache{};
  c::LedgerEvidence source;
  c::ContactEvidence contact;
  c::sc::NodalWallDeviceResults published;
  double regular_maximum=0,hourglass_maximum=0,impulse=0,impulse_error=0;
  bool initialized=false;
  Participants(const Config&,const Model&);
  Participants(const Participants&)=delete;
  Participants& operator=(const Participants&)=delete;
  void Discard() { wall.Discard(); }
};
struct StepStage {
  c::Snapshot base;
  c::Trial trial;
  c::Staged components;
  c::NativeProposal native;
  c::LedgerEvidence source;
  c::ContactEvidence contact;
  double regular_maximum=0,hourglass_maximum=0;
  EndpointInput endpoint;
  Sample sample;
  Summary summary;
  bool physical_ready=false,observation_ready=false,save_sample=false;
};
// Initialization publishes the known epoch-zero sample only after authenticated
// initial assembly, native agreement and both owning observer checks succeed.
bool Initialize(Run&,Participants&,std::string& error);
// No native accepted state, accepted observer, cache or event changes here.
bool PrepareStep(Run&,Participants&,StepStage&,std::string& error,bool reverse_order=false);
// Clears only the private ready flag on failure; prior staged Sample/Summary and
// all accepted output remain unchanged. This is also the real rollback-test seam.
bool ObserveStep(const Run&,StepStage&,std::string& error);
// All preflight precedes one existing owner/history commit. Afterwards only
// fixed value copies and a prospectively reserved Sample append occur.
bool PublishStep(Run&,Participants&,const StepStage&,std::string& error);
} // namespace tl::qualification::qeph::wall_response::runtime_detail
