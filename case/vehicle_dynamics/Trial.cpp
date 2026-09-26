#include "Storage.h"
#include "Reports.h"
#include "lib_src/solvers/NodalCinRuntime.h"

namespace crash::cases::vehicle_dynamics {
namespace {
template<StepStage S,class Call> void Timed(StepTimer& timer,Call&& call,const char* name) {
    timer.Measure<S>([&] {
        detail::Require(call(),name);
        return true;
    });
}
}

void VehiclePhysicalDynamics::Storage::Prepare() {
    auto& s=state();
    output::Require(tl::fea::trial_identity::SameStamp(stamp,s.owner.accepted()),"Physical owner changed outside its case");
    Timed<StepStage::AcceptedWitness>(timer,[&] {return activity->CaptureAcceptedPhysical(s.owner,s.publication,{&s.qeph,&s.t3,&s.qbat,&s.type25});},
        "Accepted physical CIN witness activity");
    candidate()={};
    candidate().base=stamp;
    tl::fea::NodalAssemblyView assembly;
    Timed<StepStage::BeginTrial>(timer,[&] {
        return s.owner.BeginTrial(&token,&assembly);
    },"Begin physical assembly");
    Timed<StepStage::AssembleQeph>(timer,[&] {
        return s.qeph.AssembleMappedAccepted(s.owner,token,assembly);
    },"QEPH accepted assembly");
    Timed<StepStage::AssembleT3>(timer,[&] {
        return s.t3.AssembleMappedAccepted(s.owner,token,assembly);
    },"T3 accepted assembly");
    Timed<StepStage::AssembleQbat>(timer,[&] {
        return s.qbat.AssembleMappedAccepted(s.owner,token,assembly);
    },"QBAT accepted assembly");
    Timed<StepStage::AssembleType25>(timer,[&] {
        return s.type25.AssembleMappedAccepted(s.owner,token,assembly);
    },"Weld accepted assembly");
    Timed<StepStage::AssembleType13>(timer,[&] {
        return s.type13.AssembleMappedAccepted(s.owner,token,assembly);
    },"Beam accepted assembly");
    Timed<StepStage::AssembleSolids>(timer,[&] {
        return s.solids.AssembleAccepted(s.owner,token,assembly);
    },"Solid accepted assembly");
    if(s.beam18) {
        Timed<StepStage::AssembleBeam18>(timer,[&] {
            return s.beam18->AssembleAccepted(s.owner,token,assembly);
        },"Structural beam accepted assembly");
    }
    if(s.type45) {
        Timed<StepStage::AssembleType45>(timer,[&] {
            return s.type45->AssembleAccepted(s.owner,token,assembly);
        },"Joint accepted assembly");
    }
    if(wall) {
        timer.Measure<StepStage::AssembleWall>([&] {
            wall->Assemble(s.owner,token,assembly,candidate().wall);
            return true;
        });
    }
    if(self_contact) {
        timer.Measure<StepStage::AssembleSelfContact>([&] {
            self_contact->Assemble(
                s.owner,token,assembly,candidate().self_contact);
            return true;
        });
    }
    Timed<StepStage::UploadWitness>(timer,[&] {
        return activity->UploadAttempt(s.owner,token);
    },"Actual accepted CIN witness upload");
    Timed<StepStage::SealAssembly>(timer,[&] {
        return s.owner.SealAssembly(token);
    },"Seal complete physical assembly");
    timer.Measure<StepStage::AdvanceCin>([&] {
        const auto advanced=tl::fea::AdvanceStaggeredCin(s.owner,token,
            {stamp.owner_id,stamp.epoch,assembly.attempt,config.startup.qualification_id,
             config.startup.reserved_step_s,config.maximum_rotation_increment,true,config.structural});
        if(advanced.status==tl::fea::NodalStatus::StepTooLarge) {
            throw StepSizeError(advanced.stable_dt,advanced.node);
        }
        detail::Require(advanced,"Advance physical CIN/rigid owner");
        candidate().structural_step_limit=advanced.stable_dt;
        if(config.structural.capture_limiter)
            detail::Require(s.owner.CopyPreparedCinStructuralLimit(token,
                &candidate().structural_limiter),"Copy actual structural limiter");
        return true;
    });
    Timed<StepStage::BorrowPrepared>(timer,[&] {
        return s.owner.BorrowPrepared(token,&prepared);
    },"Borrow complete prepared physical owner");
}
void VehiclePhysicalDynamics::Storage::Evaluate() {
    auto& s=state();
    tl::fea::ShellPhysicalDiagnostics d;
    Timed<StepStage::EvaluateQeph>(timer,[&] {
        return s.qeph.EvaluateCandidate(s.owner,token,prepared,&d.qeph);
    },"QEPH candidate");
    Timed<StepStage::EvaluateT3>(timer,[&] {
        return s.t3.EvaluateCandidate(s.owner,token,prepared,&d.t3);
    },"T3 candidate");
    Timed<StepStage::EvaluateQbat>(timer,[&] {
        return s.qbat.EvaluateCandidate(s.owner,token,prepared,&d.qbat);
    },"QBAT candidate");
    Timed<StepStage::EvaluateType25>(timer,[&] {
        return s.type25.EvaluateCandidate(s.owner,token,prepared,&d.type25);
    },"Weld candidate");
    Timed<StepStage::EvaluateType13>(timer,[&] {
        return s.type13.EvaluateCandidate(s.owner,token,prepared,&d.type13);
    },"Beam candidate");
    Timed<StepStage::EvaluateSolids>(timer,[&] {
        return s.solids.EvaluateCandidate(s.owner,token,prepared,&d.solids);
    },"Solid candidate");
    if(s.beam18) {
        Timed<StepStage::EvaluateBeam18>(timer,[&] {
            return s.beam18->EvaluateCandidate(s.owner,token,prepared,&d.beam18);
        },"Structural beam candidate");
    }
    if(s.type45) {
        Timed<StepStage::EvaluateType45>(timer,[&] {
            return s.type45->EvaluateCandidate(s.owner,token,prepared,&d.type45);
        },"Joint candidate");
    }
    timer.Measure<StepStage::PreparePublication>([&] {
        detail::Require(s.publication.PreparePhysical(s.owner,token,
            {&d.qeph,&d.t3,&d.qbat,&d.type25,&d.type13,&d.solids,s.type45 ? &d.type45 : nullptr,s.beam18 ? &d.beam18 : nullptr},
            &candidate().mechanics),"Prepare complete publication");
        return true;
    });
    if(wall) {
        timer.Measure<StepStage::EvaluateWall>([&] {
            wall->Evaluate(s.owner,token,prepared,candidate().mechanics,candidate().wall);
            return true;
        });
    }
    if(self_contact) {
        timer.Measure<StepStage::EvaluateSelfContact>([&] {
            self_contact->SealCandidate(
                s.owner,token,candidate().mechanics,prepared,
                candidate().self_contact);
            return true;
        });
    }
}
void VehiclePhysicalDynamics::Storage::Capture() {
    tl::fea::NodalUniformMotionObservation observed;
    // Preserve complete snapshot validation and its prepared phase; only the
    // readback payload changes. Full archive fields use the sampled capture.
    Timed<StepStage::CaptureFields>(timer,[&] {
        return motion.ObservePrepared(state().owner,token,
            {vehicle_runtime::InitialSpeedMps,0,0},&observed);
    },"Prepared physical motion observation");
    output::Require(tl::fea::trial_identity::SamePrepared(prepared,observed.prepared),
        "Prepared physical observation identity differs");
    auto& out=candidate();
    out.proposed_time=observed.prepared.proposed_time;
    timer.Measure<StepStage::ObserveMotion>([&] {
        const auto& value=observed.motion;
        out.uniform_motion={value.nodes,value.maximum_position_error,value.maximum_velocity_error,
            value.maximum_orientation_error,value.maximum_spin};
        return true;
    });
}
} // namespace crash::cases::vehicle_dynamics
