// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in LICENSES/Chrono-BSD-3-Clause.txt at the Robodyna root and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// Robodyna adaptation: canonical body-family implementation; legacy archive tags and equations retained.
// =============================================================================

#include "robodyna/mbd/RbBodyAuxRef.h"
#include "chrono/physics/ChBodyAuxRef.h"

namespace chrono {
// Keep the exact inherited factory and casting identities through reverse aliases.
CH_FACTORY_REGISTER(ChBodyAuxRef)
CH_UPCASTING(ChBodyAuxRef, ChBody)
}

namespace robodyna::mbd {
// Implementation-only lookup keeps inherited helper resolution and arithmetic unchanged.
using namespace ::chrono;

// The legacy registration block above preserves dynamic creation and persistence.
RbBodyAuxRef::RbBodyAuxRef(const RbBodyAuxRef& other) : RbBody(other) {
    ref_to_com = other.ref_to_com;
    ref_to_abs = other.ref_to_abs;
}

void RbBodyAuxRef::SetFrameCOMToRef(const ChFramed& frame) {
    ChFrameMoving<> old_com_to_abs = *this;

    ref_to_abs = TransformLocalToParent(ref_to_com);

    ChFrameMoving<> new_com_to_abs = ref_to_abs.TransformLocalToParent(ChFrameMoving<>(frame));

    RbBody::SetCoordsys(new_com_to_abs.GetCoordsys());
    RbBody::SetCoordsysDt(new_com_to_abs.GetCoordsysDt());
    RbBody::SetCoordsysDt2(new_com_to_abs.GetCoordsysDt2());

    ref_to_com = frame.GetInverse();
    ref_to_abs = TransformLocalToParent(ref_to_com);

    // Restore marker/forces positions, keeping unchanged respect to aux ref.
    ChFrameMoving<> com_oldnew = old_com_to_abs >> new_com_to_abs.GetInverse();

    for (auto& marker : marklist) {
        marker->ConcatenatePreTransformation(com_oldnew);
        marker->Update(GetChTime(), UpdateFlags::UPDATE_ALL);
    }
}

void RbBodyAuxRef::SetFrameRefToAbs(const ChFramed& frame) {
    auto cog_to_abs = frame.TransformLocalToParent(ref_to_com.GetInverse());
    RbBody::SetCoordsys(cog_to_abs.GetCoordsys());
    ref_to_abs = frame;
}

void RbBodyAuxRef::SetFrameCOMToAbs(const ChFramed& frame) {
    RbBody::SetCoordsys(frame.GetCoordsys());
    ref_to_abs = frame.TransformLocalToParent(ref_to_com);
}

void RbBodyAuxRef::Update(double time, UpdateFlags update_flags) {
    // update parent class
    RbBody::Update(time, update_flags);

    // update own data
    ref_to_abs = TransformLocalToParent(ref_to_com);
}

// -----------------------------------------------------------------------------

void RbBodyAuxRef::SetPos(const ChVector3<>& pos) {
    SetFrameCOMToAbs(ChFramed(pos, GetRot()));
}

void RbBodyAuxRef::SetRot(const ChMatrix33<>& R) {
    SetFrameCOMToAbs(ChFramed(GetPos(), R));
}

void RbBodyAuxRef::SetRot(const ChQuaternion<>& q) {
    SetFrameCOMToAbs(ChFramed(GetPos(), q));
}

void RbBodyAuxRef::SetCoordsys(const ChCoordsysd& C) {
    SetFrameCOMToAbs(ChFramed(C));
}

void RbBodyAuxRef::SetCoordsys(const ChVector3<>& v, const ChQuaternion<>& q) {
    SetFrameCOMToAbs(ChFramed(v, q));
}

void RbBodyAuxRef::SetPosDt(const ChVector3<>& p_dt) {
    RbBody::SetPosDt(p_dt);
    ref_to_abs.SetPosDt(GetPosDt());
    ref_to_abs.SetRotDt(GetRotDt());
}

void RbBodyAuxRef::SetLinVel(const ChVector3<>& p_dt) {
    SetPosDt(p_dt);
}

void RbBodyAuxRef::SetRotDt(const ChQuaternion<>& q_dt) {
    RbBody::SetRotDt(q_dt);
    ref_to_abs.SetPosDt(GetPosDt());
    ref_to_abs.SetRotDt(GetRotDt());
}

void RbBodyAuxRef::SetAngVelLocal(const ChVector3<>& w) {
    RbBody::SetAngVelLocal(w);
    ref_to_abs.SetPosDt(GetPosDt());
    ref_to_abs.SetRotDt(GetRotDt());
}

void RbBodyAuxRef::SetAngVelParent(const ChVector3<>& w) {
    RbBody::SetAngVelParent(w);
    ref_to_abs.SetPosDt(GetPosDt());
    ref_to_abs.SetRotDt(GetRotDt());
}

void RbBodyAuxRef::SetCoordsysDt(const ChCoordsysd& csys_dt) {
    RbBody::SetCoordsysDt(csys_dt);
    ref_to_abs.SetPosDt(GetPosDt());
    ref_to_abs.SetRotDt(GetRotDt());
}

// -----------------------------------------------------------------------------

void RbBodyAuxRef::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<RbBodyAuxRef>();

    // serialize parent class
    RbBody::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(ref_to_com);
    archive_out << CHNVP(ref_to_abs);
}

/// Method to allow de serialization of transient data from archives.
void RbBodyAuxRef::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<RbBodyAuxRef>();

    // deserialize parent class
    RbBody::ArchiveIn(archive_in);

    // stream in all member data:
    archive_in >> CHNVP(ref_to_com);
    archive_in >> CHNVP(ref_to_abs);
}

}  // namespace robodyna::mbd
