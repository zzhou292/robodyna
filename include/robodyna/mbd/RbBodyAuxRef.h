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
// Robodyna adaptation: canonical body-family ownership; inherited behavior and identities retained.
// =============================================================================

#ifndef ROBODYNA_MBD_RBBODYAUXREF_H
#define ROBODYNA_MBD_RBBODYAUXREF_H

#include "robodyna/mbd/RbBody.h"

namespace robodyna::mbd {

/// Class for rigid bodies defined with respect to a non-centroidal reference frame.
///
/// An auxiliary reference frame is added to the base ChBody class offering the flexibility of placing collision
/// and visual shapes, as well as markers, relative to a potentially more convenient frame than relative to the
/// centroidal reference frame. The caller is responsible for specifying the location and orientation of the
/// centroidal frame at the body Center Of Mass (COM).
///
/// Additional information can be found in the @ref rigid_bodies manual page.
class ChApi RbBodyAuxRef : public RbBody {
  public:
    RbBodyAuxRef() : RbBody() {}
    RbBodyAuxRef(const RbBodyAuxRef& other);
    ~RbBodyAuxRef() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual RbBodyAuxRef* Clone() const override { return new RbBodyAuxRef(*this); }

    /// Set the auxiliary reference frame with respect to the absolute frame.
    /// This moves the entire body; the body COM is rigidly moved as well.
    void SetFrameRefToAbs(const chrono::ChFramed& frame);

    /// Get the auxiliary reference frame with respect to the absolute frame.
    /// Note that, in general, this is different from GetFrameCOMToAbs().
    virtual const chrono::ChFrameMoving<>& GetFrameRefToAbs() const override { return ref_to_abs; }

    /// Set the body COM frame with respect to the absolute frame.
    /// This moves the entire body; the body REF is rigidly moved as well.
    void SetFrameCOMToAbs(const chrono::ChFramed& frame);

    /// Set the COM frame with respect to the auxiliary reference frame.
    /// Note that this also moves the body absolute COM (the REF is fixed).
    /// The position of contained ChMarker objects, if any, is not changed with respect to the reference.
    void SetFrameCOMToRef(const chrono::ChFramed& frame);

    /// Get the COM frame with respect to the auxiliary reference frame.
    chrono::ChFramed GetFrameCOMToRef() const { return ref_to_com.GetInverse(); }

    /// Set the auxiliary reference frame with respect to the COM frame.
    /// Note that this does not move the body absolute COM (the COM is fixed).
    void SetFrameRefToCOM(const chrono::ChFramed& frame) { ref_to_com = frame; }

    /// Get the auxiliary reference frame with respect to the COM frame.
    const chrono::ChFramed& GetFrameRefToCOM() const { return ref_to_com; }

    /// Update all auxiliary data of the rigid body and of
    /// its children (markers, forces..)
    virtual void Update(double time, chrono::UpdateFlags update_flags) override;

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(chrono::ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(chrono::ChArchiveIn& archive_in) override;

  public:
    // These functions override the ChBodyFrame (ChFrame) functions for setting position and rotation.
    // In addition to setting the COM frame, they also must adjust ref_to_abs. Indeed, any of these
    // functions move the entire body and as such the body REF frame must also be moved.

    virtual void SetPos(const chrono::ChVector3<>& pos) override;
    virtual void SetRot(const chrono::ChMatrix33<>& R) override;
    virtual void SetRot(const chrono::ChQuaternion<>& q) override;
    virtual void SetCoordsys(const chrono::ChCoordsysd& C) override;
    virtual void SetCoordsys(const chrono::ChVector3<>& v, const chrono::ChQuaternion<>& q) override;

    virtual void SetPosDt(const chrono::ChVector3<>& p_dt) override;
    virtual void SetLinVel(const chrono::ChVector3<>& p_dt) override;
    virtual void SetRotDt(const chrono::ChQuaternion<>& q_dt) override;
    virtual void SetAngVelLocal(const chrono::ChVector3<>& w) override;
    virtual void SetAngVelParent(const chrono::ChVector3<>& w) override;
    virtual void SetCoordsysDt(const chrono::ChCoordsysd& csys_dt) override;

  private:
    chrono::ChFrameMoving<> ref_to_com;  ///< auxiliary REF location, relative to COM
    chrono::ChFrameMoving<> ref_to_abs;  ///< auxiliary REF location, relative to abs coordinates
};

}  // namespace robodyna::mbd

namespace chrono {
CH_CLASS_VERSION(::robodyna::mbd::RbBodyAuxRef, 0)
}

#endif
