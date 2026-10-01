// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in LICENSES/Chrono-BSD-3-Clause.txt at the Robodyna root and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// Robodyna adaptation: canonical inertia ownership and names; numerical operations unchanged.
// =============================================================================

#ifndef ROBODYNA_MECHANICS_MASS_PROPERTIES_H
#define ROBODYNA_MECHANICS_MASS_PROPERTIES_H

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChMatrix33.h"
#include "chrono/core/ChFrame.h"

namespace robodyna::mechanics {

/// Definition of mass properties of a solid object.
struct RbMassProperties {
    double mass;
    chrono::ChVector3d com;
    chrono::ChMatrix33d inertia;
};

/// Utility functions for computations with inertia tensors.
class ChApi RbInertiaUtils {
  public:
    /// Compute inertia of a cluster.
    /// Given a cluster of bodies, each with local inertia tensor, and position and rotation respect to absolute coordinates,
    /// compute the total mass, the barycenter and the inertia tensor of the cluster.
    /// Notes:
    /// - the resulting total inertia tensor is expressed in absolute coordinates
    /// - we assume that body masses are not overlapping
    /// - we assume that all local inertias are expressed in body local coordinates
    /// - we assume that all local inertias are expressed relative to body barycenter
    static void InertiaFromCluster(const std::vector<chrono::ChVector3d>& positions,
                                   const std::vector<chrono::ChMatrix33<> >& rotations,
                                   const std::vector<chrono::ChMatrix33<> >& Jlocal,
                                   const std::vector<double>& masses,
                                   chrono::ChMatrix33<>& totJ,
                                   double& totmass,
                                   chrono::ChVector3d& barycenter);

    /// Rotate an inertia tensor, given a rotation matrix R.
    static void RotateInertia(const chrono::ChMatrix33<> inertiaIn, const chrono::ChMatrix33<> R, chrono::ChMatrix33<>& inertiaOut);

    /// Rotate an inertia tensor, given a rotation matrix R.
    static chrono::ChMatrix33<> RotateInertia(const chrono::ChMatrix33<> inertiaIn, const chrono::ChMatrix33<> R);

    /// Translate an inertia tensor to a non-barycentric reference, given a displacement 'dist', using the Huygens-Steiner parallel axis theorem.
    static void TranslateInertia(const chrono::ChMatrix33<> inertiaIn, const chrono::ChVector3d dist, const double mass, chrono::ChMatrix33<>& inertiaOut);

    /// Translate an inertia tensor to a non-barycentric reference, given a displacement 'dist', using the Huygens-Steiner parallel axis theorem.
    static chrono::ChMatrix33<> TranslateInertia(const chrono::ChMatrix33<> inertiaIn, const chrono::ChVector3d dist, const double mass);

    /// Compute principal moments of inertia and the principal axes.
    /// The principal moments of inertia are sorted in ascending order.
    /// The principal axes are returned as the columns of a 3x3 rotation matrix.
    static void PrincipalInertia(const chrono::ChMatrix33<>& inertia, chrono::ChVector3d& principal_inertia, chrono::ChMatrix33<>& principal_axes);
};

/// Utility class for calculating inertia properties of a composite body.
/// A composite body is a collection of sub-components, each specified through its mass, a centroidal frame, and its inertia tensor
/// (w.r.t. the centroidal frame). New sub-components can be included in the composite body and optionally marked as "voids" (holes),
/// by specifying:
/// - the mass of the sub-component
/// - a sub-component centroidal frame, relative to the composite reference frame;
///   the location of this frame represents the sub-component COM location
/// - the sub-component inertia tensor w.r.t. its centroidal frame
class ChApi CompositeInertia {
  public:
    CompositeInertia();

    /// Get the composite mass.
    double GetMass() const { return m_mass; }

    /// Get the location of the COM.
    const chrono::ChVector3d& GetCOM() const { return m_com; }

    /// Get the inertia tensor w.r.t. a centroidal frame.
    /// The return 3x3 symmetric matrix represents the inertia tensor with respect to a centroidal frame of the composite,
    /// aligned with the axes of the reference frame.
    chrono::ChMatrix33<> GetInertia() const;

    /// Get the inertia tensor w.r.t. the reference frame.
    const chrono::ChMatrix33<>& GetInertiaReference() const { return m_inertia; }

    /// Include sub-component inertia properties.
    /// Update the inertia properties of the composite object with the specified mass and inertia of a sub-component.
    void AddComponent(const chrono::ChFrame<>& frame,       ///< centroidal frame of sub-component (relative to the composite reference frame)
                      double mass,                  ///< mass of sub-component
                      const chrono::ChMatrix33<>& inertia,  ///< sub-component inertia tensor w.r.t. its centroidal frame
                      bool is_void = false          ///< indicate if sub-component represents a material void
    );

    /// Utility function for calculating an inertia shift matrix from a given vector.
    /// This matrix is used when applying the parallel axis theorem.
    static chrono::ChMatrix33<> InertiaShiftMatrix(const chrono::ChVector3d& v);

  private:
    chrono::ChMatrix33<> m_inertia;  ///< inertia tensor w.r.t reference frame
    chrono::ChVector3d m_com;        ///< location of COM (relative to reference frame)
    double m_mass;           ///< mass of composite body
};

}  // namespace robodyna::mechanics

#endif
