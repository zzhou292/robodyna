! Exact VELROT_EXPLICIT arithmetic in independently authored double context.
! OpenRadioss Copyright (C) 2026 Siemens; AGPL-3.0-or-later.
module rigid_native_finite
  use iso_c_binding, only: c_double
  implicit none
contains
  subroutine native_finite_velocity(vr,lsm,vs,dt) bind(C,name="nodal_rigid_native_finite_velocity")
    real(c_double),intent(in)::vr(3),lsm(3),dt
    real(c_double),intent(out)::vs(3)
    call native_finite_velocity_threshold(vr,lsm,vs,dt,1.d-8)
  end subroutine
  subroutine native_finite_velocity_threshold(vr,lsm,vs,dt,em08)
    real(c_double),intent(in)::vr(3),lsm(3),dt,em08
    real(c_double),intent(out)::vs(3)
    real(c_double)::ang2,rz(3,3),localz(3),localx(3),trans(3,3),localy(3), &
      lsmlocal(3),lsmltr(3),lsmgtr(3),norm,vs2,angelv,vrm(3)
    real(c_double),parameter::one=1.d0,zero=0.d0,em06=1.d-6,em20=1.d-20
#include "extracted/FiniteVelocity.inc"
  end subroutine
  subroutine cross_product(x,y,z)
    real(c_double),intent(in)::x(3),y(3)
    real(c_double),intent(out)::z(3)
#include "extracted/FiniteCross.inc"
  end subroutine
end module
