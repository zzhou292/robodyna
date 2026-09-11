! SPDX-License-Identifier: AGPL-3.0-or-later
! Dormant native context and fixed packets; no element or material equations.
module SOLID6Z_FORCE_MVSIZ_MOD
  implicit none
#include "mvsiz_p.inc"
end module
module SOLID6Z_FORCE_PROP_PARAM_MOD
  implicit none
#include "property_bound.inc"
end module
module SOLID6Z_FORCE_ELEMENT_MOD
  implicit none
  integer,parameter :: NIXS=14
end module
module SOLID6Z_FORCE_MESSAGE_MOD
  implicit none
  integer,parameter :: ANINFO=1
  integer :: geometry_errors=0
contains
  subroutine ANCMSG(MSGID,ANMODE,I1)
    integer,intent(in) :: MSGID,ANMODE,I1
    geometry_errors=geometry_errors+1
  end subroutine
end module
module SOLID6Z_FORCE_ALEANIM_MOD
  implicit none
  type FANI_CELL_
    logical :: IS_VORT_X_REQUESTED=.false.,IS_VORT_Y_REQUESTED=.false.,IS_VORT_Z_REQUESTED=.false.
    real(kind=8) :: VORT_X(1),VORT_Y(1),VORT_Z(1)
  end type
end module
module SOLID6Z_FORCE_ELBUFDEF_MOD
  implicit none
  type G_BUFEL_
    integer :: G_PLA=0
  end type
  type ELBUF_STRUCT_
    type(G_BUFEL_) :: GBUF
  end type
end module
module SOLID6Z_FORCE_ALE_MOD
  implicit none
  type global_context
    integer :: ICAA=0,ISFINT=0
  end type
  type ale_context
    type(global_context) :: GLOBAL
  end type
  type(ale_context) :: ALE
end module
module SOLID6Z_FORCE_OBSERVATIONS
  implicit none
  real(kind=8) :: observer_rate(3,4),observer_mode(3,4)
  real(kind=8) :: observer_shear,observer_damping,observer_first_energy,observer_final_energy
end module
module SOLID6Z_FORCE_PACKETS
  use SOLID6Z_FORCE_MVSIZ_MOD
  implicit none
  type native_geometry
    real(kind=8) :: x(MVSIZ,6,3),v(MVSIZ,6,3),frame(MVSIZ,9)
    real(kind=8) :: p(MVSIZ,6,3),jacobian(MVSIZ,9),volume(MVSIZ),length(MVSIZ)
    real(kind=8) :: world_gradient(MVSIZ,9),material_gradient(MVSIZ,9)
    real(kind=8) :: velocity_gradient(MVSIZ,9),rate(MVSIZ,6)
  end type
end module
subroutine SOLID6Z_FORCE_ARRET(code)
  integer,intent(in) :: code
  error stop 'S6Z native geometry left the admitted branch'
end subroutine
subroutine SOLID6Z_FORCE_SORTHDIR3()
  error stop 'S6Z isotropic profile called orthotropic frame'
end subroutine
subroutine SOLID6Z_FORCE_SLENA()
  error stop 'S6Z Lagrangian profile called ALE length'
end subroutine
subroutine SOLID6Z_FORCE_SLDEGE()
  error stop 'S6Z DTSDE OFF profile called alternate length'
end subroutine
subroutine SOLID6Z_FORCE_MDAMA24()
  error stop 'S6Z LAW42 profile called LAW24 damage'
end subroutine
subroutine SOLID6Z_FORCE_SZSVM()
  error stop 'S6Z LAW42 profile called plastic stabilization'
end subroutine
