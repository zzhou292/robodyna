! SPDX-License-Identifier: AGPL-3.0-or-later
! Test-only storage/context, with no constitutive or geometry equations.
module LAW90_ENGINE_MESSAGE_MOD
  implicit none
  integer, parameter :: ANINFO=1
  integer :: geometry_errors=0
contains
  subroutine ANCMSG(MSGID,ANMODE,I1)
    integer,intent(in) :: MSGID,ANMODE,I1
    geometry_errors=geometry_errors+1
  end subroutine
end module

subroutine LAW90_ENGINE_ARRET(code)
  integer,intent(in) :: code
  error stop 'Native solid18 left the qualified positive-Jacobian branch'
end subroutine

module LAW90_ENGINE_PACKETS
  use LAW90_ENGINE_CONSTANT_MOD
  implicit none
#include "mvsiz_p.inc"
  ! Arrays passed to complete native leaves retain their actual leading bound.
  type native_geometry
    real(kind=8) :: x(MVSIZ,8,3),v(MVSIZ,8,3),frame(MVSIZ,9)
    real(kind=8) :: center_j(MVSIZ,9),higher(MVSIZ,4,3)
    real(kind=8) :: jacobian(MVSIZ,8,9),inverse(MVSIZ,8,9)
    real(kind=8) :: p(MVSIZ,8,8,3),shear(MVSIZ,8,8,6),cross(MVSIZ,8,8,6)
    real(kind=8) :: center_p(MVSIZ,4,3),volume(MVSIZ,8),center_volume(MVSIZ),smax(MVSIZ)
  end type
end module

subroutine LAW90_ENGINE_JACOB_J33()
  error stop 'Excluded LAW90 ICP0 branch reached JACOB_J33'
end subroutine
