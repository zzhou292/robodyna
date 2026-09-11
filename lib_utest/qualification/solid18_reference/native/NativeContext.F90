! SPDX-License-Identifier: AGPL-3.0-or-later
! Test-only context. The selected invocation is one ordinary Lagrangian cell.
module SOLID18_POINT_CAPTURE
  use iso_c_binding
  implicit none
  real(c_double) :: point_values(33,8)
end module

module SOLID18_REF_MESSAGE_MOD
  implicit none
  integer, parameter :: MSGERROR=1, ANINFO=1
  integer :: geometry_errors=0
contains
  subroutine ANCMSG(MSGID,MSGTYPE,ANMODE,I1)
    integer, intent(in) :: MSGID,MSGTYPE,ANMODE,I1
    geometry_errors=geometry_errors+1
  end subroutine
end module

! SMASS3B's complete dormant JALE branch only names ALE%GRID%NWALE.
! Supply that context, set JALE=JEUL=0, and retain the original branch untouched.
module SOLID18_REF_ALE_MOD
  implicit none
  type GridContext
    integer :: NWALE=0
  end type
  type AleContext
    type(GridContext) :: GRID
  end type
  type(AleContext) :: ALE
end module
