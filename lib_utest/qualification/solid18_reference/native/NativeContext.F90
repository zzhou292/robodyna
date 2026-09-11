! SPDX-License-Identifier: AGPL-3.0-or-later
! Test-only context. The selected invocation is one ordinary Lagrangian cell.
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
