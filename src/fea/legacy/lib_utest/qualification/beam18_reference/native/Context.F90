! SPDX-License-Identifier: AGPL-3.0-or-later
! One-parent, three-node qualification context; no material/geometry equations.
module BEAM18_REF_ELEMENT_MOD
  implicit none
  integer,parameter :: NIXP=6
end module
module BEAM18_REF_NAMES_AND_TITLES_MOD
  implicit none
  integer,parameter :: NCHARTITLE=100
end module
module BEAM18_REF_MESSAGE_MOD
  implicit none
  integer,parameter :: MSGERROR=1,MSGWARNING=2,ANINFO=1,ANINFO_BLIND_1=2,MSG_CUMU=1,MSG_PRINT=2
  integer :: errors=0,warnings=0
contains
  subroutine ANCMSG(MSGID,MSGTYPE,ANMODE,I1,C1,PRMOD)
    integer,intent(in) :: MSGID,MSGTYPE,ANMODE
    integer,intent(in),optional :: I1,PRMOD
    character(*),intent(in),optional :: C1
    if(present(PRMOD))then
      if(PRMOD==MSG_PRINT)return
    endif
    if(MSGTYPE==MSGERROR)errors=errors+1
    if(MSGTYPE==MSGWARNING)warnings=warnings+1
  end subroutine
end module
