! SPDX-License-Identifier: AGPL-3.0-or-later
module contribution_messages
  implicit none
  integer,parameter::msgerror=1,aninfo_blind_1=1
  integer::length_diagnostics=0
contains
  subroutine ancmsg(msgid,msgtype,anmode,i1,c1,i2)
    integer,intent(in)::msgid,msgtype,anmode,i1,i2
    character(*),intent(in)::c1
    if(msgid/=328.or.msgtype/=msgerror)error stop 'Unselected RINIT3 length diagnostic'
    length_diagnostics=length_diagnostics+1
  end subroutine
end module
