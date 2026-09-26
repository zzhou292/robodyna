module solid_support_element
  implicit none
  integer,parameter::nixs=11
end module
module solid_support_observation
  implicit none
  integer::observed_matches=0,observed_effective=0,warning_calls=0
end module
module solid_support_message
  use solid_support_observation
  implicit none
  integer,parameter::MSGWARNING=1,ANINFO_BLIND_1=1,MSG_CUMU=1
contains
  subroutine ANCMSG(MSGID,MSGTYPE,ANMODE,I1,I2,PRMOD)
    integer,intent(in)::MSGID
    integer,intent(in),optional::MSGTYPE,ANMODE,I1,I2,PRMOD
    ! Diagnostics only. No source branch depends on this stub's values.
    warning_calls=warning_calls+1
  end subroutine
end module
