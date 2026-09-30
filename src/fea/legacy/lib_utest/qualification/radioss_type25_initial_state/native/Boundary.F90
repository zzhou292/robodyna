module initial_state_names
  implicit none
  integer,parameter::NCHARTITLE=128
end module
module initial_state_messages
  use iso_c_binding
  implicit none
  integer,parameter::MSGWARNING=0,ANINFO_BLIND_1=0,MSG_CUMU=0
contains
  subroutine ANCMSG(MSGID,MSGTYPE,ANMODE,I1,I2,I3,I4,I5,R1,PRMOD)
    integer,intent(in)::MSGID,MSGTYPE,ANMODE,I1,I2,I3,I4,I5,PRMOD
    real(c_double),intent(in)::R1
    ! IPRI1 excludes the sole warning output in the unchanged PWR3 body.
    error stop 'Unexpected initial-history diagnostic'
  end subroutine
end module
