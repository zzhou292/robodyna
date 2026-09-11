! Test-only I/O context. No material or interpolation arithmetic lives here.
module LAW90_REF_MESSAGE_MOD
  use iso_c_binding, only: c_double
  implicit none
  integer, parameter :: MSGERROR=1, MSGWARNING=2
  integer, parameter :: ANINFO_BLIND=1, ANINFO_BLIND_1=2
contains
  subroutine ANCMSG(MSGID,MSGTYPE,ANMODE,I1,C1,I2,R1)
    integer,intent(in) :: MSGID,MSGTYPE,ANMODE,I1
    character(*),intent(in) :: C1
    integer,optional,intent(in) :: I2
    real(c_double),optional,intent(in) :: R1
    if (MSGTYPE == MSGERROR) error stop 'native LAW90 setup rejected'
    if (MSGID /= 865) error stop 'unexpected native LAW90 warning'
  end subroutine
end module

module LAW90_REF_TABLE_MOD
  implicit none
end module

module LAW90_REF_NAMES_AND_TITLES_MOD
  implicit none
  integer,parameter :: NCHARTITLE=100
end module
