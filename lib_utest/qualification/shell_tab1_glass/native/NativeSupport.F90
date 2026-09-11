! Test-only fail-closed dependencies of complete native table/failure routines.
! None of these diagnostic/unselected function branches is in the supported case.
module GLASS_TAB1_MESSAGE_MOD
  implicit none
  integer,parameter :: ANINFO=1,MSGWARNING=2
contains
  subroutine ANCMSG(MSGID,MSGTYPE,ANMODE,I1,C1)
    integer,optional,intent(in) :: MSGID,MSGTYPE,ANMODE,I1
    character(*),optional,intent(in) :: C1
    error stop 'Unexpected native TAB1 diagnostic branch'
  end subroutine
end module
subroutine GLASS_TAB1_ARRET(code)
  implicit none
  integer,intent(in) :: code
  error stop 'Unexpected native TAB1 error branch'
end subroutine
double precision function GLASS_TAB1_FINTER(index,x,npf,tf,derivative)
  implicit none
  integer,intent(in) :: index,npf(*)
  double precision,intent(in) :: x,tf(*)
  double precision,intent(out) :: derivative
  error stop 'Unsupported TAB1 extra function branch'
end function
