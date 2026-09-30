! Test-only allocation/property/message context. Numerical leaves are complete
! pinned originals; no force/frame/stiffness supplied by the production port.
module T45_NAMES_AND_TITLES_MOD
  implicit none
  integer, parameter :: NCHARTITLE=100
end module
module T45_ELEMENT_MOD
  implicit none
  integer, parameter :: NIXR=6
end module
module T45_SENSOR_MOD
  implicit none
  type SENSOR_STR_
    integer :: SENS_ID=0
    real(8) :: TSTART=0
  end type
end module
module T45_MESSAGE_MOD
  implicit none
  integer, parameter :: MSGERROR=1, MSGWARNING=2, ANINFO_BLIND_1=1, ANINFO_BLIND_2=2
  integer :: errors=0, warnings=0
contains
  subroutine ANCMSG(MSGID,MSGTYPE,ANMODE,I1,I2,I3,I4,C1,C2,R1,R2)
    integer, intent(in) :: MSGID,MSGTYPE,ANMODE
    integer, optional, intent(in) :: I1,I2,I3,I4
    character(len=*), optional, intent(in) :: C1,C2
    real(8), optional, intent(in) :: R1,R2
    if(MSGTYPE==MSGERROR) errors=errors+1
    if(MSGTYPE==MSGWARNING) warnings=warnings+1
  end subroutine
end module
module T45_PROPERTY
  implicit none
  real(8) :: GEO(64)=0
contains
  subroutine SetProperty(kind,values)
    integer, intent(in) :: kind
    real(8), intent(in) :: values(14) ! ScF, native resolved Cr, six free K, six free C
    GEO=0
    GEO(1)=kind
    GEO(4:9)=values(3:8)
    GEO(11)=values(1)
    GEO(12)=values(2)
    GEO(21:26)=values(9:14)
    GEO(27:28)=1
    select case(kind)
    case(1)
      GEO(15:17)=values(2)
    case(2)
      GEO(15:17)=values(2)
      GEO(19:20)=values(2)
    case(3)
      GEO(16:17)=values(2)
      GEO(19:20)=values(2)
    end select
  end subroutine
end module
real(8) function T45_GET_U_GEO(slot,property)
  use T45_PROPERTY, only: GEO
  implicit none
  integer, intent(in) :: slot,property
  if(property/=1.or.slot<1.or.slot>64) error stop 'TYPE45 property slot'
  T45_GET_U_GEO=GEO(slot)
end function
integer function T45_GET_U_PNU(slot,property,kind)
  implicit none
  integer, intent(in) :: slot,property,kind
  if(property/=1.or.kind/=29.or.slot<1.or.slot>18) error stop 'TYPE45 property function slot'
  T45_GET_U_PNU=0
end function
real(8) function T45_GET_U_FUNC(function_id,x,derivative)
  implicit none
  integer, intent(in) :: function_id
  real(8), intent(in) :: x
  real(8), intent(out) :: derivative
  error stop 'Unsupported TYPE45 function called'
end function
real(8) function T45_GET_U_FUNC_DERI(function_id)
  implicit none
  integer, intent(in) :: function_id
  error stop 'Unsupported TYPE45 function derivative called'
end function
integer function T45_GET_U_SKEW(skew,n1,n2,n3,values)
  implicit none
  integer, intent(in) :: skew
  integer, intent(out) :: n1,n2,n3
  real(8), intent(out) :: values(9)
  error stop 'Unsupported TYPE45 external skew called'
end function
subroutine T45_FRETITL2(title,encoded,n)
  implicit none
  integer, intent(in) :: n,encoded(n)
  character(len=*), intent(out) :: title
  title='explicit supplied TYPE45 context'
end subroutine
