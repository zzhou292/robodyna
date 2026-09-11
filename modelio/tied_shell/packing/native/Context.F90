module packing_context
  implicit none
  integer :: numnod=0, ipri=0, n2d=0, iout=6
end module
module names_and_titles_mod
  implicit none
  integer, parameter :: nchartitle=128
end module
module format_mod
  implicit none
  character(len=*), parameter :: fmw_4i='(4I12)', fmw_10i='(10I12)'
end module
module message_mod
  implicit none
  integer, parameter :: msgwarning=1, aninfo_blind_2=2
contains
  subroutine ancmsg(msgid,msgtype,anmode,i1,c1,i2,i3,i4,i5,i6,i7,i8,i9)
    integer, optional :: msgid,msgtype,anmode,i1,i2,i3,i4,i5,i6,i7,i8,i9
    character(len=*), optional :: c1
    error stop 'Unexpected native packing warning/error'
  end subroutine
end module
module my_alloc_mod
  implicit none
contains
  subroutine my_alloc(values,count,name)
    integer, allocatable :: values(:)
    integer :: count
    character(len=*) :: name
    allocate(values(count))
  end subroutine
  subroutine my_dealloc(values)
    integer, allocatable :: values(:)
    deallocate(values)
  end subroutine
end module
module setdef_mod
  implicit none
  type set_
    integer :: nb_sh4n=0, nb_sh3n=0
    integer, allocatable :: sh4n(:), sh3n(:)
  end type
end module
