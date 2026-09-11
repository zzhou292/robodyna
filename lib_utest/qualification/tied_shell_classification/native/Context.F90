! Test-only allocation, messages and the three accessed INTBUF components.
! Complete original ITAGSL2/KINSET/KININI retain their numerical statements.
module names_and_titles_mod
  implicit none
  integer, parameter :: nchartitle=100, ncharline=256
end module
module message_mod
  implicit none
  integer, parameter :: msgwarning=1, aninfo_blind_1=1, aninfo_blind_2=2
  integer, parameter :: msg_cumu=1, msg_print=2
  integer :: penalty_count=0
contains
  subroutine ancmsg(msgid,msgtype,anmode,i1,i2,c1,c2,prmod)
    integer, intent(in) :: msgid,msgtype,anmode
    integer, intent(in), optional :: i1,i2,prmod
    character(len=*), intent(in), optional :: c1,c2
    if(msgid==1179.and.present(prmod)) then
      if(prmod==msg_cumu) penalty_count=penalty_count+1
    endif
  end subroutine
end module
module intbufdef_mod
  implicit none
  type intbuf_struct_
    integer, allocatable :: nsv(:),msr(:),irupt(:)
  end type
end module
module my_alloc_mod
  implicit none
  interface my_alloc
    module procedure allocate_integer
  end interface
  interface my_dealloc
    module procedure deallocate_integer
  end interface
contains
  subroutine allocate_integer(p,n,label)
    integer, allocatable, intent(inout) :: p(:)
    integer, intent(in) :: n
    character(len=*), intent(in) :: label
    allocate(p(n))
  end subroutine
  subroutine deallocate_integer(p)
    integer, allocatable, intent(inout) :: p(:)
    deallocate(p)
  end subroutine
end module
subroutine fretitl2(title,encoded,n)
  implicit none
  integer, intent(in) :: n,encoded(n)
  character(len=*), intent(out) :: title
  title='supplied classification context'
end subroutine
