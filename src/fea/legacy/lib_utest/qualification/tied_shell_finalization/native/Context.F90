! Test-only context for complete I2TID3 and its selected native registration.
module names_and_titles_mod
  implicit none
  integer,parameter :: nchartitle=100,ncharline=256
end module
module message_mod
  implicit none
  integer,parameter :: msgwarning=1,msgerror=2,aninfo_blind_1=1,aninfo_blind_2=2
  integer,parameter :: msg_cumu=1,msg_print=2
  integer,allocatable :: event_integer(:,:)
  double precision,allocatable :: event_real(:,:)
  integer :: event_count=0
  logical :: event_overflow=.false.
contains
  subroutine ancmsg(msgid,msgtype,anmode,i1,i2,i3,i4,i5,i6,r1,r2,r3,c1,c2,prmod)
    integer,intent(in) :: msgid,msgtype,anmode
    integer,intent(in),optional :: i1,i2,i3,i4,i5,i6,prmod
    double precision,intent(in),optional :: r1,r2,r3
    character(*),intent(in),optional :: c1,c2
    integer :: action
    action=3
    if(present(prmod)) action=prmod
    event_count=event_count+1
    if(event_count>size(event_integer,2)) then
      event_overflow=.true.
      return
    endif
    event_integer(:,event_count)=0
    event_real(:,event_count)=0
    event_integer(1:3,event_count)=[msgid,action,msgtype]
    if(present(i1)) event_integer(4,event_count)=i1
    if(present(i2)) event_integer(5,event_count)=i2
    if(present(i3)) event_integer(6,event_count)=i3
    if(present(i4)) event_integer(7,event_count)=i4
    if(present(i5)) event_integer(8,event_count)=i5
    if(present(i6)) event_integer(9,event_count)=i6
    if(present(r1)) event_real(1,event_count)=r1
    if(present(r2)) event_real(2,event_count)=r2
    if(present(r3)) event_real(3,event_count)=r3
  end subroutine
end module
module element_mod
  implicit none
  ! Only unused ILEV27 type-census dummy extents consume these constants.
  integer,parameter :: nixs=10,nixc=6,nixtg=5
end module
module nod2el_mod
  implicit none
  integer,allocatable :: knod2els(:),knod2elc(:),knod2eltg(:)
  integer,allocatable :: nod2els(:),nod2elc(:),nod2eltg(:)
end module
module intbufdef_mod
  implicit none
  type intbuf_struct_
    integer,allocatable :: nsv(:),msr(:),irtlm(:),irupt(:),msegtyp2(:)
    double precision,allocatable :: csts(:),csts_bis(:),dpara(:),nmas(:),smas(:),siner(:)
    double precision,allocatable :: areas2(:),uvar(:),xm0(:),spenalty(:),stfr_penalty(:)
    double precision,allocatable :: skew(:),dsm(:),fsm(:),fini(:),rupt(:)
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
    integer,allocatable,intent(inout) :: p(:)
    integer,intent(in) :: n
    character(*),intent(in) :: label
    allocate(p(n))
  end subroutine
  subroutine deallocate_integer(p)
    integer,allocatable,intent(inout) :: p(:)
    deallocate(p)
  end subroutine
end module
