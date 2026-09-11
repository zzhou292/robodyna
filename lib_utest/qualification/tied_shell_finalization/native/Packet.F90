subroutine native_tied_finalize(nn,nm,ns,nmain,event_capacity,mode,irect,nsv,msr,selected,st,distance, &
    counts,output_nsv,output_msr,output_selected,output_st,output_stb,output_dpara,output_irupt, &
    output_nmas,events,event_values,status) bind(C,name='native_tied_finalize')
  use iso_c_binding
  use ieee_arithmetic
  use finalization_interfaces
  use finalization_storage
  use message_mod
  use names_and_titles_mod
  implicit none
#include "com04_c.inc"
#include "scr03_c.inc"
#include "units_c.inc"
  integer(c_int),value :: nn,nm,ns,nmain,event_capacity,mode
  integer(c_int),intent(in) :: irect(4,nm),nsv(ns),msr(nmain),selected(ns)
  real(c_double),intent(in) :: st(2,ns),distance(ns)
  integer(c_int),intent(inout) :: counts(4),output_nsv(ns),output_msr(nmain),output_selected(ns),output_irupt(ns)
  real(c_double),intent(inout) :: output_st(2,ns),output_stb(2,ns),output_dpara(7*ns),output_nmas(2*nmain)
  integer(c_int),intent(inout) :: events(9,event_capacity)
  real(c_double),intent(inout) :: event_values(3,event_capacity)
  integer(c_int),intent(out) :: status
  type(intbuf_struct_) :: b
  integer,allocatable :: tag(:),itab(:),ikine(:),scratch(:),connection_counts(:),add(:),connection(:)
  real(c_double),allocatable :: x(:,:)
  integer :: ipari(80),dummy(12,1),multi,i,id,iddlevel,iproj
  real(c_double) :: tzinf,dsearch
  character(nchartitle) :: title
  status=1
  if(nn<=0.or.nn>1048576.or.nm<=0.or.nm>524288.or.ns<=0.or.ns>65536.or.nmain<=0.or.nmain>nn) return
  if(event_capacity<=0.or.event_capacity>65544.or.mode<0.or.mode>1) return
  if(any(irect<1).or.any(irect>nn).or.any(nsv<1).or.any(nsv>nn).or.any(msr<1).or.any(msr>nn)) return
  if(any(selected<0).or.any(selected>nm).or.any(.not.ieee_is_finite(st))) return
  if(any(.not.ieee_is_finite(distance)).or.any(distance<0)) return
  allocate(tag(nn))
  tag=0
  do i=1,nmain
    if(tag(msr(i))/=0) return
    tag(msr(i))=1
  enddo
  do i=1,nm
    if(any(iand(tag(irect(:,i)),1)==0)) return
  enddo
  do i=1,ns
    if(iand(tag(nsv(i)),2)/=0) return
    tag(nsv(i))=ior(tag(nsv(i)),2)
  enddo
  numnod=nn
  ninter=1
  numels8=0
  numels10=0
  numels16=0
  kwarn=0
  ipri=0
  err_category=''
  allocate(itab(nn),ikine(5*nn),scratch(5*nn),x(3,nn),connection_counts(nn),add(nn),connection(1))
  itab=[(i,i=1,nn)]
  ikine=0
  scratch=0
  x=0
  add=0
  connection=0
  call kinini(ikine)
  call first_connections(nn,ns,nsv,2,connection_counts,multi)
  if(multi/=0) return
  call initialize_buffer(b,ns,nmain,nm)
  b%nsv=nsv
  b%msr=msr
  b%irtlm=selected
  b%csts=reshape(st,[2*ns])
  b%dpara(1:ns)=distance
  if(mode==1) then
    do i=ns+1,7*ns
      b%dpara(i)=1000+i
    enddo
  endif
  ipari=0
  ipari(4)=nm
  ipari(5)=ns
  ipari(6)=nmain
  ipari(20)=28
  ipari(34)=2
  dummy=0
  id=71
  iddlevel=0
  iproj=1
  tzinf=0
  dsearch=0
  title='serial first-interface finalized search'
  allocate(event_integer(9,event_capacity),event_real(3,event_capacity))
  event_count=0
  event_overflow=.false.
  open(newunit=iout,status='scratch',action='write')
  call i2tid3(x,irect,b%csts,b%msr,b%nsv,b%irtlm,itab,ikine,scratch,b%dpara,ipari,tzinf,iddlevel, &
      id,title,b,dsearch,iproj,dummy,dummy,dummy,dummy,dummy,b%csts_bis,multi,add,connection_counts,connection,dummy)
  close(iout)
  if(.not.event_overflow) then
    counts=[ipari(5),ipari(6),event_count,multi]
    output_nsv=b%nsv
    output_msr=b%msr
    output_selected=b%irtlm
    output_st=reshape(b%csts,[2,ns])
    output_stb=reshape(b%csts_bis,[2,ns])
    output_dpara=b%dpara
    output_irupt=b%irupt
    output_nmas=b%nmas
    events(:,1:event_count)=event_integer(:,1:event_count)
    event_values(:,1:event_count)=event_real(:,1:event_count)
    status=0
  endif
  deallocate(event_integer,event_real)
end subroutine

subroutine native_tied_connection_count(nn,ns,nsv,is1,counts,multi,status) bind(C,name='native_tied_connection_count')
  use iso_c_binding
  use finalization_interfaces
  implicit none
  integer(c_int),value :: nn,ns,is1
  integer(c_int),intent(in) :: nsv(ns)
  integer(c_int),intent(inout) :: counts(nn),multi
  integer(c_int),intent(out) :: status
  integer,allocatable :: next(:)
  integer :: next_multi
  status=1
  if(nn<=0.or.nn>1048576.or.ns<=0.or.ns>65536) return
  if(is1/=2.and.is1/=-1) return
  if(any(nsv<1).or.any(nsv>nn)) return
  allocate(next(nn))
  call first_connections(nn,ns,nsv,is1,next,next_multi)
  counts=next
  multi=next_multi
  status=0
end subroutine
