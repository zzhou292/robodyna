! Test context only: explicit native node indices, complete Q/T family arrays,
! and unique slave nodes. Native routines determine surface and node ordering.
subroutine native_tied_packing(nn,nq,nt,ns,ixq,ixt,slaves,order,rect,msr,nm,nsv,cleared) &
    bind(C,name='native_tied_packing')
  use iso_c_binding
  use packing_context
  use setdef_mod
  use my_alloc_mod
  implicit none
  integer(c_int), intent(in) :: nn,nq,nt,ns
  integer(c_int), intent(in) :: ixq(5,nq),ixt(4,nt),slaves(ns)
  integer(c_int), intent(out) :: order(nq+nt),rect(4,nq+nt),msr(nn),nm,nsv(ns),cleared
  integer :: i,k,nseg,iad,nix,l,ir,id,msvsize,sirect,limit,ind
  integer, allocatable :: buftmpsurf(:),itri(:,:),index(:),iwork(:),surf(:,:),tags(:),itab(:)
  integer, allocatable :: clause_node(:),tagnod(:),sort(:),idx(:)
  logical :: type18
  type(set_) :: clause
  character(len=128) :: title
  numnod=nn
  ipri=0
  n2d=0
  allocate(buftmpsurf(6*(nq+nt)),itri(5,nq+nt),index(2*max(nq+nt,ns)),iwork(70000))
  allocate(surf(nq+nt,4),tags(0:2*nn),itab(nn))
  allocate(clause%sh4n(nq),clause%sh3n(nt))
  clause%nb_sh4n=nq
  clause%nb_sh3n=nt
  clause%sh4n=[(i,i=1,nq)]
  clause%sh3n=[(i,i=1,nt)]
  itab=[(i,i=1,nn)]
  nseg=0
  iad=1
  call shell_surface_buffer(ixq,5,2,5,3,nseg,iad,0,buftmpsurf,clause)
  call shell_surface_buffer(ixt,4,2,4,7,nseg,iad,0,buftmpsurf,clause)
  include 'SurfaceSort.inc'
  order=index(1:nseg)
  do i=1,nseg
    do k=1,4
      surf(i,k)=buftmpsurf(6*(order(i)-1)+k)
    end do
  end do
  tags=0
  type18=.false.
  title='Native packing packet'
  ir=0
  id=1
  msvsize=nn
  sirect=4*nseg
  call insurf(nseg,nm,ir,rect,surf,itab,msr,id,title,tags,msvsize,sirect,limit,ind,type18)
  cleared=merge(1,0,all(tags(1:)==0))
  ! Exact CREATE_NODE_FROM_ELEMENT sorting block, after physical incidence
  ! has supplied the packet's distinct node list.
  allocate(clause_node(ns),tagnod(nn))
  clause_node=slaves
  tagnod=0
  tagnod(slaves)=1
  ind=ns
  include 'NodeSort.inc'
  call inpoint(ind,id,clause_node,itab,nsv)
end subroutine
