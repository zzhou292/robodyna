! SPDX-License-Identifier: MIT
! Supplied ordinary-property packet around the complete independent donor.
subroutine native_tied_search_geometry(nn,nq,nt,master,q_nodes,t_nodes, &
    q_order,t_order,geo_thickness,young_modulus,part_thickness, &
    element_thickness,selected,consumed,status) &
    bind(C,name='native_tied_search_geometry')
  use iso_c_binding, only : c_int,c_double
  use search_geometry_context
  use search_geometry_packet_tools
  use search_geometry_native_interfaces
  implicit none
  integer(c_int), value, intent(in) :: nn,nq,nt
  integer(c_int), intent(in) :: master(*),q_nodes(*),t_nodes(*)
  integer(c_int), intent(in) :: q_order(*),t_order(*)
  real(c_double), intent(in) :: geo_thickness(*),young_modulus(*)
  real(c_double), intent(in) :: part_thickness(*),element_thickness(*)
  integer(c_int), intent(inout) :: selected(2)
  real(c_double), intent(inout) :: consumed(2)
  integer(c_int), intent(out) :: status
  integer :: count,i,j,node,nint,is,nty,irect(4,1),next_selected(2)
  integer, allocatable :: ixc(:,:),ixtg(:,:),igeo(:,:),iworksh(:,:)
  integer, allocatable :: knod2elc(:),knod2eltg(:),nod2elc(:),nod2eltg(:)
  integer, allocatable :: ipartc(:),iparttg(:),quad_nodes(:,:),triangle_nodes(:,:)
  integer, allocatable :: quad_order(:),triangle_order(:)
  real(c_double), allocatable :: geo(:,:),pm(:,:),thk(:),thk_part(:)
  real(c_double) :: pm_stack(20,1),next_consumed(2)

  status=1
  ! Count gates precede every borrowed array read and local allocation.
  if(nn<1.or.nn>4096.or.nq<0.or.nt<0.or.nq>128.or.nt>128) return
  count=nq+nt
  if(count<1.or.count>128) return
  do i=1,4
    if(master(i)<1.or.master(i)>nn) return
  enddo
  do i=1,4*nq
    if(q_nodes(i)<1.or.q_nodes(i)>nn) return
  enddo
  do i=1,3*nt
    if(t_nodes(i)<1.or.t_nodes(i)>nn) return
  enddo
  if(.not.valid_order(q_order,nq).or..not.valid_order(t_order,nt)) return
  do i=1,count
    if(.not.ieee_is_finite(geo_thickness(i)).or.geo_thickness(i)<=0) return
    if(.not.ieee_is_finite(young_modulus(i)).or.young_modulus(i)<0) return
    if(.not.ieee_is_finite(part_thickness(i)).or.part_thickness(i)<0) return
    if(.not.ieee_is_finite(element_thickness(i)).or.element_thickness(i)<0) return
  enddo

  allocate(ixc(6,max(1,nq)),ixtg(5,max(1,nt)),igeo(npropgi,count))
  allocate(geo(npropg,count),pm(npropm,count),iworksh(3,count))
  allocate(thk(count),thk_part(count),ipartc(max(1,nq)),iparttg(max(1,nt)))
  allocate(knod2elc(nn+1),knod2eltg(nn+1))
  allocate(nod2elc(max(1,4*nq)),nod2eltg(max(1,3*nt)))
  allocate(quad_nodes(4,nq),triangle_nodes(3,nt),quad_order(nq),triangle_order(nt))
  ixc=0
  ixtg=0
  igeo=0
  geo=0
  pm=0
  iworksh=0
  pm_stack=0
  ipartc=0
  iparttg=0
  nod2elc=0
  nod2eltg=0
  do i=1,count
    ! Independent slots retain each input layer's original field association.
    igeo(11,i)=1
    geo(1,i)=geo_thickness(i)
    pm(20,i)=young_modulus(i)
    thk(i)=element_thickness(i)
    thk_part(i)=part_thickness(i)
  enddo
  do i=1,nq
    ixc(1,i)=i
    ixc(6,i)=i
    ipartc(i)=i
    quad_order(i)=q_order(i)
    do j=1,4
      node=q_nodes(4*(i-1)+j)
      ixc(j+1,i)=node
      quad_nodes(j,i)=node
    enddo
  enddo
  do i=1,nt
    ixtg(1,i)=nq+i
    ixtg(5,i)=nq+i
    iparttg(i)=nq+i
    triangle_order(i)=t_order(i)
    do j=1,3
      node=t_nodes(3*(i-1)+j)
      ixtg(j+1,i)=node
      triangle_nodes(j,i)=node
    enddo
  enddo
  call build_incidence(quad_nodes,quad_order,knod2elc,nod2elc)
  call build_incidence(triangle_nodes,triangle_order,knod2eltg,nod2eltg)
  numnod=nn
  numelc=nq
  numeltg=nt
  iintthick=0
  irect(:,1)=master(1:4)
  nint=1
  is=1
  nty=2
  next_selected=0
  call incoq3(irect,ixc,ixtg,nint,next_selected(1),next_selected(2),is, &
      geo,pm,knod2elc,knod2eltg,nod2elc,nod2eltg,thk,nty,igeo,pm_stack,iworksh)
  if(next_selected(1)<0.or.next_selected(1)>nq) return
  if(next_selected(2)<0.or.next_selected(2)>nt) return
  call native_consumed_thickness(ixc,ixtg,ipartc,iparttg,geo,thk,thk_part, &
      next_selected(1),next_selected(2),next_consumed)
  if(.not.all(ieee_is_finite(next_consumed))) return
  selected=next_selected
  consumed=next_consumed
  status=0
end subroutine
