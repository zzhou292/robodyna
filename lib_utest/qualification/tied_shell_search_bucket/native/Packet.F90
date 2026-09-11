! SPDX-License-Identifier: MIT
subroutine native_tied_bucket(nn,nm,ns,nmn,capacity,x,irect,nsv,msr, &
    bounds_thickness,projection_input,selected,st,dist,count,pairs,bounds,cells,status) &
    bind(C,name='native_tied_bucket')
  use iso_c_binding,only:c_int,c_double
  use, intrinsic :: ieee_arithmetic
  use constant_mod,only:zero
  use element_mod
  use stack_mod
  use i2trivox_mod
  use bucket_packet_context
  implicit none
#include "mvsiz_p.inc"
  integer(c_int),value,intent(in) :: nn,nm,ns,nmn,capacity
  real(c_double),intent(in) :: x(3,nn),bounds_thickness(nm),projection_input(nm)
  integer(c_int),intent(in) :: irect(4,nm),nsv(ns),msr(nmn)
  integer(c_int),intent(inout) :: selected(ns),count,pairs(2,capacity),cells(3)
  real(c_double),intent(inout) :: st(2,ns),dist(ns),bounds(6)
  integer(c_int),intent(out) :: status
  integer :: i,j,domain_status,next_cells(3),nvsiz
  integer,allocatable :: next_selected(:),node_roles(:)
  real(c_double),allocatable :: next_st(:,:),next_dist(:),segments(:,:)
  real(c_double) :: next_bounds(6),tzinf,gapmin,gapmax
  ! These arguments are deliberately unconsumed by the adapted I2COR3 seam.
  ! They are opaque conforming storage, not a fabricated vehicle incidence set.
  integer :: opaque_s(nixs,0),opaque_s10(nixs10,0),opaque_s16(nixs16,0),opaque_s20(nixs20,0)
  integer :: opaque_c(nixc,0),opaque_t(nixtg,0),opaque_work(3,0),opaque_ip(0)
  integer :: opaque_i(1),opaque_geo(1,1)
  real(c_double) :: opaque_thk(0),opaque_r(1),opaque_mat(1,1)
  type(stack_ply) :: stack
  status=1
  if(nn<1.or.nn>1048576.or.nm<1.or.nm>524288.or.ns<1.or.ns>65536) return
  if(nmn<1.or.nmn>nn.or.capacity<0.or.capacity>8388608) return
  if(allocated(projection_thickness).or.allocated(enumerated_pairs)) return
  if(.not.all(ieee_is_finite(x))) return
  if(.not.all(ieee_is_finite(bounds_thickness)).or.any(bounds_thickness<=zero)) return
  if(.not.all(ieee_is_finite(projection_input)).or.any(projection_input<=zero)) return
  if(any(irect<1).or.any(irect>nn).or.any(nsv<1).or.any(nsv>nn)) return
  if(any(msr<1).or.any(msr>nn)) return
  allocate(node_roles(nn))
  node_roles=0
  do i=1,nmn
    if(node_roles(msr(i))/=0) return
    node_roles(msr(i))=1
  enddo
  do i=1,nm
    do j=1,4
      if(node_roles(irect(j,i))/=1) return
    enddo
  enddo
  node_roles=0
  do i=1,ns
    if(node_roles(nsv(i))/=0) return
    node_roles(nsv(i))=1
  enddo
  allocate(segments(nm,2),next_selected(ns),next_st(2,ns),next_dist(ns))
  call bucket_domain(nn,nm,nmn,x,irect,msr,bounds_thickness, &
      segments,next_bounds,next_cells,tzinf,domain_status)
  if(domain_status/=0) return
  allocate(projection_thickness(nm),enumerated_pairs(2,capacity))
  projection_thickness=projection_input
  pair_count=0
  pair_overflow=.false.
  next_selected=0
  next_st=zero
  next_dist=huge(zero)
  opaque_i=0
  opaque_geo=0
  opaque_r=zero
  opaque_mat=zero
  gapmin=huge(gapmin)
  gapmax=-huge(gapmax)
  nvsiz=mvsiz
  call i2trivox(nvsiz,nn,0,0,0,0,0,0,1,1, &
      opaque_s,opaque_s10,opaque_s16,opaque_s20,opaque_c,opaque_t,opaque_work,ns,nm, &
      28,1,1,1,1,1,1,2,next_cells,nsv,next_selected,opaque_ip,opaque_ip, &
      opaque_i,opaque_i,opaque_i,opaque_i,opaque_i,opaque_i,irect,opaque_geo, &
      zero,next_bounds,tzinf,segments,next_dist,opaque_thk,opaque_r,x,opaque_mat,next_st, &
      opaque_mat,stack,gapmin,gapmax)
  status=0
  if(pair_overflow) status=2
  if(any(next_selected<0).or.any(next_selected>nm)) status=3
  if(.not.all(ieee_is_finite(next_st)).or..not.all(ieee_is_finite(next_dist))) status=3
  if(status==0) then
    selected=next_selected
    st=next_st
    dist=next_dist
    count=pair_count
    pairs(:,1:pair_count)=enumerated_pairs(:,1:pair_count)
    bounds=next_bounds
    cells=next_cells
  endif
  deallocate(projection_thickness,enumerated_pairs)
end subroutine
