! SPDX-License-Identifier: AGPL-3.0-or-later
! Complete one-occurrence COR3_1 -> DST3_1 -> GLOB_1 stage.
! Coordinates are four main corners and one secondary; IDs are equality-preserving
! local indices checked by the C++ adapter. Source global main identity is retained.
subroutine rd_selection_retained(coords,scalars,normals,bisectors,flags,seed, &
    values,sector_flags,row_markers,row_values,defined) bind(c)
  use iso_c_binding
  use selection_constants
  use selection_observations
  implicit none
  real(c_double),intent(in) :: coords(3,5),scalars(12),seed
  real(c_float),intent(in) :: normals(3,4),bisectors(3,2,4)
  integer(c_int),intent(in) :: flags(19)
  real(c_double),intent(out) :: values(9,4),row_values(4)
  integer(c_int),intent(out) :: sector_flags(4,4),row_markers(4),defined
  integer,parameter :: mvsiz=1
  integer :: jlt,nsn,nin,igap,inacti,nrtm,i,j,node,slot
  integer :: irect(4,1),nsv(1),cand_e(1),cand_n(1),irtlm(4,1)
  integer :: ix1(1),ix2(1),ix3(1),ix4(1),nsvg(1),admsr(4,1),subtria(1)
  integer :: mvoisin(4,1),mvoisn(1,4),lbound(4),ibound(4,1),etyp(1),msegtyp(1)
  integer :: mseglo(1),itab(5),icont_i(1),far(1,4),farm(4,1)
  real(c_double) :: x(3,5),stf(1),stfn(1),stif(1),gap_s(1),gaps(1),gap_m(1),gapm(1)
  real(c_double) :: gapn_m(4,1),gapnm(4,1),gap_s_l(1),gap_m_l(1),gapmxl(1)
  real(c_double) :: xi(1),yi(1),zi(1),xx(1,5),yy(1,5),zz(1,5)
  real(c_double) :: nnx(1,5),nny(1,5),nnz(1,5),time_s(2,1),drad,dgapload
  real(c_double) :: pent(1,4),dist(1),lb(1,4),lc(1,4),lbp(1,4),lcp(1,4)
  real(c_double) :: penm(4,1),lbm(4,1),lcm(4,1)
  real(c_float) :: nod_normal(3,4,1),vtx_bisector(3,2,4)
  logical :: active,clamped_defined
  defined=1
  jlt=1; nsn=1; nin=1; igap=1; inacti=5; nrtm=1
  x=zero
  do i=1,4
    node=flags(i)
    if(node<1.or.node>4) error stop 'Invalid main-node map in native reference'
    x(:,node)=coords(:,i)
    irect(i,1)=node
  enddo
  x(:,5)=coords(:,5)
  nsv=5; cand_e=1; cand_n=1
  irtlm(:,1)=flags(15:18)
  stf=scalars(1); stfn=scalars(2)
  gap_s=scalars(3); gap_m=scalars(4); gapn_m(:,1)=scalars(5:8)
  drad=scalars(9); dgapload=scalars(10); time_s(:,1)=scalars(11:12)
  gap_s_l=zero; gap_m_l=zero ! Unread by the explicitly selected IGAP1 branch.
  nod_normal(:,:,1)=normals; vtx_bisector=bisectors
  mvoisin(:,1)=flags(5:8)
  lbound=0
  do i=1,4
    ! ADMSR always indexes a defined local normal reference. Only LBOUND's
    ! zero/nonzero property is consumed in COR1; IBOUND retains alias identity.
    slot=flags(8+i)
    if(slot<0.or.slot>4) error stop 'Invalid boundary map in native reference'
    if(slot>0)then
      admsr(i,1)=slot
      lbound(slot)=1
    else
      admsr(i,1)=i
    endif
  enddo
  ! The C++ adapter assigns each bound ID its first bound corner's slot;
  ! unbound corners therefore remain distinct in this constructed LBOUND view.
  msegtyp=flags(13); mseglo=flags(14); icont_i=flags(19)
  itab=(/1,2,3,4,5/)
  lb=seed; lc=seed; lbp=seed; lcp=seed
  call i25cor3_1(jlt,x,irect,nsv,cand_e,cand_n,irtlm,stf,stfn,stif,igap, &
      xi,yi,zi,ix1,ix2,ix3,ix4,nsvg,nsn,nin,gap_s,gaps,admsr,nod_normal, &
      xx,yy,zz,nnx,nny,nnz,gap_m,gapm,gapn_m,gapnm,subtria,mvoisin,mvoisn, &
      gap_s_l,gap_m_l,gapmxl,lbound,ibound,etyp,msegtyp,nrtm)
  if(stif(1)>zero.and.(subtria(1)<1.or.subtria(1)>4))then
    defined=0 ! Qualification observation, not a fabricated native status.
    return
  endif
  call i25dst3_1(jlt,cand_n,cand_e,xx,yy,zz,xi,yi,zi,nin,nsn,ix1,ix2,ix3,ix4, &
      nsvg,stif,inacti,mseglo,gaps,gapm,gapmxl,irect,irtlm,time_s,gapnm,itab, &
      icont_i,nnx,nny,nnz,far,pent,dist,lb,lc,lbp,lcp,subtria,mvoisn,ibound, &
      vtx_bisector,drad,dgapload,etyp)
  call i25glob_1(jlt,cand_n,cand_e,nin,nsn,ix1,ix2,ix3,ix4,nsvg,stif,inacti, &
      mseglo,irtlm,time_s,itab,far,pent,lbp,lcp,farm,penm,lbm,lcm)
  active=stif(1)>zero
  do j=1,4
    ! These masks follow complete source write domains, not numerical guesses.
    clamped_defined=active.and.(ix3(1)/=ix4(1).or.j==1)
    values(:,j)=(/lb(1,j),lc(1,j),lbp(1,j),lcp(1,j),pent(1,j),penm(j,1),lbm(j,1),lcm(j,1),retained_distance_squared(1,j)/)
    sector_flags(:,j)=(/far(1,j),farm(j,1),merge(1,0,active),merge(1,0,clamped_defined)/)
  enddo
  row_markers=irtlm(:,1)
  row_values=(/stif(1),dist(1),time_s(1,1),time_s(2,1)/)
end subroutine
