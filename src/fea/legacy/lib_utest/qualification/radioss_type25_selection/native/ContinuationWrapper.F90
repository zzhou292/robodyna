! SPDX-License-Identifier: AGPL-3.0-or-later
! Complete local COR3_21 -> DST3_21 -> GLOB stage. Native reference IDs and NRTM
! remain supplied values; only physical node/main array addresses are private.
subroutine rd_selection_continuation(coords,scalars,normals,bisectors,flags, &
    references,sliding,bounds,reference_count,segment_count,constraints,skews,seed, &
    values,sector_flags,row_markers,row_values,axes,decision) bind(c)
  use iso_c_binding
  use selection_constants
  use selection_observations
  implicit none
  integer(c_int),intent(in) :: reference_count,segment_count
  real(c_double),intent(in) :: coords(3,5),scalars(12),seed
  real(c_float),intent(in) :: normals(3,4),bisectors(3,2,reference_count)
  integer(c_int),intent(in) :: flags(19),references(4),sliding(4),bounds(reference_count)
  integer(c_int),intent(in) :: constraints(5),skews(5)
  real(c_double),intent(out) :: values(9,4),row_values(4)
  integer(c_int),intent(out) :: sector_flags(6,4),row_markers(4),axes(3),decision(2)
  integer :: jlt,nsn,nin,igap,inacti,nrtm,i,j,node,ispmd
  common /selection_process/ ispmd
  integer :: irect(4,1),nsv(1),cand_e(1),cand_n(1),irtlm(4,1)
  integer :: ix1(1),ix2(1),ix3(1),ix4(1),nsvg(1),admsr(4,1)
  integer :: mvoisin(4,1),mvoisn(1,4),ibound(4,1),etyp(1),msegtyp(1)
  integer :: mseglo(1),itab(5),far(1,4),farm(4,1),islide(4,1),kslide(1,4)
  integer :: icodt(5),iskew(5),index(1)
  real(c_double) :: x(3,5),stf(1),stfn(1),stif(1),gap_s(1),gaps(1),gap_m(1),gapm(1)
  real(c_double) :: gapn_m(4,1),gapnm(4,1),gap_s_l(1),gap_m_l(1),gapmxl(1)
  real(c_double) :: xi(1),yi(1),zi(1),xx(1,5),yy(1,5),zz(1,5)
  real(c_double) :: nnx(1,5),nny(1,5),nnz(1,5),time_s(2,1),drad,dgapload
  real(c_double) :: pent(1,4),dist(1),lb(1,4),lc(1,4),lbp(1,4),lcp(1,4)
  real(c_double) :: penm(4,1),lbm(4,1),lcm(4,1)
  real(c_float) :: nod_normal(3,4,1)
  logical :: active,clamped_defined
  if(reference_count<1.or.segment_count<1) error stop 'Invalid reference-library bounds'
  jlt=1; nsn=1; nin=1; igap=1; inacti=5; nrtm=segment_count; ispmd=0
  x=zero; icodt=0; iskew=0
  do i=1,4
    node=flags(i)
    if(node<1.or.node>4) error stop 'Invalid node map in native reference'
    if(references(i)<1.or.references(i)>reference_count) error stop 'Invalid native normal reference'
    if(sliding(i)<0) error stop 'Invalid native sliding reference'
    x(:,node)=coords(:,i); irect(i,1)=node
    icodt(node)=constraints(i); iskew(node)=skews(i)
  enddo
  x(:,5)=coords(:,5); icodt(5)=constraints(5); iskew(5)=skews(5)
  nsv=5; cand_e=1; cand_n=1; index=1
  irtlm(:,1)=flags(15:18)
  stf=scalars(1); stfn=scalars(2)
  gap_s=scalars(3); gap_m=scalars(4); gapn_m(:,1)=scalars(5:8)
  drad=scalars(9); dgapload=scalars(10); time_s(:,1)=scalars(11:12)
  gap_s_l=zero; gap_m_l=zero ! Unread by the selected IGAP1 branch.
  nod_normal(:,:,1)=normals; mvoisin(:,1)=flags(5:8)
  admsr(:,1)=references; islide(:,1)=sliding
  msegtyp=flags(13); mseglo=flags(14); itab=(/1,2,3,4,5/)
  lb=seed; lc=seed; lbp=seed; lcp=seed
  continuation_selected=0; continuation_won=0
  call i25cor3_21(jlt,x,irect,nsv,cand_e,cand_n,stf,stfn,stif,igap, &
      xi,yi,zi,ix1,ix2,ix3,ix4,nsvg,nsn,msegtyp,etyp,nin,gap_s,gaps, &
      admsr,nod_normal,xx,yy,zz,nnx,nny,nnz,gap_m,gapm,gapn_m,gapnm, &
      islide,kslide,mvoisin,mvoisn,gap_s_l,gap_m_l,gapmxl,bounds,ibound)
  call i25dst3_21(jlt,cand_n,cand_e,nrtm,xx,yy,zz,xi,yi,zi,nin,nsn, &
      ix1,ix2,ix3,ix4,nsvg,stif,inacti,mseglo,gaps,gapm,irect,irtlm,time_s, &
      gapnm,itab,nnx,nny,nnz,far,pent,dist,lb,lc,lbp,lcp,kslide,mvoisn, &
      gapmxl,ibound,bisectors,etyp,icodt,iskew,drad,dgapload)
  call i25glob(jlt,cand_n,cand_e,nin,nsn,ix1,ix2,ix3,ix4,nsvg,stif, &
      inacti,mseglo,irtlm,time_s,itab,far,pent,lbp,lcp,index,farm,penm,lbm,lcm)
  active=stif(1)>zero
  do j=1,4
    clamped_defined=active.and.(ix3(1)/=ix4(1).or.j==1)
    values(:,j)=(/lb(1,j),lc(1,j),lbp(1,j),lcp(1,j),pent(1,j),penm(j,1), &
        lbm(j,1),lcm(j,1),continuation_distance_squared(1,j)/)
    sector_flags(:,j)=(/far(1,j),farm(j,1),merge(1,0,active),merge(1,0,clamped_defined), &
        kslide(1,j),continuation_ingap(1,j)/)
  enddo
  row_markers=irtlm(:,1); row_values=(/stif(1),dist(1),time_s(1,1),time_s(2,1)/)
  axes=continuation_axes(:,1)
  ! Inactive selected-subtriangle was not defined by the source; private zero is
  ! API-only. Winner flags arise only from the actual original local branch.
  decision=(/continuation_selected(1),continuation_won(1)/)
end subroutine
