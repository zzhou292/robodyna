! SPDX-License-Identifier: AGPL-3.0-or-later
! Complete source-indexed COR22 -> DST22/helpers -> GLOB22 for one occurrence.
! Packed startup tables are unpacked without arithmetic; actual local IDs, NRTM
! and signed MSEGTYP stay unchanged. Unselected main rows are never consumed.
subroutine rd_selection_new_impact(counts,kinematics,source_int,source_real,normals, &
    bounds,bisectors,control,prior_markers,prior_metrics,seed,projection,side_values, &
    side_flags,cache_values,cache_far,row_markers,row_metrics,scalar_values,decision) bind(c)
  use iso_c_binding
  use selection_constants
  use selection_observations
  implicit none
  integer(c_int),intent(in) :: counts(4) ! NRTM, references, primary local main, ICONT_I.
  integer(c_int),intent(in) :: source_int(14,counts(1)),bounds(counts(2)),prior_markers(4)
  real(c_double),intent(in) :: kinematics(3,5,2),source_real(6,counts(1))
  real(c_float),intent(in) :: normals(3,4,counts(1)),bisectors(3,2,counts(2))
  real(c_double),intent(in) :: control(3),prior_metrics(2),seed
  real(c_double),intent(out) :: projection(5,4),side_values(2,4),cache_values(3,4)
  real(c_double),intent(out) :: row_metrics(2),scalar_values(4)
  integer(c_int),intent(out) :: side_flags(4,4),cache_far(4),row_markers(4),decision(10)
  integer :: nrtm,jlt,nsn,nin,igap,inacti,i,j,node,l,ispmd
  common /selection_process/ ispmd
  real(c_double) :: dt1
  common /selection_clock/ dt1
  integer :: irect(4,counts(1)),msegtyp(counts(1)),mseglo(counts(1))
  integer :: mvoisin(4,counts(1)),admsr(4,counts(1))
  real(c_double) :: stf(counts(1)),gap_m(counts(1)),gapn_m(4,counts(1)),gap_m_l(counts(1))
  integer :: nsv(1),cand_e(1),cand_n(1),global_cand_e(1),irtlm(4,1),index(1)
  integer :: ix1(1),ix2(1),ix3(1),ix4(1),nsvg(1),ishel(1),subtria(1),etyp(1)
  integer :: mvoisa(1,4),mvoisb(1,4),ibounda(4,1),iboundb(4,1),icont_i(1),itab(5)
  integer :: far(1),farm(4,1)
  real(c_double) :: x(3,5),v(3,5),stfn(1),stif(1),gap_s(1),gaps(1),gapm(1)
  real(c_double) :: gapnm(4,1),gap_s_l(1),gapmxl(1),xi(1),yi(1),zi(1),vxi(1),vyi(1),vzi(1)
  real(c_double) :: xx(1,5),yy(1,5),zz(1,5),vx1(1),vx2(1),vx3(1),vx4(1)
  real(c_double) :: vy1(1),vy2(1),vy3(1),vy4(1),vz1(1),vz2(1),vz3(1),vz4(1)
  real(c_double) :: nax(1,5),nay(1,5),naz(1,5),nbx(1,5),nby(1,5),nbz(1,5)
  real(c_double) :: time_s(2,1),pene_old(5,1),stif_old(2,1),penmin,eps0,marge,drad,dgapload
  real(c_double) :: pent(1),lbs(1),lcs(1),lbp(1,4),lcp(1,4),penm(4,1),lbm(4,1),lcm(4,1)
  nrtm=counts(1)
  if(nrtm<1.or.counts(2)<1.or.counts(3)<1.or.counts(3)>nrtm) &
      error stop 'Invalid new-impact reference dimensions'
  irect=source_int(1:4,:); msegtyp=source_int(5,:); mseglo=source_int(6,:)
  mvoisin=source_int(7:10,:); admsr=source_int(11:14,:)
  stf=source_real(1,:); gap_m=source_real(2,:); gapn_m=source_real(3:6,:)
  gap_m_l=zero ! Selected IGAP1 does not read the limited-gap branch.
  jlt=1; nsn=1; nin=1; igap=1; inacti=5; ispmd=0
  x=zero; v=zero
  do i=1,4
    node=irect(i,counts(3))
    if(node<1.or.node>4) error stop 'Invalid current source node mapping'
    x(:,node)=kinematics(:,i,1); v(:,node)=kinematics(:,i,2)
  enddo
  x(:,5)=kinematics(:,5,1); v(:,5)=kinematics(:,5,2)
  nsv=5; cand_n=1; cand_e=counts(3); global_cand_e=cand_e; index=1
  irtlm(:,1)=prior_markers; time_s(:,1)=prior_metrics; itab=(/1,2,3,4,5/)
  stfn=control(1); gap_s=control(2); dt1=control(3); icont_i=counts(4)
  drad=zero; dgapload=zero; gap_s_l=zero
  ! Unread legacy arguments in this selected donor stage; no force/history law
  ! or artificial gap is evaluated through these storage placeholders.
  pene_old=zero; stif_old=zero; penmin=zero; eps0=zero; marge=zero
  pent=seed; lbs=seed; lcs=seed; far=-91
  lbp=seed; lcp=seed; nbx=seed; nby=seed; nbz=seed
  impact_side_choice=0; impact_won=0
  call i25cor3_22(jlt,x,irect,nsv,cand_e,cand_n,stf,stfn,stif,igap,xi,yi,zi, &
      vxi,vyi,vzi,ix1,ix2,ix3,ix4,nsvg,nsn,v,nin,gap_s,gaps,admsr,normals, &
      xx,yy,zz,vx1,vx2,vx3,vx4,vy1,vy2,vy3,vy4,vz1,vz2,vz3,vz4, &
      nax,nay,naz,nbx,nby,nbz,gap_m,gapm,gapn_m,gapnm,mvoisin,nrtm,msegtyp,ishel, &
      mvoisa,mvoisb,gap_s_l,gap_m_l,gapmxl,bounds,ibounda,iboundb,etyp)
  call i25dst3_22(jlt,cand_n,cand_e,ishel,xx,yy,zz,xi,yi,zi,vx1,vx2,vx3,vx4,vxi, &
      vy1,vy2,vy3,vy4,vyi,vz1,vz2,vz3,vz4,vzi,nin,nsn,ix1,ix2,ix3,ix4,nsvg, &
      stif,inacti,mseglo,gaps,gapm,irect,irtlm,time_s,gapnm,pene_old,stif_old,itab, &
      penmin,eps0,icont_i,marge,nax,nay,naz,nbx,nby,nbz,far,pent,subtria,lbs,lcs, &
      lbp,lcp,mvoisa,mvoisb,gapmxl,ibounda,iboundb,bisectors,drad,dgapload,etyp)
  call i25glob_22(jlt,cand_n,cand_e,global_cand_e,nin,nsn,ix1,ix2,ix3,ix4,nsvg, &
      stif,inacti,mseglo,irtlm,time_s,itab,subtria,far,pent,lbs,lcs,index,farm,penm,lbm,lcm)
  do j=1,4
    projection(:,j)=(/impact_raw_lb(1,j),impact_raw_lc(1,j),lbp(1,j),lcp(1,j), &
        impact_distance_squared(1,j)/)
    side_values(:,j)=(/impact_primary_penetration(1,j),impact_opposite_penetration(1,j)/)
    side_flags(:,j)=(/impact_primary_far(1,j),impact_opposite_far(1,j), &
        impact_primary_gap(1,j),impact_opposite_gap(1,j)/)
    cache_values(:,j)=(/penm(j,1),lbm(j,1),lcm(j,1)/)
  enddo
  cache_far=farm(:,1); row_markers=irtlm(:,1); row_metrics=time_s(:,1)
  scalar_values=(/stif(1),pent(1),lbs(1),lcs(1)/)
  decision=(/subtria(1),far(1),global_cand_e(1),impact_side_choice(1),impact_won(1), &
      impact_side_selector(1,1),impact_side_selector(2,1),impact_intersection(1,1), &
      impact_intersection(2,1),impact_recontact(1)/)
end subroutine
