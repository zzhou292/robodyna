! SPDX-License-Identifier: AGPL-3.0-or-later
subroutine rd_native_pack(coords,velocities,controls,flags,gap,ibc) bind(c)
  use iso_c_binding
  implicit none
  real(c_double),intent(in) :: coords(3,5),velocities(3,5),controls(7)
  integer(c_int),intent(in) :: flags(7)
  real(c_double),intent(out) :: gap
  integer(c_int),intent(out) :: ibc
  integer :: irect(4,1),nsv(1),cand_e(1),cand_n(1),ityp,igap,segment(1),icodt(5),iskew(5)
  integer :: ix1(2),ix2(2),ix3(2),ix4(2),etyp(2),bcs(2)
  real(c_double) :: x1(2),x2(2),x3(2),x4(2),y1(2),y2(2),y3(2),y4(2)
  real(c_double) :: z1(2),z2(2),z3(2),z4(2),xi(2),yi(2),zi(2),stif(2),gapv(2)
  real(c_double) :: gap_s(1),gap_m(1),curvature(1),unused_gap(1),drad,load
  real(c_double) :: dt1
  common /qual_cor3t_dt/ dt1
  irect(:,1)=[1,2,3,4]
  if(flags(1)==1)irect(4,1)=3
  nsv=5;cand_e=1;cand_n=1;ityp=-991;igap=1;segment=flags(2)
  icodt=flags(3:7);iskew=0
  gap_s=controls(1);gap_m=controls(2);curvature=controls(3)
  drad=controls(4);load=controls(5);dt1=controls(6);unused_gap=0
  stif=-991
  call i25cor3t(1,irect,coords,nsv,cand_e,cand_n,x1,x2,x3,x4,y1,y2,y3,y4, &
     z1,z2,z3,z4,xi,yi,zi,stif,ix1,ix2,ix3,ix4,1,gap_s,gap_m,gapv, &
     curvature,ityp,1,velocities,igap,unused_gap,unused_gap,segment,etyp, &
     icodt,iskew,bcs,drad,load)
  gap=gapv(1);ibc=bcs(1)
end subroutine
