! SPDX-License-Identifier: AGPL-3.0-or-later
subroutine rd_nodal_bulk(values,kind,seed,slots,extra) bind(C)
  use iso_c_binding
  implicit none
  real(c_double),intent(in)::values(3),seed
  integer(c_int),intent(in)::kind
  real(c_double),intent(out)::slots(8,2),extra(12,2)
  integer::lft,llt
  common /CONTRIBUTION_RANGE/lft,llt
  integer::nc(512,8),mat(1),nnc
  real(c_double)::volu(1),fill(1),pm(32,1),volnod(1),bvolnod(1),vns(8,1),bns(8,1),vnsx(12,1),bnsx(12,1)
  if(kind/=6.and.kind/=8)error stop 'Unselected SBULK3 family'
  lft=1;llt=1;nnc=kind;nc=0;mat=1;pm=0
  volu=values(1);fill=values(2);pm(32,1)=values(3)
  volnod=seed;bvolnod=seed;vns=seed;bns=seed;vnsx=seed;bnsx=seed
  call SBULK3(volu,nc,nnc,mat,pm,volnod,bvolnod,vns,bns,vnsx,bnsx,fill)
  slots(:,1)=vns(:,1);slots(:,2)=bns(:,1)
  extra(:,1)=vnsx(:,1);extra(:,2)=bnsx(:,1)
end subroutine
