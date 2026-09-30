! SPDX-License-Identifier: AGPL-3.0-or-later
! Exact REDEF3 H1 excerpts; wrapper supplies one resolved channel and no secondary curve.
subroutine type13_h1_channel(old,new_deformation,stiffness,length,step,active,curve,oldpos,result,newpos) bind(C)
  use iso_c_binding
  implicit none
  real(c_double),intent(in)::old(5),new_deformation,stiffness,length,step,active,curve(2,5)
  integer(c_int),intent(in)::oldpos
  real(c_double),intent(out)::result(5)
  integer(c_int),intent(out)::newpos
  integer,parameter::nel=1
  real(c_double),parameter::zero=0d0,one=1d0,half=.5d0,ep30=1d30
  integer::i,jecrou(-1:12),ifunc(1),iecrou(1),iad(1),ipos(1),ilen(1)
  real(c_double)::dt11,dx(1),dxold(1),dpx(1),dpx2(1),e(1),xl0(1),fold(1),ddx(1),dvx(1),dvxs(1),ff(1)
  real(c_double)::fx(1),fxep(1),xk(1),xx(1),yy(1),dydx(1),ak(1),off(1)
  real(c_double)::dvv,dfac,d(1),b(1),ee(1),gx(1),xc(1),gf3(1),gx2(1)
  d=1d0;b=0d0;ee=0d0;gx=0d0;xc=0d0;gf3=0d0;gx2=0d0
  dt11=step
  dx=new_deformation;dxold=old(1);dpx=old(2);dpx2=0d0;e=old(5);xl0=length
  fx=old(4);fxep=old(3);ff=1d0;xk=stiffness;off=active;ak=1d0
  jecrou=0;jecrou(1)=1;ifunc=1;iecrou=1
#include "extracted/NormalizeHistory.inc"
#include "extracted/H1Trial.inc"
  iad=1;ipos=oldpos;ilen=4-oldpos
  call type13_native_vinter2(curve,iad,ipos,ilen,nel,xx,dydx,yy)
#include "extracted/H1Clip.inc"
#include "extracted/MaskedWork.inc"
#include "extracted/RestoreHistory.inc"
  result=(/dx(1),dpx(1),fxep(1),fx(1),e(1)/);newpos=ipos(1)
end subroutine
