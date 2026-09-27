! SPDX-License-Identifier: AGPL-3.0-or-later
! Raw operand binding only; complete native SHOUR_CTL supplies every result.
subroutine controlled_hourglass_native(p,velocity,projection,base,inforce,values) &
    bind(C,name='controlled_hourglass_native')
  use iso_c_binding,only:c_double
  use HEPH_NATIVE_CONSTANT_MOD
  use IC1_NATIVE_MVSIZ_MOD
  use IC1_NATIVE_SHOUR_CTL_MOD
  use CONTROLLED_LEAF_OBSERVATIONS
  implicit none
  real(c_double),intent(in)::p(9),velocity(3,8),projection(3,4),base(4,3),inforce(3,8)
  real(c_double),intent(out)::values(63)
  interface
    subroutine slots_native(p,s) bind(C,name='ic1_native_slots')
      import c_double
      real(c_double),intent(in)::p(4)
      real(c_double),intent(out)::s(6)
    end subroutine
  end interface
  real(c_double)::pm(110,1),rho(1),off(mvsiz),v(mvsiz,8,3),f(mvsiz,8,3),ph(mvsiz,4,3)
  real(c_double)::vol(mvsiz),vol0(mvsiz),hour(1,3,4),cxx(mvsiz),energy(mvsiz),sti(mvsiz)
  real(c_double)::parameters(4),slots(6)
  integer::mat(mvsiz),n,k,h,cursor
  parameters=[p(1),p(2),p(3),one]
  call slots_native(parameters,slots)
  pm=zero;pm(1,1)=p(3);pm(20,1)=slots(1);pm(21,1)=slots(2);pm(22,1)=slots(3)
  pm(32,1)=slots(4);pm(100,1)=slots(5);pm(107,1)=slots(6)
  rho=p(3);off=one;mat=1;v=zero;f=zero;ph=zero;vol=zero;vol0=zero;cxx=zero;energy=zero;sti=zero
  vol(1)=p(6);vol0(1)=p(7);cxx(1)=p(4);energy(1)=p(8);sti(1)=p(9)
  do k=1,3
    do n=1,8
      v(1,n,k)=velocity(k,n);f(1,n,k)=inforce(k,n)
    enddo
    do h=1,4
      ph(1,h,k)=projection(k,h);hour(1,k,h)=base(h,k)
    enddo
  enddo
  work=zero;rate=zero;force=zero;work_calls=0;mode_calls=0
  call IC1_NATIVE_SHOUR_CTL(pm,rho,off, &
    v(:,1,1),v(:,2,1),v(:,3,1),v(:,4,1),v(:,5,1),v(:,6,1),v(:,7,1),v(:,8,1), &
    v(:,1,2),v(:,2,2),v(:,3,2),v(:,4,2),v(:,5,2),v(:,6,2),v(:,7,2),v(:,8,2), &
    v(:,1,3),v(:,2,3),v(:,3,3),v(:,4,3),v(:,5,3),v(:,6,3),v(:,7,3),v(:,8,3), &
    f(:,1,1),f(:,1,2),f(:,1,3),f(:,2,1),f(:,2,2),f(:,2,3), &
    f(:,3,1),f(:,3,2),f(:,3,3),f(:,4,1),f(:,4,2),f(:,4,3), &
    f(:,5,1),f(:,5,2),f(:,5,3),f(:,6,1),f(:,6,2),f(:,6,3), &
    f(:,7,1),f(:,7,2),f(:,7,3),f(:,8,1),f(:,8,2),f(:,8,3), &
    ph(:,1,1),ph(:,1,2),ph(:,1,3),ph(:,2,1),ph(:,2,2),ph(:,2,3), &
    ph(:,3,1),ph(:,3,2),ph(:,3,3),ph(:,4,1),ph(:,4,2),ph(:,4,3), &
    vol,hour,42,p(5),mat,cxx,energy,110,1,vol0,ZEP1,sti,1)
  if(work_calls/=1.or.mode_calls/=1)error stop 'Controlled leaf observer call count'
  cursor=1
  do k=1,3
    do h=1,4
      values(cursor)=hour(1,k,h);cursor=cursor+1
    enddo
  enddo
  do n=1,8
    do k=1,3
      values(cursor)=f(1,n,k);cursor=cursor+1
    enddo
  enddo
  values(37:39)=[energy(1),sti(1),work]
  values(40:51)=rate;values(52:63)=force
end subroutine
