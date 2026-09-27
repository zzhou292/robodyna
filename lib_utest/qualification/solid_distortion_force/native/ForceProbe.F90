! SPDX-License-Identifier: AGPL-3.0-or-later
! Array packing only. Arithmetic remains the qualified a62 full native leaf.
subroutine distortion_force_native(count,parameters,flags,xworld,vworld,base,step,result,contacts) &
  bind(C,name='distortion_force_native')
  use iso_c_binding,only:c_double,c_int
  use HEPH_NATIVE_PACKETS
  use IC1_NATIVE_INTERFACES
  use IC1_NATIVE_OBSERVATIONS
  implicit none
  integer(c_int),value,intent(in)::count
  real(c_double),intent(in)::parameters(5,2),xworld(3,8,2),vworld(3,8,2),base(26,2),step
  integer(c_int),intent(in)::flags(2)
  real(c_double),intent(out)::result(26,2)
  integer(c_int),intent(out)::contacts(2)
  real(c_double)::x(MVSIZ,8,3),v(MVSIZ,8,3),f(MVSIZ,8,3),sti(MVSIZ),stic(2)
  real(c_double)::fld(MVSIZ),ll(MVSIZ),energy(2)
  integer::istab(MVSIZ),i,n,k
  if(count<1.or.count>2)error stop 'Native distortion probe count'
  x=zero;v=zero;f=zero;sti=zero;stic=zero;fld=zero;ll=zero;energy=zero;istab=0;result=zero
  center_contacts=0;corner_contacts=0
  do i=1,count
    if(parameters(4,i)/=parameters(4,1).or.parameters(5,i)/=parameters(5,1))error stop 'Native common material operands'
    do n=1,8
      do k=1,3
        x(i,n,k)=xworld(k,n,i);v(i,n,k)=vworld(k,n,i);f(i,n,k)=base(3*(n-1)+k,i)
      enddo
    enddo
    sti(i)=base(25,i);energy(i)=base(26,i)
    stic(i)=parameters(1,i);fld(i)=parameters(2,i);ll(i)=parameters(3,i);istab(i)=flags(i)
  enddo
  call IC1_NATIVE_S8FOR_DISTOR( &
    x(1,1,1),x(1,2,1),x(1,3,1),x(1,4,1),x(1,5,1),x(1,6,1),x(1,7,1),x(1,8,1), &
    x(1,1,2),x(1,2,2),x(1,3,2),x(1,4,2),x(1,5,2),x(1,6,2),x(1,7,2),x(1,8,2), &
    x(1,1,3),x(1,2,3),x(1,3,3),x(1,4,3),x(1,5,3),x(1,6,3),x(1,7,3),x(1,8,3), &
    v(1,1,1),v(1,2,1),v(1,3,1),v(1,4,1),v(1,5,1),v(1,6,1),v(1,7,1),v(1,8,1), &
    v(1,1,2),v(1,2,2),v(1,3,2),v(1,4,2),v(1,5,2),v(1,6,2),v(1,7,2),v(1,8,2), &
    v(1,1,3),v(1,2,3),v(1,3,3),v(1,4,3),v(1,5,3),v(1,6,3),v(1,7,3),v(1,8,3), &
    f(1,1,1),f(1,2,1),f(1,3,1),f(1,4,1),f(1,5,1),f(1,6,1),f(1,7,1),f(1,8,1), &
    f(1,1,2),f(1,2,2),f(1,3,2),f(1,4,2),f(1,5,2),f(1,6,2),f(1,7,2),f(1,8,2), &
    f(1,1,3),f(1,2,3),f(1,3,3),f(1,4,3),f(1,5,3),f(1,6,3),f(1,7,3),f(1,8,3), &
    sti,stic,fld,parameters(4,1),ll,istab,parameters(5,1),count,energy,step)
  do i=1,count
    do n=1,8
      do k=1,3
        result(3*(n-1)+k,i)=f(i,n,k)
      enddo
    enddo
    result(25,i)=sti(i);result(26,i)=energy(i)
  enddo
  contacts=[center_contacts,corner_contacts]
end subroutine
