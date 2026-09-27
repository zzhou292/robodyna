! SPDX-License-Identifier: AGPL-3.0-or-later
module IC1_NATIVE_OBSERVATIONS
  implicit none
  real(kind=8)::hourglass_work=0
  integer::hourglass_calls=0,center_contacts=0,corner_contacts=0
  real(kind=8)::distortion_sigma(6)=0,distortion_parameters(5)=0
  integer::distortion_flag=0
end module
subroutine IC1_NATIVE_HOUR_WORK(index,work)
  use IC1_NATIVE_OBSERVATIONS
  implicit none
  integer,intent(in)::index
  real(kind=8),intent(in)::work
  if(index/=1)error stop 'Controlled HEPH observation index outside one-parent coupon'
  hourglass_work=work
  hourglass_calls=hourglass_calls+1
end subroutine

subroutine IC1_NATIVE_GEOMETRY_FORCE(category,index,force)
  use IC1_NATIVE_OBSERVATIONS
  implicit none
  integer,intent(in)::category,index
  real(kind=8),intent(in)::force
  if(index/=1)error stop 'Controlled geometry observation outside one-parent coupon'
  if(force>0)then
    if(category==1)center_contacts=center_contacts+1
    if(category==2)corner_contacts=corner_contacts+1
  endif
end subroutine
subroutine ic1_native_geometry_counts(center,corner) bind(C,name='ic1_native_geometry_counts')
  use iso_c_binding,only:c_int
  use IC1_NATIVE_OBSERVATIONS
  implicit none
  integer(c_int),intent(out)::center,corner
  center=center_contacts;corner=corner_contacts
end subroutine

subroutine ic1_native_distortion_observation(sig,parameters,flag) bind(C,name='ic1_native_distortion_observation')
  use iso_c_binding,only:c_double,c_int
  use IC1_NATIVE_OBSERVATIONS
  implicit none
  real(c_double),intent(out)::sig(6),parameters(5)
  integer(c_int),intent(out)::flag
  sig=distortion_sigma;parameters=distortion_parameters;flag=distortion_flag
end subroutine
