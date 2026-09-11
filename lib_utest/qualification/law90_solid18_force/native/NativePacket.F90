! SPDX-License-Identifier: AGPL-3.0-or-later
! Rehydrate only the independently evaluated native current packet. No equations.
subroutine LAW90_FORCE_PACKET(values,g)
  use LAW90_ENGINE_PACKETS
  implicit none
  real(kind=8),intent(in) :: values(991)
  type(native_geometry),intent(out) :: g
  integer :: k,n,a,ip
  g%frame=zero;g%x=zero;g%v=zero;g%center_p=zero
  g%volume=zero;g%inverse=zero;g%p=zero;g%shear=zero;g%cross=zero
  g%center_volume=zero;g%smax=zero
  g%frame(1,:)=values(1:9);k=9
  do n=1,8
    g%x(1,n,:)=values(k+1:k+3);k=k+3
  enddo
  do n=1,8
    g%v(1,n,:)=values(k+1:k+3);k=k+3
  enddo
  do a=1,3
    g%center_p(1,:,a)=values(k+1:k+4);k=k+4
  enddo
  g%center_volume(1)=values(k+1);g%smax(1)=values(k+2);k=k+2
  do ip=1,8
    g%volume(1,ip)=values(k+1);k=k+1
    g%inverse(1,ip,:)=values(k+1:k+9);k=k+9
    do a=1,3
      g%p(1,ip,:,a)=values(k+1:k+8);k=k+8
    enddo
    do a=1,6
      g%shear(1,ip,:,a)=values(k+1:k+8);k=k+8
    enddo
  enddo
end subroutine
