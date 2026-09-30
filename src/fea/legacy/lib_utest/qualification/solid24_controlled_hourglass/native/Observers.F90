! SPDX-License-Identifier: AGPL-3.0-or-later
module H24_ADAPTER_OBSERVATIONS_MOD
  use iso_c_binding,only:c_double,c_int
  use HEPH_NATIVE_PACKETS
  implicit none
  real(c_double)::snapshot(201)=0
  integer::calls(5)=0
  logical::published=.false.
contains
  subroutine H24_RESET()
    snapshot=zero;calls=0;published=.false.
  end subroutine
  subroutine H24_GEOMETRY(g)
    type(geometry_packet),intent(in)::g
    integer::cursor,n,k
    cursor=1
    call append(g%frame(1,:))
    do n=1,8
      call append(g%x(1,n,:))
    enddo
    do n=1,8
      call append(g%v(1,n,:))
    enddo
    call append([g%volume(1),g%length(1)])
    do k=1,3
      call append(g%p(1,:,k))
    enddo
    do n=1,4
      call append(g%ph(1,n,:))
    enddo
    call append([g%jac(1,1),g%jac(1,5),g%jac(1,7)])
    call append(g%gradient(1,:));call append(g%rate(1,:))
    if(cursor/=106)error stop 'H24 geometry observation extent'
    calls(1)=calls(1)+1
  contains
    subroutine append(a)
      real(c_double),intent(in)::a(:)
      snapshot(cursor:cursor+size(a)-1)=a;cursor=cursor+size(a)
    end subroutine
  end subroutine
  subroutine H24_FORCE(f,offset,stage)
    real(c_double),intent(in)::f(MVSIZ,8,3)
    integer,intent(in)::offset,stage
    integer::n,k
    do n=1,8
      do k=1,3
        snapshot(105+offset+3*(n-1)+k)=f(1,n,k)
      enddo
    enddo
    calls(stage)=calls(stage)+1
  end subroutine
  subroutine H24_PUBLISH()
    if(any(calls/=1))error stop 'H24 missing or repeated stage observation'
    published=.true.
  end subroutine
end module
subroutine H24_ADAPTER_MODES(index,r,f)
  use H24_ADAPTER_OBSERVATIONS_MOD
  implicit none
  integer,intent(in)::index
  real(c_double),intent(in)::r(12),f(12)
  if(index/=1)error stop 'H24 modal observer index'
  snapshot(178:189)=r;snapshot(190:201)=f;calls(5)=calls(5)+1
end subroutine
subroutine h24_adapter_observations(values,counts,valid) bind(C,name='h24_adapter_observations')
  use H24_ADAPTER_OBSERVATIONS_MOD
  implicit none
  real(c_double),intent(out)::values(201)
  integer(c_int),intent(out)::counts(5),valid
  valid=0;counts=calls
  if(.not.published)return
  values=snapshot;valid=1
end subroutine
