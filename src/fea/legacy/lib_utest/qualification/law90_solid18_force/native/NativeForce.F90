! SPDX-License-Identifier: AGPL-3.0-or-later
! Full native geometry/material/force leaves; independently recurrent native state.
subroutine law90_solid_force_native(prepared,cx,cy,n,x0,rho,x,v,step,base,cursors,values,next_cursors,status) &
    bind(C,name='law90_solid_force_native')
  use iso_c_binding,only:c_double,c_int
  use LAW90_ENGINE_PACKETS
  implicit none
  integer(c_int),intent(in) :: n,cursors(3,8)
  real(c_double),intent(in) :: prepared(33),cx(n),cy(n),x0(3,8),rho,x(3,8),v(3,8),step(2),base(20,8)
  real(c_double),intent(out) :: values(357)
  integer(c_int),intent(out) :: next_cursors(3,8),status
  interface
    subroutine reference(x,rho,out,perm,tags,status) bind(C,name='law90_reference_native')
      import c_double,c_int
      real(c_double),intent(in) :: x(3,8),rho
      real(c_double),intent(out) :: out(755)
      integer(c_int),intent(out) :: perm(8),tags(4),status
    end subroutine
    subroutine current(x0,rho,x,v,dt,out,status) bind(C,name='law90_current_native')
      import c_double,c_int
      real(c_double),intent(in) :: x0(3,8),rho,x(3,8),v(3,8),dt
      real(c_double),intent(out) :: out(991)
      integer(c_int),intent(out) :: status
    end subroutine
    subroutine caller(p,cx,cy,n,base,cursors,tensor,rate,step,out,next,status) &
        bind(C,name='law90_solid_caller_native')
      import c_double,c_int
      integer(c_int),intent(in) :: n,cursors(3)
      real(c_double),intent(in) :: p(33),cx(n),cy(n),base(20),tensor(6),rate(6),step(5)
      real(c_double),intent(out) :: out(37)
      integer(c_int),intent(out) :: next(3),status
    end subroutine
  end interface
  type(native_geometry) :: g
  real(c_double) :: initial(755),geometry(991),staged(357),observation(37),material_step(5)
  real(c_double) :: etotsh(1,6),b(6),rate(6),force(MVSIZ,8,3),global(10),stiff,work,dtmin,length
  integer(c_int) :: perm(8),tags(4),staged_cursors(3,8)
  integer :: ir,is,it,ip,k,node,a
  call reference(x0,rho,initial,perm,tags,status)
  if(status/=0)return
  call current(x0,rho,x,v,step(2),geometry,status)
  if(status/=0)return
  call LAW90_FORCE_PACKET(geometry,g)
  force=zero;global=zero;stiff=zero;work=zero;dtmin=1.d30;length=1.d30
  staged=zero;staged_cursors=0
  ! Retain the native r/s/t visitation, independently of stored point numbering.
  do ir=1,2
    do is=1,2
      do it=1,2
        ip=ir+2*(is-1)+4*(it-1)
        call LAW90_FORCE_LENGTH(g,ip,length)
        k=752+30*(ip-1)
        b=geometry(k+18:k+23);rate=geometry(k+24:k+29)
        call LAW90_FORCE_S8ETOTSH10(etotsh,b(1),b(2),b(3),b(4),b(5),b(6),1)
        material_step=[step(1),step(2),g%volume(1,ip),initial(54+10*ip),length]
        call caller(prepared,cx,cy,n,base(:,ip),cursors(:,ip),etotsh,rate,material_step, &
                    observation,staged_cursors(:,ip),status)
        if(status/=0)return
        k=40*(ip-1)
        staged(k+1:k+37)=observation
        staged(k+38:k+40)=material_step(3:5)
        call LAW90_FORCE_POINT_ASSEMBLY(g,ip,observation(11:16),observation(19),force)
        call LAW90_FORCE_REDUCE(g,ip,observation(1:20),material_step(4),initial(135), &
                               observation(37),global,stiff)
        dtmin=min(dtmin,observation(36));work=work+observation(35)
      enddo
    enddo
  enddo
  call LAW90_FORCE_WORLD(g,force)
  staged(321:330)=global;staged(331:333)=[dtmin,stiff,work]
  do node=1,8
    k=333+3*perm(node)
    do a=1,3
      staged(k+a)=force(1,node,a)
    enddo
  enddo
  values=staged;next_cursors=staged_cursors
end subroutine
