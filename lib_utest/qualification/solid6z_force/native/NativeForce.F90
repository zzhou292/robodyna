! SPDX-License-Identifier: AGPL-3.0-or-later
! One-element selected caller. All equations are complete native leaves.
subroutine SOLID6Z_FORCE_NATIVE(PARAMETERS,XREF,JAC_I,BASE,X,V,STEP, &
    GEOMETRY,POINT,HISTORY,FORCES,STABILIZATION,STATUS) bind(C,name='solid6z_force_native')
  use iso_c_binding
  use, intrinsic :: ieee_arithmetic
  use SOLID6Z_FORCE_PACKETS
  implicit none
  real(c_double),intent(in) :: PARAMETERS(4),XREF(3,6),JAC_I(11),BASE(21),X(3,6),V(3,6),STEP(3)
  real(c_double),intent(out) :: GEOMETRY(98),POINT(33),HISTORY(21),FORCES(54),STABILIZATION(28)
  integer(c_int),intent(out) :: STATUS
  type(native_geometry) :: G
  real(kind=8) :: D(MVSIZ,6,3),MATERIAL_STEP(4),NATIVE_POINT(33),NATIVE_HISTORY(21)
  real(kind=8) :: NATIVE_FORCES(54),NATIVE_STABILIZATION(28),GEOMETRY_PACKET(98)
  integer :: K,N,INDEX
  interface
    subroutine law42_solid_caller_native(P,B,M,R,S,O,I) bind(C,name='law42_solid_caller_native')
      use iso_c_binding
      real(c_double),intent(in) :: P(4),B(9),M(9),R(6),S(4)
      real(c_double),intent(out) :: O(33)
      integer(c_int),intent(out) :: I
    end subroutine
  end interface
  STATUS=1
  if (.not.all(ieee_is_finite(PARAMETERS)).or..not.all(ieee_is_finite(XREF))) return
  if (.not.all(ieee_is_finite(JAC_I)).or..not.all(ieee_is_finite(BASE))) return
  if (.not.all(ieee_is_finite(X)).or..not.all(ieee_is_finite(V))) return
  if (.not.all(ieee_is_finite(STEP))) return
  if (STEP(1)<=0.or.STEP(2)<0.or.STEP(3)<0.or.JAC_I(10)<=0.or.JAC_I(11)<=0) return
  G%X=0
  G%V=0
  G%FRAME=0
  G%P=0
  G%JACOBIAN=0
  G%VOLUME=0
  G%LENGTH=0
  G%WORLD_GRADIENT=0
  G%MATERIAL_GRADIENT=0
  G%VELOCITY_GRADIENT=0
  G%RATE=0
  D=0
  call SOLID6Z_FORCE_GLOBALS()
  call SOLID6Z_FORCE_COORDINATES(X,V,XREF,BASE(7),G,D,STATUS)
  if (STATUS/=0) return
  call SOLID6Z_FORCE_CURRENT(G,D,JAC_I(1:10),STEP(1),STATUS)
  if (STATUS/=0) return
  MATERIAL_STEP=[STEP(1),G%VOLUME(1),JAC_I(11),G%LENGTH(1)]
  call law42_solid_caller_native(PARAMETERS,BASE(1:9),G%MATERIAL_GRADIENT(1,:), &
    G%RATE(1,:),MATERIAL_STEP,NATIVE_POINT,STATUS)
  if (STATUS/=0) return
  call SOLID6Z_FORCE_RESULTANTS(G,PARAMETERS,JAC_I(11),BASE,NATIVE_POINT, &
    STEP(1),STEP(2),STEP(3),NATIVE_HISTORY,NATIVE_FORCES,NATIVE_STABILIZATION)
! Native R and public Matrix3 both store world-frame columns in row-major order.
  GEOMETRY_PACKET(1:9)=G%FRAME(1,:)
  do N=1,6
    do K=1,3
      GEOMETRY_PACKET(9+3*(N-1)+K)=G%X(1,N,K)
      GEOMETRY_PACKET(27+3*(N-1)+K)=G%V(1,N,K)
      GEOMETRY_PACKET(45+6*(K-1)+N)=G%P(1,N,K)
    end do
  end do
  GEOMETRY_PACKET(64:72)=G%WORLD_GRADIENT(1,:)
  GEOMETRY_PACKET(73:81)=G%MATERIAL_GRADIENT(1,:)
  GEOMETRY_PACKET(82:90)=G%VELOCITY_GRADIENT(1,:)
  GEOMETRY_PACKET(91:96)=G%RATE(1,:)
  GEOMETRY_PACKET(97:98)=[G%VOLUME(1),G%LENGTH(1)]
  STATUS=3
  if (.not.all(ieee_is_finite(GEOMETRY_PACKET))) return
  if (.not.all(ieee_is_finite(NATIVE_POINT))) return
  if (.not.all(ieee_is_finite(NATIVE_HISTORY))) return
  if (.not.all(ieee_is_finite(NATIVE_FORCES))) return
  if (.not.all(ieee_is_finite(NATIVE_STABILIZATION))) return
  GEOMETRY=GEOMETRY_PACKET
  POINT=NATIVE_POINT
  HISTORY=NATIVE_HISTORY
  FORCES=NATIVE_FORCES
  STABILIZATION=NATIVE_STABILIZATION
  STATUS=0
end subroutine
