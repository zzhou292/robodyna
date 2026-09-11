! SPDX-License-Identifier: AGPL-3.0-or-later
! ABI/storage only around complete S6ZRCOOR3 and SGCOOR3.
subroutine SOLID6Z_FORCE_COORDINATES(X,V,XREF,DENSITY,G,DISPLACEMENT,STATUS)
  use SOLID6Z_FORCE_PACKETS
  use SOLID6Z_FORCE_ELEMENT_MOD
  use SOLID6Z_FORCE_PROP_PARAM_MOD
  use SOLID6Z_FORCE_S6ZRCOOR3_MOD
  implicit none
  real(kind=8),intent(in) :: X(3,6),V(3,6),XREF(3,6),DENSITY
  type(native_geometry),intent(inout) :: G
  real(kind=8),intent(out) :: DISPLACEMENT(MVSIZ,6,3)
  integer,intent(out) :: STATUS
  integer :: IXS(NIXS,1),NC(MVSIZ,6),NGL(1),MXT(1),NGEO(1),IPARG(N_VAR_IPARG),IDTMIN(102)
  integer :: N,K
  real(kind=8) :: GAMA0(1,6),GAMA(MVSIZ,6),GAMAR(1,6),SAVED(1,15)
  real(kind=8) :: OFF(MVSIZ),OFFG(MVSIZ),OFF0(MVSIZ),RHO(MVSIZ),RHOO(MVSIZ)
  real(kind=8) :: LOCAL_REAL(MVSIZ,6,3),INITIAL(MVSIZ,6,3),AUX(MVSIZ,15),D(3,6)
  IXS=1
  IXS(2:9,1)=[1,2,3,3,4,5,6,6]
  IPARG=0
  IDTMIN=0
  GAMA0=0
  GAMA=0
  GAMAR=0
  INITIAL=0
  AUX=0
  OFF=1
  OFFG=1
  OFF0=1
  RHO=DENSITY
  RHOO=0
  D=0
  STATUS=0
  do K=1,3
    do N=1,5
      SAVED(1,5*(K-1)+N)=XREF(K,N)-XREF(K,6)
    end do
  end do
  call SOLID6Z_FORCE_S6ZRCOOR3( &
    6, X, IXS, V, GAMA0, &
    GAMA, LOCAL_REAL(:,1,1), LOCAL_REAL(:,2,1), LOCAL_REAL(:,3,1), LOCAL_REAL(:,4,1), &
    LOCAL_REAL(:,5,1), LOCAL_REAL(:,6,1), LOCAL_REAL(:,1,2), LOCAL_REAL(:,2,2), LOCAL_REAL(:,3,2), &
    LOCAL_REAL(:,4,2), LOCAL_REAL(:,5,2), LOCAL_REAL(:,6,2), LOCAL_REAL(:,1,3), LOCAL_REAL(:,2,3), &
    LOCAL_REAL(:,3,3), LOCAL_REAL(:,4,3), LOCAL_REAL(:,5,3), LOCAL_REAL(:,6,3), G%V(:,1,1), &
    G%V(:,2,1), G%V(:,3,1), G%V(:,4,1), G%V(:,5,1), G%V(:,6,1), &
    G%V(:,1,2), G%V(:,2,2), G%V(:,3,2), G%V(:,4,2), G%V(:,5,2), &
    G%V(:,6,2), G%V(:,1,3), G%V(:,2,3), G%V(:,3,3), G%V(:,4,3), &
    G%V(:,5,3), G%V(:,6,3), AUX(:,1), AUX(:,2), OFFG, &
    OFF, SAVED, RHO, RHOO, G%FRAME(:,1), &
    G%FRAME(:,2), G%FRAME(:,3), G%FRAME(:,4), G%FRAME(:,5), G%FRAME(:,6), &
    G%FRAME(:,7), G%FRAME(:,8), G%FRAME(:,9), NC(:,1), NC(:,2), &
    NC(:,3), NC(:,4), NC(:,5), NC(:,6), NGL, &
    MXT, NGEO, 0, AUX(:,3), AUX(:,4), &
    AUX(:,5), AUX(:,6), 1, AUX(:,7), AUX(:,8), &
    AUX(:,9), AUX(:,10), AUX(:,11), AUX(:,12), AUX(:,13), &
    AUX(:,14), AUX(:,15), IPARG, GAMAR, NIXS, &
    0, 10, 0, 1, G%X(:,1,1), &
    G%X(:,2,1), G%X(:,3,1), G%X(:,4,1), G%X(:,5,1), G%X(:,6,1), &
    G%X(:,1,2), G%X(:,2,2), G%X(:,3,2), G%X(:,4,2), G%X(:,5,2), &
    G%X(:,6,2), G%X(:,1,3), G%X(:,2,3), G%X(:,3,3), G%X(:,4,3), &
    G%X(:,5,3), G%X(:,6,3), 24, IDTMIN, INITIAL(:,:,1), &
    INITIAL(:,:,2), INITIAL(:,:,3), 0, X)
  call SOLID6Z_FORCE_SGCOOR3(0.0D0,6,X,IXS, &
    INITIAL(:,:,1),INITIAL(:,:,2),INITIAL(:,:,3), &
    DISPLACEMENT(:,:,1),DISPLACEMENT(:,:,2),DISPLACEMENT(:,:,3), &
    SAVED,D,OFF,OFF0,1,X,42,10)
end subroutine
