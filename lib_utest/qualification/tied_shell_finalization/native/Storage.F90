module finalization_storage
  use intbufdef_mod
  implicit none
contains
  subroutine initialize_buffer(b,ns,nm,masters)
    type(intbuf_struct_),intent(out) :: b
    integer,intent(in) :: ns,nm,masters
    allocate(b%nsv(ns),b%msr(nm),b%irtlm(ns),b%irupt(ns),b%msegtyp2(masters))
    allocate(b%csts(2*ns),b%csts_bis(2*ns),b%dpara(7*ns),b%nmas(2*nm),b%smas(ns),b%siner(ns))
    allocate(b%spenalty(ns),b%stfr_penalty(ns),b%skew(9*ns),b%dsm(3*ns),b%fsm(3*ns),b%fini(3*ns))
    allocate(b%areas2(0),b%uvar(0),b%xm0(0),b%rupt(0))
    ! Corresponding selected intbuf_ini_starter fresh initializations.
    b%irupt=0
    b%csts=0
    b%csts_bis=0
    b%dpara=0
    b%nmas=0
    b%smas=0
    b%siner=0
    b%spenalty=0
    b%stfr_penalty=0
    b%skew=0
    b%dsm=0
    b%fsm=0
    b%fini=0
    b%msegtyp2=0
  end subroutine
end module
