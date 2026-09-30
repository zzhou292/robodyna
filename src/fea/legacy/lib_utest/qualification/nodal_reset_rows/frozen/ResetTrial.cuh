__global__ void ResetTrial(Control* c, std::uint64_t epoch, std::uint64_t attempt) {
  c->assembly = {};
  c->assembly.base_epoch = epoch; c->assembly.attempt = attempt;
  c->limit = {}; c->node = UINT32_MAX; c->status = NodalStatus::Ok;
  c->structural_limiter = {};
  if (stability::ResetRows(&c->rows, epoch, attempt) != sc::Status::kOk)
    c->status = NodalStatus::InvalidOutput;
}
