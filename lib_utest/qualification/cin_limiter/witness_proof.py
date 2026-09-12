"""Checked additive diagnostic view, with every mechanics line retained."""
def replace_once(text, addition, prior=''):
    assert text.count(addition) == 1, addition
    return text.replace(addition, prior, 1)

def legacy_owner(text):
    text = replace_once(text, '#include "cin_limiter/Capture.h"\n')
    return replace_once(text, '''    if (structural.capture_limiter)
      control->structural_limiter = cin_limiter::Capture(sources, structural.factor,
          result, epoch, attempt);
''')

def legacy_kernels(text):
    text = replace_once(text, '#include "../cin_limiter/Capture.h"\n')
    return replace_once(text, '''  if (input.structural.capture_limiter)
    input.control->structural_limiter = cin_limiter::Capture(source, input.structural.factor,
        result, input.epoch, input.attempt);
''')

def legacy_values(text):
    text = replace_once(text, '    std::uint32_t group, double* trace_upper = nullptr) noexcept {',
                        '    std::uint32_t group) noexcept {')
    return replace_once(text, '  if (trace_upper) *trace_upper = trace;\n')
