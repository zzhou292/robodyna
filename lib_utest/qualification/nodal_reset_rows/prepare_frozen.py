"""Adapt the complete frozen reset without changing its statements."""
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent


def generated():
    source = (HERE / 'frozen/ExplicitStepStability.h').read_text()
    begin = source.index('struct RowBounds {')
    end = source.index('\n};', begin) + len('\n};')
    # The same public POD lets the complete old kernel operate on today's
    # Control. Only this type declaration and the namespace are adapted.
    source = source[:begin] + 'using RowBounds = ::tl::fea::stability::RowBounds;' + source[end:]
    source = source.replace('tl::fea::stability', 'tl::fea::reset_frozen_stability', 1)
    source = source.replace('}  // namespace tl::fea::stability',
                            '}  // namespace tl::fea::reset_frozen_stability')
    # Avoid ADL through the reused RowBounds POD in the unrelated finalizer.
    source = source.replace('InvalidateRows(rows)',
                            '::tl::fea::reset_frozen_stability::InvalidateRows(rows)')
    return '#include "lib_src/solvers/ExplicitStepStability.h"\n' + source


if __name__ == '__main__':
    output = Path(sys.argv[1])
    output.mkdir(parents=True, exist_ok=True)
    (output / 'FrozenStability.h').write_text(generated())
