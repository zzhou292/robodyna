"""Verify the immutable original 1504-cell rubber source fixture."""
import hashlib
from pathlib import Path

root=Path(__file__).resolve().parent
records={
    'YarisRubberSourceFixture.h': (493851,'871f4457fb833c4c0c3c4b2a4929d5d421c43469b0befb75a7b390505e1cdbd2'),
    'source-manifest.json': (17991,'807d4975a8246c8f5d67207114c82b0b30b4f173fc9798aee3e146131a62970a'),
}
for name,(size,digest) in records.items():
    value=(root/name).read_bytes()
    if len(value)!=size or hashlib.sha256(value).hexdigest()!=digest:
        raise RuntimeError('Original rubber fixture changed: '+name)
print('Verified original 1504 cells /2308 nodes and source-card receipts')
