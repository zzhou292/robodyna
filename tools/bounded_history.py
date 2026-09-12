"""Bound retained guard telemetry without changing live limit evaluation.

Append owns each fresh sample record; callers must not mutate it afterward.
The first samples and rolling tail stay chronological and never overlap.
Whole-run counters include samples that are later evicted from the tail.
"""

from collections import deque


DEFAULT_CAPACITY = 16384
DEFAULT_FIRST_SAMPLES = 2048


class SampleHistory:
    def __init__(self, capacity=DEFAULT_CAPACITY,
                 first_samples=DEFAULT_FIRST_SAMPLES):
        if (type(capacity) is not int or type(first_samples) is not int or
                not 0 <= first_samples < capacity):
            raise ValueError('sample history requires 0 <= first samples < capacity')
        self.capacity = capacity
        self.first_capacity = first_samples
        self.first = []
        self.last = deque(maxlen=capacity - first_samples)
        self.total_samples = 0
        self.peak_rss_bytes = 0

    def append(self, sample):
        self.total_samples += 1
        self.peak_rss_bytes = max(self.peak_rss_bytes, sample['rss_bytes'])
        if len(self.first) < self.first_capacity:
            self.first.append(sample)
        else:
            self.last.append(sample)

    def samples(self):
        return self.first + list(self.last)

    def metadata(self):
        retained = len(self.first) + len(self.last)
        dropped = self.total_samples - retained
        return dict(policy='first_and_rolling_last_v1',
                    capacity=self.capacity,
                    first_capacity=self.first_capacity,
                    last_capacity=self.last.maxlen,
                    total_samples=self.total_samples,
                    retained_samples=retained,
                    dropped_samples=dropped,
                    retained_first_samples=len(self.first),
                    retained_last_samples=len(self.last),
                    complete_trace=dropped == 0)
