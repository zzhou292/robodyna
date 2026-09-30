# Physical runtime source

`Source::Original` retains the existing execution, physical attachment and optional
joint wrappers after their original authority checks. `Source::WithEnvironment`
retains the qualified combined source, including its genuine fixed-wall domain,
execution, CIN witnesses and complete joint model. A private immutable variant
owns their lifetimes; public construction cannot supply arbitrary borrowed TL
views. The shared accessors create no numerical state.

The existing startup, participant and dynamics implementations consume these
views. Original-only accessors reject the environment source explicitly. Source
handles outlive all GPU consumers. The environment producer reports its retained graph separately from historical
construction peaks, with exact shared witness backing checks. No guessed overlap
or unused module reservation is charged as a live runtime allocation.
