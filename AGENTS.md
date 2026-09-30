# Robodyna development

Read docs/migration/EXECUTION.md and docs/architecture/MODULES.md first. Current
work is a source-preserving migration, not a new numerical implementation.

- One product and Bazel root. Imported compatibility sources are first-party
  transition backends; preserve all inherited functionality and provenance.
- FEA and MBD are peer modules. Share neutral mechanics contracts; mixed assembly
  and concrete adapters depend on the domains. Do not split the qualified Yaris
  FE/rigid/CIN owner or add a second stepper as a directory reorganization.
- Reuse existing algorithms/utilities. Preserve target-specific arithmetic flags,
  state ownership, rollback and device-view lifetimes. CUDA coverage requires tests.
- Keep interfaces and implementation modular. Make changes at owning boundaries;
  avoid broad reformatting, mass symbol renames or a giant umbrella source file.
- Source imports are immutable checkpoints. Record later build/API edits separately.
  Preserve original license notices, class/archive identity and required fixtures.
- Use focused verification. Heavy builds and GPU runs share the workstation lock;
  follow the enclosing workspace guard settings. Other GPU jobs remain running.
- Local commits/branches are authorized. Do not push without explicit publication
  authorization. Do not modify or reset preserved source worktrees or run evidence.
