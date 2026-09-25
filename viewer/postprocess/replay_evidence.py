"""Require an executed exact Chrono replay test before starting capture."""
from output.gtest_evidence import require_completed_test

EXACT_REPLAY_TEST = (
    "PhysicalSceneArchive."
    "ExactOriginalArchiveWallActivityAndFailedSeekPreserveDisplay"
)


def require_exact_replay(report):
    """The exact Chrono identity stays fixed; callers cannot substitute a test."""
    require_completed_test(report, EXACT_REPLAY_TEST)
