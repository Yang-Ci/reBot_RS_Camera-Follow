"""Compatibility imports; new code uses robot_follow.runtime.diagnostics."""
from robot_follow.runtime.diagnostics import (
    ActualFeedbackSampler, DIAG_FIELDS, DiagnosticRecorder, feedback_age_ms,
)

__all__ = ['ActualFeedbackSampler', 'DIAG_FIELDS', 'DiagnosticRecorder', 'feedback_age_ms']
