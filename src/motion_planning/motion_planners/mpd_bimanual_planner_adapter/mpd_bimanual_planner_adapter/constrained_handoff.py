from __future__ import annotations


def can_handoff_bimanual(*, velocities, closure_translation_error_m, closure_rotation_error_rad, velocity_tolerance=0.02, translation_tolerance=0.002, rotation_tolerance=0.01745):
    return max(abs(float(value)) for value in velocities) <= velocity_tolerance and closure_translation_error_m <= translation_tolerance and closure_rotation_error_rad <= rotation_tolerance
