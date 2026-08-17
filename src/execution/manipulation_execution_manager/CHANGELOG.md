# Changelog

All notable user-visible changes are documented here.

## Unreleased

- Removed internal milestone, fake-source, and one-off validation executables
  from the public package.
- Replaced development-stage launch files with
  `execution_manager.launch.py`.
- Rewrote architecture, ROS contract, and configuration documentation around
  the stable public API.
- Removed the controller-specific debug safety monitor and direct-to-controller
  Twist relay. Their future ownership and safety requirements are documented in
  `docs/future_safety_monitor.md`.

## 0.1.0

Initial public release candidate.

- Standard ROS 2 Jazzy `ament_python` package.
- Multi-source validation, normalization, priority arbitration, takeover, and
  stale-source fallback.
- Mutually exclusive joint-space and task-space streaming routes.
- Independent `FollowJointTrajectory` goal forwarding.
- Versioned JSON execution status schema.
- Independent diagnostic safety monitor.
- Strict JSON source configuration with startup validation.
- Unit, release-contract, clean-build, and installed-package test coverage.
