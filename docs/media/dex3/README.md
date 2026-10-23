# Complete Dex3 contact-only grasp recordings

Each animation shows the entire ten-second tabletop sequence: approach, close,
lift, hold, lower and release. The wrist is moved by actuated slide joints in a
mechanical test rig. The object has a free joint, no equality constraint, and
no manually applied force. The fingers support it through contact and friction.

Both hands use the pinned official Unitree Dex3 geometry and the same seven-axis
closing targets as the ROS grasp controller. Each GIF contains 101 frames at
10 fps, plus a one-second final outcome hold. Adjacent JSON files record the
grasp result, source/result hashes and complete frame coverage.

This is complete hand-bench evidence, not a complete autonomous delivery mission.
The full-robot pickup trial and retained delivery failures are described in the
[Dex3 migration report](../../DEX3_CONTACT_MANIPULATION_20261020.md).
