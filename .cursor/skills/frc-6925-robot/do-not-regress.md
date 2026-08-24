# Bugs that stay fixed

Do not reintroduce these. They were match-breakers vs Jonathan `n4` (`ba957fb`). Logic only — not new shot numbers.

| Bug | Regression looks like | Keep |
| --- | --- | --- |
| Idle overwrites shot RPM while feeding | Flywheels drop to 3000 as Fuel enters | Hold last shot RPM while `feeder.isFeeding()`; default command uses `holdRPM` |
| At-speed only motor 8 / true at idle | Auto fires or skips wait | Three columns ±300, `isIdleHold()` → false |
| Feeder `TalonFX.get()` | `isFeeding()` lies | Commanded `FeederSpeed` |
| Shared PathPlanner named command | Autos hang on second marker | `NamedCommandRegistry` + `Commands.defer` |
| MegaTag2 yaw rate 0 | Bad pose while turning | Pigeon yaw rate + pose yaw; XY fuse only |
| Rewrite crop/IMU/tag-filter every cycle | Camera “funny”, tags vanish | Boot pose + offset + pipeline 0 only |
| Pose fuse rejects first jump | Odometry stuck at origin | `VisionGates.allowVisionJump` from unseeded pose |
| Aim uses trench tags | Yaws at the wrong wall | `HubAimMath` hub-only pick |
| Auto-feed on RB | Operator 1 unused | `autoshootFeed` false; RB `aimAndWindUp` only |
| Alliance unknown → silent red hub | Aims wrong color | `Landmarks.targetPositionOptional()` empty until FMS alliance |

Intentionally unchanged (would force a range day): live table, lateral 8 in, fiducial −0.5842 m, 26° camera, 35 A, idle 3000, kP 0.15, hood direction, Xbox/X3D map.
