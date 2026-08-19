# PathPlanner named commands (6925)

All names are registered with `Commands.defer` so each schedule gets a fresh command. Climber / hopper dump names stay no-ops — this robot **shoots** Fuel.

## Shoot / feed

| Name | Finishes? | Notes |
| --- | --- | --- |
| `shoot` | No (hold) | Feeder + intake bounce. Use as a path event, not a sequential step. |
| `shootHold` | No | Alias of `shoot`. |
| `autoShoot` | Yes (4 s) | Sequential autos. Does **not** stop flywheels; returns hold to 3000 after the volley. |
| `StopFeed` | Yes | Feeder off + shooter idle 3000. |

## Windup

| Name | Finishes? | Notes |
| --- | --- | --- |
| `windUp` / `windUpOnce` | Yes | Snap 3350 / default hood. Sequential-safe. |
| `autoWindUp` / `autoWindUpClose` / `autoWindUpCloser` | Yes (≤2 s) | Wait until all three columns are in the 300 RPM band. |
| `AdjustedWindUp` | No | Continuous table RPM from pose. Path events / deadlines only. |
| `AdjustedWindUpOnce` | Yes (≤2 s) | Snap table RPM, wait at speed. Use in sequential autos. |
| `AdjustedShootWhileMoving` | No | Deadline with a path. Keep `StopFeed` after. |

## Intake

| Name | Alias | Notes |
| --- | --- | --- |
| `StopIntake` | `intakeStop` | `LB-NZI-RB-S.path` uses `intakeStop`. |
| `IntakeFast` | `IntakeFast` | |
| `IntakeMid` | | |
| `intakeDeploy` | `intakeDeploy` | Rotator to -14.5. |
| `intakeBounce` | | No-op (bounce is inside `autoShoot`). |

## No-ops (motor removed)

`jolt`, `ClimbUp`, `ClimbDown`, `climbDown`, `StopClimber`, `hopperDeploy`, `hopperDeploy`, `VisionUpdate`.

## Hood

`hoodReset` / `hoodReset` → hood 0.

## New autos

- **Taxi-Shoot** — mild accel taxi, `AdjustedWindUpOnce` + `autoShoot`.
- **Center-VisionShot** — repaired hub-front shot with vision in auto.
- **Bump-2Piece** — bump ramp, not trench. No climb.
