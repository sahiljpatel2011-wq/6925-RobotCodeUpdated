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

| `hoodReset` | `hoodReset` | Hood **0** (clamped to servo min 0.01). |

## New autos

- **Taxi-Shoot** — drive the taxi path (table windup as a path event), then `AdjustedWindUpOnce` at the shoot pose, then `autoShoot`.
- **Center-VisionShot** / **M-S** — hub-front path first (range updates on the path), then `AdjustedWindUpOnce` + `autoShoot`.
- **Bump-2Piece** — bump ramp collect, `StopIntake`, `RB-S` pointed at the hub, then windup + shoot. No climb.

## Hub autos (no bump)

All stay on alliance-side carpet in front of the hub (`x ≤ 3.2 m` on blue). They do **not** cross the bump ramps (`x ≈ 3.96–5.09 m`) or the trench. PathPlanner folder **Hub-NoBump**. Flip for red.

| Auto / path | Start (blue) | Shot pose | Table range |
| --- | --- | --- | --- |
| `Center-Hub` | wall center `(0.70, 4.04)` | `(2.72, 4.04)` | ~75 in |
| `Close-Hub` | wall center | `(3.15, 4.04)` | ~58 in |
| `Far-Hub` | wall center | `(2.08, 4.04)` | ~100 in |
| `Left-Hub` | left wall `(0.70, 6.85)` | same 75 in pose | curves in on carpet |
| `Right-Hub` | right wall `(0.70, 1.20)` | same 75 in pose | curves in on carpet |
| `Alley-Left-Hub` | left of hub `(2.40, 5.10)` | 75 in pose | short align |
| `Alley-Right-Hub` | right of hub `(2.40, 2.95)` | 75 in pose | short align |

Each auto: path (points at hub, `AdjustedWindUp` on the way) → `AdjustedWindUpOnce` → `autoShoot` → `StopFeed`.
