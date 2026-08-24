# 6925 hard numbers (do not retune)

Source of truth in code: `Constants.java`, `HubAimMath.java`, `ShotTable.java`, `Landmarks.java`, `TunerConstants.java`. This file is a checklist for the agent, not a second table to edit instead of code.

## CAN / PWM

| ID | Device |
| --- | --- |
| 8 / 9 / 10 | Flywheels (8 inverted vs 9/10) |
| 11 | Fuel feed |
| 45 | Intake roller |
| 50 | Intake rotator |
| 51 | Feeder |
| PWM 0 / 1 | Hood servos, same sign |
| Pigeon 0 | Gyro |
| Swerve | TunerConstants — drive supply **35 A**, steer **25 A** |

## Shooter

| Item | Value |
| --- | --- |
| Idle | 3000 RPM |
| Live table | Constants RPM **+150** (`ShotTable.kLiveRpmOffset`) |
| Clamp | 47–110 in |
| At-speed | all three columns **±300 RPM**, false at idle |
| Feeder isFeeding | commanded state, not `TalonFX.get()` |
| Aim | `-tx * kAimP` with `kAimP = 0.15` (kD unused) |
| Look-ahead | 0.25 s |
| If tag lost >0.15 s | yaw tx → 0 |

Live table (Constants + 150): 47"→3500/0.00, 50"→3250/0.02, 75.125"→3500/0.05, 84"→3500/0.15, 92"→3500/0.30, 100"→3600/0.45, 110"→3650/0.50.

Presets (not interpolator): closer 3350/0.0, close 3350/0.3, pass 3500/0.75, full-field pass 5450/0.7, POV-up 75" uses Constants 3350/0.05 (no +150).

## Camera / field

| Item | Value |
| --- | --- |
| Camera XY | −1.46 in X, 0 Y, 25.39 in up, 26° pitch |
| Hub tag height | 44.25 in |
| Hub depth | 23.5 in |
| Lateral | ±8 in (3,8,19,24 = −8; 9,11,25,27 = +8) |
| Fiducial 3D offset | −0.5842 m (MegaTag pose, not added into ty range) |
| Blue hub | 182.105 / 158.845 in |
| Red hub | 469.115 / 158.845 in |

Hub tags: red `2,3,4,5,8,9,10,11`; blue `18,19,20,21,24,25,26,27`; station faces 26 / 10. Trench `1,6,7,12,17,22,23,28` — Y pass only (live pass 7/12/23/28).

## FeatureFlags (compiled)

`visionInAuto=true`, `hubTagAimFilter=true`, `autoshootFeed=false`, `ntShotTable=false`, `visionPoseFuse=true`. Getters must ignore Dashboard.

## Git remotes

- `origin` → `JonathanV0/6925-Rebuilt` — fetch only
- `personal` → `sahiljpatel2011-wq/6925-RobotCodeUpdated` — push `n4` here
- Do not commit `simgui-ds.json`
