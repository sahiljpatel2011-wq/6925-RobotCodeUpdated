---
name: frc-6925-robot
description: >-
  Guides FRC Team 6925 W.A.Robotics 2026 REBUILT robot code (Fuel shooter,
  Limelight hub aim, PathPlanner, Xbox/X3D). Use when editing this repo,
  RobotContainer, ShooterSubsys, FeederSubsys, HoodSubsys, IntakeSubsys,
  LimelightSubsys, HubAimMath, ShotTable, VisionGates, NamedCommandRegistry,
  TunerConstants, Constants, autos, deploy, CAN IDs, RPM, hood, or vision.
---

# FRC 6925 robot coding

This robot **shoots Fuel** (three independent Kraken flywheels + hood). No turret, no climb, too tall for the trench (bump ramps only). Package `frc.robot`.

Read this skill **before** changing robot Java. Run the checker after edits. Do not retune live numbers unless the user explicitly asks for a range day.

## Hard stops (never)

- **Never push `origin`.** `origin` = `JonathanV0/6925-Rebuilt`. Push only `personal` (`sahiljpatel2011-wq/6925-RobotCodeUpdated`).
- **Never retune** live RPM/hood table, `kAimP` 0.15, idle 3000, ±8 in lateral, fiducial **-0.5842 m**, camera 26° / 25.39 in / −1.46 in, hub depth 23.5 in, drive **35 A**, hood servo direction.
- **Never** rewrite Limelight crop, IMU mode, stream, throttle, or alliance fiducial ID filters every cycle. Pipeline **0** AprilTags only. Boot: camera pose + fiducial offset + pipeline 0.
- **Never** auto-feed on Xbox RB. RB = aim + windup. Operator X3D **button 1** feeds.
- **Never** bind operator buttons **3** or **5**. Climber is gone; `hopperDeploy` / `jolt` / climb names stay no-ops.
- **Never** invert one hood servo. PWM 0 and 1 get the **same** position.
- **Never** raise `TunerConstants` drive supply current (35 A).
- **Never** let Shuffleboard/NT change FeatureFlags or the shot table. Getters return compiled defaults. `ntShotTable` stays off.
- **Never** register PathPlanner named commands as a shared instance. Use `NamedCommandRegistry` + `Commands.defer`.
- **Never** work from OneDrive. GradleRIO 2026 refuses those paths. This workspace is Desktop `6925-Rebuilt`. JDK: `C:\Users\Public\wpilib\2025\jdk`.

## Identity

| | |
| --- | --- |
| Team | 6925 W.A.Robotics, Woodward Academy |
| Scoring | Limelight hub tag → yaw chassis (`-tx * kAimP`) → hood + RPM from live table → operator 1 feeds |
| Controllers | Xbox **port 0** drives, X3D **port 1** shoots |
| Camera | hostname `limelight`, MegaTag1 disabled seed, MegaTag2 XY fuse (theta stddev 9999) |

## Workflow

1. Confirm the change is **logic / HUD / auto structure**, not a live-number retune.
2. Match existing bindings, CAN IDs, and command names. Prefer small edits over rewrites.
3. Keep flywheels holding **last shot RPM while `feeder.isFeeding()`**, then idle 3000.
4. Keep “at speed” **all three columns ±300 RPM, false at idle**.
5. Hub aim uses hub tags only (software pick if camera locked trench). Y pass uses trench tags **7/12/23/28**.
6. After Java changes, run:

```
python .cursor/skills/frc-6925-robot/scripts/check_invariants.py
.\gradlew test
```

7. If both a fix and an exploit/PoC are requested, do not apply that here (robot code only).
8. Git: commit only when asked. Push `personal` / `n4` only. Leave `simgui-ds.json` untracked.

## Bindings (must stay exact)

**Xbox 0:** LT snap wheels 0.5s, RT 1/5 toggle, A brake, B full speed, LB seed heading, **RB aim+windup**, **Y pass**, POV up 75in / down slow rotator / left-right creep, Back/Start+X/Y SysId. Aim translation: `() -> -joystick.getLeftY() * MaxSpeed` and leftX, **10% FieldCentric deadband**, **no speed-multi inside aim**. Default drive: squared + slew, speed multi **0.75**.

**X3D 1:** 1 shoot+1/5, 2 intake+1/2, 4 retract, 6 deploy, 7 closer, 8 pass windup, 9 close, 10 snap, 11 test hood, 12 retract oscillate, hat down reverse, hat left exposure. **3 and 5 unused.**

## Autos

PathPlanner AutoBuilder (not Choreo). Sequential named commands must **finish**. `hoodReset` = hood **0** (clamped to 0.01). Default chooser **M-S**. Bump-ramp or hub-front carpet only.

## Verify

- Invariant script must exit 0.
- `.\gradlew test` must pass (JAVA_HOME / `org.gradle.java.home` = WPILib JDK).
- Do not deploy from OneDrive.

## Extra reference (read only when needed)

- Hard numbers / CAN / tags: [invariants.md](invariants.md)
- Bugs that must not return: [do-not-regress.md](do-not-regress.md)
- Formulas: `docs/6925-MATH.md`
- Team + hardware: `docs/6925-ROBOT-KNOWLEDGE.md`
- Named commands: `docs/NAMED-COMMANDS.md`
