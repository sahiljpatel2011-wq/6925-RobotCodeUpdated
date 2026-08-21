# 6925 W.A.Robotics — 2026 REBUILT

Woodward Academy, College Park, GA. This robot **shoots Fuel** with three independent Kraken flywheels. It is not a dump-bin, does not climb, and is too tall for the trench (use bump ramps).

Full robot knowledge (YouTube, OA, hardware, aim): [docs/6925-ROBOT-KNOWLEDGE.md](docs/6925-ROBOT-KNOWLEDGE.md). **All formulas:** [docs/6925-MATH.md](docs/6925-MATH.md). Named commands: [docs/NAMED-COMMANDS.md](docs/NAMED-COMMANDS.md).

## How it scores

Xbox **right bumper** aims at hub tags and winds the live shot table. Operator **button 1** feeds. Flywheels idle at **3000 RPM** between volleys and **hold the last shot RPM while feeding**.

## CAN / PWM

| ID | Device |
| --- | --- |
| CANivore 8 / 9 / 10 | Shooter columns |
| 11 | Fuel feed |
| 45 | Intake roller |
| 50 | Intake rotator |
| 51 | Feeder |
| PWM 0 / 1 | Hood servos |
| Pigeon 0 + swerve | See `TunerConstants` — **do not raise 35 A drive supply** |

## Driver / operator (code, not the old header)

**Xbox 0:** left stick drive, right stick turn, LB seed heading, RB hub aim+windup, Y pass, A brake, B full speed, RT 1/5 speed, LT snap wheels 0.5 s, POV up 75 in preset.

**X3D 1:** 1 feed, 2 intake, 4 retract, 6 deploy, 7 closer windup, 8 pass windup, 9 close, 10 snap wheels, 11 test hood, 12 retract oscillate, hat down reverse, hat left exposure. Buttons **3 and 5 unused**.

## Voltage rules

Keep 35 A drive, 3000 idle between shots, open-loop teleop. Do not spool from 0 on a high-accel path. FeatureFlags are compiled defaults (Dashboard cannot change a match).

## Limelight

Name `limelight`. Pipeline **0 only** = 2026 AprilTags / MegaTag2 (XY-only fusion, MegaTag1 disabled seed). No Fuel neural pipeline in code. Fiducial offset **-0.5842 m** stays until a range day. RB uses the hood table + drivetrain yaw (`-tx * 0.15`).

## Build / sim (this PC)

WPILib VS Code + Cursor. Tasks in `.vscode/tasks.json`.

```
.\gradlew test
.\gradlew build
.\gradlew simulateJava
.\gradlew deploy
```

Sim: Glass / AdvantageScope + PathPlanner GUI. Field JSON `edu/wpi/first/fields/2026-rebuilt.json`. Not SimScale. Shot “success” in sim means the table commanded the right RPM/hood.

## What this upgrade fixed vs left alone

**Fixed:** shooter idle overwriting shot RPM; 3-column at-speed; auto named-command deadlocks; vision in auto; MegaTag heading loop; hub-tag aim + tags 3/19; pass requires drive; deferred named commands; telemetry/Field2d; stale operator header.

**Not changed:** live RPM/hood table, ±8 in lateral, fiducial offset, 35 A drive, 300 RPM band, bumper + button 1 UX, climber no-ops.
