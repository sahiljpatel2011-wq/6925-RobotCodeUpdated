# Team 6925 research (REBUILT 2026)

Ground truth for **this robot only**: this repo, 6925 Open Alliance (five January 2026 posts), TBA/FIRST for 6925, 6925’s YouTube, and muted 2026 match VODs of 6925. WCP sample CAD, 10376’s dump hopper, Valor, and generic “2026 dumper” notes are **not** 6925.

## Identity

- **Name:** W.A.Robotics, Woodward Academy, College Park, Georgia
- **FIRST:** Peachtree (PCH), rookie 2018, school is the only sponsor
- **Site:** https://warobotics.webflow.io/ — FTC 17075 and 18547
- **2026 code they posted:** https://github.com/JonathanV0/6925-Rebuilt (this workspace)
- **Org GitHub listed on the site:** https://github.com/WARbotics
- **OA thread:** https://www.chiefdelphi.com/t/frc-team-6925-woodward-academy-robotics-2026-build-thread-open-alliance/512655 — five posts, 24–31 Jan 2026, author Voidthe (Gavin Parker). No competition recaps, no finished CAD, no match photos.

## 2026 record (TBA / FIRST)

- Official **21-30-0**. District **#26 / 91**, **115** points.
- TBA has **no 2026 photos or CAD** for 6925.
- **Columbus (W3):** 38 district pts (17+13+8). Rank 6, **8-7-0**. Alliance 4 captain: 3329 / 4468, backup 4240. Out DE R3. District EI (SpaceX); DCMP Leadership semi (Rhythm B).
- **Albany (W4):** 26 pts (15+11). Rank 10, **6-8-0**. Alliance 6 captain: 3635 / 7538. Out DE R2. Leadership semi (Gavin P).
- **Peachtree DCMP:** 51 pts (27+24). Rank 34, **4-8**. DCMP EI + Championship qualifying.
- **Milstein:** rank 60, 3-7. Endgame **None** every 2026 match checked — they never climbed.

## How they built it (OA, Jan 2026)

Small team after graduation (“two new members and five returning”). They skipped a custom mechanism, **cloned the WCP CC baseplate**, modified it for **SDS MK4i** (stock CC is SwerveX), and started from **WCP example code**. Old batteries were a 2025 failure mode; they bought new batteries/chargers. That is why drive stays at **35 A**, idle **3000 RPM**, and teleop stays open-loop voltage.

This repo is **WPILib command-based + PathPlannerLib AutoBuilder**, not Choreo.

## What this robot actually is (code + muted VODs)

**Shooter, not a dump bin.** Three independent Kraken flywheels CAN 8/9/10, dual-servo hood PWM 0/1, feeder 51 + fuel 11. Ground roller 45 + rotator 50. One Limelight named `limelight`. No turret: yaw the chassis. `hopperDeploy` / climb / jolt are `Commands.none()`.

- [Columbus Quals 29](https://www.youtube.com/watch?v=v7nugBt2hx0) (Georgia FRC, muted): red **6925 / 10376 / 7451** beat **9057 / 4112 / 9770**, TBA **181–50**, all Fuel, no tower, no climbs. Alliance **shoots Fuel into the hub**. Do **not** ID 10376’s labeled clear hopper as 6925.
- FIRST Championship Milstein Day 3 (muted): announcers call 6925 **“the googly-eyed machine”**, collecting near the hub and firing Fuel.
- Official YouTube [@6925_W.A.Robotics](https://www.youtube.com/@6925_W.A.Robotics): 151 subs, 71 videos, **no 2026 reveal**. Latest is 2025 REEFSCAPE. Do not treat those mechanisms as this robot.

Too tall for the trench (code header); they use **bump ramps**.

## Driver UX (keep)

Xbox right bumper = hub aim + live-table windup. Operator button 1 = feed/oscillate. Y = pass. Do not replace with one-button auto-fire as the default.

## Live shot table (what aim actually commands)

Interpolator is **Constants + 150 RPM**. Do not replace with physics.

| Distance (in) | RPM | Hood |
| --- | --- | --- |
| 47 | 3500 | 0.00 |
| 50 | 3250 | 0.02 |
| 75.125 | 3500 | 0.05 |
| 84 | 3500 | 0.15 |
| 92 | 3500 | 0.30 |
| 100 | 3600 | 0.45 |
| 110 | 3650 | 0.50 |

Idle 3000 between volleys. Hold last shot RPM **while feeding**. After the volley, return to 3000.

## Aim / camera (keep until tape)

- Camera: −1.46 in X, 0 Y, 25.39 in up, **26°** pitch (header still says 20° — ignore).
- Hub tag height for ty-range: **44.25 in**. Hub depth in software: **23.5 in**.
- Limelight `fiducial_offset` **−0.5842 m**. Shots were tuned with this on. **Do not zero** until a range day.
- Lateral **±8.0 in** on offset hub tags (keep 8, not drawing ~14 in, until tape). Hub IDs include 3 and 19 with the same 8 in signs.
- Aim P 0.15, D 0.01 (enable D, do not change P).

`cameraToTag = (44.25 − 25.39) / tan(26° + ty)`, then add 23.5 in depth and ±8 in lateral, `atan2` for yaw. Guard tan near 0°/90°. If no hub tag, pose-to-hub for both range and heading.

## Field / FUEL (manual)

- FUEL: ~5.91 in, ~0.215 kg foam. 504 per match, preload up to 8.
- Carpet ~ 317.7 × 651.2 in. Hub lip 72 in. Blue hub XY 182.105 / 158.845 in, red 469.115 / 158.845 in.
- Hub tags 2–5, 8–11 (red) and 18–21, 24–27 (blue) at 44.25 in. Trench 1,6,7,12,17,22,23,28 at 35 in.

## Limelight setup (on the camera)

- Pipeline 0: 2026 REBUILT map, MegaTag2, Light/Medium black level, stream low. Crop hopper/ceiling of **this** robot so onboard Fuel is not a detector target.
- Pipeline 1: official Fuel B1 neural detector. Hold operator 3 to yaw to the largest detection. Default off.
- `SetRobotOrientation` from **Pigeon / field yaw**, never fused pose. Fuse **XY only**.

## Simulation on this PC

WPILib `.\gradlew simulateJava` + Glass / AdvantageScope + PathPlanner GUI. Field JSON: `edu/wpi/first/fields/2026-rebuilt.json`. **Not SimScale.** Shot success in sim means “commanded RPM/hood from the table,” not foam aerodynamics.

## Voltage rules (not bugs)

Drive supply **35 A** / LowerLimit 60, steer 25 A. Do not raise. Independent flywheel loops. Climber named commands stay `none()`.
