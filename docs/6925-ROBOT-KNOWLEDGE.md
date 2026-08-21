# Team 6925 robot knowledge (one file)

Compiled 21 Aug 2026 for **this robot only**: Woodward Academy **W.A.Robotics**, FRC **6925**, 2026 **REBUILT**. Sources are labeled. Do not mix in stock WCP dump-bin CAD, 10376’s hopper, Valor, or 6925’s **2024/2025** mechanisms.

This code **still scores the way 6925 scores**: Limelight finds a hub AprilTag → **drivetrain yaws** onto the hub → **hood servos** set launch angle from the live shot table → operator **button 1** feeds Fuel. There is **no turret**.

---

## 1. Who they are

| | |
| --- | --- |
| Name | W.A.Robotics |
| School | Woodward Academy, College Park, Georgia |
| District | Peachtree (PCH), rookie **2018** |
| Sponsor | School only (Open Alliance) |
| Site | https://warobotics.webflow.io/ (also FTC 17075 / 18547) |
| YouTube | https://www.youtube.com/@6925_W.A.Robotics |
| 2026 code they posted | https://github.com/JonathanV0/6925-Rebuilt |
| This working copy | `personal` remote `sahiljpatel2011-wq/6925-RobotCodeUpdated`, branch `n4` |
| Open Alliance 2026 | https://www.chiefdelphi.com/t/frc-team-6925-woodward-academy-robotics-2026-build-thread-open-alliance/512655 (Gavin Parker / Voidthe, 24–31 Jan 2026) |

**2026 record (The Blue Alliance / FIRST):** 21–30–0. District **#26 / 91**, **115** points.

- **Columbus (W3):** rank 6, 8–7–0, Alliance 4 captain (3329 / 4468, backup 4240), out DE R3. District Engineering Inspiration (SpaceX).
- **Albany (W4):** rank 10, 6–8–0, Alliance 6 captain. Leadership semi (Gavin P).
- **Peachtree DCMP:** rank 34, 4–8. DCMP EI + Championship qualifying.
- **Milstein (Houston):** rank 60, 3–7. Endgame **None** in every 2026 match checked — **they never climbed**.

TBA has **no 2026 CAD or team-uploaded robot photos** for 6925.

---

## 2. YouTube and match video (what you can actually see)

### Official team channel

[@6925_W.A.Robotics](https://www.youtube.com/@6925_W.A.Robotics) — team-run channel (Twitter/X also pointed here for swerve and auto path clips in earlier seasons).

**There is no 2026 robot reveal on that channel.** Latest team-posted mechanism videos are **2025 REEFSCAPE** and earlier. Those are **not** this year’s shooter/hood/intake. Use them only as team culture / shop history.

**2024 Crescendo (CRESCENDO notes, not 2026 Fuel)** — Open Alliance 2024 said shooter tests were posted on the same YouTube. That robot was a **two-Kraken speaker/amp shooter** (Colson, ~1 in compression) with an over-bumper intake and a climber. **Do not copy those RPMs or hood ideas onto the 2026 Fuel robot.**

Older team posts also mention **Swerverus** (football t-shirt cannon) and GRITS/Everybot offseason bots. Not the competition robot.

### 2026 match VODs (this year’s machine)

Georgia FRC event stream, **Columbus Quals 29**, muted/live call:

- Video: https://www.youtube.com/watch?v=v7nugBt2hx0
- TBA: https://www.thebluealliance.com/match/2026gacol_qm29
- Red: **6925 / 10376 / 7451** beat **9057 / 4112 / 9770**, **181–50**, all Fuel, **no tower, no climbs**.

Announcer / picture of play (not CAD):

- 6925 starts auto, **backs up**, puts **eight preloaded Fuel** into the hub.
- They collect from the **depot**, **line up at the hub**, and **launch Fuel** (not dump a bin).
- They **cross the bump / hump** with a load; they are **not** running the trench as a pathing strategy.
- Late match: three red robots including 6925 send **rainbows of Fuel into the hub** from midfield.
- **Do not ID 10376’s clear dump hopper as 6925.** 6925 is the shooter lining up and firing.

FIRST Championship **Milstein** streams (muted): announcers call 6925 **“the googly-eyed machine”**, collecting near the hub and **firing** Fuel. Still no climb.

TBA does not host those VODs on the team media tab. Event YouTube (Georgia FRC / FIRST Championship) is the source.

---

## 3. How they decided to build it (Open Alliance, Jan 2026)

Small team after graduation (“two new members and five returning”). Goals: stay simple, PCH competitive, fifth DCMP, train new people. 2025 Albany **Quality Award**. New batteries/chargers after 2025 voltage pain.

**Design choice (their words):** skip a custom from-scratch mechanism. **Clone the WCP 2026 Competitive Concept baseplate**, only modify as needed — **especially SDS MK4i** (stock WCP CC is SwerveX). CNC some plates in-house. Start from **WCP example code** because programmers graduated.

That is why this repo looks like WCP sample **plus** 6925 hardware:

| Stock WCP “Big Dumper” | **This 6925 robot** |
| --- | --- |
| Huge hopper, dump Fuel | **Shoots** Fuel with three flywheels + **hood** |
| SwerveX | **MK4i + Kraken**, Tuner X, CANivore |
| Often Choreo sample | **PathPlanner AutoBuilder** (not Choreo) |
| Climb / hopper deploy in sample autos | **No climb**, `hopperDeploy` / `jolt` / climb names are **no-ops** |
| Can go under trench (WCP Q&A) | **Too tall for trench** — bump ramps only |

Chief Delphi WCP CC thread: three **independent** Kraken flywheels (not one shaft). 6925 kept that: CAN **8 / 9 / 10**, motor 8 inverted vs 9/10.

---

## 4. How this robot scores (code + video)

**No turret.** Aim = **spin the swerve**. Shot shape = **hood servos** + flywheel RPM.

### Teleop shot (do not change)

1. Driver **Xbox port 0**, **right bumper**: chassis yaws with Limelight hub `tx` (`rotation = -tx * 0.15`), driver still translates on left stick (10% FieldCentric deadband, **no speed-multi inside aim**).
2. Same bumper **winds** the live table: **hood position + RPM** from Limelight range (`ty` → ground range + 23.5 in hub depth, clamp 47–110 in).
3. Operator **X3D port 1**, **button 1**: feeder + intake bounce. Flywheels **keep last shot RPM while feeding**, then idle **3000**.
4. **Y** on Xbox: trench-tag pass aim (tags **7 / 12 / 23 / 28** only, 15° inward) + **5450 RPM** / hood **0.7**.

Hood PWM **0 left / 1 right**, same sign (**do not invert**). Table hood **0.00–0.50** on live shots; closer preset **0.0**; pass preset **0.75**.

### Live shot table (Constants + 150 RPM) — do not retune

| Distance (in) | RPM | Hood |
| --- | --- | --- |
| 47 | 3500 | 0.00 |
| 50 | 3250 | 0.02 |
| 75.125 | 3500 | 0.05 |
| 84 | 3500 | 0.15 |
| 92 | 3500 | 0.30 |
| 100 | 3600 | 0.45 |
| 110 | 3650 | 0.50 |

Idle **3000** between volleys. “At speed” = all three columns within **±300 RPM**, **false at idle**.

### Aim math (keep until a range day)

- Camera: **−1.46 in** behind center, **25.39 in** up, **26°** pitch.
- Hub tag height for `ty`: **44.25 in**.
- `cameraToTag = (44.25 − 25.39) / tan(26° + ty)`, guard tan near 0°/90°.
- Add **23.5 in** hub depth and **±8 in** lateral on offset faces (3,8,19,24 = −8; 9,11,25,27 = +8). Then `atan2` for yaw.
- Limelight fiducial offset **−0.5842 m** — shots were tuned with this **on**. Do not zero.
- Hub tags only for **aim**: red **2–5, 8–11**; blue **18–21, 24–27**. Station faces **10 / 26**. Trench **1,6,7,12,17,22,23,28** are **not** hub aim (Y pass only).
- Software picks a hub tag if the camera locked a trench tag.
- Moving: existing **0.25 s** look-ahead on range + lead `tx` while strafing.

### Field map (every cycle, not only RB)

Pipeline **0** AprilTags. MegaTag1 **seeds** pose while disabled. MegaTag2 **fuses XY** in teleop/auto (gyro keeps heading). Field Map + Match Ready on SmartDashboard.

---

## 5. Hardware (this repo / Tuner X)

**Do not raise drive supply 35 A.**

| ID | Device |
| --- | --- |
| CANivore swerve 0–7 + CANcoder 0–3 | MK4i modules (TunerConstants) |
| Pigeon 2 id **0** | Gyro |
| TalonFX **8 / 9 / 10** CANivore | Three independent flywheels (8 CW, 9/10 CCW) |
| TalonFX **11** | Fuel feed |
| TalonFX **45** | Intake roller |
| TalonFX **50** | Intake rotator |
| TalonFX **51** | Feeder |
| PWM **0 / 1** | Hood servos, bounds 1000–2000 µs, 0.01–0.77 |

Default drive speed multi **0.75**. Open-loop FieldCentric voltage. Squared translation + slew; rotate ^1.5.

---

## 6. Driver / operator map (must stay exact)

Two people. **Xbox 0 drives. X3D 1 shoots.** Buttons **3 and 5 are unused** (climber removed).

**Xbox 0**

- Left stick translate, right stick rotate
- LT snap wheels 0.5 s, RT 1/5 toggle, A brake, B full speed
- LB seed field-centric heading
- **RB aim + windup only (no auto-feed)**
- Y pass
- POV up 75 in preset / down slow rotator / left-right creep
- Back/Start + X/Y SysId

**X3D 1**

- 1 Shoot + hold 1/5 drive
- 2 intake + hold 1/2 drive
- 4 retract, 6 deploy
- 7 closer windup (3350 / 0.0), 8 pass windup (3500 / 0.75), 9 close, 10 snap, 11 test hood, 12 retract oscillate
- Hat down reverse, hat left exposure tune

---

## 7. Bugs that were real (and stay fixed)

These were match-breakers vs original `n4`. They are **logic**, not new shot numbers — no range-day retest required.

| Bug | What it did on the robot | Fix |
| --- | --- | --- |
| Idle overwrote shot RPM while feeding | Flywheels dumped to 3000 as Fuel went in | Hold last shot RPM while feeder commanded on |
| At-speed only motor 8 / true at idle | Auto/teleop thought it was ready when it was not | All three columns, false at idle |
| Feeder `TalonFX.get()` | `isFeeding()` lied | Use commanded feeder state |
| PathPlanner one shared named-command instance | Autos hung | `NamedCommandRegistry` + `Commands.defer` |
| MegaTag2 yaw rate 0 | Bad pose | Pigeon yaw rate + pose yaw |
| Alliance tag filter written every cycle | Camera “went funny”, tags vanished | **Do not** rewrite crop/IMU/stream/tag-filter from code |
| Pose fuse rejected first jump | Odometry stuck at origin | VisionGates allow first jump off origin |
| Aim used trench tags | Aimed at the wrong wall | Hub-only software aim |
| Auto-feed on RB (briefly added) | Changed two-person flow | Removed — still operator 1 |

**Intentionally not changed (would force retest):** live RPM/hood table, ±8 in lateral, −0.5842 m offset, 26° camera, 35 A, idle 3000, aim **kP 0.15** (D is in Constants but live aim is still `-tx * kP`), hood servo direction, bindings.

Dashboard FeatureFlags and the NT shot table **cannot** change those numbers mid-match.

---

## 8. Autos (PathPlanner)

Default chooser **M-S**. Sequential names must finish. Aliases: `intakeStop` = `StopIntake`, `hopperDeploy` / climb / `jolt` = none, `hoodReset` = hood **0**.

Too tall for trench: bump-ramp or hub-front carpet autos only. No climb auto.

---

## 9. Pit checklist

Limelight hostname **`limelight`**, **pipeline 0** AprilTags / MegaTag2, 2026 field map on the camera. Point at tags while **disabled** until **VisionSeeded** and **Match Ready**. Pick an auto that is not **None**. Xbox 0 + stick 1. Battery ≥ 12 V.

---

## 10. What this file is not

- Not stock WCP Big Dumper dump-hopper CAD.
- Not 10376 Plutonium Panthers’ hopper in Columbus Q29.
- Not 6925’s 2024 speaker/amp YouTube tests.
- Not physics of foam Fuel. Sim “makes” a shot when the **table commanded** RPM/hood.

Primary sources: this repo, OA 2026 posts, TBA/FIRST 2026, Georgia FRC Columbus Q29 VOD, team YouTube (historical only).

**Formulas (do not retune):** [6925-MATH.md](6925-MATH.md)
