#!/usr/bin/env python3
"""Fail if 6925 hard invariants drifted. Run from anywhere in the repo."""

from __future__ import annotations

import re
import sys
from pathlib import Path

FAILS: list[str] = []
PASSES: list[str] = []


def repo_root() -> Path:
    here = Path(__file__).resolve()
    for parent in [here, *here.parents]:
        if (parent / "build.gradle").is_file() and (parent / "src/main/java/frc/robot").is_dir():
            return parent
    raise SystemExit("Could not find 6925 repo root (build.gradle + src/main/java/frc/robot)")


def read(root: Path, rel: str) -> str:
    path = root / rel
    if not path.is_file():
        FAILS.append(f"missing file: {rel}")
        return ""
    return path.read_text(encoding="utf-8", errors="replace")


def must_contain(rel: str, text: str, needle: str, label: str) -> None:
    if needle in text:
        PASSES.append(label)
    else:
        FAILS.append(f"{rel}: expected `{needle}` ({label})")


def must_regex(rel: str, text: str, pattern: str, label: str) -> None:
    if re.search(pattern, text):
        PASSES.append(label)
    else:
        FAILS.append(f"{rel}: expected /{pattern}/ ({label})")


def must_absent(rel: str, text: str, needle: str, label: str) -> None:
    if needle in text:
        FAILS.append(f"{rel}: forbidden `{needle}` ({label})")
    else:
        PASSES.append(label)


def java_team_files(root: Path) -> list[Path]:
    base = root / "src/main/java/frc/robot"
    return [p for p in base.rglob("*.java") if p.name != "LimelightHelpers.java"]


def main() -> int:
    root = repo_root()

    constants = read(root, "src/main/java/frc/robot/Constants.java")
    must_contain("Constants.java", constants, "kIdleRPM = 3000", "idle 3000")
    must_contain("Constants.java", constants, "kAimP = 0.15", "aim kP 0.15")
    must_contain("Constants.java", constants, "kLookAheadSeconds = 0.25", "look-ahead 0.25")
    must_contain("Constants.java", constants, "kHubCenterOffsetInches = 23.5", "hub depth 23.5")
    must_contain("Constants.java", constants, "kPassAimOffsetDegrees = 15.0", "pass inward 15")

    tuner = read(root, "src/main/java/frc/robot/generated/TunerConstants.java")
    must_regex("TunerConstants.java", tuner, r"withSupplyCurrentLimit\(\s*35\s*\)", "drive 35 A")

    shot = read(root, "src/main/java/frc/robot/vision/ShotTable.java")
    must_contain("ShotTable.java", shot, "kLiveRpmOffset = 150.0", "live table +150 RPM")
    must_contain("ShotTable.java", shot, "return compiled(distance);", "get() uses compiled table")

    flags = read(root, "src/main/java/frc/robot/FeatureFlags.java")
    must_contain("FeatureFlags.java", flags, "DEFAULT_AUTOSHOOT_FEED = false", "no auto-feed")
    must_contain("FeatureFlags.java", flags, "DEFAULT_NT_SHOT_TABLE = false", "no NT shot table")
    must_contain("FeatureFlags.java", flags, "return DEFAULT_AUTOSHOOT_FEED", "autoshootFeed compiled")
    must_contain("FeatureFlags.java", flags, "return DEFAULT_NT_SHOT_TABLE", "ntShotTable compiled")

    landmarks = read(root, "src/main/java/frc/robot/Landmarks.java")
    must_contain("Landmarks.java", landmarks, "182.105", "blue hub X")
    must_contain("Landmarks.java", landmarks, "469.115", "red hub X")
    must_contain("Landmarks.java", landmarks, "158.845", "hub Y")

    aim = read(root, "src/main/java/frc/robot/vision/HubAimMath.java")
    must_contain("HubAimMath.java", aim, "kCameraHeightInches = 25.39", "camera height")
    must_contain("HubAimMath.java", aim, "kCameraMountAngleDegrees = 26.0", "camera 26 deg")
    must_contain("HubAimMath.java", aim, "kLateralOffsetInches = 8.0", "lateral 8 in")
    must_contain("HubAimMath.java", aim, "{2, 3, 4, 5, 8, 9, 10, 11}", "red hub tags")
    must_contain("HubAimMath.java", aim, "{18, 19, 20, 21, 24, 25, 26, 27}", "blue hub tags")
    must_contain("HubAimMath.java", aim, "{1, 6, 7, 12, 17, 22, 23, 28}", "trench tags")
    must_contain("HubAimMath.java", aim, "return -txDegrees * kP;", "omega = -tx * kP")

    ll = read(root, "src/main/java/frc/robot/subsystems/LimelightSubsys.java")
    must_contain("LimelightSubsys.java", ll, "kCameraForwardInches = -1.46", "camera X")
    must_contain("LimelightSubsys.java", ll, "setFiducial3DOffset(name, -0.5842, 0.0, 0.0)", "fiducial offset")

    shooter = read(root, "src/main/java/frc/robot/subsystems/ShooterSubsys.java")
    must_contain("ShooterSubsys.java", shooter, 'new TalonFX(8, "CANivore")', "shooter CAN 8")
    must_contain("ShooterSubsys.java", shooter, 'new TalonFX(9, "CANivore")', "shooter CAN 9")
    must_contain("ShooterSubsys.java", shooter, 'new TalonFX(10, "CANivore")', "shooter CAN 10")
    must_contain("ShooterSubsys.java", shooter, "setVelocityRPM(holdRPM)", "default holds last RPM")
    must_contain("ShooterSubsys.java", shooter, "isIdleHold()", "at-speed false at idle")
    must_contain("ShooterSubsys.java", shooter, "kVelocityToleranceRPM = 300.0", "+/-300 RPM band")

    feeder = read(root, "src/main/java/frc/robot/subsystems/FeederSubsys.java")
    must_contain("FeederSubsys.java", feeder, 'new TalonFX(51, "CANivore")', "feeder CAN 51")
    must_contain("FeederSubsys.java", feeder, 'new TalonFX(11, "CANivore")', "fuel CAN 11")
    must_contain("FeederSubsys.java", feeder, "commanded == FeederSpeed.FEED_FAST", "isFeeding commanded")

    intake = read(root, "src/main/java/frc/robot/subsystems/IntakeSubsys.java")
    must_contain("IntakeSubsys.java", intake, 'new TalonFX(45, "CANivore")', "intake CAN 45")
    must_contain("IntakeSubsys.java", intake, 'new TalonFX(50, "CANivore")', "rotator CAN 50")

    hood = read(root, "src/main/java/frc/robot/subsystems/HoodSubsys.java")
    must_contain("HoodSubsys.java", hood, "kLeftServoPWM = 0", "hood PWM 0")
    must_contain("HoodSubsys.java", hood, "kRightServoPWM = 1", "hood PWM 1")
    must_contain("HoodSubsys.java", hood, "leftServo.set(smoothedPosition)", "left hood same sign")
    must_contain("HoodSubsys.java", hood, "rightServo.set(smoothedPosition)", "right hood same sign")
    must_absent("HoodSubsys.java", hood, "1.0 - smoothedPosition", "do not invert hood")

    container = read(root, "src/main/java/frc/robot/RobotContainer.java")
    must_contain("RobotContainer.java", container, "RobotCommands.aimAndWindUp(", "RB aim+windup")
    must_contain("RobotContainer.java", container, "operator.button(1).whileTrue(RobotCommands.Shoot())", "op 1 feeds")
    must_absent("RobotContainer.java", container, "operator.button(3)", "op 3 unused")
    must_absent("RobotContainer.java", container, "operator.button(5)", "op 5 unused")
    must_contain("RobotContainer.java", container, "VisionGates.allowVisionJump", "first vision jump allowed")

    commands = read(root, "src/main/java/frc/robot/RobotCommands.java")
    must_contain("RobotCommands.java", commands, "if (!feederSubsys.isFeeding())", "hold RPM while feeding")
    must_contain("RobotCommands.java", commands, "HubAimMath.aimAssistOmega(tx, kAimP)", "aim uses kP")

    named = read(root, "src/main/java/frc/robot/NamedCommandRegistry.java")
    must_contain("NamedCommandRegistry.java", named, "Commands.defer(factory", "named commands deferred")

    gradle_props = read(root, "gradle.properties")
    must_absent("gradle.properties", gradle_props, "OneDrive", "JDK not on OneDrive")
    must_contain("gradle.properties", gradle_props, "wpilib", "WPILib public JDK")

    forbidden_calls = (
        "setCropWindow(",
        "setIMUMode(",
        "setStreamMode_Standard(",
        "setStreamMode_PiPMain(",
        "setStreamMode_PiPSecondary(",
        "setFiducialIDFiltersOverride(",
    )
    for path in java_team_files(root):
        text = path.read_text(encoding="utf-8", errors="replace")
        rel = path.relative_to(root).as_posix()
        for call in forbidden_calls:
            if call in text:
                FAILS.append(f"{rel}: forbidden Limelight override {call}")

    print(f"6925 invariant check - {root}")
    for line in PASSES:
        print(f"  OK  {line}")
    if FAILS:
        print()
        for line in FAILS:
            print(f"  FAIL  {line}")
        print(f"\n{len(FAILS)} failed, {len(PASSES)} passed")
        return 1
    print(f"\nAll {len(PASSES)} checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
