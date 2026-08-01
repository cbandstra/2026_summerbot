# 2026 Summer Bot

FRC swerve-drive robot code (Team 4296 hardware) built on WPILib command-based Java and CTRE
Phoenix 6, with PhotonVision AprilTag alignment. Driven with a Thrustmaster T.16000M flight stick.

## Controls

Driver station USB port 0, Thrustmaster T.16000M. **Verify axis/button numbers on the Driver
Station's USB Devices tab before driving** — the indices below are the stick's standard USB report
order, not guaranteed on every setup.

### Axes (always active — normal field-centric driving)

| Control | Axis | Action |
|---|---|---|
| Stick forward / back | Y (1) | Drive forward / backward |
| Stick left / right | X (0) | Strafe left / right |
| Stick twist | Twist (2) | Rotate in place |
| Throttle slider | Slider (3) | Sets top translation speed live — **all the way back = fastest, all the way forward = slowest/safest** |

Driving is **field-centric**: forward on the stick always drives away from the driver station
regardless of which way the robot is facing.

### Buttons

| Button | Name | Behavior |
|---|---|---|
| 1 | Trigger | **Hold** — lock the wheels in an X pattern (brake; resists being pushed). |
| 2 | Thumb | **Hold** — spin in place looking for any AprilTag, then auto-align rotation to it once seen. You still steer translation with the stick; only rotation is taken over. Logs "Looking for April tags" while searching. |
| 3 | — | Unbound (this align behavior used to live here). |
| 4 | — | Unbound. |

### Autonomous

No autonomous routine is wired up — the robot sits idle during the autonomous period.

## Driver-feel details

These are all tunable in [`Constants.java`](src/main/java/frc/robot/Constants.java) under
`OperatorConstants`:

- **Deadband** — stick deflection below ~10% produces no output (translation and rotation).
- **Response curve** — above the deadband, output is `stickFraction ^ 2.0`, softening low-speed
  response for finer control while full deflection still reaches top speed.
- **Throttle slider** — scales the translation top speed between 5% and 100% of the drivetrain's
  true top speed.
- **Minimum output floor** — once past the deadband, translation is floored to ~5% of true top
  speed so the wheels always move usefully (clamped so it never exceeds the slider's current cap).
- **Input smoothing** — a slew-rate limiter on each axis filters flight-stick pot noise to stop
  steer-motor chatter.

## Vision

A single PhotonVision camera (named **`front cam`** in the PhotonVision UI) provides AprilTag
targets. Yaw readings are latency-compensated against the robot's odometry history so alignment
doesn't overshoot on stale frames. Alignment PID gains and the search-spin rate live in
`VisionConstants` in [`Constants.java`](src/main/java/frc/robot/Constants.java).

## Project layout

| File | Purpose |
|---|---|
| [`RobotContainer.java`](src/main/java/frc/robot/RobotContainer.java) | Subsystems, button bindings, driving/align math. |
| [`Constants.java`](src/main/java/frc/robot/Constants.java) | Tunable operator + vision constants. |
| [`subsystems/Vision.java`](src/main/java/frc/robot/subsystems/Vision.java) | PhotonVision camera wrapper. |
| [`subsystems/CommandSwerveDrivetrain.java`](src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java) | CTRE swerve drivetrain. |
| [`generated/TunerConstants.java`](src/main/java/frc/robot/generated/TunerConstants.java) | Swerve hardware config (CAN IDs, gear ratios, offsets). |
| [`Robot.java`](src/main/java/frc/robot/Robot.java) | Mode lifecycle + scheduler. |

## Build & deploy

Uses the WPILib toolchain (bundled JDK). From the WPILib VS Code extension, or on the command line:

```sh
./gradlew build      # compile + tests
./gradlew deploy     # build and deploy to the roboRIO (robot must be connected)
./gradlew simulateJava   # run in simulation
```
