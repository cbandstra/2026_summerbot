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
regardless of which way the robot is facing. This relies on the robot knowing which way is
"forward" in the first place — the very first time the robot is enabled after powering on, it
automatically treats whatever direction it's currently facing as forward, so make sure it's
pointed away from the driver station before that first enable. Use the Recenter button
(below) any time after that to redefine forward again, e.g. after it's been hand-placed at
an angle.

### Buttons

| Button | Name | Behavior |
|---|---|---|
| 1 | Trigger | **Hold** — lock the wheels in an X pattern (brake; resists being pushed). |
| 2 | Target lock | **Tap** (under 1 second) to toggle on/off — spins in place looking for any AprilTag (in 60° pulses, pausing 0.5s between each so the camera gets a steady look), then auto-align rotation to it once seen, hands-free until tapped again. **Hold** it instead to activate the same behavior only while held, exactly like a plain hold button — always ends off on release, even if it was already toggled on. You still steer translation with the stick either way; only rotation is taken over. Logs "Looking for April tags" while searching. Turns off automatically if the robot is disabled. |
| 3 | Force spin | **Hold** — only has an effect while target lock (button 2) is active. Normally, once target lock finds a tag it stops spinning and smoothly aligns to it; holding this button instead forces the pulsed search spin to keep happening even though a tag is in view, as if none had been found. Releasing it goes straight back to aligning if a tag is still visible. |
| 4 | Recenter | **Press** — makes wherever the robot is currently facing the new "forward" for field-centric driving. Doesn't move the robot or change its tracked field position, just which way the stick's forward points from now on. Useful if the robot was hand-placed at an angle, or you want to redefine forward mid-match. |

### Autonomous

Autonomous is scripted in [`src/main/deploy/autonomous.json`](src/main/deploy/autonomous.json) — a
JSON array of plain-English instruction strings, run in order top to bottom. Editing it doesn't
need a code change, just a redeploy (drag the file's content changes over with `./gradlew deploy`
or the WPILib VS Code "Deploy Robot Code" command).

Example:

```json
[
  "drive forward 3 feet",
  "wait 1 seconds",
  "rotate 90 degrees",
  "drive right 2 feet",
  "align with april tag 4"
]
```

**Supported instructions** (not case-sensitive):

| Instruction | Example | Notes |
|---|---|---|
| `drive <forward\|backward\|left\|right> <number> feet` | `drive forward 3 feet` | Distance is measured from odometry, not timed — accurate regardless of battery voltage or carpet friction. `foot`/`feet` both work. Direction is robot-relative (whichever way the robot is facing when the step starts), not field-relative. |
| `rotate <number> degrees` | `rotate 90 degrees` | Turns in place. A bare number is signed: positive = counterclockwise, negative = clockwise (matches the rest of the codebase's convention). You can instead write `rotate left <number> degrees` or `rotate right <number> degrees`, which is usually clearer. `degree`/`degrees` both work. |
| `wait <number> seconds` | `wait 1.5 seconds` | Just sits still. `second`/`seconds` both work. |
| `align with april tag <number>` | `align with april tag #4` | Spins (in the same 60°-pulse, 0.5s-pause pattern as target lock) to search for that specific AprilTag ID, then turns to face it. Also accepts `align to april tag 4` (with or without the `#`). Gives up after a few seconds if the tag is never found, so a missing tag can't stall the rest of the script. |

If a line doesn't match one of these exactly, **the whole file is rejected** (logged to the RioLog
console with the exact bad line) and autonomous does nothing that run, rather than guessing at a
typo and running a partial or wrong script.

All the speeds/tolerances used above (drive speed, rotate speed, align tolerance, align timeout)
are tunable in `AutoConstants` in [`Constants.java`](src/main/java/frc/robot/Constants.java) — the
script only controls direction/distance/angle/tag ID, not speed.

New instruction types are added in [`AutoScript.java`](src/main/java/frc/robot/AutoScript.java)
(the text parser) and [`AutoStep.java`](src/main/java/frc/robot/AutoStep.java) (the parsed data),
with the actual robot behavior for each wired up in `RobotContainer.autoStepCommand()`.

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

A single PhotonVision camera (named **`OV9281_April_Tags`** in the PhotonVision UI) provides
AprilTag targets. Yaw readings are latency-compensated against the robot's odometry history so alignment
doesn't overshoot on stale frames. When no tag is in view, the robot searches by spinning in
pulses (60° of spin, then a 0.5s pause, repeated) rather than spinning continuously. Alignment PID
gains and the search pulse's spin rate/degrees/pause live in `VisionConstants` in
[`Constants.java`](src/main/java/frc/robot/Constants.java).

### Driver camera

A Logitech C920 plugged directly into the roboRIO's own USB port streams to the Driver Station as
a plain video feed for the human driver — separate from the PhotonVision cameras above, which are
for vision processing, not for a person to look at. It shows up automatically in the Driver
Station's camera tab (or a Camera Stream widget in Shuffleboard/Elastic), no extra setup needed.
Kept to a low resolution/fps (set in [`Robot.java`](src/main/java/frc/robot/Robot.java)) since
match radio bandwidth is limited and shared with everything else the robot sends.

## Project layout

| File | Purpose |
|---|---|
| [`RobotContainer.java`](src/main/java/frc/robot/RobotContainer.java) | Subsystems, button bindings, driving/align math, autonomous command building. |
| [`Constants.java`](src/main/java/frc/robot/Constants.java) | Tunable operator, vision, and autonomous constants. |
| [`AutoScript.java`](src/main/java/frc/robot/AutoScript.java) | Parses `deploy/autonomous.json` into `AutoStep`s. |
| [`AutoStep.java`](src/main/java/frc/robot/AutoStep.java) | The parsed autonomous instruction types. |
| [`deploy/autonomous.json`](src/main/deploy/autonomous.json) | The autonomous script itself — edit this, not code. |
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
