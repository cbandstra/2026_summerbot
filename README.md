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
| 2 | Target lock | **Tap** (under 1 second) to toggle on/off — spins in place looking for any AprilTag (fast for 15° at a time, then slower for 0.75s between each pulse — never a full stop — so the camera gets a steadier look), then auto-align rotation to it once seen, hands-free until tapped again. If the tag is lost while under 1 meter away, it's almost certainly just out of frame from being too close, not actually gone — target lock holds still instead of spinning off looking for a different one; see Force spin (button 3) to override that. **Hold** it instead to activate the same behavior only while held, exactly like a plain hold button — always ends off on release, even if it was already toggled on. You still steer translation with the stick either way; only rotation is taken over. Logs "Looking for April tags" while searching. Turns off automatically if the robot is disabled. |
| 3 | Force spin | **Hold** — turns target lock on hands-free (same as tapping button 2) if it isn't already running, then, once target lock finds a tag it would normally stop spinning and smoothly align to it; holding this button instead forces the pulsed search spin to keep happening even though a tag is in view, as if none had been found. Also the only way to make target lock resume searching after it's held still for a lost close-up tag (see button 2). If target lock is currently suspended (see buttons 8/9 - released with no tag back in view yet), pressing this clears the suspension immediately and performs its usual search, instead of doing nothing until a tag reappears on its own. Releasing it goes straight back to aligning if a tag is still visible. Only button 2 can turn target lock back off — this button never disables it. |
| 4 | Force spin (clockwise) | **Press** — a single tap is enough, it doesn't need to be held. Turns target lock on hands-free (same as tapping button 2) if it isn't already running, same as button 3, including clearing a suspension (see button 3) immediately instead of waiting for a tag to reappear. Forces the search to spin clockwise instead of the default counterclockwise, ignoring whatever tag's currently in view (even one target lock's already aligned to) — same as button 3's override, just spinning the other way, and overrides holding still for a lost close-up tag too. Once a tag's actually lost and then reacquired, it locks onto that tag normally. Press it again to re-arm the clockwise override. Any *later* search — from this button, button 3, or losing that tag naturally — defaults back to counterclockwise. Only button 2 can turn target lock back off — this button never disables it. |
| 7 | Recenter | **Press** — makes wherever the robot is currently facing the new "forward" for field-centric driving. Doesn't move the robot or change its tracked field position, just which way the stick's forward points from now on. Useful if the robot was hand-placed at an angle, or you want to redefine forward mid-match. |
| 8 | Drive toward locked target | **Hold** — the held version of the autonomous "drive toward target" instruction: drives forward, correcting aim toward whichever AprilTag's in view the whole time it's visible, for as long as the button's held instead of a fixed distance. Takes over the stick completely (translation included) while held. Speed scales directly with distance to the tag: `kDriveTowardMinSpeedMps` (0.6 m/s floor, also used when no tag's in view yet since distance isn't known) plus distance times a gain that itself steps at 2 m away (`kDriveTowardNearDistanceMeters`) - `kDriveTowardFarGain` (1.0) past that, `kDriveTowardNearGain` (0.7) closer than that - capped at `kDriveTowardMaxSpeedPercent` (90%) of the drivetrain's true top speed (~5.85 m/s, so a ~5.3 m/s cap). Speed changes (including the initial ramp-up from a stop) are ramped rather than instant (`kDriveTowardSlewMpsPerSec` — avoids spinning the wheels). Releasing it stops the robot; if target lock was on, it's suspended (not turned off) until an AprilTag's back in view, so it doesn't immediately spin off searching the instant you let go — once a tag's seen again it resumes normally on its own. If target lock was already off, releasing does nothing extra. Takes priority over buttons 9 and 10 if more than one is held. |
| 9 | Drive away from locked target | **Hold** — the held version of the autonomous "drive away from target" instruction: drives backward at a fixed speed (`kDriveTowardBackupSpeedMps`), ramped up from a stop instead of snapping straight there (`kDriveTowardSlewMpsPerSec` — avoids spinning the wheels), correcting aim toward whichever AprilTag's in view the whole time it's visible, for as long as the button's held instead of a fixed distance. Takes over the stick completely (translation included) while held. Releasing it stops the robot; if target lock was on, it's suspended (not turned off) until an AprilTag's back in view, so it doesn't immediately spin off searching the instant you let go — once a tag's seen again it resumes normally on its own. If target lock was already off, releasing does nothing extra. If button 8 is also held, button 8 takes priority and this does nothing. |
| 10 | Line up with locked target | **Hold** — the held version of the autonomous "align with april tag ... and go to it" instruction: searches for, then drives up to and squares up on, whichever tag is currently best-seen (the same tag target lock would align to), continuing to refine (or hold position once centered) for as long as it's held. Takes over the stick completely while held. Releasing it stops the robot immediately. If button 8 or 9 is also held, that one takes priority and this does nothing. |

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
| `drive toward target <number> feet` | `drive toward target 5 feet` | Drives forward (robot-relative, whichever way the robot's currently facing). Meant to follow a `rotate` step that aimed the robot roughly at a target: while any AprilTag is in view (not a specific ID - whichever one's currently seen) it corrects aim toward it as it drives, same as `align with april tag`'s rotation correction; once the tag goes out of view (e.g. too close for the camera to see the whole thing) it stops correcting and just keeps driving straight for the rest of the distance, rather than searching for it. `toward`/`towards` both work. |
| `drive away from target <number> feet` | `drive away from target 5 feet` | Same as `drive toward target`, but backward — simply reverses the wheel direction rather than re-pointing them, same relationship as `drive backward` to `drive forward`. Still corrects aim toward whichever AprilTag is in view the whole time, even while backing away from it, using the same rules as `drive toward target` for when it stops correcting. |
| `rotate <number> degrees` | `rotate 90 degrees` | Turns in place. A bare number is signed: positive = counterclockwise, negative = clockwise (matches the rest of the codebase's convention). You can instead say the direction explicitly, either before or after the number: `rotate left/right/clockwise/counterclockwise <number> degrees` or `rotate <number> degrees left/right/clockwise/counterclockwise`. `degree`/`degrees` both work. |
| `wait <number> seconds` | `wait 1.5 seconds` | Just sits still. `second`/`seconds` both work. |
| `align with april tag <number>` | `align with april tag #4` | Spins (in the same fast/slow pulse pattern as target lock) to search for that specific AprilTag ID, then turns to face it. Also accepts `align to april tag 4` (with or without the `#`). Gives up after a few seconds if the tag is never found, so a missing tag can't stall the rest of the script. |
| `align with april tag <number> and go to it` | `align with april tag 1 and go to it` | Spins to search for that specific AprilTag ID (same as `align with april tag`), then drives at it until it's about 1 meter away - fast while there's real ground to cover, slowing down within 2 meters of the tag so the final approach doesn't come in too hot to fine-tune alignment. While still approaching, it aims at the tag's center and nudges sideways toward square; once stopped, it switches to correcting squareness by rotating and centering by strafing directly, rather than only aiming at the tag's center. "Centered" is judged from the tag's own pose, not just yaw, against targets measured on the actual robot (the camera isn't mounted perfectly square/centered, so dead-on doesn't read as exactly 180°/0). Gives up after a few seconds if it never gets there, so a missing tag can't stall the rest of the script. Tunable in `AutoConstants` (`kLineUp*`). Also accepts `align to april tag <number> and go to it`. |
| `recenter` | `recenter` | Same as button 7 - makes wherever the robot is currently facing the new "forward" for field-centric driving. Doesn't move the robot or change its tracked field position, just which way "forward" points from then on for any `drive`/`drive toward`/`drive away` steps after it. |
| `vision rotation test` | `vision rotation test` | A diagnostic step, not a real match instruction - see [Vision rotation test](#vision-rotation-test) below. Usually the only thing in the script while it's there, since it doesn't return control until it's completely done. |
| `play beep <number> times` | `play beep 3 times` | Plays the same beep normally played after a step finishes (tone/duration tunable via `kStepCompleteBeepHz`/`kStepCompleteBeepSeconds`, same as always), back to back this many times. Always plays, even if `kStepCompleteBeepEnabled` is `false` - the whole point of this step is to beep - but doesn't also get the normal automatic trailing beep after it, so it's not one extra time on top. `time`/`times` both work. |

A line that's blank, or starts with `#` or `//` (leading whitespace is fine), is a comment and is
skipped — handy for disabling a step without deleting it.

If a line doesn't match one of these exactly (and isn't a comment), **the whole file is rejected**
(logged to the RioLog console with the exact bad line) and autonomous does nothing that run,
rather than guessing at a typo and running a partial or wrong script.

Each step beeps once it finishes, played through one of the drivetrain's own Kraken motors (no
extra speaker hardware needed) — tone and duration are tunable via `kStepCompleteBeepHz`/
`kStepCompleteBeepSeconds` in `AutoConstants`. Set `kStepCompleteBeepEnabled` to `false` there to
skip the beep entirely (not just silence it) and speed up cycle times, since every step normally
waits for its beep to fully finish before the next one starts.

All the speeds/tolerances used above (drive speed, rotate speed, align tolerance, align timeout)
are tunable in `AutoConstants` in [`Constants.java`](src/main/java/frc/robot/Constants.java) — the
script only controls direction/distance/angle/tag ID, not speed.

New instruction types are added in [`AutoScript.java`](src/main/java/frc/robot/AutoScript.java)
(the text parser) and [`AutoStep.java`](src/main/java/frc/robot/AutoStep.java) (the parsed data),
with the actual robot behavior for each wired up in `RobotContainer.autoStepCommand()`.

#### Vision rotation test

The `vision rotation test` instruction is a diagnostic for finding how fast the robot can spin
while still reliably seeing AprilTags. Set the field up with tags visible from wherever the robot
starts, put `vision rotation test` in `autonomous.json`, then run autonomous like normal (a match
isn't needed - works fine standalone in the autonomous period).

It runs a series of full 360° rotations, each faster than the last, counting how many *unique*
tag IDs came into view during each one, logging the duration and count to the RioLog console
after every rotation. All other routine console logging (target lock, "April Tags in view",
etc.) is silenced for the whole test so only these result lines show up - restored automatically
once the test ends, however it ends:

1. **Steady-speed sequences** — spin at one constant rate, starting at 0.5 rad/s (0.1 turned out
   to be too far below the wheels' static friction threshold to actually move at all). As long as
   the tag count doesn't drop from the previous sequence, repeats with the speed increased by
   another 0.1 rad/s. Once it drops (or the speed maxes out at the drivetrain's actual top turn
   rate), moves on to...
2. **Pulse sequences** — same idea, but spinning in target lock's fast/slow pulse pattern instead
   of one steady rate, starting at 1.0 rad/s fast / 0.5 rad/s slow and increasing both by 0.5/0.1
   rad/s each time. Ends the whole test once the tag count drops here too (or the fast speed maxes
   out).

The console logs a line the instant each sequence starts, not just once it finishes, so it's
clear the test is progressing even during a slow rotation. Once the whole test ends (either way),
it logs the single best sequence seen across the entire run - whichever one saw the most unique
tags, using duration as a tiebreaker if more than one sequence tied for the top tag count. Before
every measured rotation, the robot first spins slowly until no tag is in view (or 10 seconds pass,
in case a tag's visible from every angle), so each measurement starts from the same clean state.
All the starting speeds/increments are tunable in `RotationTestConstants` in
[`Constants.java`](src/main/java/frc/robot/Constants.java).

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
doesn't overshoot on stale frames. When no tag is in view, the robot searches by alternating
fast/slow spin pulses (never a full stop) instead of spinning continuously at one speed - set
`kSearchPulseEnabled` to `false` to spin at one steady rate instead. Alignment PID gains and the
search pulse's speeds/degrees/duration live in `VisionConstants` in
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
| [`PulsedSearch.java`](src/main/java/frc/robot/PulsedSearch.java) | Fast/slow pulsed search-spin pattern - shared by target lock, "align with april tag", and the rotation test. |
| [`VisionRotationTest.java`](src/main/java/frc/robot/VisionRotationTest.java) | Builds the "vision rotation test" diagnostic step. |
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
