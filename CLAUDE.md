# Instructions for Claude

Project-specific preferences and reference info. Claude Code reads this automatically each session.

## Working preferences

- **Deploy to the robot whenever it's available.** After making code changes, build and deploy to
  the roboRIO without asking first. If the robot isn't connected, just say so.
- **Write code like you're explaining it to a 5-year-old** — clear, simple, and concise. Prefer
  obvious code over clever code.
- **Keep comments few and simple.** Comment only what isn't obvious from the code; keep each one
  short and plain. Don't over-explain.
- **Always update [`README.md`](README.md) when functionality changes** — add, remove, or change a
  button binding / behavior, and the README's controls and behavior sections must be updated in the
  same change.

## Build & deploy

Uses the WPILib toolchain (bundled JDK). WPILib doesn't put Java on PATH, so set `JAVA_HOME` when
building from the command line:

```sh
JAVA_HOME="/c/Users/Public/wpilib/2026/jdk" ./gradlew deploy --offline   # build + deploy to roboRIO
JAVA_HOME="/c/Users/Public/wpilib/2026/jdk" ./gradlew compileJava --offline   # compile only
```

## Hardware & configuration

**Team 4296.** Details below aren't fully derivable from the code — treat as reference.

### Control & compute
- **roboRIO v1** — main robot controller (too weak to run AprilTag detection itself).
- **OrangePi 5** coprocessor running **PhotonVision**, with two cameras plugged directly into the
  board's own USB ports (no hub — a powered hub was tried and caused bandwidth-related "Camera
  Lost" errors at higher resolutions, confirmed 2026-08-07):
  - **Arducam OV9281** (global shutter) for AprilTags. Must be named **`OV9281_April_Tags`** in
    the PhotonVision UI (must match `VisionConstants.kCameraName` in code). Replaced the original
    Logitech C920 to fix rolling-shutter motion blur during the search-and-align spin.
  - **Arducam OV9782** for Object Detection (RKNN models on the RK3588S's NPU), nicknamed
    `OV9782_Object_Detection` in PhotonVision. Not currently wired into robot code - PhotonVision
    only, no `frc.robot` integration yet.
  - AprilTag pipeline: 36h11, 2026 field layout. The 2026 season has two layouts (welded vs
    AndyMark) — verify which applies before an event.
- **Driver controller:** Thrustmaster T.16000M flight stick on USB port 0.

### Network
- Radio: **Vivid-Hosting VH-109** (1 PoE uplink + 4 LAN ports). roboRIO and OrangePi each plug into
  a LAN port (roboRIO v1 has only one Ethernet port, used for the radio uplink).
- Static IPs: **roboRIO `10.42.96.2`**, **OrangePi `10.42.96.11`** (team `10.TE.AM.x` subnet).
- OrangePi is powered via a 12V→5V buck regulator fed by PoE from the VH-109 radio.
- PhotonVision SSH default creds: user `pi`, password `raspberry`.

### Drivetrain (see [`generated/TunerConstants.java`](src/main/java/frc/robot/generated/TunerConstants.java))
- Swerve: SDS **Mk5n** modules — **Kraken X60** drive, **Kraken X44** steer, CTRE **CANcoder**
  azimuth encoder, CTRE **Pigeon 2** gyro. All on the roboRIO onboard CAN bus.
- CAN ID convention: steer motors 0–3, drive motors 4–7, CANcoders 11–14, Pigeon 20.
- Measured top speed ~5.85 m/s at 12 V. Wheelbase/trackwidth 19.75 in × 19.75 in, 4 in wheels.
- **Note:** `createDrivetrain()` currently instantiates only two modules (FrontRight + BackLeft) —
  a bench/test configuration, not the full 4-module robot. Re-enable all four before full driving.
