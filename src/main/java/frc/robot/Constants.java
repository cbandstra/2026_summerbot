// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    // Thrustmaster T.16000M raw HID axis indices (standard USB joystick report order).
    // Verify against the Driver Station's USB Devices tab before relying on these -
    // move each axis/twist/slider individually and confirm which channel number moves.
    public static final int kThrustmasterXAxis = 0; // stick left(-1)/right(+1)
    public static final int kThrustmasterYAxis = 1; // stick forward(-1)/back(+1)
    public static final int kThrustmasterTwistAxis = 2; // stick rotate CCW(-1)/CW(+1)
    public static final int kThrustmasterSliderAxis = 3; // throttle slider

    // Button numbers (1-indexed, as reported by WPILib), verify against Driver Station.
    public static final int kThrustmasterTriggerButton = 1;
    public static final int kThrustmasterThumbButton = 2;
    public static final int kThrustmasterSearchButton = 3;
    public static final int kThrustmasterTagSearchButton = 4;

    // Deadband is a fraction of the CURRENT slider-selected top speed - below this much stick
    // deflection, translation output is zero, regardless of where the slider is set.
    public static final double kTranslationDeadband = 0.10;
    public static final double kRotationDeadband = 0.10;

    // Shapes the response between the deadband edge and full stick deflection: output fraction
    // = stickFraction ^ kTranslationCurveExponent. At 1.0 the response is linear (unchanged).
    // Above 1.0, small stick movements near the deadband produce proportionally less speed than
    // linear would, giving finer control at low speed, while full deflection still reaches
    // maxSpeed exactly - this is what makes "100% slider" driving usable at low speeds instead
    // of every small push feeling like a lot of speed. 2.0 (squared) is a common starting point;
    // go higher (e.g. 3.0) for even gentler low-speed response.
    public static final double kTranslationCurveExponent = 2.0;

    // Same idea as kTranslationCurveExponent above, but for the twist axis: output fraction =
    // stickFraction ^ kRotationCurveExponent above the deadband. No floor here (unlike
    // translation) since low-speed rotation being weak isn't a usability problem the way
    // barely-moving wheels is - the ask here was specifically to tone down rotation at low
    // input, not guarantee a minimum.
    public static final double kRotationCurveExponent = 2.0;

    // Once stick deflection is past the deadband above, translation output is floored to at
    // least this fraction of the drivetrain's TRUE top speed (not the slider-scaled speed) -
    // below this absolute speed the wheels don't move usefully, so there's no reason to ever
    // command less than this once the driver is actively asking for movement. This is an
    // absolute floor, unlike the deadband above, so it does not shrink when the slider is
    // turned down. If the slider's cap is below this floor, the floor is clamped to the slider's
    // cap instead (see RobotContainer) so commands never exceed what the slider currently allows.
    public static final double kMinOutputPercent = 0.05;

    // Translation top speed is set live by the throttle slider (kThrustmasterSliderAxis), mapped
    // from its full range [-1, 1] to [kSliderMinSpeedPercent, kSliderMaxSpeedPercent] of the
    // drivetrain's true top speed - slider all the way back (-1) is full speed (kSliderMaxSpeedPercent),
    // all the way forward (+1) is the slowest/safest setting (kSliderMinSpeedPercent). Keep this
    // at or above kMinOutputPercent, or the slider's low end becomes unreachable (see above).
    public static final double kSliderMinSpeedPercent = 0.05;
    public static final double kSliderMaxSpeedPercent = 1.00;

    // Rotation has no slider control - it's capped at a fixed fraction of the drivetrain's true
    // top rotational speed, higher than the slider's minimum translation cap since spinning in
    // place is inherently less dangerous than driving into something at speed.
    public static final double kMaxRotationOutputPercent = .3;
  }

  public static class VisionConstants {
    // Name assigned to the C920 in the PhotonVision web UI's Cameras tab - must match exactly.
    public static final String kCameraName = "C920_1";

    // PID gains for rotating the robot to center the best-seen AprilTag in frame (see
    // RobotContainer's button 2 binding). Input/setpoint are yaw error in degrees, output is
    // rad/s. Start with P-only and tune kP on the robot; add kD if the heading oscillates
    // before settling rather than converging smoothly.
    public static final double kAlignRotationKP = 0.06;
    public static final double kAlignRotationKI = 0.0;
    public static final double kAlignRotationKD = 0.0;

    // Within this many degrees of dead-center, the target is considered "aligned" - not
    // currently gating anything, but available via PIDController's atSetpoint() for a future
    // command that should finish once aligned.
    public static final double kAlignYawToleranceDegrees = 1.5;

    // Absolute search-spin rate (rad/s) while holding the search button with no tag visible -
    // NOT scaled off the drivetrain's MaxAngularRate, so it stays put regardless of drivetrain
    // tuning. Kept deliberately below full speed even though the blur math at the current
    // exposure setting allows faster - user preference, not a camera limitation.
    public static final double kSearchRotationRadPerSec = 1.0;

    // Priority order of AprilTag IDs to look for when holding the tag-search-and-approach button
    // (button 4) - tries each ID in order and targets whichever is found first, so if more than
    // one is visible at once the earlier ID in this list wins. Edit this array and redeploy to
    // change which tags it looks for; no runtime input for this yet.
    public static final int[] kSearchTagIdOrder = {1, 2, 3, 4};

    // How far (meters) to stop from the target tag once approaching it - measured along the
    // ground plane (hypot of the camera-to-target transform's X/Y, ignoring the height
    // difference between camera and tag) rather than full 3D line-of-sight distance, since
    // "32 inches away" means 32 inches away on the floor, not slant distance.
    public static final double kApproachDistanceMeters = 0.8128; // 32 inches

    // The tag-search button won't start closing distance until the (latency-compensated) yaw
    // error is within this many degrees of dead-center - keeps it from driving toward the tag at
    // a steep angle while still mid-rotation, only moving forward once roughly squared up.
    public static final double kApproachYawToleranceDegrees = 30.0;

    // The tag-search button considers itself "arrived" once within this many meters of
    // kApproachDistanceMeters and within kAlignYawToleranceDegrees of squared-up - at that point
    // it holds station for kApproachSettleSeconds before finishing on its own.
    public static final double kApproachDistanceToleranceMeters = 0.05; // ~2 inches

    // How many degrees the TAG ITSELF may be turned away from facing the camera head-on
    // (distinct from kAlignYawToleranceDegrees, which is about the tag's left/right position in
    // frame, not its own facing angle) - re-enabled 2026-07-24 (see RobotContainer's comments on
    // why it was disabled and why it's worth re-testing now that the camera's 3D mode is on).
    public static final double kApproachTagFaceToleranceDegrees = 15.0;

    // How long (seconds) the tag-search button holds station once arrived before the command
    // finishes on its own and hands control back to normal driving.
    public static final double kApproachSettleSeconds = 0.5;

    // How long (seconds) to hold station (not resume spinning to search) after briefly losing
    // sight of the current tour stop's tag, before treating it as genuinely lost and resuming
    // the search spin / give-up sweep. Confirmed on the robot: without this, a single missed
    // frame right as the robot was about to arrive (distance error down to ~0.03m) threw away
    // all that progress and forced a full new 360-degree search - spinning away is exactly the
    // wrong reaction to a target that's 99% acquired. ~5x the measured vision pipeline latency
    // (~60ms), generous enough to ride out a brief flicker without waiting too long on a genuine
    // loss.
    public static final double kLostGracePeriodSeconds = 0.3;

    // PID gains for closing distance to the target tag (see the tag-search button). Input is
    // measured ground-plane distance (m) to the tag, setpoint is kApproachDistanceMeters, output
    // is forward speed (m/s). Conservative starting point since this is the first autonomous-
    // translation behavior on this robot (button 2 only ever touched rotation) - start
    // P-only and tune kP on the robot, same approach as kAlignRotationKP above.
    public static final double kApproachDistanceKP = 1.5;
    public static final double kApproachDistanceKI = 0.0;
    // Kept at 0 - a nonzero kD here caused violent full-torque stop/start jitter, confirmed on
    // the robot. Root cause: vision only delivers a fresh distance reading every ~30-60ms (one
    // per camera frame), but PIDController.calculate() runs every 20ms loop tick regardless - the
    // input is frozen between frames then jumps in one discrete step, which a derivative term
    // (assuming smooth continuous sampling) reads as a huge velocity spike, producing one big
    // corrective slam per frame instead of smooth damping. Smoothing is handled on the OUTPUT
    // side instead (see the SlewRateLimiters in RobotContainer.tagSearchAndApproachCommand()).
    public static final double kApproachDistanceKD = 0.0;

    // PID gains for the lateral (strafe) correction that actually squares the robot up with the
    // tag's face, rather than just gating on it - input is the SIGNED tag-face angle (degrees,
    // see RobotContainer), setpoint 0, output is sideways speed (m/s). Without this, centering
    // the tag in frame and driving straight at it (a pursuit curve) does not by itself converge
    // to a square final heading - it only happens to end up square if the approach started
    // already on the tag's normal line. Direction confirmed backwards on the robot and fixed by
    // negating the output in RobotContainer - see the comment there.
    public static final double kTagFaceAlignKP = 0.015;
    public static final double kTagFaceAlignKI = 0.0;
    // Kept at 0 - same derivative-kick-on-discrete-vision-updates reasoning as
    // kApproachDistanceKD above; smoothing is handled on the output side instead.
    public static final double kTagFaceAlignKD = 0.0;

    // Safety cap (m/s) on the autonomous approach speed - deliberately well below the
    // drivetrain's true top speed (~5.85 m/s) given this is new, untested autonomous-driving
    // behavior. Raise only after confirming the distance PID's sign/behavior is correct.
    public static final double kApproachMaxSpeedMps = 1.0;

    // Floor (m/s) on the approach speed once actively closing distance (not yet arrived) - a
    // P-only controller's output shrinks proportionally with error, and near the standoff
    // distance that output can get too small for the drivetrain to actually overcome static
    // friction, stalling just short of arriving. Same idea as
    // OperatorConstants.kMinOutputPercent for manual driving. Direction is preserved (this floors
    // the magnitude, not a fixed positive value) - safe to apply unconditionally while not yet
    // arrived, since the "arrived" check overrides forwardSpeed to 0 the instant it's satisfied,
    // regardless of this floor.
    public static final double kApproachMinSpeedMps = 0.15;
  }

  public static class UltrasonicConstants {
    // roboRIO DIO channel numbers - verify against actual wiring before trusting these, same as
    // the joystick axis/button numbers in OperatorConstants. IMPORTANT: the HC-SR04's Echo pin
    // outputs 5V logic, but roboRIO DIO inputs are only 3.3V-tolerant - a voltage divider (e.g.
    // 1k series + 2k to ground) MUST sit between Echo and this DIO input, or it risks damaging
    // the roboRIO. The Trig pin can go directly from a DIO output - HC-SR04 boards generally
    // accept a 3.3V trigger HIGH fine.
    public static final int kTriggerChannel = 0;
    public static final int kEchoChannel = 1;
  }
}
