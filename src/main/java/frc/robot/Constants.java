// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Robot-wide numerical and boolean constants. Declare everything public static; put nothing
 * functional here. Static-import an inner class where its constants are used to cut verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    // Thrustmaster T.16000M raw HID axis indices (standard USB report order). Verify on the
    // Driver Station's USB Devices tab before relying on these.
    public static final int kThrustmasterXAxis = 0; // stick left(-1)/right(+1)
    public static final int kThrustmasterYAxis = 1; // stick forward(-1)/back(+1)
    public static final int kThrustmasterTwistAxis = 2; // stick rotate CCW(-1)/CW(+1)
    public static final int kThrustmasterSliderAxis = 3; // throttle slider

    // Button numbers (1-indexed, as reported by WPILib). Verify on the Driver Station.
    public static final int kThrustmasterTriggerButton = 1;
    public static final int kThrustmasterThumbButton = 2;
    public static final int kThrustmasterTagSearchButton = 4;

    // Stick deflection below this fraction produces zero output.
    public static final double kTranslationDeadband = 0.10;
    public static final double kRotationDeadband = 0.10;

    // Response shaping above the deadband: output fraction = stickFraction ^ exponent. 1.0 is
    // linear; higher softens low-speed response while full deflection still reaches max. 2.0
    // (squared) is a common starting point; go higher (e.g. 3.0) for gentler low-speed response.
    public static final double kTranslationCurveExponent = 2.0;
    public static final double kRotationCurveExponent = 2.0;

    // Absolute floor (fraction of the drivetrain's TRUE top speed) on translation once past the
    // deadband - below this the wheels don't move usefully. Does not shrink with the slider, but
    // is clamped to the slider's current cap so it never exceeds what the slider allows (see
    // RobotContainer).
    public static final double kMinOutputPercent = 0.05;

    // Throttle slider maps its [-1, 1] range to [max, min] fraction of true top translation
    // speed: all the way back (-1) is fastest, all the way forward (+1) is slowest/safest. Keep
    // the min at or above kMinOutputPercent or the slider's low end becomes unreachable.
    public static final double kSliderMinSpeedPercent = 0.05;
    public static final double kSliderMaxSpeedPercent = 1.00;

    // Rotation has no slider - fixed fraction of true top rotational speed. Higher than the
    // slider's translation minimum since spinning in place is less dangerous than driving at speed.
    public static final double kMaxRotationOutputPercent = .3;
  }

  public static class VisionConstants {
    // Must match the C920's name in the PhotonVision UI's Cameras tab exactly.
    public static final String kCameraName = "front cam";

    // PID for rotating to center the best-seen AprilTag (button 2). Input is yaw error (deg),
    // output is rad/s. Start P-only; add kD only if the heading oscillates before settling.
    public static final double kAlignRotationKP = 0.06;
    public static final double kAlignRotationKI = 0.0;
    public static final double kAlignRotationKD = 0.0;

    // Yaw error (deg) within which a target counts as "aligned"; also the approach's squared-up
    // arrival gate.
    public static final double kAlignYawToleranceDegrees = 1.5;

    // Search-spin rate (rad/s) while holding a search button with no tag visible. Absolute, not
    // scaled off kMaxAngularRate, so it's independent of drivetrain tuning. Kept below full speed
    // by preference, not a camera/blur limitation.
    public static final double kSearchRotationRadPerSec = 1.0;

    // AprilTag IDs the tag-search-and-approach button (button 4) visits, in tour order. Edit and
    // redeploy to change; no runtime input yet.
    public static final int[] kSearchTagIdOrder = {1, 2, 3, 4};

    // Standoff distance (m) to stop from the target tag, measured along the ground plane (hypot
    // of the camera-to-target X/Y, ignoring the camera/tag height difference), not 3D
    // line-of-sight.
    public static final double kApproachDistanceMeters = 0.8128; // 32 inches

    // The approach won't start closing distance until yaw error is within this many degrees -
    // keeps it from crabbing in at a steep angle while still mid-rotation.
    public static final double kApproachYawToleranceDegrees = 30.0;

    // Distance tolerance (m) for the approach to count as "arrived".
    public static final double kApproachDistanceToleranceMeters = 0.05; // ~2 inches

    // How far the tag itself may be turned from facing the camera head-on for "arrived" (distinct
    // from kAlignYawToleranceDegrees, which is the tag's left/right position in frame). Re-enabled
    // 2026-07-24 now that the camera's 3D mode is on.
    public static final double kApproachTagFaceToleranceDegrees = 15.0;

    // How long (s) to hold station once arrived before the command finishes on its own.
    public static final double kApproachSettleSeconds = 0.5;

    // How long (s) to hold station after briefly losing the current tag before treating it as
    // genuinely lost and resuming the search spin. Without this, a single missed frame near
    // arrival (~0.03m error, confirmed on the robot) threw away all progress and forced a full
    // new 360 search. ~5x the measured ~60ms pipeline latency.
    public static final double kLostGracePeriodSeconds = 0.3;

    // PID for closing distance to the tag (button 4). Input is ground-plane distance (m),
    // setpoint is kApproachDistanceMeters, output is forward speed (m/s). Start P-only and tune
    // on the robot.
    public static final double kApproachDistanceKP = 1.5;
    public static final double kApproachDistanceKI = 0.0;
    // Kept 0: vision delivers a fresh distance only every ~30-60ms while calculate() runs every
    // 20ms, so the frozen-then-jumping input reads as a huge velocity spike to a derivative term,
    // causing violent stop/start jitter (confirmed on the robot). Output is smoothed via
    // SlewRateLimiters in RobotContainer instead.
    public static final double kApproachDistanceKD = 0.0;

    // PID for the strafe that squares the robot up with the tag's face (button 4). Input is the
    // signed tag-face angle (deg), setpoint 0, output is sideways speed (m/s). Without it, driving
    // straight at a centered tag (a pursuit curve) only ends square if it started on the tag's
    // normal. Output is negated in RobotContainer - direction confirmed backwards on the robot.
    public static final double kTagFaceAlignKP = 0.015;
    public static final double kTagFaceAlignKI = 0.0;
    // Kept 0: same derivative-kick-on-discrete-vision reasoning as kApproachDistanceKD.
    public static final double kTagFaceAlignKD = 0.0;

    // Safety cap (m/s) on autonomous approach speed - well below the drivetrain's ~5.85 m/s top
    // speed given this is new autonomous-driving behavior. Raise only after confirming PID signs.
    public static final double kApproachMaxSpeedMps = 1.0;

    // Floor (m/s) on approach speed while still closing - a P-only output shrinks near the
    // setpoint until it can't overcome static friction, stalling short of arrival. Magnitude only
    // (direction preserved); safe to apply unconditionally since "arrived" forces speed to 0.
    public static final double kApproachMinSpeedMps = 0.15;
  }
}
