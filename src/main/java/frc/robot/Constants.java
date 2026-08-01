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

    // Search-spin rate (rad/s) while holding button 2 with no tag visible. Absolute, not scaled
    // off kMaxAngularRate, so it's independent of drivetrain tuning. Kept below full speed by
    // preference, not a camera/blur limitation.
    public static final double kSearchRotationRadPerSec = 1.0;
  }
}
