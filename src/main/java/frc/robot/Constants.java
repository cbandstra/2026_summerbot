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

    // Thrustmaster stick axis numbers. Check the Driver Station's USB tab if these seem wrong.
    public static final int kThrustmasterXAxis = 0; // stick left(-1)/right(+1)
    public static final int kThrustmasterYAxis = 1; // stick forward(-1)/back(+1)
    public static final int kThrustmasterTwistAxis = 2; // stick rotate CCW(-1)/CW(+1)
    public static final int kThrustmasterSliderAxis = 3; // throttle slider

    // Button numbers (check the Driver Station's USB tab if these seem wrong).
    public static final int kThrustmasterTriggerButton = 1;
    public static final int kThrustmasterThumbButton = 2;

    // Stick deflection below this fraction produces zero output.
    public static final double kTranslationDeadband = 0.10;
    public static final double kRotationDeadband = 0.10;

    // Shapes stick response above the deadband: output = stickFraction ^ exponent. 1.0 is
    // linear; higher makes low-speed response gentler while full stick still reaches top speed.
    public static final double kTranslationCurveExponent = 2.0;
    public static final double kRotationCurveExponent = 2.0;

    // Slowest the drivetrain will actually command once past the deadband, as a fraction of true
    // top speed - stops the robot from asking for a speed too small to move the wheels. Never
    // higher than what the slider currently allows.
    public static final double kMinOutputPercent = 0.05;

    // The throttle slider maps to a speed range: all the way back = fastest, all the way
    // forward = slowest.
    public static final double kSliderMinSpeedPercent = 0.05;
    public static final double kSliderMaxSpeedPercent = 1.00;

    // Rotation has no slider, so it's always this fraction of true top spin speed.
    public static final double kMaxRotationOutputPercent = .3;
  }

  public static class VisionConstants {
    // Must match the camera's name in the PhotonVision UI.
    public static final String kCameraName = "front cam";

    // PID gains for turning to face the best-seen AprilTag (button 2). Tune kP first; only add
    // kD if it oscillates before settling.
    public static final double kAlignRotationKP = 0.06;
    public static final double kAlignRotationKI = 0.0;
    public static final double kAlignRotationKD = 0.0;

    // How fast (rad/s) to spin while searching for a tag with button 2. Kept slow on purpose.
    public static final double kSearchRotationRadPerSec = 1.0;
  }

  public static class AutoConstants {
    // Speed used for every scripted "drive" step. Slow on purpose for testing.
    public static final double kAutoDriveSpeedMps = 0.3;

    // Spin rate (rad/s) used for every scripted "rotate" step.
    public static final double kAutoRotateSpeedRadPerSec = 1.0;

    // Yaw error (degrees) within which "align with april tag" counts as done.
    public static final double kAutoAlignToleranceDegrees = 1.5;

    // How long "align with april tag" searches before giving up, so a missing tag can't stall
    // the whole autonomous sequence forever.
    public static final double kAutoAlignTimeoutSeconds = 5.0;
  }
}
