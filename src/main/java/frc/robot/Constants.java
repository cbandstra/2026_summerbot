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

    // Deadband is a fraction of the CURRENT slider-selected top speed - below this much stick
    // deflection, translation output is zero, regardless of where the slider is set.
    public static final double kTranslationDeadband = 0.10;
    public static final double kRotationDeadband = 0.10;

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
    public static final double kMaxRotationOutputPercent = 0.30;
  }
}
