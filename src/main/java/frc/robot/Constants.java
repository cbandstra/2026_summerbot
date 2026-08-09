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
    public static final int kThrustmasterTargetLockButton = 2;
    public static final int kThrustmasterRecenterButton = 7;
    public static final int kThrustmasterForceSpinButton = 3;
    public static final int kThrustmasterForceSpinClockwiseButton = 4;
    public static final int kThrustmasterDriveAwayFromLockedTargetButton = 8;
    public static final int kThrustmasterDriveTowardLockedTargetButton = 9;
    public static final int kThrustmasterLineUpLockedTargetButton = 10;

    // A press shorter than this toggles target lock on/off. A press this long or longer acts
    // like plain hold-to-activate instead, and always ends off on release (even if it was
    // already toggled on) - so holding "just in case you forgot" never leaves it stuck on.
    public static final double kTargetLockTapThresholdSeconds = 1.0;

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
    public static final String kCameraName = "OV9281_April_Tags";

    // PID gains for turning to face the best-seen AprilTag (target lock). Tune kP first; only
    // add kD if it oscillates before settling. Separate from the search-spin speeds below - only
    // affects rotation while actively aiming at a tag that's already in view.
    public static final double kAlignRotationKP = 0.04;
    public static final double kAlignRotationKI = 0.0;
    public static final double kAlignRotationKD = 0.0;

    // How fast (rad/s) to spin during each fast pulse while searching for a tag.
    public static final double kSearchRotationRadPerSec = 2.0;

    // How fast (rad/s) to spin during the slow phase between pulses. Slower than the fast pulse,
    // but not stopped, so the camera gets a steadier look without giving up ground entirely.
    public static final double kSearchSlowRotationRadPerSec = 0.5;

    // Search alternates fast/slow - it spins fast this many degrees, then spins slow for
    // kSearchSlowPhaseSeconds, then spins fast again, and so on until a tag is seen. Gives the
    // camera a steadier look during the slow phase instead of only ever seeing tags blur past.
    public static final double kSearchSpinDegrees = 20.0;

    // How long (seconds) to spin at the slow rate between each fast pulse of the search spin.
    public static final double kSearchSlowPhaseSeconds = 1;

    // If target lock loses a tag while it was closer than this (meters), the tag was probably
    // lost because the robot got too close for the camera to see the whole thing - not because
    // it wandered off. Don't spin away looking for another one in that case; just hold still
    // until a tag comes back into view or the driver forces a new search (see Force spin button).
    public static final double kCloseTargetLossDistanceMeters = 1.0;
  }

  public static class AutoConstants {
    // Speed used for every scripted "drive" step. Slow on purpose for testing.
    public static final double kAutoDriveSpeedMps = 1.0;

    // Spin rate (rad/s) used for every scripted "rotate" step. Only affects the plain "rotate #
    // degrees" instruction - "align with april tag" and "align with april tag ... and go to it"
    // use their own separate speed/gain constants, so this is safe to tune independently.
    public static final double kAutoRotateSpeedRadPerSec = 2.0;

    // Set false to skip the step-complete beep entirely (not just silence it) - speeds up
    // autonomous cycle times when you don't need the audible confirmation, since every step
    // otherwise waits for its beep to fully finish before the next one starts.
    public static final boolean kStepCompleteBeepEnabled = true;

    // The beep every drivetrain motor plays when an autonomous step finishes alternates between
    // two tones (Hz), switching every kStepCompleteBeepNoteSeconds, for a total of
    // kStepCompleteBeepSeconds - see RobotContainer.beepCommand().
    public static final double kStepCompleteBeepHz = 880.0;
    public static final double kStepCompleteBeepHz2 = 660.0;
    public static final double kStepCompleteBeepNoteSeconds = 0.15;
    public static final double kStepCompleteBeepSeconds = 0.5;

    // "Align with april tag" and "align with april tag ... and go to it" hold still (instead of
    // immediately spinning to search) for this long at the very start, in case the tag is
    // already in view but the camera just hasn't reported this frame's detection yet (e.g. right
    // as a preceding rotate step finishes) - avoids a needless spin burst before vision catches
    // up on something that was already there.
    public static final double kAutoSearchInitialGraceSeconds = 0.5;

    // Yaw error (degrees) within which "align with april tag" counts as done.
    public static final double kAutoAlignToleranceDegrees = 1.5;

    // How long "align with april tag" searches before giving up, so a missing tag can't stall
    // the whole autonomous sequence forever.
    public static final double kAutoAlignTimeoutSeconds = 5.0;

    // "Find id <n> and line up with it" stops driving once this close (meters) to the tag.
    public static final double kLineUpDistanceMeters = 1.0;

    // How close (meters) to kLineUpDistanceMeters counts as "close enough" to finish the step.
    public static final double kLineUpDistanceToleranceMeters = 0.1;

    // The approach speed isn't one constant - it's fast (kLineUpFarApproachSpeedMps) until
    // within kLineUpNearDistanceMeters of the tag, then slows to kLineUpNearApproachSpeedMps for
    // the final approach so it doesn't come in too hot to fine-tune its alignment. Independent of
    // kAutoDriveSpeedMps, which is used by the plain drive/align steps instead.
    public static final double kLineUpFarApproachSpeedMps = 1.5;
    public static final double kLineUpNearApproachSpeedMps = 0.5;
    public static final double kLineUpNearDistanceMeters = 2.0;

    // While still far out, aiming at the tag's center normally allows up to the drivetrain's
    // full turn speed (kMaxAngularRate) - but a big initial yaw error right after the search
    // finds the tag can spin fast enough to outrun the camera and lose the tag again before it
    // can even start correcting. Close-range squaring never has this problem since its errors
    // are already small by the time it's driving, so it never gets close to the full turn speed
    // anyway. This caps the far-range aim turn to roughly match that same gentler pace.
    public static final double kLineUpFarAimMaxAngularRateRadPerSec = 2.0;

    // "Centered" (for finishing the step) is judged straight from the tag's own pose, not from
    // yaw-to-target: the tag's Z rotation (how squarely it's facing the camera) must be within
    // kLineUpCenteredZAngleToleranceDegrees of kLineUpCenteredZAngleTargetDegrees, and its Y
    // translation (how far left/right of the camera it is) must be within
    // kLineUpCenteredYToleranceMeters of kLineUpCenteredYTargetMeters.
    //
    // The targets aren't exactly 180/0 - measured on the robot 2026-08-08 by lining it up
    // manually (by eye) with a target and reading what the camera actually reported. The camera
    // isn't mounted perfectly centered/square on the chassis, so "physically lined up" reads as
    // slightly off from the theoretical dead-on numbers - these targets correct for that offset.
    public static final double kLineUpCenteredZAngleTargetDegrees = -176.0;
    public static final double kLineUpCenteredYTargetMeters = 0.054;

    public static final double kLineUpCenteredZAngleToleranceDegrees = 3.0;
    public static final double kLineUpCenteredYToleranceMeters = 0.05;

    // How far off-center (degrees) is still "close enough to drive straight at it" while lining
    // up - the rotation PID keeps refining aim the whole approach, this just gates when it's safe
    // to start driving forward instead of turning in place first.
    public static final double kLineUpSteerToleranceDegrees = 10.0;

    // How long "find id <n> and line up with it" searches/drives before giving up, so a missing
    // tag can't stall the rest of the autonomous sequence forever. Longer than a plain align
    // since it also has to drive up to the tag afterward.
    public static final double kLineUpTimeoutSeconds = 20.0;

    // While still driving up to the tag, keep pointing at its center like a normal approach,
    // but also nudge sideways toward square (proportional to how far the Z angle is from
    // kLineUpCenteredZAngleTargetDegrees) so there's less left to fix once it stops. This alone
    // can't fully square the robot up - see kLineUpSquareKP below for why - it's just a head
    // start. Confirmed correct direction on the robot 2026-08-08.
    public static final double kLineUpStrafeKP = 0.006;

    // Once the robot's stopped at kLineUpDistanceMeters, squaring up is a rotation problem - the
    // Z angle is purely a function of heading relative to the tag's face, not position, so
    // sliding sideways without turning can't change it. (The old approach only worked
    // indirectly: strafing shifted bearing, which the gentle bearing PID reacted to by rotating,
    // which incidentally changed Z angle too - roundabout and weak.) This instead rotates
    // straight at the Z-angle error (rad/s per degree), same as a normal turn-in-place. Confirmed
    // correct direction on the robot 2026-08-08.
    public static final double kLineUpSquareKP = 0.05;

    // Once stopped, centering is a translation problem, corrected directly the same way (m/s per
    // meter of Y error) instead of relying on it falling out of the rotation correction above.
    // Confirmed correct direction on the robot 2026-08-08 (was inverted before - flip it back if
    // it starts strafing away from center again).
    public static final double kLineUpCenterStrafeKP = -3.0;

    // Cap (m/s) for the close-up centering strafe above.
    public static final double kLineUpCloseStrafeMaxMps = 0.4;

    // A small error times a gentle gain can come out too small to actually move the robot at all
    // (motor/gearbox static friction). Whenever a strafe correction is needed (Z angle out of
    // tolerance while still driving up, or Y out of tolerance once stopped), it's floored to at
    // least this speed (direction still comes from the gain math) so it's never a no-op.
    public static final double kLineUpStrafeMinMps = 0.08;

    // Caps how fast the commanded speeds are allowed to change (per second) during "find id <n>
    // and line up with it" - smooths out noisy vision readings and the strafe floor snapping
    // on/off, which otherwise show up as jerky, jumpy motion instead of a smooth correction.
    public static final double kLineUpTranslationSlewMpsPerSec = 1.5;
    public static final double kLineUpRotationSlewRadPerSecSquared = 6.0;
  }
}
