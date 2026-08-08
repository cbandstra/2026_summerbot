// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/** Sets up the robot: subsystems, driver controls, and button bindings. */
public class RobotContainer {
  // Distance from the robot's center to a wheel - used to turn top speed (m/s) into top spin
  // rate (rad/s).
  private static final double kDriveBaseRadiusMeters =
      Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY);

  // True top speed of the drivetrain. The throttle slider scales this down live
  // (see throttleSpeedPercent()).
  private static final double kMaxSpeedMps = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  // Rotation has no slider, so it's always this fraction of true top spin speed.
  private static final double kMaxAngularRate =
      (kMaxSpeedMps / kDriveBaseRadiusMeters) * OperatorConstants.kMaxRotationOutputPercent;

  // Subsystems
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final Vision vision = new Vision();

  private final Telemetry logger = new Telemetry(kMaxSpeedMps);

  // Autonomous instructions parsed from deploy/autonomous.json. Loaded once here so a typo
  // shows up in the console right away instead of hiding until autonomous actually starts.
  private final List<AutoStep> m_autoSteps = AutoScript.load();

  // Turns the robot to face an AprilTag - shared by target lock and the "align with april tag"
  // autonomous instruction. Input/output: yaw error in degrees, rad/s out.
  private final PIDController m_alignRotationController = new PIDController(
      VisionConstants.kAlignRotationKP,
      VisionConstants.kAlignRotationKI,
      VisionConstants.kAlignRotationKD
  );

  // So "Looking for April tags" only logs once per search, not every loop.
  private boolean m_loggedSearching = false;

  // Drives target lock's pulsed search spin (see PulsedSearch below). Autonomous's "align with
  // april tag" gets its own instance per call, since it isn't a long-lived field like this one.
  private final PulsedSearch m_targetLockSearch = new PulsedSearch();

  // How long the target lock button has been held this press - used to tell a quick tap
  // (toggle on/off) from a hold (plain hold-to-activate) on release.
  private final Timer m_targetLockPressTimer = new Timer();

  // True while target lock is toggled on. Target lock is active whenever this is true OR the
  // button is physically held (see configureBindings).
  private boolean m_targetLockToggleOn = false;

  // Distance (meters) to the last tag target lock actually saw. Starts far away so a fresh
  // search behaves normally. Used to tell "lost the tag because we drove right up to it" (don't
  // spin away looking for another) from "lost the tag because it's just not in view" (do spin).
  private double m_targetLockLastDistanceMeters = Double.MAX_VALUE;

  // True until the robot's been enabled once since power-on. The gyro zeroes itself to whichever
  // way the robot happens to be pointed when the roboRIO boots - if that's not facing away from
  // the driver station (e.g. it booted on a cart or the pit table), "forward" on the stick drives
  // the wrong way until someone fixes it with the Recenter button. Auto-seeding on the first
  // enable fixes it automatically instead, since by then the robot's actually been placed for
  // driving.
  private boolean m_needsInitialFieldCentricSeed = true;

  // Thrustmaster T.16000M flight stick
  private final CommandJoystick m_driverController =
      new CommandJoystick(OperatorConstants.kDriverControllerPort);

  // Smooths stick input so small pot jitter doesn't make the wheels twitch.
  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3.0);

  // Field-centric drive. Deadband/curve shaping is done by hand below instead of by CTRE, since
  // CTRE's built-in deadband can't apply a response curve.
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(0)
      .withRotationalDeadband(0)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  // Robot-centric drive request used for autonomous steps - "forward" means the robot's own
  // front, not a field direction.
  private final SwerveRequest.RobotCentric autoDrive = new SwerveRequest.RobotCentric()
      .withDeadband(0)
      .withRotationalDeadband(0)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public RobotContainer() {
    configureBindings();
  }

  /** Wires up driver controls: default drive command and button bindings. */
  private void configureBindings() {
    // WPILib convention: X is forward, Y is left. Axis numbers are the stick's default USB
    // order - check the Driver Station if driving feels backwards.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> {
            double maxSpeed = kMaxSpeedMps * throttleSpeedPercent();
            double[] translation = computeTranslationVelocity(maxSpeed);
            return drive.withVelocityX(translation[0])
                .withVelocityY(translation[1])
                .withRotationalRate(computeManualRotationalRate());
        })
    );

    // Idle (don't fight the brakes) while disabled. Also force target lock's toggle off, so it
    // can't silently resume searching/aligning the instant the robot is re-enabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );
    RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> {
        if (m_targetLockToggleOn) {
            m_targetLockToggleOn = false;
            RobotLog.log("Target lock: OFF (robot disabled)");
        }
    }));

    // First time the robot's enabled after booting, treat wherever it's currently facing as
    // "forward" - see m_needsInitialFieldCentricSeed above for why.
    RobotModeTriggers.disabled().onFalse(Commands.runOnce(() -> {
        if (m_needsInitialFieldCentricSeed) {
            drivetrain.seedFieldCentric();
            m_needsInitialFieldCentricSeed = false;
            RobotLog.log("Drivetrain: seeded forward direction on first enable");
        }
    }));

    // Hold the trigger to lock the wheels in an X pattern (resists being pushed).
    m_driverController.button(OperatorConstants.kThrustmasterTriggerButton)
        .whileTrue(drivetrain.applyRequest(() -> brake));

    // Press to make wherever the robot is CURRENTLY facing the new "forward" for field-centric
    // driving - useful after the robot's hand-placed at an angle, or to redefine forward
    // mid-match. Doesn't move the robot or touch its tracked field position, just which way the
    // stick's "forward" points from now on.
    m_driverController.button(OperatorConstants.kThrustmasterRecenterButton)
        .onTrue(Commands.runOnce(drivetrain::seedFieldCentric, drivetrain));

    // Target lock: spin looking for any AprilTag, then turn to face it once seen. Translation
    // stays on the stick the whole time - only rotation is taken over.
    //
    // A quick tap (shorter than kTargetLockTapThresholdSeconds) toggles it on/off, so it keeps
    // running hands-free until tapped again. Holding it down works exactly like before - active
    // the whole time it's held, and always ends off on release - in case you forget the toggle
    // and just want to hold it like usual.
    Trigger targetLockButton = m_driverController.button(OperatorConstants.kThrustmasterTargetLockButton);

    targetLockButton.onTrue(Commands.runOnce(m_targetLockPressTimer::restart));
    targetLockButton.onFalse(Commands.runOnce(() -> {
        double heldSeconds = m_targetLockPressTimer.get();
        if (heldSeconds < OperatorConstants.kTargetLockTapThresholdSeconds) {
            m_targetLockToggleOn = !m_targetLockToggleOn;
            RobotLog.log("Target lock: " + (m_targetLockToggleOn ? "ON" : "OFF")
                + String.format(" (tap, %.2fs)", heldSeconds));
        } else {
            m_targetLockToggleOn = false;
            RobotLog.log(String.format("Target lock: OFF (held %.2fs)", heldSeconds));
        }
    }));

    // Force spin: while target lock has found a tag and is aligning to it, hold this button to
    // interrupt that and force the same pulsed search spin instead - e.g. to deliberately look
    // away from the current tag. Also the only way to resume searching after target lock holds
    // still for a lost close-up tag (see kCloseTargetLossDistanceMeters below). Does nothing
    // unless target lock is currently active.
    Trigger forceSpinButton = m_driverController.button(OperatorConstants.kThrustmasterForceSpinButton);
    forceSpinButton.onTrue(Commands.runOnce(() -> {
        m_targetLockSearch.reset(drivetrain.getState().Pose.getRotation());
        m_targetLockLastDistanceMeters = Double.MAX_VALUE;
    }));

    new Trigger(() -> targetLockButton.getAsBoolean() || m_targetLockToggleOn).whileTrue(
        Commands.startRun(
            () -> {
                m_alignRotationController.reset();
                m_loggedSearching = false;
                m_targetLockSearch.reset(drivetrain.getState().Pose.getRotation());
                m_targetLockLastDistanceMeters = Double.MAX_VALUE;
            },
            () -> {
                double maxSpeed = kMaxSpeedMps * throttleSpeedPercent();
                double[] translation = computeTranslationVelocity(maxSpeed);
                boolean forceSpin = forceSpinButton.getAsBoolean();
                double rotationalRate;
                if (vision.hasTarget() && !forceSpin) {
                    rotationalRate = computeAlignRotationalRate();
                    m_loggedSearching = false;
                    m_targetLockLastDistanceMeters = vision.getTargetDistanceMeters();
                } else if (!forceSpin
                        && m_targetLockLastDistanceMeters < VisionConstants.kCloseTargetLossDistanceMeters) {
                    // Lost a tag we were right up against - it's almost certainly still there,
                    // just out of frame. Hold still instead of spinning away from it.
                    if (!m_loggedSearching) {
                        RobotLog.log("Target lock: holding still (lost a close-up tag)");
                        m_loggedSearching = true;
                    }
                    rotationalRate = 0.0;
                } else {
                    if (!m_loggedSearching) {
                        RobotLog.log(forceSpin && vision.hasTarget()
                            ? "Target lock: forcing search spin (button 3)"
                            : "Looking for April tags");
                        m_loggedSearching = true;
                    }
                    rotationalRate = m_targetLockSearch.pulse(drivetrain.getState().Pose.getRotation());
                }
                drivetrain.setControl(drive.withVelocityX(translation[0])
                    .withVelocityY(translation[1])
                    .withRotationalRate(rotationalRate));
            },
            drivetrain, vision
        )
    );

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /** Maps the throttle slider to a speed fraction: all the way back = fastest, forward = slowest. */
  private double throttleSpeedPercent() {
    double axis = m_driverController.getRawAxis(OperatorConstants.kThrustmasterSliderAxis);
    double t = (1.0 - axis) / 2.0; // -1 -> 1 (max), +1 -> 0 (min)
    return OperatorConstants.kSliderMinSpeedPercent
        + t * (OperatorConstants.kSliderMaxSpeedPercent - OperatorConstants.kSliderMinSpeedPercent);
  }

  /**
   * Turn rate (rad/s) to face the best-seen AprilTag, correcting for camera lag (see {@link
   * #computeCompensatedYawDegrees}). Confirmed on the robot: the raw (non-flipped) yaw sign
   * already turns toward the target, so no extra sign flip is needed here.
   */
  private double computeAlignRotationalRate() {
    return computeAlignRotationalRate(vision.getTargetYawDegrees(), vision.getTargetTimestampSeconds());
  }

  private double computeAlignRotationalRate(double rawYawDegrees, double frameTimestampSeconds) {
    double compensatedYawDegrees = computeCompensatedYawDegrees(rawYawDegrees, frameTimestampSeconds);
    return MathUtil.clamp(
        m_alignRotationController.calculate(compensatedYawDegrees, 0.0),
        -kMaxAngularRate, kMaxAngularRate
    );
  }

  /**
   * Corrects a camera yaw reading for lag: the frame it came from is always a little old (~60ms
   * on this rig), so this figures out how far the robot has turned since then and subtracts it
   * back out. Falls back to the raw yaw if we don't have that much odometry history yet.
   *
   * <p>Important: PhotonVision's timestamp must be converted with {@link Utils#fpgaToCurrentTime}
   * first, or {@link CommandSwerveDrivetrain#samplePoseAt} silently returns nothing and this does
   * no correction at all.
   */
  private double computeCompensatedYawDegrees(double rawYawDegrees, double frameTimestampSeconds) {
    var historicalPose = drivetrain.samplePoseAt(Utils.fpgaToCurrentTime(frameTimestampSeconds));
    if (historicalPose.isEmpty()) {
        return rawYawDegrees;
    }
    double rotationSinceFrameDegrees = drivetrain.getState().Pose.getRotation()
        .minus(historicalPose.get().getRotation())
        .getDegrees();
    return rawYawDegrees - rotationSinceFrameDegrees;
  }

  /**
   * Turn rate (rad/s) from the twist stick: zero below the deadband, then curved up to
   * {@code kMaxAngularRate} at full twist. Same shaping as {@link #computeTranslationVelocity}.
   */
  private double computeManualRotationalRate() {
    double stickTwist = -m_rotLimiter.calculate(m_driverController.getRawAxis(OperatorConstants.kThrustmasterTwistAxis)); // CCW is stick twisted left (negative twist)
    double stickMagnitude = Math.abs(stickTwist);

    if (stickMagnitude < OperatorConstants.kRotationDeadband) {
        return 0.0;
    }

    double stickFraction = Math.min(stickMagnitude, 1.0);
    double curvedFraction = Math.pow(stickFraction, OperatorConstants.kRotationCurveExponent);
    return Math.signum(stickTwist) * curvedFraction * kMaxAngularRate;
  }

  /**
   * Turns stick X/Y into robot-relative {@code {velocityX, velocityY}} (m/s). Below the
   * deadband, output is zero; above it, a curve softens low-speed response; then a floor makes
   * sure the robot never commands a speed too small to actually move the wheels (capped at
   * {@code maxSpeed} so it never exceeds what the slider currently allows).
   *
   * @param maxSpeed current top speed (m/s), already scaled by the throttle slider
   */
  private double[] computeTranslationVelocity(double maxSpeed) {
    double stickX = -m_xLimiter.calculate(m_driverController.getRawAxis(OperatorConstants.kThrustmasterYAxis)); // forward is stick pushed away (negative Y)
    double stickY = -m_yLimiter.calculate(m_driverController.getRawAxis(OperatorConstants.kThrustmasterXAxis)); // left is stick pushed left (negative X)
    double stickMagnitude = Math.hypot(stickX, stickY);

    if (stickMagnitude < OperatorConstants.kTranslationDeadband) {
        return new double[] {0.0, 0.0};
    }

    // Clamp to 1.0 in case of diagonal stick deflection (X and Y can each be at their own max).
    double stickFraction = Math.min(stickMagnitude, 1.0);
    double curvedFraction = Math.pow(stickFraction, OperatorConstants.kTranslationCurveExponent);

    double floorFraction = Math.min(OperatorConstants.kMinOutputPercent * kMaxSpeedMps, maxSpeed) / maxSpeed;
    double outputFraction = Math.max(curvedFraction, floorFraction);

    double outputSpeed = outputFraction * maxSpeed;
    double unitX = stickX / stickMagnitude;
    double unitY = stickY / stickMagnitude;

    // Negated (both axes): same chassis-mounting inversion confirmed for the autonomous
    // RobotCentric drive (see driveStepCommand) - this rig's kinematic "front" is physically its
    // back, so +velocityX/+velocityY here also drive backward/right instead of forward/left.
    // Negating both axes is the same thing as rotating the whole output 180 degrees. Confirmed
    // on the robot via the recenter button: after seeding a known forward direction, driving
    // "forward" went the opposite way.
    return new double[] {-unitX * outputSpeed, -unitY * outputSpeed};
  }

  /** Turns an AutoStep into a runnable Command using the robot's own subsystems. */
  private Command autoStepCommand(AutoStep step) {
    if (step instanceof AutoStep.Drive drive) {
        return driveStepCommand(drive);
    } else if (step instanceof AutoStep.Rotate rotate) {
        return rotateCommand(rotate.degrees());
    } else if (step instanceof AutoStep.Wait wait) {
        return Commands.waitSeconds(wait.seconds());
    } else if (step instanceof AutoStep.AlignTag alignTag) {
        return alignToTagCommand(alignTag.tagId());
    } else if (step instanceof AutoStep.LineUpTag lineUpTag) {
        return lineUpToTagCommand(lineUpTag.tagId());
    }
    throw new IllegalStateException("Unhandled AutoStep: " + step);
  }

  /** Turns a "drive <direction> <distance>" step into robot-centric velocityX/velocityY. */
  private Command driveStepCommand(AutoStep.Drive step) {
    double speed = AutoConstants.kAutoDriveSpeedMps;
    // Negated: confirmed backwards on the robot (2026-08-01) - commanding +velocityX on this
    // RobotCentric request drove the chassis backward, not forward, and +velocityY drove right
    // instead of left. Both axes are flipped from what CTRE's RobotCentric normally means.
    double vx = switch (step.direction()) {
        case FORWARD -> -speed;
        case BACKWARD -> speed;
        default -> 0.0;
    };
    double vy = switch (step.direction()) {
        case LEFT -> -speed;
        case RIGHT -> speed;
        default -> 0.0;
    };
    return driveDistanceCommand(vx, vy, step.distanceMeters());
  }

  /**
   * Drives straight (robot-centric) at {@code (vxMps, vyMps)} until the robot has moved
   * {@code distanceMeters} from where this command started, then stops. Distance is measured
   * from odometry.
   */
  private Command driveDistanceCommand(double vxMps, double vyMps, double distanceMeters) {
    Pose2d[] startPose = {null};
    return Commands.sequence(
        Commands.runOnce(() -> startPose[0] = drivetrain.getState().Pose, drivetrain),
        Commands.run(() -> drivetrain.setControl(autoDrive.withVelocityX(vxMps).withVelocityY(vyMps)), drivetrain)
            .until(() -> startPose[0].getTranslation()
                .getDistance(drivetrain.getState().Pose.getTranslation()) >= distanceMeters),
        Commands.runOnce(() -> drivetrain.setControl(autoDrive.withVelocityX(0).withVelocityY(0)), drivetrain)
    );
  }

  /**
   * Turns in place by {@code degrees} (positive = counterclockwise) and stops. Tracks how far
   * it's actually turned via odometry (added up loop by loop) rather than time, so it works for
   * any angle, including more than 180 degrees, without getting confused by heading wraparound.
   */
  private Command rotateCommand(double degrees) {
    double targetRadians = Math.toRadians(Math.abs(degrees));
    double spinRate = Math.signum(degrees)
        * Math.min(AutoConstants.kAutoRotateSpeedRadPerSec, kMaxAngularRate);

    Rotation2d[] lastHeading = {null};
    double[] turnedRadians = {0.0};

    return Commands.sequence(
        Commands.runOnce(() -> {
            lastHeading[0] = drivetrain.getState().Pose.getRotation();
            turnedRadians[0] = 0.0;
        }, drivetrain),
        Commands.run(() -> {
            Rotation2d currentHeading = drivetrain.getState().Pose.getRotation();
            turnedRadians[0] += Math.abs(currentHeading.minus(lastHeading[0]).getRadians());
            lastHeading[0] = currentHeading;
            drivetrain.setControl(autoDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(spinRate));
        }, drivetrain).until(() -> turnedRadians[0] >= targetRadians),
        Commands.runOnce(() -> drivetrain.setControl(
            autoDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)), drivetrain)
    );
  }

  /**
   * Spins to search for AprilTag {@code tagId}, then turns to face it once seen, finishing once
   * aligned within {@link AutoConstants#kAutoAlignToleranceDegrees}. Gives up after {@link
   * AutoConstants#kAutoAlignTimeoutSeconds} if the tag is never found, so a missing tag can't
   * stall the rest of the autonomous sequence forever.
   */
  private Command alignToTagCommand(int tagId) {
    double[] lastYawErrorDegrees = {Double.MAX_VALUE};
    PulsedSearch search = new PulsedSearch();

    return Commands.sequence(
        Commands.runOnce(() -> {
            m_alignRotationController.reset();
            search.reset(drivetrain.getState().Pose.getRotation());
        }, drivetrain),
        Commands.run(() -> {
            var target = vision.getTargetById(tagId);
            double rotationalRate;
            if (target.isPresent()) {
                double rawYawDegrees = target.get().getYaw();
                lastYawErrorDegrees[0] = computeCompensatedYawDegrees(rawYawDegrees, vision.getTargetTimestampSeconds());
                rotationalRate = computeAlignRotationalRate(rawYawDegrees, vision.getTargetTimestampSeconds());
            } else {
                lastYawErrorDegrees[0] = Double.MAX_VALUE;
                rotationalRate = search.pulse(drivetrain.getState().Pose.getRotation());
            }
            drivetrain.setControl(autoDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(rotationalRate));
        }, drivetrain, vision)
            .until(() -> Math.abs(lastYawErrorDegrees[0]) <= AutoConstants.kAutoAlignToleranceDegrees)
            .withTimeout(AutoConstants.kAutoAlignTimeoutSeconds),
        Commands.runOnce(() -> drivetrain.setControl(
            autoDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)), drivetrain)
    );
  }

  /**
   * Spins to search for AprilTag {@code tagId} (same pulsed search as {@link
   * #alignToTagCommand}), then drives straight at it while continuously correcting aim, stopping
   * once {@link AutoConstants#kLineUpDistanceMeters} away and centered. Gives up after {@link
   * AutoConstants#kLineUpTimeoutSeconds} if it never gets there, so a missing tag can't stall the
   * rest of the autonomous sequence forever.
   */
  private Command lineUpToTagCommand(int tagId) {
    double[] yawErrorDegrees = {Double.MAX_VALUE};
    double[] distanceMeters = {Double.MAX_VALUE};
    double[] tagZAngleDegrees = {Double.MAX_VALUE};
    double[] cameraToTagYMeters = {Double.MAX_VALUE};
    PulsedSearch search = new PulsedSearch();
    // Smooths the commanded speeds so they ramp instead of jumping instantly - a noisy vision
    // reading (or the strafe floor snapping on/off) would otherwise show up as a sudden jerk.
    // Same idea as the driver stick's smoothing (m_xLimiter etc.), just for this command.
    SlewRateLimiter vxLimiter = new SlewRateLimiter(AutoConstants.kLineUpTranslationSlewMpsPerSec);
    SlewRateLimiter vyLimiter = new SlewRateLimiter(AutoConstants.kLineUpTranslationSlewMpsPerSec);
    SlewRateLimiter rotationLimiter = new SlewRateLimiter(AutoConstants.kLineUpRotationSlewRadPerSecSquared);
    // The limiters above need SOME starting baseline, but resetting to 0 makes the very first
    // command of the whole step ramp up from a stall instead of starting at full speed - whether
    // that first command comes from the search, the approach aim, or (if a tag's already in view
    // when the step starts) the squaring/centering correction. Priming them to the first loop's
    // actual numbers - whichever branch computes them - avoids that, while still smoothing every
    // change after that.
    boolean[] limitersPrimed = {false};

    return Commands.sequence(
        Commands.runOnce(() -> {
            m_alignRotationController.reset();
            search.reset(drivetrain.getState().Pose.getRotation());
            limitersPrimed[0] = false;
        }, drivetrain),
        Commands.run(() -> {
            var target = vision.getTargetById(tagId);
            double rotationalRate;
            double vx;
            double vy;
            if (target.isPresent()) {
                double rawYawDegrees = target.get().getYaw();
                yawErrorDegrees[0] = computeCompensatedYawDegrees(rawYawDegrees, vision.getTargetTimestampSeconds());
                distanceMeters[0] = vision.getTargetDistanceMeters();

                var cameraToTarget = target.get().getBestCameraToTarget();
                tagZAngleDegrees[0] = Math.toDegrees(cameraToTarget.getRotation().getZ());
                cameraToTagYMeters[0] = cameraToTarget.getTranslation().getY();

                double zAngleErrorDegrees = Rotation2d.fromDegrees(tagZAngleDegrees[0])
                    .minus(Rotation2d.fromDegrees(AutoConstants.kLineUpCenteredZAngleTargetDegrees)).getDegrees();
                double yErrorMeters = cameraToTagYMeters[0] - AutoConstants.kLineUpCenteredYTargetMeters;

                boolean aimedWellEnoughToDrive =
                    Math.abs(yawErrorDegrees[0]) <= AutoConstants.kLineUpSteerToleranceDegrees;
                boolean stillTooFar = distanceMeters[0] - AutoConstants.kLineUpDistanceMeters
                    > AutoConstants.kLineUpDistanceToleranceMeters;
                // Fast while there's real ground to cover, slower once close so the final
                // approach doesn't come in too hot to fine-tune alignment.
                double approachSpeedMps = distanceMeters[0] <= AutoConstants.kLineUpNearDistanceMeters
                    ? AutoConstants.kLineUpNearApproachSpeedMps
                    : AutoConstants.kLineUpFarApproachSpeedMps;
                // Negative = forward - same RobotCentric convention as driveStepCommand.
                vx = (aimedWellEnoughToDrive && stillTooFar)
                    ? -Math.min(approachSpeedMps, kMaxSpeedMps)
                    : 0.0;

                if (stillTooFar) {
                    // Still approaching - aim at the tag's center like a normal approach, and
                    // nudge sideways toward square so there's less left to fix once stopped.
                    // Re-clamped tighter than the usual full turn speed - a big initial yaw error
                    // right after search finds the tag could otherwise spin fast enough to lose
                    // it again before it can even start correcting.
                    rotationalRate = MathUtil.clamp(
                        computeAlignRotationalRate(rawYawDegrees, vision.getTargetTimestampSeconds()),
                        -AutoConstants.kLineUpFarAimMaxAngularRateRadPerSec,
                        AutoConstants.kLineUpFarAimMaxAngularRateRadPerSec);
                    boolean zAngleOutOfTolerance = Math.abs(zAngleErrorDegrees)
                        > AutoConstants.kLineUpCenteredZAngleToleranceDegrees;
                    vy = flooredAndClamped(AutoConstants.kLineUpStrafeKP * zAngleErrorDegrees,
                        zAngleOutOfTolerance, AutoConstants.kLineUpStrafeMinMps, approachSpeedMps);
                } else {
                    // Stopped driving forward - squaring up is a rotation problem, and centering
                    // is a translation problem, so correct each directly instead of only aiming
                    // at the tag's center and hoping strafing fixes the skew as a side effect.
                    rotationalRate = MathUtil.clamp(AutoConstants.kLineUpSquareKP * zAngleErrorDegrees,
                        -kMaxAngularRate, kMaxAngularRate);
                    boolean yOutOfTolerance = Math.abs(yErrorMeters)
                        > AutoConstants.kLineUpCenteredYToleranceMeters;
                    vy = flooredAndClamped(AutoConstants.kLineUpCenterStrafeKP * yErrorMeters,
                        yOutOfTolerance, AutoConstants.kLineUpStrafeMinMps, AutoConstants.kLineUpCloseStrafeMaxMps);
                }
            } else {
                yawErrorDegrees[0] = Double.MAX_VALUE;
                distanceMeters[0] = Double.MAX_VALUE;
                tagZAngleDegrees[0] = Double.MAX_VALUE;
                cameraToTagYMeters[0] = Double.MAX_VALUE;
                rotationalRate = search.pulse(drivetrain.getState().Pose.getRotation());
                vx = 0.0;
                vy = 0.0;
            }

            if (!limitersPrimed[0]) {
                vxLimiter.reset(vx);
                vyLimiter.reset(vy);
                rotationLimiter.reset(rotationalRate);
                limitersPrimed[0] = true;
            }
            drivetrain.setControl(autoDrive.withVelocityX(vxLimiter.calculate(vx))
                .withVelocityY(vyLimiter.calculate(vy))
                .withRotationalRate(rotationLimiter.calculate(rotationalRate)));
        }, drivetrain, vision)
            .until(() -> isCentered(tagZAngleDegrees[0], cameraToTagYMeters[0])
                && Math.abs(distanceMeters[0] - AutoConstants.kLineUpDistanceMeters)
                    <= AutoConstants.kLineUpDistanceToleranceMeters)
            .withTimeout(AutoConstants.kLineUpTimeoutSeconds),
        Commands.runOnce(() -> drivetrain.setControl(
            autoDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)), drivetrain)
    );
  }

  /**
   * {@code gainTimesError}, but with its magnitude floored to {@code minMps} whenever {@code
   * outOfTolerance} is true (so a small error times a gentle gain can't come out too small to
   * actually move the robot) and capped to {@code maxMps}. Direction always comes from {@code
   * gainTimesError}'s sign.
   */
  private static double flooredAndClamped(double gainTimesError, boolean outOfTolerance, double minMps, double maxMps) {
    double magnitude = Math.abs(gainTimesError);
    if (outOfTolerance && magnitude < minMps) {
        magnitude = minMps;
    }
    magnitude = Math.min(magnitude, maxMps);
    return Math.copySign(magnitude, gainTimesError);
  }

  /**
   * True if the tag's own pose (not yaw-to-target) says we're squared up and centered on it:
   * its Z rotation (how squarely it's facing the camera) within {@link
   * AutoConstants#kLineUpCenteredZAngleToleranceDegrees} of {@link
   * AutoConstants#kLineUpCenteredZAngleTargetDegrees}, and its Y translation (how far left/right
   * of the camera it is) within {@link AutoConstants#kLineUpCenteredYToleranceMeters} of {@link
   * AutoConstants#kLineUpCenteredYTargetMeters}. Those targets aren't exactly 180/0 - see their
   * javadoc for why.
   */
  private static boolean isCentered(double tagZAngleDegrees, double cameraToTagYMeters) {
    double zAngleErrorDegrees = Math.abs(Rotation2d.fromDegrees(tagZAngleDegrees)
        .minus(Rotation2d.fromDegrees(AutoConstants.kLineUpCenteredZAngleTargetDegrees)).getDegrees());
    double yErrorMeters = Math.abs(cameraToTagYMeters - AutoConstants.kLineUpCenteredYTargetMeters);
    return zAngleErrorDegrees <= AutoConstants.kLineUpCenteredZAngleToleranceDegrees
        && yErrorMeters <= AutoConstants.kLineUpCenteredYToleranceMeters;
  }

  /**
   * Turns the continuous "spin looking for a tag" behavior into alternating fast/slow pulses:
   * spin fast for {@link VisionConstants#kSearchSpinDegrees} degrees, spin slow for {@link
   * VisionConstants#kSearchSlowPhaseSeconds}, then spin fast again - repeat until a tag is seen.
   * Gives the camera a steadier look during the slow phase instead of only ever seeing tags blur
   * past mid-turn, without ever fully stopping.
   *
   * <p>Call {@link #reset} once when a search starts, then {@link #pulse} every loop while no
   * target is seen.
   */
  private static final class PulsedSearch {
    private final Timer m_slowPhaseTimer = new Timer();
    private Rotation2d m_lastHeading = Rotation2d.kZero;
    private double m_spunDegrees = 0.0;
    private boolean m_slowPhase = false;

    void reset(Rotation2d currentHeading) {
        m_lastHeading = currentHeading;
        m_spunDegrees = 0.0;
        m_slowPhase = false;
    }

    /** Rotational rate (rad/s) to command this loop - fast during a pulse, slow between them. */
    double pulse(Rotation2d currentHeading) {
        if (m_slowPhase) {
            if (m_slowPhaseTimer.hasElapsed(VisionConstants.kSearchSlowPhaseSeconds)) {
                m_slowPhase = false;
                m_spunDegrees = 0.0;
                m_lastHeading = currentHeading;
            }
            return Math.min(VisionConstants.kSearchSlowRotationRadPerSec, kMaxAngularRate);
        }

        m_spunDegrees += Math.abs(currentHeading.minus(m_lastHeading).getDegrees());
        m_lastHeading = currentHeading;

        if (m_spunDegrees >= VisionConstants.kSearchSpinDegrees) {
            m_slowPhase = true;
            m_slowPhaseTimer.restart();
            return Math.min(VisionConstants.kSearchSlowRotationRadPerSec, kMaxAngularRate);
        }
        return Math.min(VisionConstants.kSearchRotationRadPerSec, kMaxAngularRate);
    }
  }

  /**
   * Beeps briefly using every drivetrain motor at once (both drive and steer motors on every
   * module) - Kraken/Falcon (TalonFX) motors can play a tone directly, so no extra speaker
   * hardware is needed, and using all of them instead of just one makes it noticeably louder.
   * Alternates between two tones every {@link AutoConstants#kStepCompleteBeepNoteSeconds} for a
   * more attention-grabbing "beep-boop" instead of one flat tone. Used to audibly mark each
   * autonomous step finishing. Silences itself again afterward so it doesn't keep humming into
   * the next step.
   *
   * <p>{@code drivetrain.setControl(null)} first releases the motors from the drivetrain's own
   * background control thread - without this, that thread keeps re-commanding them in parallel
   * at high frequency and wins the race against our tone almost every time, so nothing is heard
   * (confirmed 2026-08-08: the tone was silently losing that race). Restores normal (stopped)
   * drivetrain control afterward.
   */
  private Command beepCommand() {
    List<TalonFX> beepMotors = new ArrayList<>();
    for (var module : drivetrain.getModules()) {
        beepMotors.add(module.getDriveMotor());
        beepMotors.add(module.getSteerMotor());
    }
    Timer beepTimer = new Timer();
    return Commands.sequence(
        Commands.runOnce(() -> RobotLog.log(String.format(
            "Step complete: beeping for %.2fs", AutoConstants.kStepCompleteBeepSeconds))),
        Commands.runOnce(() -> {
            drivetrain.setControl(null);
            beepTimer.restart();
        }, drivetrain),
        Commands.run(() -> {
            long noteIndex = (long) (beepTimer.get() / AutoConstants.kStepCompleteBeepNoteSeconds);
            double hz = (noteIndex % 2 == 0)
                ? AutoConstants.kStepCompleteBeepHz
                : AutoConstants.kStepCompleteBeepHz2;
            for (TalonFX beepMotor : beepMotors) {
                beepMotor.setControl(new MusicTone(hz));
            }
        }, drivetrain).withTimeout(AutoConstants.kStepCompleteBeepSeconds)
    ).finallyDo(() -> {
        for (TalonFX beepMotor : beepMotors) {
            beepMotor.setControl(new MusicTone(0));
        }
        drivetrain.setControl(autoDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    });
  }

  /**
   * Command to run in autonomous, built from the instructions in deploy/autonomous.json (see
   * README.md for the supported instructions). Returns null (nothing runs) if that file is
   * missing or has a mistake in it - {@link Robot} skips scheduling when this is null. Each step
   * beeps once it finishes - see {@link #beepCommand}.
   *
   * @return the command to run in autonomous, or null for none
   */
  public Command getAutonomousCommand() {
    if (m_autoSteps.isEmpty()) {
        return null;
    }
    return Commands.sequence(m_autoSteps.stream()
        .map(step -> autoStepCommand(step).andThen(beepCommand()))
        .toArray(Command[]::new));
  }
}
