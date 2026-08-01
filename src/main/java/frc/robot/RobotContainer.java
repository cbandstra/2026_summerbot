// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

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

  // Turns the robot to face an AprilTag - shared by button 2 and the "align with april tag"
  // autonomous instruction. Input/output: yaw error in degrees, rad/s out.
  private final PIDController m_alignRotationController = new PIDController(
      VisionConstants.kAlignRotationKP,
      VisionConstants.kAlignRotationKI,
      VisionConstants.kAlignRotationKD
  );

  // So "Looking for April tags" only logs once per search, not every loop.
  private boolean m_loggedSearching = false;

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

    // Idle (don't fight the brakes) while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    // Hold the trigger to lock the wheels in an X pattern (resists being pushed).
    m_driverController.button(OperatorConstants.kThrustmasterTriggerButton)
        .whileTrue(drivetrain.applyRequest(() -> brake));

    // Hold button 2: spin looking for any AprilTag, then turn to face it once seen. Translation
    // stays on the stick the whole time - only rotation is taken over.
    m_driverController.button(OperatorConstants.kThrustmasterThumbButton).whileTrue(
        Commands.startRun(
            () -> {
                m_alignRotationController.reset();
                m_loggedSearching = false;
            },
            () -> {
                double maxSpeed = kMaxSpeedMps * throttleSpeedPercent();
                double[] translation = computeTranslationVelocity(maxSpeed);
                double rotationalRate;
                if (vision.hasTarget()) {
                    rotationalRate = computeAlignRotationalRate();
                    m_loggedSearching = false;
                } else {
                    if (!m_loggedSearching) {
                        RobotLog.log("Looking for April tags");
                        m_loggedSearching = true;
                    }
                    rotationalRate = Math.min(VisionConstants.kSearchRotationRadPerSec, kMaxAngularRate);
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
    return new double[] {unitX * outputSpeed, unitY * outputSpeed};
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

    return Commands.sequence(
        Commands.runOnce(() -> m_alignRotationController.reset(), drivetrain),
        Commands.run(() -> {
            var target = vision.getTargetById(tagId);
            double rotationalRate;
            if (target.isPresent()) {
                double rawYawDegrees = target.get().getYaw();
                lastYawErrorDegrees[0] = computeCompensatedYawDegrees(rawYawDegrees, vision.getTargetTimestampSeconds());
                rotationalRate = computeAlignRotationalRate(rawYawDegrees, vision.getTargetTimestampSeconds());
            } else {
                lastYawErrorDegrees[0] = Double.MAX_VALUE;
                rotationalRate = Math.min(VisionConstants.kSearchRotationRadPerSec, kMaxAngularRate);
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
   * Command to run in autonomous, built from the instructions in deploy/autonomous.json (see
   * README.md for the supported instructions). Returns null (nothing runs) if that file is
   * missing or has a mistake in it - {@link Robot} skips scheduling when this is null.
   *
   * @return the command to run in autonomous, or null for none
   */
  public Command getAutonomousCommand() {
    if (m_autoSteps.isEmpty()) {
        return null;
    }
    return Commands.sequence(m_autoSteps.stream().map(this::autoStepCommand).toArray(Command[]::new));
  }
}
