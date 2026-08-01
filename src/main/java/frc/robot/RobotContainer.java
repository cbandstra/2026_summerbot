// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Distance from the robot's center to a module - the radius the wheels turn on when spinning
  // in place. Used to convert the drivetrain's true top translational speed into its true top
  // rotational speed (rad/s = m/s / radius), so the rotation safety cap below is scaled from the
  // same physical ceiling as the translation cap instead of an arbitrary fixed rotation rate.
  private static final double kDriveBaseRadiusMeters =
      Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY);

  // The drivetrain's true top translational speed - the throttle slider scales this down live
  // (see throttleSpeedPercent()) rather than a fixed fraction being applied here.
  private static final double kMaxSpeedMps = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  // Rotation has no slider control, so this stays a fixed fraction of the drivetrain's true top
  // rotational speed (see OperatorConstants.kMaxRotationOutputPercent).
  private static final double kMaxAngularRate =
      (kMaxSpeedMps / kDriveBaseRadiusMeters) * OperatorConstants.kMaxRotationOutputPercent;

  // The robot's subsystems and commands are defined here...
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final Vision vision = new Vision();

  private final Telemetry logger = new Telemetry(kMaxSpeedMps);

  // Rotates the robot to center the best-seen AprilTag in frame (see the button 2 binding
  // below). Input/setpoint are yaw error in degrees, output is rad/s.
  private final PIDController m_alignRotationController = new PIDController(
      VisionConstants.kAlignRotationKP,
      VisionConstants.kAlignRotationKI,
      VisionConstants.kAlignRotationKD
  );

  // Tracks whether button 2 has already logged "Looking for April tags" for the current
  // no-target streak, so it only logs once per loss (including right at button press if nothing
  // is visible yet) rather than every loop while still searching.
  private boolean m_loggedSearching = false;

  // Thrustmaster T.16000M flight stick
  private final CommandJoystick m_driverController =
      new CommandJoystick(OperatorConstants.kDriverControllerPort);

  // Smooths raw joystick axis noise (flight stick pots are noisier than a gamepad's) before the
  // module-angle calculation. Units are axis-units/sec - 3.0 sweeps center to full in ~1/3 s.
  // Without it, pot jitter around a steady input flickers the commanded module angle every loop,
  // which the steer motor faithfully chases - audible as chatter even though the loop is fine.
  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3.0);

  /*
   * Field-centric driving. Translation and rotation deadband/curve shaping are both computed
   * manually each loop (see computeTranslationVelocity() and computeManualRotationalRate()
   * below) since CTRE's built-in deadband can only zero small inputs - it can't apply a response
   * curve above the deadband, which rotation now needs too. Both are disabled here.
   */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(0)
      .withRotationalDeadband(0)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link edu.wpi.first.wpilibj2.command.button.Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * edu.wpi.first.wpilibj2.command.button.CommandXboxController Xbox}/{@link
   * edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or {@link
   * CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    // Raw axis indices below are the Thrustmaster T.16000M's standard USB report order -
    // confirm on the Driver Station's USB Devices tab before flying/driving.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> {
            double maxSpeed = kMaxSpeedMps * throttleSpeedPercent();
            double[] translation = computeTranslationVelocity(maxSpeed);
            return drive.withVelocityX(translation[0])
                .withVelocityY(translation[1])
                .withRotationalRate(computeManualRotationalRate());
        })
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    // Hold the trigger to lock the wheels in an X pattern (resists being pushed)
    m_driverController.button(OperatorConstants.kThrustmasterTriggerButton)
        .whileTrue(drivetrain.applyRequest(() -> brake));

    // Hold button 2 (thumb button) to spin in place looking for any AprilTag, then auto-align to
    // it once seen. Button 3 is intentionally left unbound (2026-07-25) - this behavior used to
    // live there.
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

  /**
   * Maps the throttle slider's raw axis reading ([-1, 1]) to a fraction of the drivetrain's true
   * top translational speed, linearly between {@link OperatorConstants#kSliderMaxSpeedPercent}
   * (slider all the way back, -1) and {@link OperatorConstants#kSliderMinSpeedPercent} (slider
   * all the way forward, +1).
   */
  private double throttleSpeedPercent() {
    double axis = m_driverController.getRawAxis(OperatorConstants.kThrustmasterSliderAxis);
    double t = (1.0 - axis) / 2.0; // -1 -> 1 (max), +1 -> 0 (min)
    return OperatorConstants.kSliderMinSpeedPercent
        + t * (OperatorConstants.kSliderMaxSpeedPercent - OperatorConstants.kSliderMinSpeedPercent);
  }

  /**
   * Rotational rate (rad/s, clamped to kMaxAngularRate) to turn toward the best-seen AprilTag,
   * latency-compensated (see {@link #computeCompensatedYawDegrees}).
   *
   * <p>Confirmed on the robot: this camera/mount needs the raw (non-negated) yaw sign to turn
   * toward the target rather than away from it - PhotonVision's yaw and WPILib's Rotation2d share
   * the positive-CCW convention, so no extra sign flip is needed here.
   */
  private double computeAlignRotationalRate() {
    double compensatedYawDegrees = computeCompensatedYawDegrees(
        vision.getTargetYawDegrees(), vision.getTargetTimestampSeconds());
    return MathUtil.clamp(
        m_alignRotationController.calculate(compensatedYawDegrees, 0.0),
        -kMaxAngularRate, kMaxAngularRate
    );
  }

  /**
   * Latency-compensates a raw camera yaw (deg). The frame it came from is always some
   * pipeline/network latency old (~60ms on this rig), during which the robot kept rotating, so
   * reacting to the raw yaw as if it were current overshoots. {@link
   * CommandSwerveDrivetrain#samplePoseAt} reconstructs the robot's heading at the frame's capture
   * time from odometry history; the difference from the current heading is how far it has rotated
   * since, which is subtracted back out. Falls back to the raw yaw if odometry doesn't reach back
   * that far (e.g. at startup).
   *
   * <p>PhotonVision's timestamp is in the FPGA/NT4 epoch, but samplePoseAt() expects CTRE's
   * {@code Utils.getCurrentTimeSeconds()} epoch - different clocks in Phoenix 6, so it must go
   * through {@link Utils#fpgaToCurrentTime} first or samplePoseAt() returns empty every time and
   * this silently does nothing.
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
   * Reads the twist axis, applies input smoothing, and returns a rotational rate in rad/s -
   * the manual rotation control used both for normal driving and as the align command's
   * fallback when no tag is visible to auto-rotate toward. Mirrors
   * computeTranslationVelocity()'s deadband + curve shaping (see there for the full rationale):
   * below {@link OperatorConstants#kRotationDeadband}, output is zero; above it, the response is
   * raised to {@link OperatorConstants#kRotationCurveExponent} so small twists produce
   * proportionally less rotation than a linear mapping would, while full deflection still
   * reaches {@code kMaxAngularRate}.
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
   * Reads the driving stick, applies input smoothing, and returns robot-relative
   * {@code {velocityX, velocityY}} in m/s. Direction is taken straight from the stick; the
   * requested speed (as a fraction of {@code maxSpeed}) goes through three rules in order:
   *
   * <ul>
   *   <li>Below {@link OperatorConstants#kTranslationDeadband} of full stick deflection, output
   *       is zero.
   *   <li>Above that deadband, the stick fraction is raised to {@link
   *       OperatorConstants#kTranslationCurveExponent} before being scaled by {@code maxSpeed} -
   *       this compresses the low end of the stick's range so small movements move slower than a
   *       linear mapping would, without affecting the top end (full stick is still full
   *       {@code maxSpeed}).
   *   <li>The result is then floored to at least {@link OperatorConstants#kMinOutputPercent} of
   *       the drivetrain's TRUE top speed - an absolute floor that does not shrink with the
   *       slider, since it represents the speed below which the wheels don't move usefully
   *       regardless of what the slider is set to. The floor is clamped to {@code maxSpeed} so it
   *       can never command more than the slider currently allows.
   * </ul>
   *
   * @param maxSpeed the slider-scaled top speed (m/s) to scale stick deflection by
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

  /**
   * Command to run in autonomous. No autonomous routine is wired up - {@link Robot} skips
   * scheduling when this is null.
   *
   * @return the command to run in autonomous, or null for none
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
