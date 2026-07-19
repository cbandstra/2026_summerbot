// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

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
      Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY);

  // The drivetrain's true top translational speed - the throttle slider scales this down live
  // (see throttleSpeedPercent()) rather than a fixed fraction being applied here.
  private static final double kMaxSpeedMps = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  // Rotation has no slider control, so this stays a fixed fraction of the drivetrain's true top
  // rotational speed (see OperatorConstants.kMaxRotationOutputPercent).
  private double MaxAngularRate = (kMaxSpeedMps / kDriveBaseRadiusMeters) * OperatorConstants.kMaxRotationOutputPercent;

  // The robot's subsystems and commands are defined here...
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final Telemetry logger = new Telemetry(kMaxSpeedMps);

  // Thrustmaster T.16000M flight stick
  private final CommandJoystick m_driverController =
      new CommandJoystick(OperatorConstants.kDriverControllerPort);

  // Smooths raw joystick axis noise (flight stick pots are noisier than a gamepad's) before it
  // reaches the module angle calculation. Units are axis-units/sec - 3.0 means it takes about
  // 1/3 second to sweep from center to full deflection. Without this, small pot jitter around a
  // steady input (e.g. holding mostly-forward with a slight strafe) can flicker the commanded
  // module angle by a fraction of a degree every loop, which the steer motor faithfully chases -
  // audible as chatter even though the control loop itself is behaving correctly.
  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(3.0);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3.0);

  /*
   * Field-centric driving. Translation deadband/floor are computed manually each loop in
   * computeTranslationVelocity() (see below) since CTRE's built-in deadband can only zero small
   * inputs, not enforce a minimum nonzero output - so translation deadband is disabled here.
   * Rotation still uses the built-in deadband since it has no floor requirement.
   */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(0)
      .withRotationalDeadband(MaxAngularRate * OperatorConstants.kRotationDeadband)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

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
                .withRotationalRate(-m_rotLimiter.calculate(m_driverController.getRawAxis(OperatorConstants.kThrustmasterTwistAxis)) * MaxAngularRate); // CCW is stick twisted left (negative twist)
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

    // Hold the thumb button to point all wheels toward the stick's direction (module testing aid)
    m_driverController.button(OperatorConstants.kThrustmasterThumbButton)
        .whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(
                -m_driverController.getRawAxis(OperatorConstants.kThrustmasterYAxis),
                -m_driverController.getRawAxis(OperatorConstants.kThrustmasterXAxis)))
        ));

    // Run SysId routines when the driver holds the POV hat up/down - each should only be run
    // once per log. See CommandSwerveDrivetrain for how to switch which routine (translation/
    // steer/rotation) these bindings exercise.
    m_driverController.povUp().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    m_driverController.povDown().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));

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
   * Reads the driving stick, applies input smoothing, and returns robot-relative
   * {@code {velocityX, velocityY}} in m/s, with two rules layered on top of the raw
   * {@code stick * maxSpeed} scaling:
   *
   * <ul>
   *   <li>Below {@link OperatorConstants#kTranslationDeadband} of {@code maxSpeed}, output is
   *       zero (the deadband shrinks/grows with the slider, since it's a fraction of the
   *       currently-selected top speed).
   *   <li>Above that deadband, output magnitude is floored to at least {@link
   *       OperatorConstants#kMinOutputPercent} of the drivetrain's TRUE top speed - an absolute
   *       floor that does not shrink with the slider, since it represents the speed below which
   *       the wheels don't move usefully regardless of what the slider is set to. The floor is
   *       clamped to {@code maxSpeed} so it can never command more than the slider currently
   *       allows.
   * </ul>
   *
   * @param maxSpeed the slider-scaled top speed (m/s) to scale stick deflection by
   */
  private double[] computeTranslationVelocity(double maxSpeed) {
    double rawX = -m_xLimiter.calculate(m_driverController.getRawAxis(OperatorConstants.kThrustmasterYAxis)) * maxSpeed; // forward is stick pushed away (negative Y)
    double rawY = -m_yLimiter.calculate(m_driverController.getRawAxis(OperatorConstants.kThrustmasterXAxis)) * maxSpeed; // left is stick pushed left (negative X)
    double magnitude = Math.hypot(rawX, rawY);

    double deadbandThreshold = maxSpeed * OperatorConstants.kTranslationDeadband;
    if (magnitude < deadbandThreshold) {
        return new double[] {0.0, 0.0};
    }

    double floor = Math.min(OperatorConstants.kMinOutputPercent * kMaxSpeedMps, maxSpeed);
    double targetMagnitude = Math.max(magnitude, floor);
    double scale = targetMagnitude / magnitude;
    return new double[] {rawX * scale, rawY * scale};
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // No autonomous routine has been built yet.
    return Commands.none();
  }
}
