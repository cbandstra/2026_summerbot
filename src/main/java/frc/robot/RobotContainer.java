// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
      Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY);

  // The drivetrain's true top translational speed - the throttle slider scales this down live
  // (see throttleSpeedPercent()) rather than a fixed fraction being applied here.
  private static final double kMaxSpeedMps = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  // Rotation has no slider control, so this stays a fixed fraction of the drivetrain's true top
  // rotational speed (see OperatorConstants.kMaxRotationOutputPercent).
  private double MaxAngularRate = (kMaxSpeedMps / kDriveBaseRadiusMeters) * OperatorConstants.kMaxRotationOutputPercent;

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

  // Closes distance to the target tag during the tag-search-and-approach button (see the button
  // 4 binding below). Input is measured ground-plane distance (m), setpoint is
  // kApproachDistanceMeters, output is forward speed (m/s).
  private final PIDController m_approachDistanceController = new PIDController(
      VisionConstants.kApproachDistanceKP,
      VisionConstants.kApproachDistanceKI,
      VisionConstants.kApproachDistanceKD
  );

  // Strafes to actually square the robot up with the target tag's face during the tag-search-
  // and-approach button (see the button 4 binding below), rather than just gating on it - input
  // is the signed tag-face angle (degrees), setpoint 0, output is sideways speed (m/s).
  private final PIDController m_tagFaceAlignController = new PIDController(
      VisionConstants.kTagFaceAlignKP,
      VisionConstants.kTagFaceAlignKI,
      VisionConstants.kTagFaceAlignKD
  );

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

  // Robot-centric (not field-centric) since the tag-search-and-approach button (button 4) always
  // means "drive toward whatever the camera currently sees," relative to the robot's own facing,
  // not a fixed field direction.
  private final SwerveRequest.RobotCentric approach = new SwerveRequest.RobotCentric()
      .withDeadband(0)
      .withRotationalDeadband(0)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

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

    // Hold button 2 (thumb button) to auto-rotate toward the best-seen AprilTag while
    // driving/strafing normally - translation still comes straight from the stick (see
    // computeTranslationVelocity), only rotation is overridden. If no tag is visible, rotation
    // falls back to the twist stick (same as normal manual driving) so the driver isn't locked
    // out of rotating while searching for a tag to align to.
    m_driverController.button(OperatorConstants.kThrustmasterThumbButton).whileTrue(
        Commands.startRun(
            () -> m_alignRotationController.reset(),
            () -> {
                double maxSpeed = kMaxSpeedMps * throttleSpeedPercent();
                double[] translation = computeTranslationVelocity(maxSpeed);
                double rotationalRate;
                if (vision.hasTarget()) {
                    rotationalRate = computeAlignRotationalRate();
                } else {
                    rotationalRate = computeManualRotationalRate();
                }
                drivetrain.setControl(drive.withVelocityX(translation[0])
                    .withVelocityY(translation[1])
                    .withRotationalRate(rotationalRate));
            },
            drivetrain, vision
        )
    );

    // Hold button 3 to spin in place looking for any AprilTag, then auto-align to it once seen.
    // Reuses m_alignRotationController - safe since button 2 and button 3 both require
    // drivetrain/vision, so the command scheduler only ever runs one of these at a time even if
    // both buttons are held, and startRun() resets the controller on whichever one (re)starts.
    m_driverController.button(OperatorConstants.kThrustmasterSearchButton).whileTrue(
        Commands.startRun(
            () -> m_alignRotationController.reset(),
            () -> {
                double maxSpeed = kMaxSpeedMps * throttleSpeedPercent();
                double[] translation = computeTranslationVelocity(maxSpeed);
                double rotationalRate;
                if (vision.hasTarget()) {
                    rotationalRate = computeAlignRotationalRate();
                } else {
                    rotationalRate = Math.min(VisionConstants.kSearchRotationRadPerSec, MaxAngularRate);
                }
                drivetrain.setControl(drive.withVelocityX(translation[0])
                    .withVelocityY(translation[1])
                    .withRotationalRate(rotationalRate));
            },
            drivetrain, vision
        )
    );

    // Press button 4 to toggle the tag-search-and-approach routine on: searches (in
    // VisionConstants.kSearchTagIdOrder priority order) for one of a specific set of AprilTag
    // IDs, then autonomously drives to within kApproachDistanceMeters of it once found, holds
    // there for kApproachSettleSeconds once arrived, then finishes on its own (control reverts
    // to the normal driving default command). Pressing button 4 again while it's still running
    // cancels it early. Unlike buttons 2/3, this takes over BOTH rotation and translation - a
    // fully automatic "go do this" action, not a driving assist. This is the first autonomous-
    // translation behavior on this robot (buttons 2/3 only ever touched rotation) - test
    // cautiously (blocks first) until the distance PID's sign/behavior is confirmed correct.
    m_driverController.button(OperatorConstants.kThrustmasterTagSearchButton)
        .toggleOnTrue(tagSearchAndApproachCommand());

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /**
   * Builds the tag-search-and-approach command bound to button 4 (see the binding's comment for
   * the overall behavior). Structured as search/approach (runs until "arrived": both squared up
   * within {@link VisionConstants#kAlignYawToleranceDegrees} and within {@link
   * VisionConstants#kApproachDistanceToleranceMeters} of the target standoff distance) followed
   * by a hold phase that keeps station for {@link VisionConstants#kApproachSettleSeconds} before
   * finishing on its own. {@code arrivedTimer} is captured by both phases' lambdas - {@link
   * edu.wpi.first.wpilibj.Timer#start()} is a no-op if already running, so calling it every loop
   * while arrived doesn't reset the elapsed time, but any loop where the robot drifts back out of
   * tolerance stops and resets it, requiring a fresh, continuous
   * {@code kApproachSettleSeconds} of being settled rather than a cumulative one.
   *
   * <p>Priority-order search does NOT just grab whichever of {@link
   * VisionConstants#kSearchTagIdOrder} happens to sweep into view first - {@code soughtIndex}
   * tracks which single ID it's currently committed to, starting at index 0 (highest priority).
   * If that specific tag hasn't turned up after a full 360-degree search sweep, it gives up on it
   * and advances to the next ID, rather than settling for a lower-priority tag just because it
   * happened to be seen first. The sweep is measured from the robot's ACTUAL odometry rotation
   * (accumulated per-loop heading deltas, since a single current-vs-start {@code Rotation2d}
   * comparison would wrap back near zero once a full revolution completes, hiding exactly the
   * event we're trying to detect) rather than assumed from the commanded search rate x time -
   * {@code OpenLoopVoltage} control has real acceleration ramp-up and no closed-loop guarantee of
   * hitting the commanded rate exactly, so a timing-based estimate can run out before a real 360
   * degrees has actually been swept, prematurely restarting the sweep right as the tag was about
   * to come into view.
   */
  private Command tagSearchAndApproachCommand() {
    Timer arrivedTimer = new Timer();
    int[] soughtIndex = {0};
    double[] sweptDegrees = {0.0};
    Rotation2d[] lastHeading = {null};

    Runnable driveTowardTarget = () -> {
        int soughtId = VisionConstants.kSearchTagIdOrder[
            Math.min(soughtIndex[0], VisionConstants.kSearchTagIdOrder.length - 1)];
        Optional<PhotonTrackedTarget> target = vision.getTargetById(soughtId);
        SmartDashboard.putNumber("TagSearch/SoughtId", soughtId);

        double rotationalRate;
        double forwardSpeed;
        double lateralSpeed = 0.0;
        boolean arrived = false;

        if (target.isPresent()) {
            sweptDegrees[0] = 0.0;
            lastHeading[0] = null;

            PhotonTrackedTarget seenTarget = target.get();
            int seenTargetId = seenTarget.getFiducialId();
            double compensatedYawDegrees = computeCompensatedYawDegrees(
                seenTarget.getYaw(), vision.getTargetTimestampSeconds());
            rotationalRate = MathUtil.clamp(
                m_alignRotationController.calculate(compensatedYawDegrees, 0.0),
                -MaxAngularRate, MaxAngularRate
            );

            // Ground-plane distance (ignores the camera/tag height difference) from the
            // camera-to-target 3D transform - requires the PhotonVision pipeline to have a valid
            // camera calibration for this to be meaningful.
            var cameraToTarget = seenTarget.getBestCameraToTarget().getTranslation();
            double distanceMeters = Math.hypot(cameraToTarget.getX(), cameraToTarget.getY());
            double distanceErrorMeters = distanceMeters - VisionConstants.kApproachDistanceMeters;
            // Negated: PIDController.calculate(measurement, setpoint) gives a NEGATIVE output
            // when measurement > setpoint (too far away), but driving forward (positive
            // velocityX) is exactly what CLOSES that distance - the opposite of a typical PID
            // relationship, where positive output increases the measurement. Confirmed backwards
            // on the robot without this negation (drove away from the tag instead of toward it).
            double approachOutput = -m_approachDistanceController.calculate(
                distanceMeters, VisionConstants.kApproachDistanceMeters);

            // Don't start closing distance until roughly squared up with the tag - avoids
            // crabbing in at a steep angle while still mid-rotation.
            boolean squaredUp = Math.abs(compensatedYawDegrees)
                <= VisionConstants.kApproachYawToleranceDegrees;
            forwardSpeed = squaredUp
                ? MathUtil.clamp(approachOutput,
                    -VisionConstants.kApproachMaxSpeedMps, VisionConstants.kApproachMaxSpeedMps)
                : 0.0;

            // --- Squaring-up (tag-face alignment) DISABLED 2026-07-23 ---
            // This actively strafed to close the tag's own face-angle error (distinct from
            // compensatedYawDegrees, which is the tag's left/right position in frame - this is
            // whether the tag's face itself is turned away from the camera). Removed because
            // running it alongside the rotation-centering loop caused the robot to circle the
            // target: strafing shifts the tag's bearing, which triggers a rotation correction,
            // which changes what "squared" reads as, which triggers more strafing - the two
            // reactive loops fighting each other instead of converging. The correct fix is NOT
            // more gain tuning but a different architecture: compute a single target transform
            // (the point kApproachDistanceMeters directly in front of the tag, facing it) via
            // seenTarget.getBestCameraToTarget().plus(...) and drive at that one computed point
            // with simple proportional control on its resulting x/y/theta, instead of reacting to
            // yaw/distance/face-angle as three independent numbers. Re-enable via that redesign,
            // not by uncommenting this block as-is.
            //
            // double signedTagFaceAngleDegrees = MathUtil.inputModulus(
            //     Math.toDegrees(seenTarget.getBestCameraToTarget().getRotation().getZ()) - 180.0,
            //     -180.0, 180.0
            // );
            // double tagFaceAngleDegrees = Math.abs(signedTagFaceAngleDegrees);
            // lateralSpeed = squaredUp
            //     ? MathUtil.clamp(-m_tagFaceAlignController.calculate(signedTagFaceAngleDegrees, 0.0),
            //         -VisionConstants.kApproachMaxSpeedMps, VisionConstants.kApproachMaxSpeedMps)
            //     : 0.0;

            arrived = Math.abs(compensatedYawDegrees) <= VisionConstants.kAlignYawToleranceDegrees
                && Math.abs(distanceErrorMeters) <= VisionConstants.kApproachDistanceToleranceMeters;

            SmartDashboard.putNumber("TagSearch/TargetId", seenTargetId);
            SmartDashboard.putNumber("TagSearch/YawErrorDegrees", compensatedYawDegrees);
            SmartDashboard.putNumber("TagSearch/DistanceMeters", distanceMeters);
            SmartDashboard.putNumber("TagSearch/DistanceErrorMeters", distanceErrorMeters);
        } else {
            Rotation2d currentHeading = drivetrain.getState().Pose.getRotation();
            if (lastHeading[0] != null) {
                sweptDegrees[0] += Math.abs(currentHeading.minus(lastHeading[0]).getDegrees());
            }
            lastHeading[0] = currentHeading;

            if (sweptDegrees[0] >= 360.0
                && soughtIndex[0] < VisionConstants.kSearchTagIdOrder.length - 1) {
                soughtIndex[0]++;
                sweptDegrees[0] = 0.0;
            }

            SmartDashboard.putNumber("TagSearch/SweptDegrees", sweptDegrees[0]);
            rotationalRate = Math.min(VisionConstants.kSearchRotationRadPerSec, MaxAngularRate);
            forwardSpeed = 0.0;
            SmartDashboard.putNumber("TagSearch/TargetId", -1);
        }

        if (arrived) {
            arrivedTimer.start();
            // Hold station once arrived, ignoring any residual PID jitter.
            forwardSpeed = 0.0;
            lateralSpeed = 0.0;
        } else {
            arrivedTimer.stop();
            arrivedTimer.reset();
        }

        SmartDashboard.putBoolean("TagSearch/Arrived", arrived);
        SmartDashboard.putNumber("TagSearch/ArrivedTimerSeconds", arrivedTimer.get());

        drivetrain.setControl(approach.withVelocityX(forwardSpeed)
            .withVelocityY(lateralSpeed)
            .withRotationalRate(rotationalRate));
    };

    return Commands.runOnce(() -> {
            m_alignRotationController.reset();
            m_approachDistanceController.reset();
            m_tagFaceAlignController.reset();
            arrivedTimer.stop();
            arrivedTimer.reset();
            soughtIndex[0] = 0;
            sweptDegrees[0] = 0.0;
            lastHeading[0] = null;
        }, drivetrain, vision)
        .andThen(Commands.run(driveTowardTarget, drivetrain, vision)
            .until(() -> arrivedTimer.hasElapsed(VisionConstants.kApproachSettleSeconds)));
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
   * Rotational rate (rad/s, clamped to MaxAngularRate) to turn the robot toward the best-seen
   * AprilTag, latency-compensated. The camera frame behind {@link Vision#getTargetYawDegrees()}
   * is always some pipeline/network latency old (measured ~60ms on this rig) - by the time it's
   * read here, the robot has kept rotating for that whole delay, so reacting to the raw yaw as if
   * it were current causes a real overshoot (robot turns past where the tag actually is "now").
   * This corrects for that using {@link CommandSwerveDrivetrain#samplePoseAt}, which reconstructs
   * the robot's own heading at the frame's capture timestamp from its odometry history: the
   * difference between that historical heading and the current one is exactly how far the robot
   * has rotated since the frame was taken, which gets subtracted back out of the raw yaw. Falls
   * back to the raw (uncompensated) yaw if the odometry buffer doesn't reach back that far (e.g.
   * right at startup). PhotonVision's timestamp is in the FPGA/NT4 epoch, but samplePoseAt()
   * expects CTRE's own {@code Utils.getCurrentTimeSeconds()} epoch - those are different clocks
   * in Phoenix 6, so the timestamp must go through {@link Utils#fpgaToCurrentTime} first or
   * samplePoseAt() just returns empty every time and this silently does nothing.
   *
   * <p>Confirmed on the robot: this camera/mount needs the raw (non-negated) yaw sign to turn
   * toward the target rather than away from it - both PhotonVision's yaw and WPILib's Rotation2d
   * use the same positive-CCW convention, so this compensation needs no extra sign flip either.
   */
  private double computeAlignRotationalRate() {
    return computeAlignRotationalRate(vision.getTargetYawDegrees(), vision.getTargetTimestampSeconds());
  }

  /**
   * Same latency-compensated align math as {@link #computeAlignRotationalRate()}, but for an
   * arbitrary target's yaw/frame-timestamp instead of always reading PhotonVision's overall
   * "best" target - used by the tag-search-and-approach button (button 4) to align to a
   * specific tag ID rather than whichever target PhotonVision considers best.
   */
  private double computeAlignRotationalRate(double rawYawDegrees, double frameTimestampSeconds) {
    double compensatedYawDegrees = computeCompensatedYawDegrees(rawYawDegrees, frameTimestampSeconds);
    return MathUtil.clamp(
        m_alignRotationController.calculate(compensatedYawDegrees, 0.0),
        -MaxAngularRate, MaxAngularRate
    );
  }

  /**
   * The latency-compensation math shared by both {@link #computeAlignRotationalRate(double,
   * double)} and the tag-search-and-approach button (button 4), which also needs the compensated
   * yaw value itself (not just the resulting rotational rate) to gate forward approach speed on
   * how squared-up the robot currently is. See computeAlignRotationalRate's javadoc for why this
   * compensation is needed and the FPGA/CTRE epoch conversion it depends on.
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
   * reaches {@code MaxAngularRate}.
   */
  private double computeManualRotationalRate() {
    double stickTwist = -m_rotLimiter.calculate(m_driverController.getRawAxis(OperatorConstants.kThrustmasterTwistAxis)); // CCW is stick twisted left (negative twist)
    double stickMagnitude = Math.abs(stickTwist);

    if (stickMagnitude < OperatorConstants.kRotationDeadband) {
        return 0.0;
    }

    double stickFraction = Math.min(stickMagnitude, 1.0);
    double curvedFraction = Math.pow(stickFraction, OperatorConstants.kRotationCurveExponent);
    return Math.signum(stickTwist) * curvedFraction * MaxAngularRate;
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
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // No autonomous routine has been built yet.
    return Commands.none();
  }
}
