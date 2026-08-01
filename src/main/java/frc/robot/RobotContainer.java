// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.UltrasonicConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ObstacleSensor;
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
  // Read from FrontLeft since only FrontLeft/BackRight are actually wired up right now (see
  // TunerConstants.createDrivetrain()) - numerically identical regardless of which module this
  // reads given the chassis's symmetric 19.75in x 19.75in footprint.
  private static final double kDriveBaseRadiusMeters =
      Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY);

  // The drivetrain's true top translational speed - the throttle slider scales this down live
  // (see throttleSpeedPercent()) rather than a fixed fraction being applied here.
  private static final double kMaxSpeedMps = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  // Rotation has no slider control, so this stays a fixed fraction of the drivetrain's true top
  // rotational speed (see OperatorConstants.kMaxRotationOutputPercent).
  private double MaxAngularRate = (kMaxSpeedMps / kDriveBaseRadiusMeters) * OperatorConstants.kMaxRotationOutputPercent;

  // The Pigeon 2 is mounted backwards on this chassis - its raw yaw reads 180 degrees off from
  // the robot's true physical front. Used both by the recenter button (to bake the correction
  // into the seeded field-centric forward) and by the POV rotate-to-heading feature (to correct
  // the raw gyro reading it reads directly - see the POV bindings below for why that reads the
  // gyro directly instead of going through the recenter-affected pose heading).
  private static final double kPigeonMountOffsetDegrees = 180.0;

  // The robot's subsystems and commands are defined here...
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final Vision vision = new Vision();
  // No avoidance behavior wired up yet - just publishes distance readings to the dashboard so
  // wiring/orientation can be confirmed first (see Constants.UltrasonicConstants for the
  // required voltage divider on the Echo line).
  public final ObstacleSensor obstacleSensor = new ObstacleSensor();

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

  // Rotates the robot to an exact absolute field-relative heading while a POV (hat switch)
  // direction is held (see the POV bindings below). Input/setpoint are the robot's current/target
  // heading in degrees - continuous input lets it always turn the shorter way around the
  // +/-180 degree wraparound instead of spinning the long way when the target is, e.g., just
  // across the line from the current heading (179 deg to -179 deg).
  private final PIDController m_headingHoldController = new PIDController(
      OperatorConstants.kHeadingHoldKP,
      OperatorConstants.kHeadingHoldKI,
      OperatorConstants.kHeadingHoldKD
  );
  {
      m_headingHoldController.enableContinuousInput(-180.0, 180.0);
  }

  // Ordered list of poses recorded while button 5 is held (see recordWaypointCommand()) - button
  // 6 drives through them in recorded order, button 7 clears the whole list. Empty (not null)
  // when nothing is recorded. Odometry poses, not raw Pigeon heading, since both recording and
  // playback go through the same drivetrain.getState().Pose frame, so it's self-consistent
  // regardless of any recentering (button 1) done in between. Read LIVE (by index) from
  // driveWaypointPathCommand() rather than a frozen snapshot taken at button-6-press time - see
  // that method's comment for why appending or clearing mid-run both do something sensible as a
  // result.
  private final List<Pose2d> m_waypoints = new ArrayList<>();

  // Field-relative heading (absolute, not robot-relative) each AprilTag ID was last seen at,
  // keyed by fiducial ID - persists across separate tag-search-and-approach runs (a class field,
  // not local to tagSearchAndApproachCommand()) so a later search for the same ID can start by
  // turning the shorter way toward it instead of always sweeping the same fixed direction. Only
  // ever written to, never cleared - a stale entry just means a possibly-wrong first guess at
  // search direction, not a correctness problem, since the full-360 give-up sweep still covers
  // every direction regardless.
  private final Map<Integer, Rotation2d> m_lastKnownTagBearings = new HashMap<>();

  // Tracks whether button 2 has already logged "Looking for April tags" for the current
  // no-target streak, so it only logs once per loss (including right at button press if nothing
  // is visible yet) rather than every loop while still searching.
  private boolean m_loggedSearching = false;

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

  // Used ONLY by driveLegCommand() (3x3 ft Square auto) and driveWaypointPathCommand() (button 6) -
  // both compute their velocity vectors directly from drivetrain.getState().Pose, which is always
  // in the TRUE, alliance-independent field frame (blue-alliance-origin axes). `drive` above
  // defaults to ForwardPerspectiveValue.OperatorPerspective, which is correct for stick input
  // (push-away-from-driver semantics, alliance-relative) but would silently rotate an
  // already-true-field-frame vector by another +/-180 degrees on red alliance, sending the robot
  // the wrong way. BlueAlliance perspective tells FieldCentric the input is already in true field
  // axes and skips that extra rotation - this is the exact "path following" use case CTRE's own
  // ForwardPerspectiveValue.BlueAlliance javadoc describes. A SEPARATE SwerveRequest instance
  // (not just changing `drive`'s perspective inline) because `with*()` mutates and returns the
  // SAME object - reusing `drive` here would leave ForwardPerspective stuck on BlueAlliance for
  // every OTHER call site (manual driving, POV, tag-align) the next time any of them run.
  private final SwerveRequest.FieldCentric driveFieldFrame = new SwerveRequest.FieldCentric()
      .withDeadband(0)
      .withRotationalDeadband(0)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance);

  // Robot-centric (not field-centric) since the tag-search-and-approach button (button 4) always
  // means "drive toward whatever the camera currently sees," relative to the robot's own facing,
  // not a fixed field direction.
  private final SwerveRequest.RobotCentric approach = new SwerveRequest.RobotCentric()
      .withDeadband(0)
      .withRotationalDeadband(0)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Lets the driver pick which autonomous routine getAutonomousCommand() returns, published as
  // a widget ("Auto Mode") on SmartDashboard/Shuffleboard/Elastic. Defaults to the existing tag-
  // search-and-approach behavior (unchanged from before the square routine existed) so nothing
  // about prior autonomous behavior silently changes just because a new option was added -
  // "3x3 ft Square" must be explicitly selected to run instead.
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_autoChooser.setDefaultOption("Tag Search And Approach", tagSearchAndApproachCommand());
    m_autoChooser.addOption("3x3 ft Square", autoSquareCommand());
    SmartDashboard.putData("Auto Mode", m_autoChooser);
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

    // Press the trigger (button 1) to recenter field-centric "forward" on wherever the robot is
    // currently facing - useful after the robot ends up turned some way that no longer matches
    // the driver's sense of forward (e.g. after auto, or after a POV rotate/tag-align leaves it
    // facing an odd direction) without having to re-seed odometry/pose entirely. Was previously
    // the brake/wheel-lock button; that behavior isn't bound to anything now.
    //
    // Seeded to kPigeonMountOffsetDegrees (180) rather than 0: seedFieldCentric(rotation) records
    // "current physical orientation minus rotation" as the new field-centric forward. The Pigeon
    // 2's mounting error (see kPigeonMountOffsetDegrees) means the heading it feeds into the pose
    // estimate already reads 180 degrees off from the robot's true physical front. Subtracting
    // that same 180 degrees here cancels the mounting error out, so after pressing this while
    // pointed at the true physical front, "drive forward" actually drives forward instead of
    // backward.
    m_driverController.button(OperatorConstants.kThrustmasterTriggerButton)
        .onTrue(Commands.runOnce(
            () -> drivetrain.seedFieldCentric(Rotation2d.fromDegrees(kPigeonMountOffsetDegrees)),
            drivetrain
        ).ignoringDisable(true));

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
    // cancels it early. Unlike button 2, this takes over BOTH rotation and translation - a
    // fully automatic "go do this" action, not a driving assist. This is the first autonomous-
    // translation behavior on this robot (button 2 only ever touched rotation) - test
    // cautiously (blocks first) until the distance PID's sign/behavior is confirmed correct.
    //
    // Checks BOTH the front and rear cameras (see Vision.getTargetById()) for the sought tag, so
    // whichever camera picks it up first during the search spin is the one used - this naturally
    // approaches with whichever side of the robot needed less rotation to find the tag in the
    // first place, without needing to explicitly compare rotation distances between the two
    // cameras. The rear-camera approach direction is UNVERIFIED on the robot as of this feature
    // (see the sign-correction comments inside tagSearchAndApproachCommand()) - confirm it drives
    // toward the tag, not away, before trusting it near anything.
    m_driverController.button(OperatorConstants.kThrustmasterTagSearchButton)
        .toggleOnTrue(tagSearchAndApproachCommand());

    // POV (hat switch) rotates the robot to an exact absolute field-relative heading while held -
    // translation still comes from the driving stick meanwhile, same "assist, don't take over"
    // pattern as button 2's rotate-to-tag. Degrees below follow WPILib's own POV-angle
    // convention (0 = Up, clockwise-positive), which is exactly the compass bearing requested:
    // Up/North = 0, UpRight/Northeast = 45, Right/East = 90, DownRight/Southeast = 135,
    // Down/South = 180, DownLeft/Southwest = 225, Left/West = 270, UpLeft/Northwest = 315.
    //
    // These are absolute headings fixed to the field frame as of robot boot - NOT relative to
    // whatever the recenter button (button 1) has redefined "forward" to be. See
    // rotateToHeadingCommand()'s comment for why it reads the Pigeon 2 directly instead of the
    // drivetrain's pose heading to get that.
    m_driverController.povUp().whileTrue(rotateToHeadingCommand(0));
    m_driverController.povUpRight().whileTrue(rotateToHeadingCommand(45));
    m_driverController.povRight().whileTrue(rotateToHeadingCommand(90));
    m_driverController.povDownRight().whileTrue(rotateToHeadingCommand(135));
    m_driverController.povDown().whileTrue(rotateToHeadingCommand(180));
    m_driverController.povDownLeft().whileTrue(rotateToHeadingCommand(225));
    m_driverController.povLeft().whileTrue(rotateToHeadingCommand(270));
    m_driverController.povUpLeft().whileTrue(rotateToHeadingCommand(315));

    // Waypoint record/playback/clear trio - 5 records (held), 6 plays back the whole list in
    // recorded order, 7 clears it. See m_waypoints' comment for why this is pose-based (not
    // raw-Pigeon-based like the POV feature above).
    //
    // Hold button 5 while driving to record a path: every {@link
    // OperatorConstants#kWaypointRecordMinIntervalMeters} of travel appends another point to the
    // waypoint list - see recordWaypointCommand() for why releasing and holding it again later
    // just continues appending to the same list (any "break" becomes its own contiguous run of
    // points within the one list, played back in the same order by button 6) rather than needing
    // separate path objects.
    m_driverController.button(OperatorConstants.kThrustmasterSaveWaypointButton)
        .whileTrue(recordWaypointCommand());

    // Press button 6 to toggle driving through the waypoint list in save order (position AND
    // heading at each stop) - only does anything if at least one waypoint is currently saved
    // (see the .and() guard below), same "fully autonomous, both rotation and translation"
    // pattern as button 4. Pressing again cancels it early; it also finishes on its own once the
    // last waypoint is reached, or the instant the list is cleared (button 7) mid-drive - see
    // driveWaypointPathCommand()'s .until() condition.
    m_driverController.button(OperatorConstants.kThrustmasterGoToWaypointButton)
        .and(() -> !m_waypoints.isEmpty())
        .toggleOnTrue(driveWaypointPathCommand());

    // Press button 7 to clear the entire waypoint list. If button 6's drive-the-path command is
    // currently running, it notices the clear and finishes on its own within one loop (see
    // driveWaypointPathCommand()'s .until() condition) rather than needing to be canceled here.
    m_driverController.button(OperatorConstants.kThrustmasterClearWaypointButton).onTrue(
        Commands.runOnce(() -> {
            m_waypoints.clear();
            RobotLog.log("Waypoints cleared");
        }).ignoringDisable(true)
    );

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /**
   * Builds a command that rotates the robot to face {@code compassDegrees} - an absolute,
   * field-relative heading following WPILib's POV-angle convention (0 = Up/North, clockwise-
   * positive) - while translation still comes from the driving stick, same as button 2's
   * rotate-to-tag assist. Held (whileTrue), not toggled: releasing the POV direction hands
   * rotation back to the manual twist axis via the default drive command.
   *
   * <p>WPILib's {@link Rotation2d} is CCW-positive, the opposite sense of a clockwise compass
   * bearing, so the target heading passed to the controller is the negation of
   * {@code compassDegrees} - this is what makes, e.g., the Right/East POV direction turn the
   * robot toward its own right side rather than its left.
   *
   * <p>Feedback comes from the Pigeon 2's raw yaw ({@code drivetrain.getPigeon2().getYaw()}),
   * not {@code drivetrain.getState().Pose.getRotation()} - the pose heading gets redefined every
   * time the recenter button (button 1) calls {@code seedFieldCentric}, which would make these
   * "absolute" compass directions silently drift to mean something different after every
   * recenter press. The raw gyro reading stays fixed to whatever the field frame was at robot
   * boot regardless of any later seeding, which is what makes these truly absolute. It still
   * needs {@link #kPigeonMountOffsetDegrees} added in, for the same backwards-mounting reason
   * the recenter button corrects for - without it, "Up" would point the robot's true physical
   * front toward what is actually the field-relative south.
   */
  private Command rotateToHeadingCommand(double compassDegrees) {
    double targetHeadingDegrees = Rotation2d.fromDegrees(-compassDegrees).getDegrees();
    return Commands.startRun(
        m_headingHoldController::reset,
        () -> {
            double maxSpeed = kMaxSpeedMps * throttleSpeedPercent();
            double[] translation = computeTranslationVelocity(maxSpeed);
            double currentHeadingDegrees = drivetrain.getPigeon2().getYaw().getValueAsDouble()
                + kPigeonMountOffsetDegrees;
            double rotationalRate = MathUtil.clamp(
                m_headingHoldController.calculate(currentHeadingDegrees, targetHeadingDegrees),
                -MaxAngularRate, MaxAngularRate
            );
            drivetrain.setControl(drive.withVelocityX(translation[0])
                .withVelocityY(translation[1])
                .withRotationalRate(rotationalRate));
        },
        drivetrain
    );
  }

  /**
   * Builds the command bound to (held) button 5: appends the robot's current pose to {@link
   * #m_waypoints} every time it has moved at least {@link
   * OperatorConstants#kWaypointRecordMinIntervalMeters} since the last point recorded, for as
   * long as the button stays held. Declares NO subsystem requirements - it only reads {@code
   * drivetrain.getState().Pose}, never commands the drivetrain, so it runs alongside whatever
   * actually IS driving (the default command's manual stick input, most likely) without
   * interrupting it. That's the whole point: the driver keeps driving normally while this
   * silently records the path being driven.
   *
   * <p>Releasing the button ends this (see the {@code whileTrue} binding), and holding it again
   * later resumes appending to the SAME {@link #m_waypoints} list rather than starting a new one
   * - there's no notion of separate path objects, just one ordered list that grows in whatever
   * order points get recorded. A "break" (release then re-hold) is invisible in the list itself;
   * it just means the points recorded after the break come after the points from before it,
   * which is exactly "the next path to follow" once button 6 plays the whole list back in order.
   */
  private Command recordWaypointCommand() {
    return Commands.run(() -> {
        Pose2d pose = drivetrain.getState().Pose;
        boolean farEnoughFromLastPoint = m_waypoints.isEmpty()
            || pose.getTranslation().getDistance(
                m_waypoints.get(m_waypoints.size() - 1).getTranslation())
                >= OperatorConstants.kWaypointRecordMinIntervalMeters;
        if (farEnoughFromLastPoint) {
            m_waypoints.add(pose);
            RobotLog.log(String.format("Waypoint %d recorded at (%.2f, %.2f, %.1fdeg)",
                m_waypoints.size(), pose.getX(), pose.getY(), pose.getRotation().getDegrees()));
        }
    });
  }

  /**
   * Builds the command bound to button 6: drives through {@link #m_waypoints} in save order,
   * fully autonomous like button 4's tag-search-and-approach - no stick input is read here at
   * all. Uses {@link #driveFieldFrame}, not {@link #drive} - the velocity vector here is computed
   * directly from {@code drivetrain.getState().Pose} (true field frame), not from the joystick,
   * so it needs {@code ForwardPerspectiveValue.BlueAlliance} rather than the default operator-
   * relative perspective - see {@link #driveFieldFrame}'s comment for why using the wrong one
   * sends the robot the wrong direction on red alliance.
   *
   * <p>Only the LAST waypoint gets a precise, decelerating stop (proportional speed, floored/
   * capped, and gated on BOTH {@link OperatorConstants#kWaypointDistanceToleranceMeters} and
   * {@link OperatorConstants#kWaypointHeadingToleranceDegrees} before the command finishes).
   * Every waypoint before that is a fast pass-through: cruise at {@link
   * OperatorConstants#kWaypointMaxSpeedMps} and advance the instant the robot is within the
   * looser {@link OperatorConstants#kWaypointPassThroughToleranceMeters}, with NO heading gate.
   * Recorded waypoints are only ~6 inches apart (see {@link
   * OperatorConstants#kWaypointRecordMinIntervalMeters}) - requiring the tight stop-and-settle
   * behavior at every single one of them (as an earlier version of this did) meant the robot
   * spent nearly all its time doing tiny corrective rotations to settle heading precisely at each
   * point rather than covering distance, which is what made the path crawl at roughly 1 inch per
   * second. Heading is still actively commanded toward each intermediate waypoint's recorded
   * heading via {@link #m_headingHoldController} the whole time (so the robot's orientation still
   * gradually tracks the path's turns), it's just not a precondition for moving on to the next
   * point anymore.
   *
   * <p>{@code waypointIndex} tracks which stop is current. The list is read LIVE by index every
   * loop, not snapshotted at start, so both button 5 and button 7 do something sensible if
   * pressed mid-run: appending (button 5) extends the remaining path with a new stop at the end,
   * since the index hasn't reached it yet (and re-designates the PREVIOUS last stop as an
   * intermediate pass-through instead of the precise-stop target, automatically, since "last" is
   * always computed live against the list's current size); clearing (button 7) makes the index
   * immediately run off the (now shorter, possibly empty) list, which {@code isFinished()} below
   * detects as "done" the same way finishing the last stop normally would.
   */
  private Command driveWaypointPathCommand() {
    int[] waypointIndex = {0};
    return Commands.startRun(
        () -> {
            waypointIndex[0] = 0;
            m_headingHoldController.reset();
        },
        () -> {
            int index = waypointIndex[0];
            if (index >= m_waypoints.size()) {
                drivetrain.setControl(driveFieldFrame.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
                return;
            }
            boolean isFinalWaypoint = index == m_waypoints.size() - 1;
            Pose2d target = m_waypoints.get(index);

            Translation2d currentTranslation = drivetrain.getState().Pose.getTranslation();
            Translation2d delta = target.getTranslation().minus(currentTranslation);
            double distance = delta.getNorm();
            double positionTolerance = isFinalWaypoint
                ? OperatorConstants.kWaypointDistanceToleranceMeters
                : OperatorConstants.kWaypointPassThroughToleranceMeters;

            double speed;
            if (!isFinalWaypoint) {
                // Cruise straight through - no decel, no precise stop.
                speed = OperatorConstants.kWaypointMaxSpeedMps;
            } else if (distance <= positionTolerance) {
                speed = 0.0;
            } else {
                speed = MathUtil.clamp(
                    distance * OperatorConstants.kWaypointDistanceKP,
                    OperatorConstants.kWaypointMinSpeedMps, OperatorConstants.kWaypointMaxSpeedMps
                );
            }
            double unitX = distance > 1e-6 ? delta.getX() / distance : 0.0;
            double unitY = distance > 1e-6 ? delta.getY() / distance : 0.0;

            double currentHeadingDegrees = drivetrain.getState().Pose.getRotation().getDegrees();
            double targetHeadingDegrees = target.getRotation().getDegrees();
            double rotationalRate = MathUtil.clamp(
                m_headingHoldController.calculate(currentHeadingDegrees, targetHeadingDegrees),
                -MaxAngularRate, MaxAngularRate
            );

            drivetrain.setControl(driveFieldFrame.withVelocityX(unitX * speed)
                .withVelocityY(unitY * speed)
                .withRotationalRate(rotationalRate));

            boolean arrived = isFinalWaypoint
                ? distance <= positionTolerance
                    && Math.abs(MathUtil.inputModulus(
                        currentHeadingDegrees - targetHeadingDegrees, -180.0, 180.0))
                        <= OperatorConstants.kWaypointHeadingToleranceDegrees
                : distance <= positionTolerance;
            if (arrived) {
                waypointIndex[0]++;
                m_headingHoldController.reset();
            }
        },
        drivetrain
    ).until(() -> waypointIndex[0] >= m_waypoints.size());
  }

  /**
   * Builds the tag-search-and-approach command bound to button 4 (see the binding's comment for
   * the overall behavior). This is a TOUR, not a priority search: {@code tourIndex} tracks which
   * stop in {@link VisionConstants#kSearchTagIdOrder} it's currently working on, starting at
   * index 0 and visiting each one in array order - 1, then 2, then 4 (editable, see that
   * constant). At each stop it repeats the same search/approach/settle cycle: search until the
   * current stop's ID is found (or a full 360-degree sweep comes up empty, in which case it gives
   * up on that stop), approach until "arrived" (both squared up within {@link
   * VisionConstants#kAlignYawToleranceDegrees} and within {@link
   * VisionConstants#kApproachDistanceToleranceMeters} of the standoff distance), then holds
   * station for {@link VisionConstants#kApproachSettleSeconds} before moving on to the next
   * stop - or, if this was the last stop in the array, finishing the whole command. {@code
   * arrivedTimer} - {@link edu.wpi.first.wpilibj.Timer#start()} is a no-op if already running, so
   * calling it every loop while arrived doesn't reset the elapsed time, but any loop where the
   * robot drifts back out of tolerance stops and resets it, requiring a fresh, continuous
   * {@code kApproachSettleSeconds} of being settled rather than a cumulative one.
   *
   * <p>The give-up sweep is measured from the robot's ACTUAL odometry rotation (accumulated
   * per-loop heading deltas, since a single current-vs-start {@code Rotation2d} comparison would
   * wrap back near zero once a full revolution completes, hiding exactly the event we're trying
   * to detect) rather than assumed from the commanded search rate x time - {@code
   * OpenLoopVoltage} control has real acceleration ramp-up and no closed-loop guarantee of
   * hitting the commanded rate exactly, so a timing-based estimate can run out before a real 360
   * degrees has actually been swept, prematurely restarting the sweep right as the tag was about
   * to come into view.
   *
   * <p>Losing sight of the current stop's tag doesn't immediately resume the search spin -
   * {@code lostTimer} holds station for {@link VisionConstants#kLostGracePeriodSeconds} first, on
   * the theory that a target lost right as it's being approached is more likely a brief vision
   * flicker than a genuine loss. Confirmed on the robot: without this grace period, a single
   * missed frame right as the robot was about to arrive (distance error ~0.03m) threw away that
   * progress and forced a whole new 360-degree search - spinning away is exactly the wrong
   * reaction to a target that's nearly acquired.
   */
  private Command tagSearchAndApproachCommand() {
    Timer arrivedTimer = new Timer();
    Timer lostTimer = new Timer();
    int[] tourIndex = {0};
    double[] sweptDegrees = {0.0};
    Rotation2d[] lastHeading = {null};
    // +1 = search CCW, -1 = search CW - picked fresh at the start of each sweep based on
    // m_lastKnownTagBearings (see where it's set).
    int[] searchDirection = {1};
    // Smooths the commanded outputs regardless of how choppy the underlying vision updates are -
    // confirmed on the robot that a nonzero kD on the distance/lateral PIDs caused violent
    // full-torque stop/start jitter (derivative kick from vision's discrete ~30-60ms update rate
    // vs the 20ms control loop - see kApproachDistanceKD's comment), so smoothing happens here on
    // the output instead of via PID derivative terms. Same technique as the joystick input
    // limiters (m_xLimiter/m_yLimiter/m_rotLimiter) elsewhere in this class.
    SlewRateLimiter forwardLimiter = new SlewRateLimiter(2.0);
    SlewRateLimiter lateralLimiter = new SlewRateLimiter(2.0);
    SlewRateLimiter rotationLimiter = new SlewRateLimiter(2.0);
    int[] printCounter = {0};
    // Set once the LAST stop in the tour is either arrived-at-and-settled or given up on -
    // ends the whole command.
    boolean[] tourComplete = {false};

    Runnable driveTowardTarget = () -> {
        int currentId = VisionConstants.kSearchTagIdOrder[
            Math.min(tourIndex[0], VisionConstants.kSearchTagIdOrder.length - 1)];
        Optional<Vision.TargetSighting> frontSighting = vision.getFrontTargetById(currentId);
        Optional<Vision.TargetSighting> rearSighting = vision.getRearTargetById(currentId);
        boolean frontPresent = frontSighting.isPresent();
        boolean rearPresent = rearSighting.isPresent();
        SmartDashboard.putNumber("TagSearch/SoughtId", currentId);
        SmartDashboard.putNumber("TagSearch/TourIndex", tourIndex[0]);

        double rotationalRate;
        double forwardSpeed;
        double lateralSpeed = 0.0;
        boolean arrived = false;
        boolean giveUpOnCurrentStop = false;
        double loggedYawErrorDegrees = 0.0;
        double loggedDistanceErrorMeters = 0.0;
        boolean loggedUsedRear = false;

        if (frontPresent || rearPresent) {
            sweptDegrees[0] = 0.0;
            lastHeading[0] = null;
            lostTimer.stop();
            lostTimer.reset();

            // Each camera's own compensated (latency-corrected) yaw error, relative to ITS OWN
            // boresight - computed directly if that camera currently sees the target, or derived
            // from the OTHER camera's real reading if not. Front/rear are mounted 180 degrees
            // apart (see VisionConstants.kFrontCameraName's comment), so "what the other camera
            // would read" is just +180 degrees, wrapped - a pure geometric identity, valid
            // regardless of which one actually produced the frame.
            double frontYawError;
            double rearYawError;
            if (frontPresent) {
                frontYawError = computeCompensatedYawDegrees(
                    frontSighting.get().target().getYaw(), frontSighting.get().timestampSeconds());
                rearYawError = rearPresent
                    ? computeCompensatedYawDegrees(
                        rearSighting.get().target().getYaw(), rearSighting.get().timestampSeconds())
                    : MathUtil.inputModulus(180.0 + frontYawError, -180.0, 180.0);
            } else {
                rearYawError = computeCompensatedYawDegrees(
                    rearSighting.get().target().getYaw(), rearSighting.get().timestampSeconds());
                frontYawError = MathUtil.inputModulus(180.0 + rearYawError, -180.0, 180.0);
            }

            // Take the shortest path: whichever camera needs LESS rotation to square up wins,
            // even if it doesn't directly see the target yet - in that case rotation continues
            // toward it, but translation waits (see chosenSighting below) until it does, rather
            // than approaching using the OTHER (non-chosen, more-rotation-needed) camera's data.
            boolean useRear = Math.abs(rearYawError) < Math.abs(frontYawError);
            double compensatedYawDegrees = useRear ? rearYawError : frontYawError;
            Optional<Vision.TargetSighting> chosenSighting = useRear ? rearSighting : frontSighting;

            rotationalRate = MathUtil.clamp(
                m_alignRotationController.calculate(compensatedYawDegrees, 0.0),
                -MaxAngularRate, MaxAngularRate
            );

            // Remember roughly where this tag is (field-relative) so a future search for the
            // same ID can start by turning the shorter way toward it - see
            // m_lastKnownTagBearings's field comment. Uses whichever camera has a REAL detection
            // this loop (front preferred if both do), not necessarily the CHOSEN one, since the
            // chosen one might only have a derived estimate right now.
            Vision.TargetSighting realSighting = frontPresent ? frontSighting.get() : rearSighting.get();
            double realYawError = frontPresent ? frontYawError : rearYawError;
            double cameraMountOffsetDegrees = realSighting.fromRearCamera()
                ? VisionConstants.kRearCameraMountOffsetDegrees : 0.0;
            m_lastKnownTagBearings.put(currentId, drivetrain.getState().Pose.getRotation()
                .plus(Rotation2d.fromDegrees(cameraMountOffsetDegrees + realYawError)));

            if (chosenSighting.isPresent()) {
                // The camera needing less rotation ALSO already has a real detection - use its
                // own live distance/tag-face data, same as the single-camera behavior,
                // parameterized by which camera (useRear) for the mounting-direction sign
                // corrections.
                PhotonTrackedTarget seenTarget = chosenSighting.get().target();

                // Ground-plane distance (ignores the camera/tag height difference) from the
                // camera-to-target 3D transform - requires the PhotonVision pipeline to have a
                // valid camera calibration for this to be meaningful.
                var cameraToTarget = seenTarget.getBestCameraToTarget().getTranslation();
                double distanceMeters = Math.hypot(cameraToTarget.getX(), cameraToTarget.getY());
                double distanceErrorMeters = distanceMeters - VisionConstants.kApproachDistanceMeters;
                // Negated: PIDController.calculate(measurement, setpoint) gives a NEGATIVE output
                // when measurement > setpoint (too far away), but driving toward the tag is
                // exactly what CLOSES that distance - the opposite of a typical PID relationship,
                // where positive output increases the measurement. After this negation, positive
                // approachOutput means "move closer to the tag."
                double approachOutput = -m_approachDistanceController.calculate(
                    distanceMeters, VisionConstants.kApproachDistanceMeters);

                // Don't start closing distance until roughly squared up with the tag - avoids
                // crabbing in at a steep angle while still mid-rotation.
                boolean squaredUp = Math.abs(compensatedYawDegrees)
                    <= VisionConstants.kApproachYawToleranceDegrees;
                // Sign depends on WHICH camera is being used: "front cam" was confirmed on the
                // robot to be mounted facing the drivetrain's kinematic "back" (RobotCentric's +X
                // is defined by the drivetrain's own front, module geometry, independent of which
                // way either camera physically points) - commanding +approachOutput while farther
                // than the standoff distance drove AWAY from the tag, not toward it, so using
                // front needs negating. "rear camera" is mounted 180 degrees from front (see
                // VisionConstants.kFrontCameraName's comment), which puts it facing the SAME
                // direction as RobotCentric's +X - so using rear needs the OPPOSITE sign
                // (un-negated). UNVERIFIED on the robot as of this feature - confirm the
                // rear-camera approach direction is actually correct before trusting it near
                // anything, same as every other sign in this method needed on-robot confirmation.
                double mountingCorrectedOutput = useRear ? approachOutput : -approachOutput;

                // Floors the magnitude (direction preserved) so a small residual error doesn't
                // produce a commanded speed too weak to overcome static friction - confirmed on
                // the robot: without this, the robot stalled short of arriving once distance
                // error shrank to ~0.07m, well outside kApproachDistanceToleranceMeters (~0.05m).
                double flooredOutput = Math.abs(mountingCorrectedOutput) < VisionConstants.kApproachMinSpeedMps
                    ? Math.copySign(VisionConstants.kApproachMinSpeedMps, mountingCorrectedOutput)
                    : mountingCorrectedOutput;

                forwardSpeed = squaredUp
                    ? MathUtil.clamp(flooredOutput,
                        -VisionConstants.kApproachMaxSpeedMps, VisionConstants.kApproachMaxSpeedMps)
                    : 0.0;

                // --- Squaring-up (tag-face alignment) RE-ENABLED 2026-07-24 ---
                // Originally disabled 2026-07-23 after it made the robot circle the target - at
                // the time this was diagnosed as the rotation-centering and face-angle-strafing
                // loops fighting each other. However, the camera's PhotonVision pipeline had 3D
                // mode OFF for the entire time this was built and tested, which means
                // getBestCameraToTarget().getRotation() (what signedTagFaceAngleDegrees reads)
                // was always a default/zero value, not real data - so the "circling" may well
                // have actually been a bogus near-constant strafe command (chasing garbage input)
                // with the yaw loop compensating for the resulting drift, not a fundamental
                // architecture problem. Re-enabled now that 3D mode is on and real distance data
                // has been confirmed working - re-verify whether circling still happens with real
                // data before assuming the single-target-transform redesign is actually necessary.
                double signedTagFaceAngleDegrees = MathUtil.inputModulus(
                    Math.toDegrees(seenTarget.getBestCameraToTarget().getRotation().getZ()) - 180.0,
                    -180.0, 180.0
                );
                double tagFaceAngleDegrees = Math.abs(signedTagFaceAngleDegrees);
                // Negated: confirmed backwards on the robot previously (strafed the wrong
                // left/right direction) - kept even though that test predates the 3D-mode fix,
                // since this sign relationship is about strafe-direction-vs-error, not about the
                // data being real. Same per-camera mirroring reasoning as mountingCorrectedOutput
                // above applies here too - a camera facing the opposite way sees its own
                // left/right mirror-flipped relative to the chassis, so using rear needs the
                // OPPOSITE sign from front. Equally UNVERIFIED on the robot - confirm before
                // trusting it.
                lateralSpeed = squaredUp
                    ? MathUtil.clamp(
                        (useRear ? 1 : -1) * m_tagFaceAlignController.calculate(signedTagFaceAngleDegrees, 0.0),
                        -VisionConstants.kApproachMaxSpeedMps, VisionConstants.kApproachMaxSpeedMps)
                    : 0.0;

                arrived = Math.abs(compensatedYawDegrees) <= VisionConstants.kAlignYawToleranceDegrees
                    && Math.abs(distanceErrorMeters) <= VisionConstants.kApproachDistanceToleranceMeters
                    && tagFaceAngleDegrees <= VisionConstants.kApproachTagFaceToleranceDegrees;

                SmartDashboard.putNumber("TagSearch/TagFaceAngleDegrees", signedTagFaceAngleDegrees);
                loggedDistanceErrorMeters = distanceErrorMeters;
                SmartDashboard.putNumber("TagSearch/DistanceMeters", distanceMeters);
                SmartDashboard.putNumber("TagSearch/DistanceErrorMeters", distanceErrorMeters);
            } else {
                // The camera needing less rotation doesn't directly see the target yet - keep
                // rotating toward it (compensatedYawDegrees/rotationalRate above already point
                // that way); no real distance/tag-face data exists for it yet, so hold
                // translation at zero and wait rather than approaching using the OTHER
                // (non-chosen) camera's numbers.
                forwardSpeed = 0.0;
                lateralSpeed = 0.0;
            }

            loggedUsedRear = useRear;
            SmartDashboard.putBoolean("TagSearch/FromRearCamera", useRear);
            SmartDashboard.putBoolean("TagSearch/ChosenCameraHasDirectSighting", chosenSighting.isPresent());
            loggedYawErrorDegrees = compensatedYawDegrees;
            SmartDashboard.putNumber("TagSearch/TargetId", currentId);
            SmartDashboard.putNumber("TagSearch/YawErrorDegrees", compensatedYawDegrees);
        } else {
            lostTimer.start();
            if (!lostTimer.hasElapsed(VisionConstants.kLostGracePeriodSeconds)) {
                // Just lost sight - probably a brief vision flicker rather than a genuine loss
                // (especially likely if this happened close to arriving). Hold station instead
                // of immediately spinning off to search again, which would be exactly the wrong
                // reaction to a target that's nearly acquired. Don't touch sweptDegrees/
                // lastHeading yet - the give-up sweep hasn't started.
                rotationalRate = 0.0;
                forwardSpeed = 0.0;
                lastHeading[0] = null;
            } else {
                Rotation2d currentHeading = drivetrain.getState().Pose.getRotation();
                if (lastHeading[0] == null) {
                    // Starting a fresh sweep - pick whichever direction is the shorter way
                    // toward this tag's last-known bearing (if we have one) instead of always
                    // sweeping the fixed CCW direction. Rotation2d subtraction already wraps to
                    // the shortest signed angle, so its sign directly gives the shorter way
                    // round: positive means CCW is shorter, negative means CW is shorter. A
                    // stale/wrong memory just means a worse first guess, not a correctness
                    // problem - the full-360 give-up sweep below still covers every direction.
                    Rotation2d remembered = m_lastKnownTagBearings.get(currentId);
                    searchDirection[0] = (remembered != null
                        && remembered.minus(currentHeading).getDegrees() < 0) ? -1 : 1;
                } else {
                    sweptDegrees[0] += Math.abs(currentHeading.minus(lastHeading[0]).getDegrees());
                }
                lastHeading[0] = currentHeading;

                if (sweptDegrees[0] >= 360.0) {
                    giveUpOnCurrentStop = true;
                }

                rotationalRate = searchDirection[0]
                    * Math.min(VisionConstants.kSearchRotationRadPerSec, MaxAngularRate);
                forwardSpeed = 0.0;
            }

            SmartDashboard.putNumber("TagSearch/SweptDegrees", sweptDegrees[0]);
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

        // Advance to the next tour stop once either settled at the current one, or given up on
        // it after a full sweep - or finish the whole command if this was the last stop.
        boolean settledAtCurrentStop = arrivedTimer.hasElapsed(VisionConstants.kApproachSettleSeconds);
        if (settledAtCurrentStop || giveUpOnCurrentStop) {
            System.out.println(settledAtCurrentStop
                ? "TagSearch: settled at id=" + currentId + ", advancing tour."
                : "TagSearch: gave up on id=" + currentId + " after a full sweep, advancing tour.");

            if (tourIndex[0] < VisionConstants.kSearchTagIdOrder.length - 1) {
                tourIndex[0]++;
                sweptDegrees[0] = 0.0;
                lastHeading[0] = null;
                arrivedTimer.stop();
                arrivedTimer.reset();
                lostTimer.stop();
                lostTimer.reset();
                m_alignRotationController.reset();
                m_approachDistanceController.reset();
                m_tagFaceAlignController.reset();
            } else {
                tourComplete[0] = true;
            }
        }

        // Prints to the console/RioLog every ~0.5s (25 loops at the default 50Hz) instead of
        // requiring a dashboard - easier to spot while testing.
        printCounter[0]++;
        if (printCounter[0] % 25 == 0) {
            System.out.printf(
                "TagSearch: tourIndex=%d soughtId=%d found=%b rear=%b swept=%.0f yawErr=%.1f distErr=%.2f arrived=%b arrivedFor=%.2f%n",
                tourIndex[0], currentId, frontPresent || rearPresent, loggedUsedRear,
                sweptDegrees[0], loggedYawErrorDegrees, loggedDistanceErrorMeters, arrived,
                arrivedTimer.get()
            );
        }

        drivetrain.setControl(approach.withVelocityX(forwardLimiter.calculate(forwardSpeed))
            .withVelocityY(lateralLimiter.calculate(lateralSpeed))
            .withRotationalRate(rotationLimiter.calculate(rotationalRate)));
    };

    return Commands.runOnce(() -> {
            m_alignRotationController.reset();
            m_approachDistanceController.reset();
            m_tagFaceAlignController.reset();
            arrivedTimer.stop();
            arrivedTimer.reset();
            lostTimer.stop();
            lostTimer.reset();
            tourIndex[0] = 0;
            sweptDegrees[0] = 0.0;
            lastHeading[0] = null;
            tourComplete[0] = false;
            searchDirection[0] = 1;
            forwardLimiter.reset(0);
            lateralLimiter.reset(0);
            rotationLimiter.reset(0);
        }, drivetrain, vision)
        .andThen(Commands.run(driveTowardTarget, drivetrain, vision)
            .until(() -> tourComplete[0]));
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
   *   <li>Finally, if {@link ObstacleSensor#isObstacleTooClose()} - the front-mounted ultrasonic
   *       sensor sees something closer than {@link UltrasonicConstants#kObstacleStopDistanceInches}
   *       - sideways output is zeroed and any remaining FORWARD component is clamped to zero,
   *       leaving only backward translation. Turning ({@link #computeManualRotationalRate()}) is
   *       a separate method entirely and is never touched by this - rotating in place doesn't
   *       risk closing distance with whatever tripped the sensor, only forward/sideways motion
   *       does.
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
    double outputX = unitX * outputSpeed;
    double outputY = unitY * outputSpeed;

    if (obstacleSensor.isObstacleTooClose()) {
        outputX = Math.min(outputX, 0.0);
        outputY = 0.0;
    }

    return new double[] {outputX, outputY};
  }

  /**
   * Builds one straight-line leg of {@link #autoSquareCommand()}: drives at a fixed field-
   * relative velocity ({@code velocityXMps}, {@code velocityYMps}) with no rotation, until the
   * robot's odometry-measured position has moved {@code distanceMeters} from wherever this leg
   * started - not a fixed duration, so it's tolerant of however long that actually takes at the
   * commanded speed. Position is field-relative and the heading never changes across any leg
   * (the drivetrain is holonomic, so tracing a square doesn't require ever turning), which is
   * what makes 4 legs of North/East/South/West bring the robot back to exactly where it
   * started, odometry drift aside.
   */
  private Command driveLegCommand(double velocityXMps, double velocityYMps, double distanceMeters) {
    Translation2d[] legStart = new Translation2d[1];
    return Commands.startRun(
        () -> legStart[0] = drivetrain.getState().Pose.getTranslation(),
        () -> drivetrain.setControl(driveFieldFrame.withVelocityX(velocityXMps)
            .withVelocityY(velocityYMps)
            .withRotationalRate(0)),
        drivetrain
    ).until(() -> drivetrain.getState().Pose.getTranslation().getDistance(legStart[0])
        >= distanceMeters);
  }

  /**
   * Builds the "3x3 ft Square" autonomous routine (see {@link #m_autoChooser}): drives a closed
   * {@link AutoConstants#kSquareSideMeters}-per-side square - North, East, South, West, each via
   * {@link #driveLegCommand} - then stops. Since each leg is field-relative and the same length,
   * completing all 4 returns the robot to its starting position (odometry drift aside), with the
   * same heading it started with throughout (no rotation is ever commanded).
   *
   * @return the command for this autonomous routine
   */
  private Command autoSquareCommand() {
    double speed = AutoConstants.kSquareSpeedMps;
    double side = AutoConstants.kSquareSideMeters;
    return Commands.sequence(
        driveLegCommand(speed, 0, side),   // North: field-relative +X
        driveLegCommand(0, -speed, side),  // East: field-relative -Y (+Y is left, so -Y is right)
        driveLegCommand(-speed, 0, side),  // South: field-relative -X
        driveLegCommand(0, speed, side)    // West: field-relative +Y - back at the start
    ).andThen(Commands.runOnce(
        () -> drivetrain.setControl(driveFieldFrame.withVelocityX(0).withVelocityY(0).withRotationalRate(0)),
        drivetrain
    ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * <p>Returns whichever routine is currently selected on the "Auto Mode" dashboard chooser (see
   * {@link #m_autoChooser}) - either {@link #tagSearchAndApproachCommand()} (the default, same
   * tour-and-approach behavior bound to button 4 in teleop: search each id in {@link
   * VisionConstants#kSearchTagIdOrder} in turn, approach, settle, move to the next) or {@link
   * #autoSquareCommand()} (drives a closed 3x3 ft square path and returns to the starting
   * position). Both commands are built once (in the constructor) and reused across autonomous
   * periods - each resets its own captured state (Timers, leg-start position, etc.) in its
   * {@code initialize()}/startRun-init block, so re-scheduling an already-completed instance is
   * safe.
   *
   * <p>The tag-search-and-approach routine runs completely unsupervised for the whole autonomous
   * period with no driver fallback - keep an eye on the squaring-up (strafe) behavior
   * specifically, since that was only just re-enabled and hasn't been fully validated yet (see
   * [[feedback-vision-alignment-design]] memory for the open "crunchy movement" question). The
   * square routine is brand new and unvalidated on hardware - test on blocks first, then in open
   * space, before trusting it near anything the robot could run into.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
