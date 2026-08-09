package frc.robot;

import java.util.Set;
import java.util.TreeSet;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Constants.RotationTestConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

/**
 * Builds the "vision rotation test" autonomous.json step - a diagnostic, not a real match step.
 * Finds how fast the robot can spin while still reliably seeing AprilTags by running a series of
 * full 360-degree rotations, each faster than the last, logging the duration and unique-tag
 * count of each one:
 * <ol>
 *   <li>Steady-speed sequences - spin at one constant rate, starting at {@link
 *       RotationTestConstants#kSteadySpeedStartRadPerSec} and adding {@link
 *       RotationTestConstants#kSteadySpeedIncrementRadPerSec} each time, for as long as the tag
 *       count doesn't drop. Once it drops (or the speed maxes out at the drivetrain's actual max
 *       turn rate), move on to...
 *   <li>Pulse sequences - same idea, but spinning in target lock's fast/slow pulse pattern
 *       instead ({@link PulsedSearch}), starting at {@link
 *       RotationTestConstants#kPulseFastSpeedStartRadPerSec}/{@link
 *       RotationTestConstants#kPulseSlowSpeedStartRadPerSec}. Ends the whole test once the tag
 *       count drops here too (or the fast speed maxes out).
 * </ol>
 * Before every measured rotation, spins slowly until no tag is in view first, so each
 * measurement starts from the same clean state. Silences {@link RobotLog#log} for the whole test
 * (restored once it ends, however it ends) so only its own result lines - logged via {@link
 * RobotLog#logAlways} - show up on the console, not the routine status chatter (target lock,
 * "April Tags in view", etc.) that'd otherwise flood it during all this rapid-fire spinning.
 * Once the whole test ends, logs the single best sequence seen across the entire run - most
 * unique tags, fastest duration as a tiebreaker.
 */
final class VisionRotationTest {
  private final CommandSwerveDrivetrain drivetrain;
  private final Vision vision;
  private final double maxAngularRateRadPerSec;

  // Robot-centric, same shaping as RobotContainer's own autonomous drive request - "forward"
  // means the robot's own front, not a field direction.
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
      .withDeadband(0)
      .withRotationalDeadband(0)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  VisionRotationTest(CommandSwerveDrivetrain drivetrain, Vision vision, double maxAngularRateRadPerSec) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.maxAngularRateRadPerSec = maxAngularRateRadPerSec;
  }

  /** The whole test, ready to run as an autonomous step. */
  Command command() {
    RotationTestResult best = new RotationTestResult();
    return Commands.runOnce(() -> RobotLog.setQuiet(true))
        .andThen(clearTagViewCommand())
        .andThen(steadySpeedSequenceCommand(RotationTestConstants.kSteadySpeedStartRadPerSec, -1, best))
        .finallyDo(() -> RobotLog.setQuiet(false));
  }

  private Command stopDrivetrainCommand() {
    return Commands.runOnce(() -> drivetrain.setControl(
        drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)), drivetrain);
  }

  /**
   * Spins slowly ({@link RotationTestConstants#kClearViewRotationRadPerSec}) until no AprilTag is
   * in view, or {@link RotationTestConstants#kClearViewTimeoutSeconds} elapses - whichever comes
   * first, so a tag visible from every angle can't hang the rotation test forever. Run before
   * every measured rotation so each one starts from the same clean "nothing visible" state.
   */
  private Command clearTagViewCommand() {
    return Commands.run(() -> drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0)
            .withRotationalRate(RotationTestConstants.kClearViewRotationRadPerSec)),
        drivetrain, vision)
        .until(() -> !vision.hasTarget())
        .withTimeout(RotationTestConstants.kClearViewTimeoutSeconds)
        .andThen(stopDrivetrainCommand());
  }

  /**
   * Spins a full 360 degrees - tracked cumulatively via odometry, ignoring any single-loop jump
   * bigger than the robot could plausibly turn (odometry/gyro can report a stale-then-real
   * reading as a bogus jump right after boot) - commanding whatever {@code
   * rotationalRateSupplier} returns each loop (re-evaluated every time, so it can pulse). Every
   * tag ID visible at any point during the rotation is added to {@code seenIds}, which the
   * caller starts empty and reads once this finishes.
   */
  private Command timedRotationCommand(DoubleSupplier rotationalRateSupplier, Set<Integer> seenIds) {
    Rotation2d[] lastHeading = {null};
    double[] turnedRadians = {0.0};
    return Commands.sequence(
        Commands.runOnce(() -> {
            lastHeading[0] = drivetrain.getState().Pose.getRotation();
            turnedRadians[0] = 0.0;
        }, drivetrain),
        Commands.run(() -> {
            Rotation2d currentHeading = drivetrain.getState().Pose.getRotation();
            double deltaRadians = Math.abs(currentHeading.minus(lastHeading[0]).getRadians());
            if (deltaRadians <= Math.toRadians(45)) {
                turnedRadians[0] += deltaRadians;
            }
            lastHeading[0] = currentHeading;
            seenIds.addAll(vision.getVisibleTagIds());
            drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0)
                .withRotationalRate(rotationalRateSupplier.getAsDouble()));
        }, drivetrain, vision).until(() -> turnedRadians[0] >= Math.toRadians(360)),
        stopDrivetrainCommand()
    );
  }

  /**
   * One steady-speed sequence: a full 360-degree rotation at a single constant {@code
   * speedRadPerSec}, counting unique AprilTag IDs seen. {@code previousTagCount} is the previous
   * sequence's count (or negative for sequence 1, which always runs once unconditionally).
   * {@code best} tracks the best sequence across the whole test run - every sequence in the whole
   * test shares the same instance. Stops early if {@code speedRadPerSec} would already exceed
   * the drivetrain's actual max turn rate - increasing it further couldn't change anything:
   * <ul>
   *   <li>If the count didn't drop (stayed the same or improved), repeats with
   *       {@code speedRadPerSec} increased by {@link
   *       RotationTestConstants#kSteadySpeedIncrementRadPerSec}.
   *   <li>If it dropped, moves on to {@link #pulseSequenceCommand}.
   * </ul>
   */
  private Command steadySpeedSequenceCommand(double speedRadPerSec, int previousTagCount, RotationTestResult best) {
    if (speedRadPerSec > maxAngularRateRadPerSec) {
        return Commands.runOnce(() -> RobotLog.logAlways(String.format(
            "Vision rotation test: steady speed reached the drivetrain's max turn rate (%.2f rad/s) - "
                + "moving to pulse sequences", maxAngularRateRadPerSec)))
            .andThen(clearTagViewCommand())
            .andThen(pulseSequenceCommand(RotationTestConstants.kPulseFastSpeedStartRadPerSec,
                RotationTestConstants.kPulseSlowSpeedStartRadPerSec, -1, best));
    }

    Set<Integer> seenIds = new TreeSet<>();
    Timer stepTimer = new Timer();
    return Commands.sequence(
        Commands.runOnce(() -> {
            stepTimer.restart();
            RobotLog.logAlways(String.format("Vision rotation test: starting steady %.2f rad/s", speedRadPerSec));
        }),
        timedRotationCommand(() -> speedRadPerSec, seenIds),
        Commands.runOnce(() -> {
            RobotLog.logAlways(String.format(
                "Vision rotation test: steady %.2f rad/s - %.2fs, %d unique tag(s) %s",
                speedRadPerSec, stepTimer.get(), seenIds.size(), seenIds));
            best.consider(seenIds.size(), stepTimer.get(), String.format("steady %.2f rad/s", speedRadPerSec));
        }),
        // Deferred because which command comes next depends on seenIds, only known once the
        // rotation above actually finishes - the recursive call itself just builds the next
        // phase's command tree, same as any other command-building method call.
        Commands.defer(() -> {
            int count = seenIds.size();
            if (previousTagCount < 0 || count >= previousTagCount) {
                return clearTagViewCommand().andThen(steadySpeedSequenceCommand(
                    speedRadPerSec + RotationTestConstants.kSteadySpeedIncrementRadPerSec, count, best));
            }
            RobotLog.logAlways("Vision rotation test: steady speed tag count dropped - moving to pulse sequences");
            return clearTagViewCommand().andThen(pulseSequenceCommand(
                RotationTestConstants.kPulseFastSpeedStartRadPerSec,
                RotationTestConstants.kPulseSlowSpeedStartRadPerSec, -1, best));
        }, Set.of(drivetrain, vision))
    );
  }

  /**
   * One pulse sequence: a full 360-degree rotation using target lock's fast/slow pulse pattern
   * at {@code fastSpeedRadPerSec}/{@code slowSpeedRadPerSec}, counting unique AprilTag IDs seen.
   * {@code previousTagCount} is the previous sequence's count (or negative for sequence 1, which
   * always runs once unconditionally). {@code best} tracks the best sequence across the whole
   * test run - see {@link #steadySpeedSequenceCommand}. Stops early (ending the whole test, and
   * logging {@code best}'s final summary) if {@code fastSpeedRadPerSec} would already exceed the
   * drivetrain's actual max turn rate. Otherwise:
   * <ul>
   *   <li>If the count didn't drop, repeats with both speeds increased by {@link
   *       RotationTestConstants#kPulseFastSpeedIncrementRadPerSec}/{@link
   *       RotationTestConstants#kPulseSlowSpeedIncrementRadPerSec}.
   *   <li>If it dropped, the whole test is done - {@code best}'s summary is logged here too.
   * </ul>
   */
  private Command pulseSequenceCommand(
      double fastSpeedRadPerSec, double slowSpeedRadPerSec, int previousTagCount, RotationTestResult best) {
    if (fastSpeedRadPerSec > maxAngularRateRadPerSec) {
        return Commands.runOnce(() -> {
            RobotLog.logAlways(String.format(
                "Vision rotation test: pulse fast speed reached the drivetrain's max turn rate (%.2f rad/s) - "
                    + "test complete", maxAngularRateRadPerSec));
            RobotLog.logAlways("Vision rotation test: best result - " + best.summary());
        });
    }

    Set<Integer> seenIds = new TreeSet<>();
    Timer stepTimer = new Timer();
    PulsedSearch search = new PulsedSearch(maxAngularRateRadPerSec);
    return Commands.sequence(
        Commands.runOnce(() -> {
            stepTimer.restart();
            search.reset(drivetrain.getState().Pose.getRotation());
            RobotLog.logAlways(String.format(
                "Vision rotation test: starting pulse fast=%.2f/slow=%.2f rad/s", fastSpeedRadPerSec, slowSpeedRadPerSec));
        }, drivetrain),
        timedRotationCommand(
            () -> search.pulse(drivetrain.getState().Pose.getRotation(), fastSpeedRadPerSec, slowSpeedRadPerSec),
            seenIds),
        Commands.runOnce(() -> {
            RobotLog.logAlways(String.format(
                "Vision rotation test: pulse fast=%.2f/slow=%.2f rad/s - %.2fs, %d unique tag(s) %s",
                fastSpeedRadPerSec, slowSpeedRadPerSec, stepTimer.get(), seenIds.size(), seenIds));
            best.consider(seenIds.size(), stepTimer.get(),
                String.format("pulse fast=%.2f/slow=%.2f rad/s", fastSpeedRadPerSec, slowSpeedRadPerSec));
        }),
        // Deferred for the same reason as steadySpeedSequenceCommand's - see its comment.
        Commands.defer(() -> {
            int count = seenIds.size();
            if (previousTagCount < 0 || count >= previousTagCount) {
                return clearTagViewCommand().andThen(pulseSequenceCommand(
                    fastSpeedRadPerSec + RotationTestConstants.kPulseFastSpeedIncrementRadPerSec,
                    slowSpeedRadPerSec + RotationTestConstants.kPulseSlowSpeedIncrementRadPerSec, count, best));
            }
            RobotLog.logAlways("Vision rotation test: pulse tag count dropped - test complete");
            RobotLog.logAlways("Vision rotation test: best result - " + best.summary());
            return Commands.none();
        }, Set.of(drivetrain, vision))
    );
  }

  /**
   * Tracks the best (most unique tags, then fastest duration as a tiebreaker) sequence seen
   * across an entire test run - passed down through every {@link #steadySpeedSequenceCommand}/
   * {@link #pulseSequenceCommand} call so the whole run shares one.
   */
  private static final class RotationTestResult {
    private int tagCount = -1;
    private double durationSeconds = Double.MAX_VALUE;
    private String description = "";

    void consider(int candidateTagCount, double candidateDurationSeconds, String candidateDescription) {
        if (candidateTagCount > tagCount
                || (candidateTagCount == tagCount && candidateDurationSeconds < durationSeconds)) {
            tagCount = candidateTagCount;
            durationSeconds = candidateDurationSeconds;
            description = candidateDescription;
        }
    }

    String summary() {
        return String.format("%s - %.2fs, %d unique tag(s)", description, durationSeconds, tagCount);
    }
  }
}
