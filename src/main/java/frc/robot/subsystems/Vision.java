package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisionConstants;
import frc.robot.RobotLog;

/**
 * Wraps PhotonVision's two camera feeds (front and rear - see {@link
 * VisionConstants#kFrontCameraName}/{@link VisionConstants#kRearCameraName}). Each camera's best
 * target is cached once per scheduler loop in periodic() so every consumer within the same loop
 * sees a consistent result. Uses getAllUnreadResults() (a camera can produce frames faster than
 * the robot loop runs) and keeps the previous target if no new frame arrived this loop, rather
 * than flickering to "no target" for a loop iteration.
 *
 * <p>{@link #hasTarget()}/{@link #getTargetYawDegrees()}/{@link #getBestTargetId()}/{@link
 * #getTargetTimestampSeconds()} only ever look at the FRONT camera (unchanged behavior from
 * before the rear camera existed) - they back button 2's align-to-any-tag behavior, which hasn't
 * been asked to look at both cameras yet. {@link #getFrontTargetById(int)}/{@link
 * #getRearTargetById(int)} check one camera each (rather than a single "whichever, front-
 * preferred" lookup) - the tag-search-and-approach behavior (button 4) needs BOTH cameras' own
 * independent readings so it can compare how much rotation each would need and pick whichever is
 * shorter, not just settle for whichever happens to be checked first.
 */
public class Vision extends SubsystemBase {
  /** A target sighting bundled with which physical camera actually saw it. */
  public record TargetSighting(PhotonTrackedTarget target, boolean fromRearCamera, double timestampSeconds) {}

  /** Tracks one camera's most-recently-processed frame. */
  private static final class CameraTracker {
    private final PhotonCamera camera;
    private Optional<PhotonTrackedTarget> bestTarget = Optional.empty();
    private double bestTargetTimestampSeconds = 0.0;
    private List<PhotonTrackedTarget> currentTargets = List.of();

    CameraTracker(String name) {
      camera = new PhotonCamera(name);
    }

    void update() {
      for (var result : camera.getAllUnreadResults()) {
        bestTarget = result.hasTargets() ? Optional.of(result.getBestTarget()) : Optional.empty();
        currentTargets = result.getTargets();
        if (bestTarget.isPresent()) {
          bestTargetTimestampSeconds = result.getTimestampSeconds();
        }
      }
    }

    Optional<PhotonTrackedTarget> getTargetById(int fiducialId) {
      return currentTargets.stream().filter(t -> t.getFiducialId() == fiducialId).findFirst();
    }
  }

  private final CameraTracker front = new CameraTracker(VisionConstants.kFrontCameraName);
  private final CameraTracker rear = new CameraTracker(VisionConstants.kRearCameraName);

  // -1 means "not currently targeting anything" - tracked so acquired/lost logging below only
  // prints on a change of the FRONT camera's overall "best" target ID, not every loop.
  private int loggedTargetId = -1;

  @Override
  public void periodic() {
    front.update();
    rear.update();

    int currentId = front.bestTarget.map(PhotonTrackedTarget::getFiducialId).orElse(-1);
    if (currentId != loggedTargetId) {
      if (loggedTargetId != -1) {
        RobotLog.log("April Tag out of view: ID " + loggedTargetId);
      }
      if (currentId != -1) {
        RobotLog.log("April Tag in view: ID " + currentId);
      }
      loggedTargetId = currentId;
    }
  }

  /**
   * The FRONT camera's best-seen target with the given AprilTag ID, if it currently sees one -
   * unlike {@link #hasTarget()}/{@link #getTargetYawDegrees()} (which track only PhotonVision's
   * own overall "best" target regardless of ID), this looks for a specific tag among all targets
   * in that camera's frame.
   */
  public Optional<TargetSighting> getFrontTargetById(int fiducialId) {
    return front.getTargetById(fiducialId)
        .map(t -> new TargetSighting(t, false, front.bestTargetTimestampSeconds));
  }

  /** Same as {@link #getFrontTargetById(int)}, but for the REAR camera. */
  public Optional<TargetSighting> getRearTargetById(int fiducialId) {
    return rear.getTargetById(fiducialId)
        .map(t -> new TargetSighting(t, true, rear.bestTargetTimestampSeconds));
  }

  /** True if the FRONT camera saw an AprilTag in its most recently processed frame. */
  public boolean hasTarget() {
    return front.bestTarget.isPresent();
  }

  /** Fiducial ID of the FRONT camera's overall "best" target, or -1 if none is visible. */
  public int getBestTargetId() {
    return front.bestTarget.map(PhotonTrackedTarget::getFiducialId).orElse(-1);
  }

  /**
   * Yaw of the FRONT camera's best-seen AprilTag relative to its center, in degrees. Positive is
   * counter-clockwise (target to the left of center), per PhotonVision's standard math
   * convention. Returns 0 if no target is visible - callers should check {@link #hasTarget()}
   * first.
   */
  public double getTargetYawDegrees() {
    return front.bestTarget.map(PhotonTrackedTarget::getYaw).orElse(0.0);
  }

  /**
   * The estimated wall-clock time (Time Sync Server base, same as {@code Timer.getFPGATimestamp()}
   * on this robot) at which the frame containing the FRONT camera's current best target was
   * actually captured - always some pipeline/network latency behind "now". Used to latency-
   * compensate the yaw reading against how far the robot has rotated since that frame was taken
   * (see RobotContainer.computeAlignRotationalRate()). Meaningless if {@link #hasTarget()} is
   * false.
   */
  public double getTargetTimestampSeconds() {
    return front.bestTargetTimestampSeconds;
  }
}
