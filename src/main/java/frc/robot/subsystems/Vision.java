package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Collectors;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisionConstants;
import frc.robot.RobotLog;

/** Wraps the PhotonVision camera. Caches the best target once per loop so everyone sees the same result. */
public class Vision extends SubsystemBase {
  private final PhotonCamera camera = new PhotonCamera(VisionConstants.kCameraName);
  private Optional<PhotonTrackedTarget> bestTarget = Optional.empty();
  private double bestTargetTimestampSeconds = 0.0;
  private List<PhotonTrackedTarget> currentTargets = List.of();

  // So we only log when the set of visible tag IDs actually changes, not every loop. Logs every
  // ID currently in view (not just PhotonVision's single "best" pick) so it's possible to tell
  // whether a specific tag ID is actually visible, not just "some" tag.
  private Set<Integer> loggedVisibleIds = Set.of();

  @Override
  public void periodic() {
    for (var result : camera.getAllUnreadResults()) {
      bestTarget = result.hasTargets() ? Optional.of(result.getBestTarget()) : Optional.empty();
      currentTargets = result.getTargets();
      if (bestTarget.isPresent()) {
        bestTargetTimestampSeconds = result.getTimestampSeconds();
      }
    }

    Set<Integer> visibleIds = currentTargets.stream()
        .map(PhotonTrackedTarget::getFiducialId)
        .collect(Collectors.toCollection(TreeSet::new));
    if (!visibleIds.equals(loggedVisibleIds)) {
      RobotLog.log("April Tags in view: " + (visibleIds.isEmpty() ? "none" : visibleIds));
      loggedVisibleIds = visibleIds;
    }
  }

  /** True if an AprilTag was seen in the most recent camera frame. */
  public boolean hasTarget() {
    return bestTarget.isPresent();
  }

  /** The target with this AprilTag ID, if it was seen in the most recent camera frame. */
  public Optional<PhotonTrackedTarget> getTargetById(int fiducialId) {
    return currentTargets.stream().filter(t -> t.getFiducialId() == fiducialId).findFirst();
  }

  /** Whichever tag is currently best-seen - the one target lock aligns to. Empty if none. */
  public Optional<PhotonTrackedTarget> getBestTarget() {
    return bestTarget;
  }

  /** Yaw to the best-seen tag, in degrees. Positive means the tag is left of center. 0 if no target. */
  public double getTargetYawDegrees() {
    return bestTarget.map(PhotonTrackedTarget::getYaw).orElse(0.0);
  }

  /** Straight-line distance from the camera to the best-seen tag, in meters. 0 if no target. */
  public double getTargetDistanceMeters() {
    return bestTarget.map(t -> t.getBestCameraToTarget().getTranslation().getNorm()).orElse(0.0);
  }

  /**
   * When the current best target's camera frame was actually captured - always a little behind
   * "now". Used to correct for lag; see {@link frc.robot.RobotContainer#computeCompensatedYawDegrees}.
   */
  public double getTargetTimestampSeconds() {
    return bestTargetTimestampSeconds;
  }
}
