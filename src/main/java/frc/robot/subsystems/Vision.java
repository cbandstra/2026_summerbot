package frc.robot.subsystems;

import java.util.Optional;

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

  // -1 means no target. Tracked so we only log when a tag comes in/out of view, not every loop.
  private int loggedTargetId = -1;

  @Override
  public void periodic() {
    for (var result : camera.getAllUnreadResults()) {
      bestTarget = result.hasTargets() ? Optional.of(result.getBestTarget()) : Optional.empty();
      if (bestTarget.isPresent()) {
        bestTargetTimestampSeconds = result.getTimestampSeconds();
      }
    }

    int currentId = bestTarget.map(PhotonTrackedTarget::getFiducialId).orElse(-1);
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

  /** True if an AprilTag was seen in the most recent camera frame. */
  public boolean hasTarget() {
    return bestTarget.isPresent();
  }

  /** Yaw to the best-seen tag, in degrees. Positive means the tag is left of center. 0 if no target. */
  public double getTargetYawDegrees() {
    return bestTarget.map(PhotonTrackedTarget::getYaw).orElse(0.0);
  }

  /**
   * When the current best target's camera frame was actually captured - always a little behind
   * "now". Used to correct for lag; see {@link frc.robot.RobotContainer#computeCompensatedYawDegrees}.
   */
  public double getTargetTimestampSeconds() {
    return bestTargetTimestampSeconds;
  }
}
