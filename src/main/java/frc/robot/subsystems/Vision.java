package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisionConstants;

/**
 * Wraps the PhotonVision coprocessor's camera feed. The best target is cached once per
 * scheduler loop in periodic() so every consumer within the same loop sees a consistent result.
 * Uses getAllUnreadResults() (the camera can produce frames faster than the robot loop runs) and
 * keeps the previous target if no new frame arrived this loop, rather than flickering to "no
 * target" for a loop iteration.
 */
public class Vision extends SubsystemBase {
  private final PhotonCamera camera = new PhotonCamera(VisionConstants.kCameraName);
  private Optional<PhotonTrackedTarget> bestTarget = Optional.empty();
  private double bestTargetTimestampSeconds = 0.0;
  private List<PhotonTrackedTarget> currentTargets = List.of();

  @Override
  public void periodic() {
    for (var result : camera.getAllUnreadResults()) {
      bestTarget = result.hasTargets() ? Optional.of(result.getBestTarget()) : Optional.empty();
      currentTargets = result.getTargets();
      if (bestTarget.isPresent()) {
        bestTargetTimestampSeconds = result.getTimestampSeconds();
      }
    }
  }

  /**
   * The best-seen target with the given AprilTag ID, if any tag with that ID is in the most
   * recently processed camera frame - unlike {@link #hasTarget()}/{@link #getTargetYawDegrees()}
   * (which track PhotonVision's own overall "best" target regardless of ID), this looks for a
   * specific tag among all targets in that frame. Its timestamp for latency-compensation
   * purposes is {@link #getTargetTimestampSeconds()} - every target in a single camera frame
   * shares the same capture time.
   */
  public Optional<PhotonTrackedTarget> getTargetById(int fiducialId) {
    return currentTargets.stream().filter(t -> t.getFiducialId() == fiducialId).findFirst();
  }

  /** True if an AprilTag was seen in the most recently processed camera frame. */
  public boolean hasTarget() {
    return bestTarget.isPresent();
  }

  /**
   * Yaw of the best-seen AprilTag relative to the camera's center, in degrees. Positive is
   * counter-clockwise (target to the left of center), per PhotonVision's standard math
   * convention. Returns 0 if no target is visible - callers should check {@link #hasTarget()}
   * first.
   */
  public double getTargetYawDegrees() {
    return bestTarget.map(PhotonTrackedTarget::getYaw).orElse(0.0);
  }

  /**
   * The estimated wall-clock time (Time Sync Server base, same as {@code Timer.getFPGATimestamp()}
   * on this robot) at which the frame containing the current best target was actually captured -
   * always some pipeline/network latency behind "now". Used to latency-compensate the yaw reading
   * against how far the robot has rotated since that frame was taken (see
   * RobotContainer.computeAlignRotationalRate()). Meaningless if {@link #hasTarget()} is false.
   */
  public double getTargetTimestampSeconds() {
    return bestTargetTimestampSeconds;
  }
}
