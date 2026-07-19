package frc.robot.subsystems;

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

  @Override
  public void periodic() {
    for (var result : camera.getAllUnreadResults()) {
      bestTarget = result.hasTargets() ? Optional.of(result.getBestTarget()) : Optional.empty();
    }
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
}
