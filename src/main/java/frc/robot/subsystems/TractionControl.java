package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants.TractionConstants;
import frc.robot.RobotLog;

/**
 * An "electronic limited slip differential" for the swerve drivetrain - backs off commanded
 * speed when a wheel or the whole chassis isn't actually achieving what was asked of it, almost
 * always because a wheel lost traction and is just spinning rather than the drivetrain genuinely
 * being too slow. Two independent checks, since they catch different kinds of slip - see {@link
 * #correctTranslation} and {@link #correctRotation}. Applied by {@link
 * CommandSwerveDrivetrain#setControl} to every drive command (teleop, buttons, and autonomous
 * alike), not wired up by hand at each call site. Logs once when each check starts and stops
 * correcting, not every loop while it persists.
 */
final class TractionControl {
  // Debounced so a normal spin-up (which also briefly lags the commanded rotational rate, same
  // signature as real slip) isn't mistaken for slip - see kRotationSlipDebounceSeconds.
  private final Debouncer m_rotationSlipDebouncer = new Debouncer(
      TractionConstants.kRotationSlipDebounceSeconds, Debouncer.DebounceType.kRising);

  private boolean m_translationSlipping = false;
  private boolean m_rotationSlipping = false;

  /**
   * Scales {@code vx}/{@code vy} down by the worst-slipping wheel's ratio of commanded to actual
   * speed, or returns them unchanged if no wheel is slipping. Compares each wheel only against
   * its OWN last commanded target ({@code targetStates}, from {@link
   * com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState#ModuleTargets}) - never against
   * the other wheels, since swerve wheels are supposed to run at different speeds during any
   * turn. See {@link TractionConstants#kTranslationSlipRatio}.
   */
  double[] correctTranslation(double vx, double vy, SwerveModuleState[] actualStates,
      SwerveModuleState[] targetStates) {
    double worstScale = 1.0;
    int worstIndex = -1;
    for (int i = 0; i < actualStates.length; i++) {
        double targetSpeedMps = Math.abs(targetStates[i].speedMetersPerSecond);
        if (targetSpeedMps < TractionConstants.kMinModuleSpeedMps) {
            continue;
        }
        double actualSpeedMps = Math.abs(actualStates[i].speedMetersPerSecond);
        if (actualSpeedMps > targetSpeedMps * TractionConstants.kTranslationSlipRatio) {
            // Cuts harder than the bare minimum needed to match this wheel's own ratio - see
            // kTranslationCorrectionAggressiveness's own comment for why.
            double scale = (targetSpeedMps / actualSpeedMps)
                * TractionConstants.kTranslationCorrectionAggressiveness;
            if (scale < worstScale) {
                worstScale = scale;
                worstIndex = i;
            }
        }
    }
    worstScale = Math.max(worstScale, TractionConstants.kMinTranslationCorrectionScale);

    boolean slipping = worstIndex >= 0;
    if (slipping && !m_translationSlipping) {
        RobotLog.log(String.format(
            "Traction control: wheel %d slipping (target %.2f m/s, actual %.2f m/s) - "
                + "scaling translation to %.0f%%",
            worstIndex, Math.abs(targetStates[worstIndex].speedMetersPerSecond),
            Math.abs(actualStates[worstIndex].speedMetersPerSecond), worstScale * 100));
    } else if (!slipping && m_translationSlipping) {
        RobotLog.log("Traction control: wheel slip cleared");
    }
    m_translationSlipping = slipping;

    return new double[] {vx * worstScale, vy * worstScale};
  }

  /**
   * Scales {@code commandedOmegaRadPerSec} down once the gyro's actual measured angular velocity
   * (ground truth, independent of the wheel encoders odometry itself is built from) has
   * persistently fallen too far behind it, or returns it unchanged otherwise. See {@link
   * TractionConstants#kRotationSlipRatio}/{@link TractionConstants#kRotationSlipDebounceSeconds}.
   */
  double correctRotation(double commandedOmegaRadPerSec, double actualOmegaRadPerSec) {
    boolean commandedEnough =
        Math.abs(commandedOmegaRadPerSec) >= TractionConstants.kMinRotationRadPerSec;
    boolean laggingBehind = Math.abs(actualOmegaRadPerSec)
        < Math.abs(commandedOmegaRadPerSec) * TractionConstants.kRotationSlipRatio;
    boolean slipping = m_rotationSlipDebouncer.calculate(commandedEnough && laggingBehind);

    if (slipping && !m_rotationSlipping) {
        RobotLog.log(String.format(
            "Traction control: rotational slip (commanded %.2f rad/s, actual %.2f rad/s) - "
                + "scaling rotation to %.0f%%",
            commandedOmegaRadPerSec, actualOmegaRadPerSec, TractionConstants.kRotationCorrectionScale * 100));
    } else if (!slipping && m_rotationSlipping) {
        RobotLog.log("Traction control: rotational slip cleared");
    }
    m_rotationSlipping = slipping;

    return slipping ? commandedOmegaRadPerSec * TractionConstants.kRotationCorrectionScale
        : commandedOmegaRadPerSec;
  }
}
