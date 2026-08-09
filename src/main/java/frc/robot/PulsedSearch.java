package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants.VisionConstants;

/**
 * Turns the continuous "spin looking for a tag" behavior into alternating fast/slow pulses: spin
 * fast for {@link VisionConstants#kSearchSpinDegrees} degrees, spin slow for {@link
 * VisionConstants#kSearchSlowPhaseSeconds}, then spin fast again - repeat until a tag is seen.
 * Gives the camera a steadier look during the slow phase instead of only ever seeing tags blur
 * past mid-turn, without ever fully stopping. Set {@link VisionConstants#kSearchPulseEnabled}
 * false to skip all that and just spin at one steady rate instead.
 *
 * <p>Call {@link #reset} once when a search starts, then {@link #pulse} every loop while no
 * target is seen. Shared by target lock, "align with april tag", "align with april tag ... and
 * go to it" (all in {@link RobotContainer}), and {@link VisionRotationTest}.
 */
final class PulsedSearch {
  private final double maxAngularRateRadPerSec;
  private final Timer m_slowPhaseTimer = new Timer();
  private Rotation2d m_lastHeading = Rotation2d.kZero;
  private double m_spunDegrees = 0.0;
  private boolean m_slowPhase = false;

  PulsedSearch(double maxAngularRateRadPerSec) {
    this.maxAngularRateRadPerSec = maxAngularRateRadPerSec;
  }

  void reset(Rotation2d currentHeading) {
    m_lastHeading = currentHeading;
    m_spunDegrees = 0.0;
    m_slowPhase = false;
  }

  /**
   * Rotational rate (rad/s) to command this loop - fast during a pulse, slow between them, using
   * the standard search speeds. Respects {@link VisionConstants#kSearchPulseEnabled} (spins at
   * one steady rate instead if that's false).
   */
  double pulse(Rotation2d currentHeading) {
    if (!VisionConstants.kSearchPulseEnabled) {
        return Math.min(VisionConstants.kSearchRotationRadPerSec, maxAngularRateRadPerSec);
    }
    return pulse(currentHeading, VisionConstants.kSearchRotationRadPerSec, VisionConstants.kSearchSlowRotationRadPerSec);
  }

  /**
   * Same as {@link #pulse(Rotation2d)}, but with caller-specified fast/slow speeds instead of the
   * standard ones - always actually pulses regardless of {@link
   * VisionConstants#kSearchPulseEnabled}, since passing custom speeds is itself an explicit
   * request to pulse. Used by {@link VisionRotationTest} to sweep speeds.
   */
  double pulse(Rotation2d currentHeading, double fastRadPerSec, double slowRadPerSec) {
    if (m_slowPhase) {
        if (m_slowPhaseTimer.hasElapsed(VisionConstants.kSearchSlowPhaseSeconds)) {
            m_slowPhase = false;
            m_spunDegrees = 0.0;
            m_lastHeading = currentHeading;
        }
        return Math.min(slowRadPerSec, maxAngularRateRadPerSec);
    }

    m_spunDegrees += Math.abs(currentHeading.minus(m_lastHeading).getDegrees());
    m_lastHeading = currentHeading;

    if (m_spunDegrees >= VisionConstants.kSearchSpinDegrees) {
        m_slowPhase = true;
        m_slowPhaseTimer.restart();
        return Math.min(slowRadPerSec, maxAngularRateRadPerSec);
    }
    return Math.min(fastRadPerSec, maxAngularRateRadPerSec);
  }
}
