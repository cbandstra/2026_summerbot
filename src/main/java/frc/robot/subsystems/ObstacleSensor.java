package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.UltrasonicConstants;
import frc.robot.RobotLog;

/**
 * Wraps a single HC-SR04 ultrasonic sensor via WPILib's built-in trigger/echo Ultrasonic class.
 * No avoidance behavior yet - this just gets a real distance reading onto the dashboard and the
 * console/RioLog so wiring and orientation can be confirmed before anything reacts to it.
 */
public class ObstacleSensor extends SubsystemBase {
  private final Ultrasonic ultrasonic = new Ultrasonic(
      UltrasonicConstants.kTriggerChannel, UltrasonicConstants.kEchoChannel);

  // Throttles the console print to ~1/sec (50 loops at the default 50Hz) instead of every loop.
  private int m_printCounter = 0;

  public ObstacleSensor() {
    // Automatic mode runs the trigger/echo ping cycle in the background - without this,
    // getRangeInches() below never updates. Static/global, but harmless to call with only one
    // Ultrasonic instance on the robot.
    Ultrasonic.setAutomaticMode(true);
  }

  @Override
  public void periodic() {
    double distanceInches = ultrasonic.getRangeInches();
    boolean rangeValid = ultrasonic.isRangeValid();

    SmartDashboard.putNumber("Ultrasonic/DistanceInches", distanceInches);
    SmartDashboard.putBoolean("Ultrasonic/RangeValid", rangeValid);

    m_printCounter++;
    if (m_printCounter % 50 == 0) {
        RobotLog.log(String.format(
            "Ultrasonic: distance=%.1fin valid=%b", distanceInches, rangeValid));
    }
  }

  /** Most recent distance reading in inches - only meaningful once the sensor has pinged. */
  public double getDistanceInches() {
    return ultrasonic.getRangeInches();
  }

  /**
   * True if something is currently closer than {@link UltrasonicConstants#kObstacleStopDistanceInches}.
   * A reading of exactly 0.0 inches is treated as "no obstacle" rather than "touching the
   * sensor" - the HC-SR04 (and WPILib's Ultrasonic wrapper) reports 0 when it hasn't gotten a
   * valid echo back yet (e.g. right at startup, before the first ping/response cycle completes),
   * not as a real zero-distance measurement, so treating it as a genuine obstacle would block
   * driving before the sensor has ever produced a real reading.
   */
  public boolean isObstacleTooClose() {
    double distanceInches = getDistanceInches();
    return distanceInches != 0.0 && distanceInches < UltrasonicConstants.kObstacleStopDistanceInches;
  }
}
