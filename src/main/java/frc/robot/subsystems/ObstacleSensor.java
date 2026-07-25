package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.UltrasonicConstants;

/**
 * Wraps a single HC-SR04 ultrasonic sensor via WPILib's built-in trigger/echo Ultrasonic class.
 * No avoidance behavior yet - this just gets a real distance reading onto the dashboard so wiring
 * and orientation can be confirmed before anything reacts to it.
 */
public class ObstacleSensor extends SubsystemBase {
  private final Ultrasonic ultrasonic = new Ultrasonic(
      UltrasonicConstants.kTriggerChannel, UltrasonicConstants.kEchoChannel);

  public ObstacleSensor() {
    // Automatic mode runs the trigger/echo ping cycle in the background - without this,
    // getRangeInches() below never updates. Static/global, but harmless to call with only one
    // Ultrasonic instance on the robot.
    Ultrasonic.setAutomaticMode(true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Ultrasonic/DistanceInches", ultrasonic.getRangeInches());
    SmartDashboard.putBoolean("Ultrasonic/RangeValid", ultrasonic.isRangeValid());
  }

  /** Most recent distance reading in inches - only meaningful once the sensor has pinged. */
  public double getDistanceInches() {
    return ultrasonic.getRangeInches();
  }
}
