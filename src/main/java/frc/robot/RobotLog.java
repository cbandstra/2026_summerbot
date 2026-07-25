package frc.robot;

import edu.wpi.first.wpilibj.Timer;

/**
 * Small helper for timestamped status lines shared across robot code. Uses System.out (not
 * DriverStation.reportWarning/reportError) since these are routine status messages, not
 * diagnostics - they still reach the Driver Station console via NetConsole either way.
 */
public final class RobotLog {
  private RobotLog() {}

  /** Prints {@code message} to the console (visible in the Driver Station log) with a timestamp. */
  public static void log(String message) {
    System.out.printf("[%.3f] %s%n", Timer.getFPGATimestamp(), message);
  }
}
