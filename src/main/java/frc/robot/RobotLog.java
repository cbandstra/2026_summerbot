package frc.robot;

import edu.wpi.first.wpilibj.Timer;

/**
 * Small helper for timestamped status lines shared across robot code. Uses System.out (not
 * DriverStation.reportWarning/reportError) since these are routine status messages, not
 * diagnostics - they still reach the Driver Station console via NetConsole either way.
 */
public final class RobotLog {
  private RobotLog() {}

  // While true, log() drops its message instead of printing it. Used by the "Vision rotation
  // test" autonomous mode so only its own summary lines show up on the console, not the routine
  // status chatter (target lock, "April Tags in view", etc.) that'd otherwise flood it during
  // its rapid-fire spinning - see setQuiet() and logAlways().
  private static volatile boolean quiet = false;

  /** Silences (true) or restores (false) log() - see the {@code quiet} field's own comment. */
  public static void setQuiet(boolean isQuiet) {
    quiet = isQuiet;
  }

  /** Prints {@code message} to the console (visible in the Driver Station log) with a timestamp. */
  public static void log(String message) {
    if (quiet) {
        return;
    }
    logAlways(message);
  }

  /** Same as {@link #log}, but always prints even while quiet - for callers whose own output IS
   * what quiet mode is trying to isolate (e.g. the rotation test's own result lines). */
  public static void logAlways(String message) {
    System.out.printf("[%.3f] %s%n", Timer.getFPGATimestamp(), message);
  }
}
