package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.Comparator;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Prunes old CTRE SignalLogger session folders on the roboRIO's flash storage. The roboRIO has
 * very little onboard space (under 400 MB total), and SignalLogger refuses to start a new log
 * once free space drops below 10 MB (and separately warns/starts auto-deleting its own old hoot
 * logs once free space drops below 50 MB) - each timestamped folder under kLogsDirectory is one
 * past session's log, so the oldest ones are safe to delete to make room.
 */
final class LogCleanup {
  private static final Path kLogsDirectory = Path.of("/home/lvuser/logs");

  // Prune until at least this much space is free. Deliberately well above CTRE's own 50 MB
  // "low disk" warning threshold (not just its 10 MB hard-failure one) - targeting 50 MB exactly
  // left zero buffer, so disk usage crossing back under 50 MB between deploys (from CTRE's own
  // logging accumulating) immediately re-triggered CTRE's warning. Confirmed on the robot.
  private static final long kTargetFreeBytes = 150L * 1024 * 1024;

  private LogCleanup() {}

  /** Deletes the oldest session folders under the logs directory until enough space is free. */
  static void pruneIfLow() {
    File logsDir = kLogsDirectory.toFile();
    if (!logsDir.isDirectory()) {
      return;
    }

    File[] sessionDirs = logsDir.listFiles(File::isDirectory);
    if (sessionDirs == null) {
      return;
    }
    Arrays.sort(sessionDirs, Comparator.comparingLong(File::lastModified));

    for (File sessionDir : sessionDirs) {
      if (logsDir.getUsableSpace() >= kTargetFreeBytes) {
        return;
      }
      deleteRecursively(sessionDir.toPath());
      DriverStation.reportWarning("LogCleanup: deleted old log folder " + sessionDir.getName(), false);
    }
  }

  private static void deleteRecursively(Path path) {
    try (Stream<Path> walk = Files.walk(path)) {
      walk.sorted(Comparator.reverseOrder()).forEach(p -> {
        try {
          Files.delete(p);
        } catch (IOException e) {
          // Best-effort cleanup - leave whatever couldn't be deleted and move on rather than
          // failing robot startup over it.
        }
      });
    } catch (IOException e) {
      // Best-effort cleanup - see above.
    }
  }
}
