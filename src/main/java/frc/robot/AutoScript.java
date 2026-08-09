package frc.robot;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * Reads autonomous.json from the deploy directory - a plain JSON array of instruction strings -
 * and turns each one into an {@link AutoStep}. See README.md for the full list of instructions
 * and their exact wording.
 *
 * <p>Editing autonomous.json doesn't need a code change, just a redeploy. If a line can't be
 * understood, the whole file is rejected (logged to the console) rather than running part of a
 * script - safer than guessing what a typo meant. Blank lines, and lines starting with {@code #}
 * or {@code //} (leading whitespace is fine), are comments and are skipped instead of run.
 */
public final class AutoScript {
  private static final double kFeetToMeters = 0.3048;

  private static final Pattern DRIVE = Pattern.compile(
      "drive\\s+(forward|backward|left|right)\\s+(\\d+(?:\\.\\d+)?)\\s+(?:foot|feet)",
      Pattern.CASE_INSENSITIVE);
  private static final Pattern DRIVE_TOWARD_TARGET = Pattern.compile(
      "drive\\s+towards?\\s+target\\s+(\\d+(?:\\.\\d+)?)\\s+(?:foot|feet)",
      Pattern.CASE_INSENSITIVE);
  private static final Pattern DRIVE_AWAY_FROM_TARGET = Pattern.compile(
      "drive\\s+away\\s+from\\s+target\\s+(\\d+(?:\\.\\d+)?)\\s+(?:foot|feet)",
      Pattern.CASE_INSENSITIVE);
  private static final Pattern ROTATE = Pattern.compile(
      "rotate\\s+(?:(left|right|clockwise|counterclockwise)\\s+)?(-?\\d+(?:\\.\\d+)?)\\s+(?:degree|degrees)"
          + "(?:\\s+(clockwise|counterclockwise))?",
      Pattern.CASE_INSENSITIVE);
  private static final Pattern WAIT = Pattern.compile(
      "wait\\s+(\\d+(?:\\.\\d+)?)\\s+(?:second|seconds)",
      Pattern.CASE_INSENSITIVE);
  private static final Pattern ALIGN_TAG = Pattern.compile(
      "align\\s+(?:with|to)\\s+april\\s*tag\\s+#?(\\d+)",
      Pattern.CASE_INSENSITIVE);
  private static final Pattern LINE_UP_TAG = Pattern.compile(
      "align\\s+(?:with|to)\\s+april\\s*tag\\s+#?(\\d+)\\s+and\\s+go\\s+to\\s+it",
      Pattern.CASE_INSENSITIVE);
  private static final Pattern RECENTER = Pattern.compile("recenter", Pattern.CASE_INSENSITIVE);
  private static final Pattern VISION_ROTATION_TEST = Pattern.compile(
      "vision\\s+rotation\\s+test", Pattern.CASE_INSENSITIVE);
  private static final Pattern PLAY_BEEP = Pattern.compile(
      "play\\s+beep\\s+(\\d+)\\s+(?:time|times)", Pattern.CASE_INSENSITIVE);

  private AutoScript() {}

  /** Loads and parses autonomous.json. Returns an empty list if it's missing or invalid. */
  public static List<AutoStep> load() {
    return load("autonomous.json");
  }

  /**
   * Loads and parses a script file (same format as autonomous.json) from the deploy directory by
   * name - e.g. {@code "goHome.json"}. Returns an empty list if it's missing or invalid.
   */
  public static List<AutoStep> load(String fileName) {
    File file = new File(Filesystem.getDeployDirectory(), fileName);
    if (!file.exists()) {
      RobotLog.log("AutoScript: no " + fileName + " found - won't run");
      return List.of();
    }

    String[] lines;
    try {
      lines = new ObjectMapper().readValue(file, String[].class);
    } catch (IOException e) {
      RobotLog.log("AutoScript: couldn't read " + fileName + " - " + e.getMessage());
      return List.of();
    }

    try {
      return parseLines(lines);
    } catch (IllegalArgumentException e) {
      RobotLog.log("AutoScript: " + fileName + ": " + e.getMessage() + " - won't run");
      return List.of();
    }
  }

  // Package-private (not private) so AutoScriptTest can exercise these directly without touching
  // the filesystem.
  static List<AutoStep> parseLines(String[] lines) {
    List<AutoStep> steps = new ArrayList<>();
    for (String rawLine : lines) {
      String line = rawLine.trim();
      if (line.isEmpty() || line.startsWith("#") || line.startsWith("//")) {
        continue;
      }
      steps.add(parseLine(line));
    }
    return steps;
  }

  static AutoStep parseLine(String line) {
    Matcher drive = DRIVE.matcher(line);
    if (drive.matches()) {
      AutoStep.Direction direction = AutoStep.Direction.valueOf(drive.group(1).toUpperCase());
      double feet = Double.parseDouble(drive.group(2));
      return new AutoStep.Drive(direction, feet * kFeetToMeters);
    }

    Matcher driveToward = DRIVE_TOWARD_TARGET.matcher(line);
    if (driveToward.matches()) {
      double feet = Double.parseDouble(driveToward.group(1));
      return new AutoStep.DriveToward(feet * kFeetToMeters);
    }

    Matcher driveAway = DRIVE_AWAY_FROM_TARGET.matcher(line);
    if (driveAway.matches()) {
      double feet = Double.parseDouble(driveAway.group(1));
      return new AutoStep.DriveAway(feet * kFeetToMeters);
    }

    Matcher rotate = ROTATE.matcher(line);
    if (rotate.matches()) {
      double degrees = Double.parseDouble(rotate.group(2));
      // The direction word can come right after "rotate" or after "degrees" - whichever's there.
      String word = rotate.group(1) != null ? rotate.group(1) : rotate.group(3);
      if ("left".equalsIgnoreCase(word) || "counterclockwise".equalsIgnoreCase(word)) {
        degrees = Math.abs(degrees);
      } else if ("right".equalsIgnoreCase(word) || "clockwise".equalsIgnoreCase(word)) {
        degrees = -Math.abs(degrees);
      }
      return new AutoStep.Rotate(degrees);
    }

    Matcher wait = WAIT.matcher(line);
    if (wait.matches()) {
      return new AutoStep.Wait(Double.parseDouble(wait.group(1)));
    }

    Matcher alignTag = ALIGN_TAG.matcher(line);
    if (alignTag.matches()) {
      return new AutoStep.AlignTag(Integer.parseInt(alignTag.group(1)));
    }

    Matcher lineUpTag = LINE_UP_TAG.matcher(line);
    if (lineUpTag.matches()) {
      return new AutoStep.LineUpTag(Integer.parseInt(lineUpTag.group(1)));
    }

    Matcher recenter = RECENTER.matcher(line);
    if (recenter.matches()) {
      return new AutoStep.Recenter();
    }

    Matcher visionRotationTest = VISION_ROTATION_TEST.matcher(line);
    if (visionRotationTest.matches()) {
      return new AutoStep.VisionRotationTest();
    }

    Matcher playBeep = PLAY_BEEP.matcher(line);
    if (playBeep.matches()) {
      return new AutoStep.PlayBeep(Integer.parseInt(playBeep.group(1)));
    }

    throw new IllegalArgumentException("don't understand instruction \"" + line + "\"");
  }
}
