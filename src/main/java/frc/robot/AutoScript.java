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
 * script - safer than guessing what a typo meant.
 */
public final class AutoScript {
  private static final double kFeetToMeters = 0.3048;

  private static final Pattern DRIVE = Pattern.compile(
      "drive\\s+(forward|backward|left|right)\\s+(\\d+(?:\\.\\d+)?)\\s+(?:foot|feet)",
      Pattern.CASE_INSENSITIVE);
  private static final Pattern ROTATE = Pattern.compile(
      "rotate\\s+(?:(left|right)\\s+)?(-?\\d+(?:\\.\\d+)?)\\s+(?:degree|degrees)",
      Pattern.CASE_INSENSITIVE);
  private static final Pattern WAIT = Pattern.compile(
      "wait\\s+(\\d+(?:\\.\\d+)?)\\s+(?:second|seconds)",
      Pattern.CASE_INSENSITIVE);
  private static final Pattern ALIGN_TAG = Pattern.compile(
      "align\\s+(?:with|to)\\s+april\\s*tag\\s+#?(\\d+)",
      Pattern.CASE_INSENSITIVE);

  private AutoScript() {}

  /** Loads and parses autonomous.json. Returns an empty list if it's missing or invalid. */
  public static List<AutoStep> load() {
    File file = new File(Filesystem.getDeployDirectory(), "autonomous.json");
    if (!file.exists()) {
      RobotLog.log("AutoScript: no autonomous.json found - autonomous will do nothing");
      return List.of();
    }

    String[] lines;
    try {
      lines = new ObjectMapper().readValue(file, String[].class);
    } catch (IOException e) {
      RobotLog.log("AutoScript: couldn't read autonomous.json - " + e.getMessage());
      return List.of();
    }

    List<AutoStep> steps = new ArrayList<>();
    for (String line : lines) {
      try {
        steps.add(parseLine(line.trim()));
      } catch (IllegalArgumentException e) {
        RobotLog.log("AutoScript: " + e.getMessage() + " - autonomous will do nothing");
        return List.of();
      }
    }
    return steps;
  }

  // Package-private (not private) so AutoScriptTest can exercise it directly without touching
  // the filesystem.
  static AutoStep parseLine(String line) {
    Matcher drive = DRIVE.matcher(line);
    if (drive.matches()) {
      AutoStep.Direction direction = AutoStep.Direction.valueOf(drive.group(1).toUpperCase());
      double feet = Double.parseDouble(drive.group(2));
      return new AutoStep.Drive(direction, feet * kFeetToMeters);
    }

    Matcher rotate = ROTATE.matcher(line);
    if (rotate.matches()) {
      double degrees = Double.parseDouble(rotate.group(2));
      String word = rotate.group(1);
      if ("left".equalsIgnoreCase(word)) {
        degrees = Math.abs(degrees);
      } else if ("right".equalsIgnoreCase(word)) {
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

    throw new IllegalArgumentException("don't understand instruction \"" + line + "\"");
  }
}
