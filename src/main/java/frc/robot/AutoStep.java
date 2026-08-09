package frc.robot;

/** One instruction parsed from autonomous.json. See {@link AutoScript} for the text format. */
public sealed interface AutoStep {
  enum Direction { FORWARD, BACKWARD, LEFT, RIGHT }

  /** Drive one direction for a distance, then stop. */
  record Drive(Direction direction, double distanceMeters) implements AutoStep {}

  /**
   * Drive straight for a distance in whichever direction the robot is currently facing (e.g.
   * after a {@code rotate} step aimed it at a target), then stop. Same robot-relative "forward"
   * as {@link Drive} with {@link Direction#FORWARD} - just named for the "rotate toward a
   * target, then drive at it" script pattern.
   */
  record DriveToward(double distanceMeters) implements AutoStep {}

  /**
   * Drive straight for a distance in the opposite direction the robot is currently facing -
   * simply reverses {@link DriveToward}'s wheel direction rather than re-pointing the wheels.
   */
  record DriveAway(double distanceMeters) implements AutoStep {}

  /** Turn in place. Positive degrees = counterclockwise, negative = clockwise. */
  record Rotate(double degrees) implements AutoStep {}

  /** Sit still for a while. */
  record Wait(double seconds) implements AutoStep {}

  /** Search for a specific AprilTag ID and turn to face it. */
  record AlignTag(int tagId) implements AutoStep {}

  /** Search for a specific AprilTag ID, then drive up to and center on it. */
  record LineUpTag(int tagId) implements AutoStep {}

  /**
   * Same as button 7 - makes wherever the robot is currently facing the new "forward" for
   * field-centric driving. Doesn't move the robot or touch its tracked field position.
   */
  record Recenter() implements AutoStep {}

  /**
   * Diagnostic, not a real match step - see {@link VisionRotationTest} for what it actually
   * does. Finds how fast the robot can spin while still reliably seeing AprilTags.
   */
  record VisionRotationTest() implements AutoStep {}

  /**
   * Plays the same beep normally played after every autonomous step finishes (see
   * RobotContainer.beepCommand()), {@code times} times back to back - always plays regardless of
   * {@code AutoConstants#kStepCompleteBeepEnabled}, since the whole point of this step is to beep.
   */
  record PlayBeep(int times) implements AutoStep {}
}
