package frc.robot;

/** One instruction parsed from autonomous.json. See {@link AutoScript} for the text format. */
public sealed interface AutoStep {
  enum Direction { FORWARD, BACKWARD, LEFT, RIGHT }

  /** Drive one direction for a distance, then stop. */
  record Drive(Direction direction, double distanceMeters) implements AutoStep {}

  /** Turn in place. Positive degrees = counterclockwise, negative = clockwise. */
  record Rotate(double degrees) implements AutoStep {}

  /** Sit still for a while. */
  record Wait(double seconds) implements AutoStep {}

  /** Search for a specific AprilTag ID and turn to face it. */
  record AlignTag(int tagId) implements AutoStep {}
}
