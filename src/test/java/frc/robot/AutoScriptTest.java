package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

class AutoScriptTest {
  @Test
  void driveForward() {
    var step = (AutoStep.Drive) AutoScript.parseLine("drive forward 3 feet");
    assertEquals(AutoStep.Direction.FORWARD, step.direction());
    assertEquals(3 * 0.3048, step.distanceMeters(), 1e-9);
  }

  @Test
  void driveRightSingularFoot() {
    var step = (AutoStep.Drive) AutoScript.parseLine("drive right 1 foot");
    assertEquals(AutoStep.Direction.RIGHT, step.direction());
  }

  @Test
  void driveFractionalFeet() {
    var step = (AutoStep.Drive) AutoScript.parseLine("drive backward 2.5 feet");
    assertEquals(2.5 * 0.3048, step.distanceMeters(), 1e-9);
  }

  @Test
  void rotateBareNumberIsSigned() {
    var positive = (AutoStep.Rotate) AutoScript.parseLine("rotate 90 degrees");
    assertEquals(90.0, positive.degrees(), 1e-9);

    var negative = (AutoStep.Rotate) AutoScript.parseLine("rotate -90 degrees");
    assertEquals(-90.0, negative.degrees(), 1e-9);
  }

  @Test
  void rotateWithDirectionWordOverridesSign() {
    var left = (AutoStep.Rotate) AutoScript.parseLine("rotate left 90 degrees");
    assertEquals(90.0, left.degrees(), 1e-9);

    var right = (AutoStep.Rotate) AutoScript.parseLine("rotate right 90 degree");
    assertEquals(-90.0, right.degrees(), 1e-9);
  }

  @Test
  void waitInstruction() {
    var step = (AutoStep.Wait) AutoScript.parseLine("wait 1.5 seconds");
    assertEquals(1.5, step.seconds(), 1e-9);
  }

  @Test
  void alignWithTagBothPhrasings() {
    var withHash = (AutoStep.AlignTag) AutoScript.parseLine("align with april tag #4");
    assertEquals(4, withHash.tagId());

    var noHash = (AutoStep.AlignTag) AutoScript.parseLine("align to april tag 7");
    assertEquals(7, noHash.tagId());
  }

  @Test
  void isCaseInsensitive() {
    var step = (AutoStep.Drive) AutoScript.parseLine("DRIVE FORWARD 3 FEET");
    assertEquals(AutoStep.Direction.FORWARD, step.direction());
  }

  @Test
  void unrecognizedInstructionThrows() {
    assertThrows(IllegalArgumentException.class, () -> AutoScript.parseLine("drive sideways 3 feet"));
    assertThrows(IllegalArgumentException.class, () -> AutoScript.parseLine("drive forward 3 fet"));
    assertThrows(IllegalArgumentException.class, () -> AutoScript.parseLine("do a barrel roll"));
  }
}
