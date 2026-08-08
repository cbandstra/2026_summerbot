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
  void rotateWithClockwiseWordingBeforeOrAfter() {
    var leadingCcw = (AutoStep.Rotate) AutoScript.parseLine("rotate counterclockwise 180 degrees");
    assertEquals(180.0, leadingCcw.degrees(), 1e-9);

    var trailingCcw = (AutoStep.Rotate) AutoScript.parseLine("rotate 180 degrees counterclockwise");
    assertEquals(180.0, trailingCcw.degrees(), 1e-9);

    var leadingCw = (AutoStep.Rotate) AutoScript.parseLine("rotate clockwise 45 degrees");
    assertEquals(-45.0, leadingCw.degrees(), 1e-9);

    var trailingCw = (AutoStep.Rotate) AutoScript.parseLine("rotate 45 degrees clockwise");
    assertEquals(-45.0, trailingCw.degrees(), 1e-9);
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
  void lineUpWithTag() {
    var step = (AutoStep.LineUpTag) AutoScript.parseLine("Align With April Tag 1 and go to it");
    assertEquals(1, step.tagId());

    var withHashAndTo = (AutoStep.LineUpTag) AutoScript.parseLine("align to april tag #12 and go to it");
    assertEquals(12, withHashAndTo.tagId());
  }

  @Test
  void isCaseInsensitive() {
    var step = (AutoStep.Drive) AutoScript.parseLine("DRIVE FORWARD 3 FEET");
    assertEquals(AutoStep.Direction.FORWARD, step.direction());
  }

  @Test
  void commentsAndBlankLinesAreSkipped() {
    var steps = AutoScript.parseLines(new String[] {
        "# this is a comment",
        "drive forward 3 feet",
        "",
        "  // also a comment",
        "wait 1 seconds",
        "   ",
    });

    assertEquals(2, steps.size());
    assertEquals(AutoStep.Direction.FORWARD, ((AutoStep.Drive) steps.get(0)).direction());
    assertEquals(1.0, ((AutoStep.Wait) steps.get(1)).seconds(), 1e-9);
  }

  @Test
  void unrecognizedInstructionThrows() {
    assertThrows(IllegalArgumentException.class, () -> AutoScript.parseLine("drive sideways 3 feet"));
    assertThrows(IllegalArgumentException.class, () -> AutoScript.parseLine("drive forward 3 fet"));
    assertThrows(IllegalArgumentException.class, () -> AutoScript.parseLine("do a barrel roll"));
  }
}
