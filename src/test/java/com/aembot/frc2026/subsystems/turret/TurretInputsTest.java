package com.aembot.frc2026.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;

class TurretInputsTest {
  @Test
  void preservesProfileDirectionAndFreshnessThroughLogRoundTrip() {
    TurretInputs original = new TurretInputs();
    original.available = true;
    original.signalsOK = true;
    original.timestampsValid = true;
    original.controlMode = "MotionMagicVoltageFOC";
    original.referenceDegrees = 210.5;
    original.referenceVelocityDegreesPerSecond = -120.0;
    original.controllerErrorDegrees = -4.5;
    original.readTimestampSeconds = 66.414;
    original.referenceAgeSeconds = 0.011;
    original.referenceVelocityAgeSeconds = 0.012;
    original.positionAgeSeconds = 0.017;
    original.velocityAgeSeconds = 0.018;
    original.outputSignalsAgeSeconds = 0.021;
    original.oldestSignalAgeSeconds = 0.022;
    LogTable table = new LogTable(0);
    original.toLog(table);
    TurretInputs restored = new TurretInputs();
    restored.fromLog(table);
    assertTrue(restored.available);
    assertTrue(restored.signalsOK);
    assertTrue(restored.timestampsValid);
    assertEquals(original.controlMode, restored.controlMode);
    assertEquals(210.5, restored.referenceDegrees);
    assertEquals(-120.0, restored.referenceVelocityDegreesPerSecond);
    assertEquals(-4.5, restored.controllerErrorDegrees);
    assertEquals(66.414, restored.readTimestampSeconds);
    assertEquals(0.011, restored.referenceAgeSeconds);
    assertEquals(0.012, restored.referenceVelocityAgeSeconds);
    assertEquals(0.017, restored.positionAgeSeconds);
    assertEquals(0.018, restored.velocityAgeSeconds);
    assertEquals(0.021, restored.outputSignalsAgeSeconds);
    assertEquals(0.022, restored.oldestSignalAgeSeconds);
  }

  @Test
  void oldLogsCannotMasqueradeAsFreshZeroErrorMeasurements() {
    TurretInputs inputs = new TurretInputs();
    inputs.available = true;
    inputs.signalsOK = true;
    inputs.timestampsValid = true;
    inputs.referenceDegrees = 180;
    inputs.controllerErrorDegrees = 0;
    inputs.oldestSignalAgeSeconds = 0;
    inputs.fromLog(new LogTable(0));
    assertFalse(inputs.available);
    assertFalse(inputs.signalsOK);
    assertFalse(inputs.timestampsValid);
    assertEquals("Unavailable", inputs.controlMode);
    assertTrue(Double.isNaN(inputs.referenceDegrees));
    assertTrue(Double.isNaN(inputs.controllerErrorDegrees));
    assertTrue(Double.isNaN(inputs.oldestSignalAgeSeconds));
  }
}
