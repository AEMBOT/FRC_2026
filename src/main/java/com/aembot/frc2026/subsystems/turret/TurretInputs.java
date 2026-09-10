package com.aembot.frc2026.subsystems.turret;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Turret-only closed-loop telemetry; unavailable fields stay NaN in old logs and no-op IOs. */
public class TurretInputs implements LoggableInputs {
  public boolean available = false;
  public boolean signalsOK = false;
  public boolean timestampsValid = false;
  public String controlMode = "Unavailable";
  public double readTimestampSeconds = Double.NaN;
  public double referenceDegrees = Double.NaN;
  public double referenceVelocityDegreesPerSecond = Double.NaN;
  public double controllerErrorDegrees = Double.NaN;
  public double referenceAgeSeconds = Double.NaN;
  public double referenceVelocityAgeSeconds = Double.NaN;
  public double positionAgeSeconds = Double.NaN;
  public double velocityAgeSeconds = Double.NaN;
  public double outputSignalsAgeSeconds = Double.NaN;
  public double oldestSignalAgeSeconds = Double.NaN;

  @Override
  public void toLog(LogTable table) {
    table.put("Available", available);
    table.put("SignalsOK", signalsOK);
    table.put("TimestampsValid", timestampsValid);
    table.put("ControlMode", controlMode);
    table.put("ReadTimestampSeconds", readTimestampSeconds);
    table.put("ReferenceDegrees", referenceDegrees);
    table.put("ReferenceVelocityDegreesPerSecond", referenceVelocityDegreesPerSecond);
    table.put("ControllerErrorDegrees", controllerErrorDegrees);
    table.put("ReferenceAgeSeconds", referenceAgeSeconds);
    table.put("ReferenceVelocityAgeSeconds", referenceVelocityAgeSeconds);
    table.put("PositionAgeSeconds", positionAgeSeconds);
    table.put("VelocityAgeSeconds", velocityAgeSeconds);
    table.put("OutputSignalsAgeSeconds", outputSignalsAgeSeconds);
    table.put("OldestSignalAgeSeconds", oldestSignalAgeSeconds);
  }

  @Override
  public void fromLog(LogTable table) {
    available = table.get("Available", false);
    signalsOK = table.get("SignalsOK", false);
    timestampsValid = table.get("TimestampsValid", false);
    controlMode = table.get("ControlMode", "Unavailable");
    readTimestampSeconds = table.get("ReadTimestampSeconds", Double.NaN);
    referenceDegrees = table.get("ReferenceDegrees", Double.NaN);
    referenceVelocityDegreesPerSecond = table.get("ReferenceVelocityDegreesPerSecond", Double.NaN);
    controllerErrorDegrees = table.get("ControllerErrorDegrees", Double.NaN);
    referenceAgeSeconds = table.get("ReferenceAgeSeconds", Double.NaN);
    referenceVelocityAgeSeconds = table.get("ReferenceVelocityAgeSeconds", Double.NaN);
    positionAgeSeconds = table.get("PositionAgeSeconds", Double.NaN);
    velocityAgeSeconds = table.get("VelocityAgeSeconds", Double.NaN);
    outputSignalsAgeSeconds = table.get("OutputSignalsAgeSeconds", Double.NaN);
    oldestSignalAgeSeconds = table.get("OldestSignalAgeSeconds", Double.NaN);
  }
}
