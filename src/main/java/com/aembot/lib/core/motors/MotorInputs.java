package com.aembot.lib.core.motors;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Generic inputs for a motor */
public class MotorInputs implements LoggableInputs {
  public double velocityUnitsPerSecond = 0.0;
  public double positionUnits = 0.0;
  public double appliedVolts = 0.0;
  public double currentStatorAmps = 0.0;
  public double currentSupplyAmps = 0.0;
  public double rawRotorPosition = 0.0;

  @Override
  public void toLog(LogTable table) {
    table.put("VelocityUnitsPerSecond", velocityUnitsPerSecond);
    table.put("PositionUnits", positionUnits);
    table.put("AppliedVolts", appliedVolts);
    table.put("CurrentStatorAmps", currentStatorAmps);
    table.put("CurrentSupplyAmps", currentSupplyAmps);
    table.put("RawRotorPosition", rawRotorPosition);
  }

  @Override
  public void fromLog(LogTable table) {
    velocityUnitsPerSecond = table.get("VelocityUnitsPerSecond", velocityUnitsPerSecond);
    positionUnits = table.get("UnitPosition", positionUnits);
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    currentStatorAmps = table.get("CurrentStatorAmps", currentStatorAmps);
    currentSupplyAmps = table.get("CurrentSupplyAmps", currentSupplyAmps);
    rawRotorPosition = table.get("RawRotorPosition", rawRotorPosition);
  }
}
