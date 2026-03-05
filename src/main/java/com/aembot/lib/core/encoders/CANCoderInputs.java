package com.aembot.lib.core.encoders;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class CANCoderInputs implements LoggableInputs {
  public double absolutePositionRotations = 0.0;
  public double velocityRotationsPerSecond = 0.0;

  @Override
  public void toLog(LogTable table) {
    table.put("AbsolutePositionRotations", absolutePositionRotations);
    table.put("VelocityRotationsPerSecond", velocityRotationsPerSecond);
  }

  @Override
  public void fromLog(LogTable table) {
    absolutePositionRotations = table.get("AbsolutePositionRotations", absolutePositionRotations);
    velocityRotationsPerSecond =
        table.get("VelocityRotationsPerSecond", velocityRotationsPerSecond);
  }
}
