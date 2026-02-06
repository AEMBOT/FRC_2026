package com.aembot.lib.subsystems.hood;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class HoodInputs implements LoggableInputs {

  @Override
  public void toLog(LogTable table) {
    table.put("isLogging", true);
  }

  @Override
  public void fromLog(LogTable table) {}
}
